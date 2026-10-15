# Build an extracted package using x64 MSVC and Qt 5; source files stay unchanged.
# Example: .\deployment\build-windows.ps1 -QtDir C:\Qt\5.14.2\msvc2017_64
[CmdletBinding()]
param(
    [string]$SourceRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$QtDir,
    [string]$BuildRoot,
    [switch]$SkipPlannerTests
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'preflight-windows.ps1') -SourceRoot $SourceRoot -QtDir $QtDir

$SourceRoot = [IO.Path]::GetFullPath($SourceRoot)
$package = Test-PackageIntegrity $SourceRoot
$QtDir = Find-QtKit $QtDir
if (-not $QtDir) { throw 'Install Qt 5.14.2 msvc2017_64 or Qt 5.15.x msvc2019_64, then pass -QtDir.' }
$qmake = Join-Path $QtDir 'bin\qmake.exe'
$qtVersion = (& $qmake -query QT_VERSION | Out-String).Trim()
$qtSpec = (& $qmake -query QMAKE_XSPEC | Out-String).Trim()
if ($qtVersion -notmatch '^5\.(14\.2|15\.\d+)$' -or $qtSpec -ne 'win32-msvc' -or (Split-Path -Leaf $QtDir) -notmatch '_64$') {
    throw "Unsupported Qt kit: $QtDir ($qtVersion, $qtSpec). Use a Qt 5 MSVC x64 kit."
}
$vs = Find-VisualStudio
if (-not $vs) { throw 'MSVC x64 build tools are missing. Install the Desktop development with C++ workload.' }
$vcvars = Join-Path $vs 'VC\Auxiliary\Build\vcvarsall.bat'
if (-not (Test-Path -LiteralPath $vcvars)) { throw "Missing $vcvars" }
$hasV142 = [bool](Get-ChildItem -Path (Join-Path $vs 'VC\Tools\MSVC') -Directory | Where-Object Name -like '14.2*')
$toolsetArgument = ''
if ($hasV142) { $toolsetArgument = '-vcvars_ver=14.2' }
if (-not $BuildRoot) { $BuildRoot = Join-Path $SourceRoot ('_build\windows-release-' + (Get-Date -Format 'yyyyMMdd-HHmmss')) }
$BuildRoot = [IO.Path]::GetFullPath($BuildRoot)
foreach ($path in @($SourceRoot, $BuildRoot, $QtDir, $vs)) {
    if ($path -match '[^\x20-\x7E]|["%!]') { throw 'Use ASCII build paths without quotes, percent signs or exclamation marks (for example C:\dev\3dCad).' }
}
$stage = Join-Path $BuildRoot 'src'
$build = Join-Path $BuildRoot 'build'
$test = Join-Path $BuildRoot 'tests'
$app = Join-Path $BuildRoot 'app'
if (Test-Path -LiteralPath $stage) { throw "Staging directory already exists; choose a fresh -BuildRoot: $stage" }
$null = New-Item -ItemType Directory -Path $stage, $build, $test, $app -Force
foreach ($entry in $package.files) {
    $from = Get-SafePackagePath $SourceRoot $entry.path
    $to = Get-SafePackagePath $stage $entry.path
    $null = New-Item -ItemType Directory -Path (Split-Path -Parent $to) -Force
    Copy-Item -LiteralPath $from -Destination $to
}

# BOM and strict decoding distinguish UTF-8 files from the legacy GBK files.
# /execution-charset:.936 keeps existing fromLocal8Bit() literals consistent.
$utf8Strict = New-Object Text.UTF8Encoding($false, $true)
$utf8Bom = New-Object Text.UTF8Encoding($true)
$gbkStrict = [Text.Encoding]::GetEncoding(936, [Text.EncoderFallback]::ExceptionFallback, [Text.DecoderFallback]::ExceptionFallback)
$converted = @()
foreach ($file in Get-ChildItem -LiteralPath $stage -Recurse -File | Where-Object Extension -in @('.cpp', '.h', '.hpp', '.inl')) {
    $bytes = [IO.File]::ReadAllBytes($file.FullName)
    $encoding = 'utf-8'
    try {
        if ($bytes.Length -ge 3 -and $bytes[0] -eq 239 -and $bytes[1] -eq 187 -and $bytes[2] -eq 191) {
            $content = $utf8Strict.GetString($bytes, 3, $bytes.Length - 3)
        } else { $content = $utf8Strict.GetString($bytes) }
    } catch {
        $content = $gbkStrict.GetString($bytes)
        $encoding = 'gbk'
    }
    [IO.File]::WriteAllText($file.FullName, $content, $utf8Bom)
    if ($encoding -eq 'gbk') { $converted += $file.FullName.Substring($stage.Length + 1) }
}

# The original qmake file remains untouched. All build-only flags live here.
$stagedPro = Join-Path $stage '3dCad-deployment.pro'
$proText = @'
CONFIG -= debug debug_and_release build_all utf8_source
CONFIG += release
include(3dCad.pro)
TARGET = 3dCad
INCLUDEPATH += $$PWD
QMAKE_CXXFLAGS -= /utf-8
QMAKE_CXXFLAGS += /source-charset:utf-8 /execution-charset:.936
QMAKE_CFLAGS -= /utf-8
QMAKE_CFLAGS += /source-charset:utf-8 /execution-charset:.936
'@
[IO.File]::WriteAllText($stagedPro, $proText, $utf8Bom)

$plannerCommands = ''
if (-not $SkipPlannerTests) {
    $plannerCommands = @"
cd /d "$test"
cl /nologo /EHsc /std:c++14 /W4 /source-charset:utf-8 /execution-charset:.936 /I"$stage" "$stage\HarmoMotionPlan.cpp" "$stage\diagnostics\test_harmo_motion_plan.cpp" /Fe:HarmoMotionPlanTests.exe
if errorlevel 1 exit /b 11
HarmoMotionPlanTests.exe
if errorlevel 1 exit /b 12
"@
}
$batchPath = Join-Path $BuildRoot 'compile.cmd'
$batch = @"
@echo off
setlocal
call "$vcvars" x64 $toolsetArgument
if errorlevel 1 exit /b 2
set "PATH=$QtDir\bin;%PATH%"
$plannerCommands
cd /d "$build"
"$qmake" "$stagedPro" -spec win32-msvc
if errorlevel 1 exit /b 3
nmake /nologo
if errorlevel 1 exit /b 4
exit /b 0
"@
[IO.File]::WriteAllText($batchPath, $batch, [Text.Encoding]::ASCII)
$logPath = Join-Path $BuildRoot 'build.log'
Write-Host "Building with Qt $qtVersion; VS $vs; v142 preferred: $hasV142"
Write-Host "Staging at $stage; legacy GBK files normalized: $($converted.Count)"
$commandLine = 'call "' + $batchPath + '" > "' + $logPath + '" 2>&1'
& $env:ComSpec /d /c $commandLine
$buildExit = $LASTEXITCODE
if (Test-Path -LiteralPath $logPath) { Get-Content -LiteralPath $logPath }
if ($buildExit -ne 0) { throw "Build failed ($buildExit). See $logPath" }
$exe = Join-Path $build 'release\3dCad.exe'
if (-not (Test-Path -LiteralPath $exe)) { $exe = Join-Path $build '3dCad.exe' }
if (-not (Test-Path -LiteralPath $exe)) { throw 'Build completed but 3dCad.exe is missing.' }
& (Join-Path $PSScriptRoot 'deploy-runtime.ps1') -SourceRoot $stage -QtDir $QtDir -Executable $exe -Destination $app
$record = [ordered]@{
    BuiltAtUtc = [DateTime]::UtcNow.ToString('o')
    QtDir = $QtDir
    QtVersion = $qtVersion
    VisualStudio = $vs
    PreferredV142Available = $hasV142
    SourcePackageCreatedAtUtc = $package.createdAtUtc
    NormalizedGbkFiles = $converted
    PlannerTestsRun = -not $SkipPlannerTests
    BuildLog = $logPath
    AppDirectory = $app
    ExecutableSha256 = (Get-FileHash -LiteralPath (Join-Path $app '3dCad.exe') -Algorithm SHA256).Hash
}
[IO.File]::WriteAllText((Join-Path $app 'build-record.json'), ($record | ConvertTo-Json -Depth 6), (New-Object Text.UTF8Encoding($false)))
Write-Host "Deployable application: $app"
Write-Host 'Run app\launch-offline.cmd in the interactive Windows desktop for the GUI check.'
