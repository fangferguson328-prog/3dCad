# Run in Windows PowerShell 5.1. This script inventories tools; it does not install them.
[CmdletBinding()]
param(
    [string]$SourceRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$QtDir,
    [string]$OutputJson,
    [switch]$VerifyPackage
)
$ErrorActionPreference = 'Stop'

function Find-QtKit {
    param([string]$ExplicitDirectory)
    if ($ExplicitDirectory) {
        $candidate = Join-Path $ExplicitDirectory 'bin\qmake.exe'
        if (-not (Test-Path -LiteralPath $candidate)) { throw "qmake.exe missing: $candidate" }
        return (Resolve-Path -LiteralPath $ExplicitDirectory).Path
    }
    $roots = @('C:\Qt', 'D:\Qt', 'C:\tools\Qt', (Join-Path $env:USERPROFILE 'Qt'))
    if ($env:QTDIR) { $roots += $env:QTDIR }
    $onPath = Get-Command qmake.exe -ErrorAction SilentlyContinue
    if ($onPath) { $roots += (Split-Path -Parent (Split-Path -Parent $onPath.Source)) }
    $candidates = @()
    foreach ($root in ($roots | Select-Object -Unique)) {
        if (-not (Test-Path -LiteralPath $root)) { continue }
        $direct = Join-Path $root 'bin\qmake.exe'
        if (Test-Path -LiteralPath $direct) { $candidates += $direct }
        foreach ($level1 in Get-ChildItem -LiteralPath $root -Directory) {
            $one = Join-Path $level1.FullName 'bin\qmake.exe'
            if (Test-Path -LiteralPath $one) { $candidates += $one }
            foreach ($level2 in Get-ChildItem -LiteralPath $level1.FullName -Directory) {
                $two = Join-Path $level2.FullName 'bin\qmake.exe'
                if (Test-Path -LiteralPath $two) { $candidates += $two }
                foreach ($level3 in Get-ChildItem -LiteralPath $level2.FullName -Directory) {
                    $three = Join-Path $level3.FullName 'bin\qmake.exe'
                    if (Test-Path -LiteralPath $three) { $candidates += $three }
                }
            }
        }
    }
    $ranked = @()
    foreach ($path in ($candidates | Select-Object -Unique)) {
        $version = (& $path -query QT_VERSION | Out-String).Trim()
        $spec = (& $path -query QMAKE_XSPEC | Out-String).Trim()
        $kit = Split-Path -Leaf (Split-Path -Parent (Split-Path -Parent $path))
        if ($spec -notmatch 'win32-msvc' -or $kit -notmatch '_64$') { continue }
        $rank = 9
        if ($version -eq '5.14.2' -and $kit -eq 'msvc2017_64') { $rank = 0 }
        elseif ($version -like '5.15.*' -and $kit -eq 'msvc2019_64') { $rank = 1 }
        if ($rank -lt 9) { $ranked += [pscustomobject]@{ Path = $path; Version = [version]$version; Rank = $rank } }
    }
    $selected = $ranked | Sort-Object Rank, @{ Expression = 'Version'; Descending = $true } | Select-Object -First 1
    if ($selected) { return Split-Path -Parent (Split-Path -Parent $selected.Path) }
    return $null
}

function Find-VisualStudio {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path -LiteralPath $vswhere)) { return $null }
    $instances = & $vswhere -all -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -format json | ConvertFrom-Json
    $instance = $instances | Sort-Object @{ Expression = { [version]$_.installationVersion }; Descending = $true } | Select-Object -First 1
    if ($instance) { return $instance.installationPath }
    return $null
}

function Get-SafePackagePath {
    param([string]$Root, [string]$Relative)
    $prefix = [IO.Path]::GetFullPath($Root).TrimEnd('\') + '\'
    $full = [IO.Path]::GetFullPath((Join-Path $Root $Relative))
    if (-not $full.StartsWith($prefix, [StringComparison]::OrdinalIgnoreCase)) { throw "Unsafe manifest path: $Relative" }
    return $full
}

function Test-PackageIntegrity {
    param([string]$Root)
    $manifestPath = Join-Path $Root 'deployment\package-integrity.json'
    if (-not (Test-Path -LiteralPath $manifestPath)) { throw "Package integrity manifest missing: $manifestPath" }
    $manifest = Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($manifest.schemaVersion -ne 1 -or $manifest.target -ne 'windows-x64-release') { throw 'Unsupported package manifest.' }
    foreach ($entry in $manifest.files) {
        $path = Get-SafePackagePath $Root $entry.path
        if (-not (Test-Path -LiteralPath $path)) { throw "Package file missing: $($entry.path)" }
        $hash = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash
        if ($hash -ne $entry.sha256 -or (Get-Item -LiteralPath $path).Length -ne $entry.size) { throw "Package file changed: $($entry.path)" }
    }
    return $manifest
}

# Dot-source this script to load reusable functions without running the inventory.
if ($MyInvocation.InvocationName -eq '.') { return }

$SourceRoot = [IO.Path]::GetFullPath($SourceRoot)
$kit = Find-QtKit $QtDir
$vs = Find-VisualStudio
$qtVersion = $null
$qtSpec = $null
if ($kit) {
    $qtVersion = (& (Join-Path $kit 'bin\qmake.exe') -query QT_VERSION | Out-String).Trim()
    $qtSpec = (& (Join-Path $kit 'bin\qmake.exe') -query QMAKE_XSPEC | Out-String).Trim()
}
$required = @(
    '3dCad.pro', 'HarmoMotionPlan.cpp', 'XPLGenerator.cpp', 'tinyxml2.cpp',
    'includes\glm\glm\detail\glm.cpp', 'includes\glfw\lib-vc2015\glfw3dll.lib',
    'includes\Intel RealSense SDK 2.0\lib\x64\realsense2.lib',
    'includes\opencv\build\x64\vc16\lib\opencv_world470.lib',
    'includes\SDK V2.5\sdk-x64\libs\x64\EftSdk.lib',
    'includes\SDK V2.5\sdk-x64\libs\x64\log4cpp.lib',
    'includes\SDK V2.5\sdk-x64\libs\x64\rlibcpp-bcc.lib',
    'includes\SDK V2.5\sdk-x64\libs\x64\rlibcpp-tool.lib',
    'includes\robot\64\libharmorobotservice.lib',
    'includes\KW_SDK_D_1.0\SDK_1.0\SDK_windows_1.0\lib\CPP\camera.lib',
    'includes\KW_SDK_D_1.0\SDK_1.0\SDK_windows_1.0\lib\CPP\enumerate.lib'
)
$missing = @($required | Where-Object { -not (Test-Path -LiteralPath (Join-Path $SourceRoot $_)) })
$packageVerified = $false
if ($VerifyPackage) { $null = Test-PackageIntegrity $SourceRoot; $packageVerified = $true }
$result = [ordered]@{
    SourceRoot = $SourceRoot
    WindowsVersion = [Environment]::OSVersion.VersionString
    Is64BitOS = [Environment]::Is64BitOperatingSystem
    PowerShellVersion = $PSVersionTable.PSVersion.ToString()
    AnsiCodePage = [Text.Encoding]::Default.CodePage
    QtDir = $kit
    QtVersion = $qtVersion
    QtSpec = $qtSpec
    VisualStudio = $vs
    HasV142 = [bool]($vs -and (Get-ChildItem -Path (Join-Path $vs 'VC\Tools\MSVC') -Directory -ErrorAction SilentlyContinue | Where-Object Name -like '14.2*'))
    HasWindowsSDK = [bool](Test-Path (Join-Path ${env:ProgramFiles(x86)} 'Windows Kits\10\Include'))
    DisplayAdapters = @(Get-CimInstance Win32_VideoController -ErrorAction SilentlyContinue | Select-Object Name, DriverVersion)
    MissingSourceOrLibraries = $missing
    PackageVerified = $packageVerified
    ReadyForBuild = [bool]($kit -and $vs -and $missing.Count -eq 0 -and [Environment]::Is64BitOperatingSystem -and (Test-Path (Join-Path ${env:ProgramFiles(x86)} 'Windows Kits\10\Include')))
}
$json = $result | ConvertTo-Json -Depth 6
Write-Output $json
if ($OutputJson) { [IO.File]::WriteAllText([IO.Path]::GetFullPath($OutputJson), $json, (New-Object Text.UTF8Encoding($false))) }
