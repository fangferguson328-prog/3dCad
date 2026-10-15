# Collect only the current executable, Qt runtime, bundled x64 SDK DLLs and assets.
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$SourceRoot,
    [Parameter(Mandatory = $true)][string]$QtDir,
    [Parameter(Mandatory = $true)][string]$Executable,
    [Parameter(Mandatory = $true)][string]$Destination
)
$ErrorActionPreference = 'Stop'
foreach ($path in @($Executable, (Join-Path $QtDir 'bin\windeployqt.exe'))) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Missing deployment input: $path" }
}
$Destination = [IO.Path]::GetFullPath($Destination)
$null = New-Item -ItemType Directory -Path $Destination -Force
Copy-Item -LiteralPath $Executable -Destination (Join-Path $Destination '3dCad.exe')
& (Join-Path $QtDir 'bin\windeployqt.exe') --release --compiler-runtime --no-translations --dir $Destination (Join-Path $Destination '3dCad.exe')
if ($LASTEXITCODE -ne 0) { throw "windeployqt failed: $LASTEXITCODE" }
$runtime = @(
    'includes\glfw\lib-vc2015\glfw3.dll',
    'includes\Intel RealSense SDK 2.0\bin\x64\realsense2.dll',
    'includes\opencv\build\x64\vc16\bin\opencv_world470.dll',
    'includes\opencv\build\x64\vc16\bin\opencv_videoio_ffmpeg470_64.dll',
    'includes\opencv\build\x64\vc16\bin\opencv_videoio_msmf470_64.dll',
    'includes\SDK V2.5\sdk-x64\libs\x64\EftSdk.dll',
    'includes\SDK V2.5\sdk-x64\libs\x64\log4cpp.dll',
    'includes\SDK V2.5\sdk-x64\libs\x64\rlibcpp-bcc.dll',
    'includes\SDK V2.5\sdk-x64\libs\x64\rlibcpp-tool.dll',
    'includes\SDK V2.5\sdk-x64\libs\x64\NTEventLogAppender.dll',
    'includes\SDK V2.5\sdk-x64\libs\x64\log4cpp.nt.property',
    'includes\KW_SDK_D_1.0\SDK_1.0\SDK_windows_1.0\lib\CPP\camera.dll',
    'includes\KW_SDK_D_1.0\SDK_1.0\SDK_windows_1.0\lib\CPP\enumerate.dll',
    'includes\robot\64\libharmorobotservice.dll'
)
foreach ($relative in $runtime) {
    $path = Join-Path $SourceRoot $relative
    if (-not (Test-Path -LiteralPath $path)) { throw "SDK runtime missing: $path" }
    Copy-Item -LiteralPath $path -Destination $Destination
}
foreach ($asset in @('0414_config.json', 'harmo_processing_design.md')) {
    $path = Join-Path $SourceRoot $asset
    if (Test-Path -LiteralPath $path) { Copy-Item -LiteralPath $path -Destination $Destination }
}
$null = New-Item -ItemType Directory -Path (Join-Path $Destination 'samples') -Force
foreach ($fixture in @('test_cube.obj', 'test_plane.obj', 'test_cylinder_mesh.obj')) {
    $path = Join-Path $SourceRoot ('pcd\' + $fixture)
    if (Test-Path -LiteralPath $path) { Copy-Item -LiteralPath $path -Destination (Join-Path $Destination 'samples') }
}
$vc = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (Test-Path -LiteralPath $vc) {
    $vs = (& $vc -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath | Out-String).Trim()
    if ($vs) {
        $redistRoot = Join-Path $vs 'VC\Redist\MSVC'
        $redistVersions = Get-ChildItem -LiteralPath $redistRoot -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending
        foreach ($version in $redistVersions) {
            $crt = Get-ChildItem -LiteralPath (Join-Path $version.FullName 'x64') -Directory -ErrorAction SilentlyContinue | Where-Object Name -like 'Microsoft.VC*.CRT' | Select-Object -First 1
            if ($crt) {
                Get-ChildItem -LiteralPath $crt.FullName -Filter '*.dll' | Copy-Item -Destination $Destination
                break
            }
        }
    }
}
# Cloud desktops can use Qt's bundled software OpenGL if the display lacks 3.3.
$softGl = Join-Path $QtDir 'bin\opengl32sw.dll'
if (Test-Path -LiteralPath $softGl) { Copy-Item -LiteralPath $softGl -Destination $Destination }
$launch = @'
@echo off
cd /d "%~dp0"
set QT_OPENGL=software
set QT_LOGGING_RULES=qt.qpa.*=true
3dCad.exe > gui-startup.log 2>&1
exit /b %errorlevel%
'@
[IO.File]::WriteAllText((Join-Path $Destination 'launch-offline.cmd'), $launch, [Text.Encoding]::ASCII)
$mandatory = @('3dCad.exe', 'Qt5Core.dll', 'Qt5Gui.dll', 'Qt5Widgets.dll', 'platforms\qwindows.dll', 'realsense2.dll', 'opencv_world470.dll', 'EftSdk.dll', 'rlibcpp-bcc.dll', 'rlibcpp-tool.dll', 'log4cpp.dll', 'camera.dll', 'libharmorobotservice.dll', 'msvcp140.dll', 'vcruntime140.dll', 'concrt140.dll')
foreach ($relative in $mandatory) {
    if (-not (Test-Path -LiteralPath (Join-Path $Destination $relative))) { throw "Runtime closure incomplete: $relative" }
}
$inventory = @(Get-ChildItem -LiteralPath $Destination -Recurse -File | ForEach-Object {
    [pscustomobject]@{ Path = $_.FullName.Substring($Destination.Length + 1); Sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash; Size = $_.Length }
})
[IO.File]::WriteAllText((Join-Path $Destination 'runtime-integrity.json'), ($inventory | ConvertTo-Json -Depth 4), (New-Object Text.UTF8Encoding($false)))
Write-Host "Runtime deployed at $Destination"
