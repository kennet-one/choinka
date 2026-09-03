[CmdletBinding()]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$vswhere = Join-Path ([Environment]::GetEnvironmentVariable('ProgramFiles(x86)')) 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path -LiteralPath $vswhere)) { throw 'MSVC Build Tools are required.' }
$vs = (& $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath).Trim()
if (-not $vs) { throw 'MSVC x64 toolchain was not found.' }
$devcmd = Join-Path $vs 'Common7\Tools\VsDevCmd.bat'
$setup = '"' + $devcmd + '" -no_logo -arch=x64 -host_arch=x64 >nul && set'
$lines = & $env:COMSPEC /d /s /c $setup
if ($LASTEXITCODE -ne 0) { throw 'MSVC environment initialization failed.' }
foreach ($line in $lines) {
    $separator = $line.IndexOf('=')
    if ($separator -gt 0) {
        [Environment]::SetEnvironmentVariable($line.Substring(0, $separator), $line.Substring($separator + 1), 'Process')
    }
}
$output = Join-Path $root 'build\host-pump-tests'
New-Item -ItemType Directory -Path $output -Force | Out-Null
Push-Location $output
try {
    $sources = @("$root\main\pump_controller.c", "$root\main\water_level_health.c",
        "$root\main\pump_controller_selftest.c", "$root\tests\pump_host_test.c")
    & cl.exe /nologo /std:c11 /W4 /WX "/I$root\main" @sources /Fe:pump-host-test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed.' }
    & (Join-Path $output 'pump-host-test.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Pump host tests failed.' }
} finally {
    Pop-Location
}
