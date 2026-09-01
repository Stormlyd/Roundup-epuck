[CmdletBinding()]
param(
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Release',
    [string]$QtRoot = 'D:\Qt\5.9.7\mingw53_32',
    [string]$MinGwRoot = 'D:\Qt\Tools\mingw530_32',
    [switch]$SkipTests
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Invoke-NativeCommand {
    param(
        [Parameter(Mandatory = $true)]
        [string]$FilePath,
        [string[]]$ArgumentList = @()
    )

    & $FilePath @ArgumentList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $FilePath $($ArgumentList -join ' ')"
    }
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if ($repoRoot -match '[^\x00-\x7F]') {
    throw "Qt 5.9 MinGW cannot build this project from a non-ASCII path. Clone it to an English path such as D:\ros2\Roundup-epuck."
}

$qmake = Join-Path $QtRoot 'bin\qmake.exe'
$deployQt = Join-Path $QtRoot 'bin\windeployqt.exe'
$compiler = Join-Path $MinGwRoot 'bin\g++.exe'
$make = Join-Path $MinGwRoot 'bin\mingw32-make.exe'

foreach ($requiredPath in @($qmake, $deployQt, $compiler, $make)) {
    if (-not (Test-Path -LiteralPath $requiredPath -PathType Leaf)) {
        throw "Required tool not found: $requiredPath"
    }
}

$configurationLower = $Configuration.ToLowerInvariant()
$otherConfiguration = if ($Configuration -eq 'Release') { 'debug' } else { 'release' }
$buildDir = Join-Path $repoRoot ".build-windows-${configurationLower}"
$deployDir = Join-Path $repoRoot ".build-windows-deploy\${configurationLower}"
$oldPath = $env:Path
$locationPushed = $false

try {
    $env:Path = "$(Join-Path $MinGwRoot 'bin');$(Join-Path $QtRoot 'bin');$oldPath"
    New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

    if (-not $SkipTests) {
        $testDir = Join-Path $repoRoot '.build-core-audit'
        New-Item -ItemType Directory -Force -Path $testDir | Out-Null
        $testExe = Join-Path $testDir 'zooid_core_tests.exe'
        $testSources = @(
            'tests\zooid_core_tests.cpp',
            'manager\ZooidSpeedCodec.cpp',
            'manager\ZooidPursuitRoles.cpp',
            'manager\ZooidPursuitGeometry.cpp',
            'manager\ZooidPursuitStateMachine.cpp',
            'manager\ZooidPursuitControl.cpp',
            'manager\ZooidTestMode.cpp',
            'manager\ZooidTestTargets.cpp'
        ) | ForEach-Object { Join-Path $repoRoot $_ }

        $testArguments = @('-std=c++14', "-I$repoRoot") + $testSources + @('-o', $testExe)
        Invoke-NativeCommand -FilePath $compiler -ArgumentList $testArguments
        Invoke-NativeCommand -FilePath $testExe -ArgumentList @()
    }

    Push-Location $buildDir
    $locationPushed = $true
    Invoke-NativeCommand -FilePath $qmake -ArgumentList @(
        (Join-Path $repoRoot 'ZooidManager.pro'),
        '-spec',
        'win32-g++',
        "CONFIG+=${configurationLower}",
        "CONFIG-=${otherConfiguration}"
    )
    $jobs = [Math]::Max(1, [Environment]::ProcessorCount)
    Invoke-NativeCommand -FilePath $make -ArgumentList @("-j${jobs}", $configurationLower)
    Pop-Location
    $locationPushed = $false

    $builtExe = Join-Path $buildDir "${configurationLower}\ZooidManager.exe"
    if (-not (Test-Path -LiteralPath $builtExe -PathType Leaf)) {
        throw "Build completed without the expected executable: $builtExe"
    }

    New-Item -ItemType Directory -Force -Path $deployDir | Out-Null
    $deployExe = Join-Path $deployDir 'ZooidManager.exe'
    Copy-Item -LiteralPath $builtExe -Destination $deployExe -Force
    Invoke-NativeCommand -FilePath $deployQt -ArgumentList @(
        "--${configurationLower}",
        '--no-translations',
        '--compiler-runtime',
        $deployExe
    )

    $jsonDir = Join-Path $deployDir 'json'
    New-Item -ItemType Directory -Force -Path $jsonDir | Out-Null
    Copy-Item -LiteralPath (Join-Path $repoRoot 'json\chargePosition.json') -Destination $jsonDir -Force

    $debugSuffix = if ($Configuration -eq 'Debug') { 'd' } else { '' }
    $requiredRuntimeFiles = @(
        $deployExe,
        (Join-Path $deployDir "Qt5Core${debugSuffix}.dll"),
        (Join-Path $deployDir "Qt5Gui${debugSuffix}.dll"),
        (Join-Path $deployDir "Qt5Widgets${debugSuffix}.dll"),
        (Join-Path $deployDir "Qt5OpenGL${debugSuffix}.dll"),
        (Join-Path $deployDir "Qt5SerialPort${debugSuffix}.dll"),
        (Join-Path $deployDir "Qt5Sql${debugSuffix}.dll"),
        (Join-Path $deployDir "platforms\qwindows${debugSuffix}.dll"),
        (Join-Path $deployDir "sqldrivers\qsqlite${debugSuffix}.dll"),
        (Join-Path $deployDir 'libgcc_s_dw2-1.dll'),
        (Join-Path $deployDir 'libstdc++-6.dll'),
        (Join-Path $deployDir 'libwinpthread-1.dll'),
        (Join-Path $deployDir 'json\chargePosition.json')
    )
    foreach ($runtimeFile in $requiredRuntimeFiles) {
        if (-not (Test-Path -LiteralPath $runtimeFile -PathType Leaf)) {
            throw "Deployment is missing a required runtime file: $runtimeFile"
        }
    }

    Write-Host "Build, tests, and deployment completed: $deployDir" -ForegroundColor Green
}
finally {
    if ($locationPushed) {
        Pop-Location
    }
    $env:Path = $oldPath
}
