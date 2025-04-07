# ======================================
# GU-Systems Autoinstaller for Windows
# ======================================

param(
    [string]$PythonVersion = "3.11",
    [string]$CondaEnvName  = "ecm",
    [string]$SourceConfigPath = "/scripts/dev_setup.ps1"
)

function Write-Title($msg) {
    Write-Host ("==================== $msg ====================") -ForegroundColor Blue
}

function Write-Step($msg) {
    Write-Host "[ECM] $msg" -ForegroundColor Yellow
}

function Ask-Question($prompt) {
    Write-Host "`n$prompt" -NoNewline
    return Read-Host " "
}

function Is-Yes($response) {
    return $response -match '^(y|yes)$'
}

$ErrorActionPreference = "Stop"

Write-Title "GU-Systems Autoinstaller"

$response = Ask-Question "Use a Conda environment? (y/n)"
if (Is-Yes $response) {
    Write-Step "Creating or checking Conda environment..."

    conda deactivate 2>$null | Out-Null

    $envList = conda env list
    $envExists = $envList -match $CondaEnvName

    if ($envExists) {
        $responseReplace = Ask-Question "The '$CondaEnvName' environment already exists. Replace it? (y/n)"
        if (Is-Yes $responseReplace) {
            Write-Step "Removing existing Conda environment..."
            conda env remove --name $CondaEnvName -y
        } else {
            Write-Step "Skipping creation; using existing environment."
            $envCreated = $true
        }
    }

    if (-not $envCreated) {
        Write-Step "Creating Conda environment $CondaEnvName with Python $PythonVersion..."
        conda create --name $CondaEnvName python=$PythonVersion -y
    }

    conda activate $CondaEnvName
}

Write-Step "Installing dependencies from requirements.txt..."
$pipCmd = "pip install --disable-pip-version-check -r .\requirements.txt"
Invoke-Expression $pipCmd

Write-Step "Installing Pre-Commit checkers"
$pipCmd = "pip install pre-commit"
Invoke-Expression $pipCmd

$pipCmd = "pre-commit install"
Invoke-Expression $pipCmd

Write-Step "Checking API Keys..."

if (-not $env:OPENAI_API_KEY) {
    $respApi = Ask-Question "OPENAI_API_KEY is not set. Would you like to add it now? (y/n)"
    if (Is-Yes $respApi) {
        $apiKey = Ask-Question "Please provide your OPENAI_API_KEY (sk-...):"

        # Save key to .env file
        $envFile = ".\.env"
        if (-not (Test-Path $envFile)) {
            New-Item $envFile -ItemType File | Out-Null
        }
        "OPENAI_API_KEY=$apiKey" | Out-File $envFile -Encoding UTF8
        Write-Step "OPENAI_API_KEY saved to $envFile."
    }
}

Write-Step "Building Profile for default configs"
$scriptPath = if ($PSCommandPath) { $PSCommandPath } else { $MyInvocation.MyCommand.Path }
$scriptDir = Split-Path $scriptPath -Parent
$devSetupPath = Join-Path $scriptDir "dev_setup.ps1"

$profileLine = ". '$devSetupPath'"

if (!(Test-Path (Split-Path $PROFILE))) {
    New-Item -ItemType Directory -Path (Split-Path $PROFILE) -Force | Out-Null
}

if (Test-Path $PROFILE) {
    $perfilContenido = Get-Content $PROFILE

    if ($perfilContenido -notcontains $profileLine) {
        Add-Content $PROFILE "`n"
        Add-Content $PROFILE $profileLine
        Write-Host "Added config to ps1 profile"
        Write-Host "Please restart the console to apply changes"
    }
}
else {
    Add-Content $PROFILE $profileLine
}
