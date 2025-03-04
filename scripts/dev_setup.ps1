$scriptDir = Split-Path $MyInvocation.MyCommand.Definition
$SOURCE = Join-Path $scriptDir '..' | Resolve-Path | Select-Object -ExpandProperty Path

if ($env:PYTHONPATH) {
    $env:PYTHONPATH = $env:PYTHONPATH + ";" + $SOURCE
} else {
    $env:PYTHONPATH = $SOURCE
}

$env:PYTHONWARNINGS = "ignore:easy_install command is deprecated,ignore:setup.py install is deprecated,ignore:"
$env:GU_SOURCE_DIR = $SOURCE

function gutree {
    tree -I interfaces -I install -I log -I __pycache__ -I docs -I build -I __init__.py
}

