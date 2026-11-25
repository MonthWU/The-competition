$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$env:PYTHONPATH = Join-Path $projectRoot 'src\car26_system'
python -m compileall -q (Join-Path $projectRoot 'src')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
python -m unittest discover -s (Join-Path $projectRoot 'tests') -v
exit $LASTEXITCODE
