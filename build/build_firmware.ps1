param([Parameter(Mandatory=$true)][string]$Uv4Path)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
$project = Join-Path $repoRoot 'firmware\bisai300\mdk\rt1064.uvprojx'
$outputDir = Join-Path $PSScriptRoot 'generated'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$logPath = Join-Path $outputDir 'keil_rebuild.log'
$proc = Start-Process -FilePath $Uv4Path -ArgumentList @('-r', ('"' + $project + '"'), '-t', 'nor_sdram_zf_dtcm', '-j0', '-o', ('"' + $logPath + '"')) -WindowStyle Hidden -PassThru -Wait
$logText = Get-Content -LiteralPath $logPath -Raw
$logText -split "`n" | Where-Object { $_ -match 'Error\(s\)|Warning\(s\)|Program Size|Using Compiler' } | Write-Output
if ($proc.ExitCode -ne 0 -or $logText -notmatch '0 Error\(s\), 0 Warning\(s\)') {
    throw ('Keil build did not pass: ' + $logPath)
}
Write-Output 'KEIL_BUILD_OK; compilation only, no flash/download requested.'
