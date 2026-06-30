$Owner = "Ridrik"
$Repo = "DroneLab-Demo"

$Dist = Join-Path $PSScriptRoot "..\dist"
New-Item -ItemType Directory -Force $Dist | Out-Null

$Releases = Invoke-RestMethod `
    -Uri "https://api.github.com/repos/$Owner/$Repo/releases"

$Release = $Releases |
Where-Object { $_.prerelease } |
Select-Object -First 1

$Asset = $Release.assets |
Where-Object { $_.name -match "win" -and $_.name -match "\.zip$" } |
Select-Object -First 1

if (-not $Asset) {
    Write-Error "No Windows SDK found."
    exit 1
}

$tmp = Join-Path $env:TEMP $Asset.name

Invoke-WebRequest $Asset.browser_download_url -OutFile $tmp

Expand-Archive $tmp -DestinationPath $Dist -Force

Remove-Item $tmp