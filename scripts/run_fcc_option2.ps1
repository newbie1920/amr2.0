Param(
  [Parameter(ValueFromRemainingArguments = $true)]
  [string[]]$Args
)

$ErrorActionPreference = "Stop"

$fccDir = "C:\\Users\\ADMIN\\free-claude-code"
$proxyHealthUrl = "http://127.0.0.1:8082/health"

function Start-FccProxy {
  Write-Host "[FCC] Starting proxy server (same terminal)..."

  $proxyProcess = $null
  try {
    $proxyProcess = Start-Process -FilePath "fcc-server" -WorkingDirectory $fccDir -NoNewWindow -PassThru
    return $proxyProcess
  } catch {
    Write-Host "[FCC] fcc-server not found; falling back to uv run fcc-server..."
    return Start-Process -FilePath "uv" -WorkingDirectory $fccDir -NoNewWindow -PassThru -ArgumentList @("run", "fcc-server")
  }
}

$proxy = Start-FccProxy

Write-Host "[FCC] Waiting for proxy health ($proxyHealthUrl) ..."
for ($i = 1; $i -le 10; $i++) {
  try {
    $resp = Invoke-WebRequest -Uri $proxyHealthUrl -UseBasicParsing -TimeoutSec 1
    if ($resp.StatusCode -ge 200 -and $resp.StatusCode -lt 400) { break }
  } catch {
    Start-Sleep -Seconds 1
  }
}

try {
  Write-Host "[FCC] Opening Free Claude Code (same terminal)..."
  Push-Location $fccDir
  try {
    & fcc-claude @Args
  } catch {
    Write-Host "[FCC] fcc-claude not found; falling back to uv run fcc-claude..."
    & uv run fcc-claude @Args
  }
} finally {
  Pop-Location
  if ($proxy -and -not $proxy.HasExited) {
    Write-Host "[FCC] Stopping proxy server..."
    try { Stop-Process -Id $proxy.Id -Force } catch {}
  }
}
