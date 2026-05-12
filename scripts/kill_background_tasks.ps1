# Script to kill background processes that might be left hanging by background tasks
$processNames = @("cargo", "rustc", "python")

foreach ($name in $processNames) {
    $processes = Get-Process -Name $name -ErrorAction SilentlyContinue
    if ($processes) {
        Write-Host "Killing $($processes.Count) $name process(es)..."
        Stop-Process -Name $name -Force -ErrorAction SilentlyContinue
    } else {
        Write-Host "No $name processes found."
    }
}

# Be careful with node as it might kill the VS Code extension host or dev server. 
# We only kill node processes that don't have a window title (background tasks).
$nodeProcesses = Get-Process -Name "node" -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowTitle -eq "" }
if ($nodeProcesses) {
    Write-Host "Killing $($nodeProcesses.Count) background node process(es)..."
    foreach ($p in $nodeProcesses) {
        Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue
    }
} else {
    Write-Host "No background node processes found."
}

Write-Host "Cleanup complete."
