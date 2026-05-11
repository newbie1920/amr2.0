param(
    [string]$Command = "help",
    [string]$Port = "COM3",
    [int]$Baud = 115200
)

try {
    $serial = New-Object System.IO.Ports.SerialPort($Port, $Baud)
    $serial.ReadTimeout = 1000
    $serial.Open()
    Start-Sleep -Milliseconds 300
    $serial.WriteLine($Command)
    Start-Sleep -Milliseconds 800
    $response = $serial.ReadExisting()
    if ($response) { Write-Host $response }
    $serial.Close()
    Write-Host "`n[OK] Sent '$Command' to $Port"
} catch {
    Write-Host "[ERROR] $($_.Exception.Message)"
    Write-Host "Make sure Serial Monitor is CLOSED before running this task."
}
