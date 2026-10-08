@echo off
title Safety Rover - Common Network Diagnostic Tool
echo ======================================================================
echo    ESP32 SAFETY ROVER - COMMON WI-FI NETWORK DIAGNOSTIC
echo ======================================================================
echo.
echo Detecting Active Network Interfaces and Local IP Addresses...
echo.

powershell -NoProfile -Command ^
  "Write-Host '---------------- Wi-Fi Connection ----------------' -ForegroundColor Cyan; " ^
  "$wifi = netsh wlan show interfaces | Out-String; " ^
  "if ($wifi -match 'SSID\s*:\s*(.+)') { Write-Host ('  Connected SSID : ' + $Matches[1].Trim()) -ForegroundColor Green } else { Write-Host '  No active Wi-Fi profile detected' -ForegroundColor Yellow }; " ^
  "Write-Host ''; " ^
  "Write-Host '---------------- Laptop Local IPv4 Addresses ----------------' -ForegroundColor Cyan; " ^
  "$ips = Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.InterfaceAlias -notmatch 'Loopback' -and $_.IPAddress -notmatch '^169\.254' }; " ^
  "foreach ($i in $ips) { " ^
  "  Write-Host ('  Interface: ' + $i.InterfaceAlias + ' -> http://' + $i.IPAddress + ':8000') -ForegroundColor White; " ^
  "}; " ^
  "Write-Host ''; " ^
  "Write-Host '---------------- Rover mDNS Resolution Test ----------------' -ForegroundColor Cyan; " ^
  "try { " ^
  "  $resolved = [System.Net.Dns]::GetHostAddresses('esp32-rover.local'); " ^
  "  Write-Host ('  [OK] esp32-rover.local resolved to: ' + $resolved[0].IPAddressToString) -ForegroundColor Green; " ^
  "} catch { " ^
  "  Write-Host '  [INFO] esp32-rover.local not found yet (Rover may be booting or offline)' -ForegroundColor Yellow; " ^
  "}; " ^
  "Write-Host '============================================================' -ForegroundColor Cyan; "

echo.
echo Configuration Advice:
echo  1. In firmware\esp32_rover\esp32_rover.ino: Ensure 'ssid' and 'password' match your Wi-Fi.
echo  2. In Mobile App Settings: Set 'DMS Host' to your Laptop IP above with port 8000.
echo  3. In Mobile App Settings: Set 'Rover Gateway' to http://esp32-rover.local (or Rover IP).
echo.
pause
