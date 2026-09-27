@echo off
rem Doble clic para abrir la terminal del carro (se reconecta sola).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0terminal_carro.ps1" %*
pause
