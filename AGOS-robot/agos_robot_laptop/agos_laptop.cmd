@echo off
rem AGOS test robot, laptop side, from Windows: double-click this file, or in
rem PowerShell (in this folder) type  .\agos_laptop start  or  .\agos_laptop explore
rem
rem It runs the ./agos_laptop script in WSL (Ubuntu), which uses Docker there,
rem so it does exactly what ./agos_laptop does in the Ubuntu terminal.
rem .\agos_laptop help lists every command. A double-click is the same as
rem .\agos_laptop start
setlocal
set "AGOS_ARGS=%*"
if not defined AGOS_ARGS set "AGOS_ARGS=start"
wsl.exe --cd "%~dp0." bash ./agos_laptop %AGOS_ARGS%
if errorlevel 1 pause
