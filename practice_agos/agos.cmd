@echo off
rem AGOS simulation from Windows: double-click this file, or in PowerShell
rem (in this folder) type  .\agos start  or  .\agos explore  or  .\agos stop
rem
rem It runs the ./agos script in WSL (Ubuntu), which uses Docker there, so it
rem does exactly what ./agos does in the Ubuntu terminal. .\agos help lists
rem every command. A double-click is the same as .\agos start
setlocal
set "AGOS_ARGS=%*"
if not defined AGOS_ARGS set "AGOS_ARGS=start"
wsl.exe --cd "%~dp0." bash ./agos %AGOS_ARGS%
if errorlevel 1 pause
