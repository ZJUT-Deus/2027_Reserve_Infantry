@echo off
echo Cleaning Keil MDK build artifacts...

rmdir /s /q "MDK-ARM\Chassis"         2>nul
rmdir /s /q "MDK-ARM\DebugConfig"     2>nul
rmdir /s /q "MDK-ARM\RTE"            2>nul
rmdir /s /q "MDK-ARM\.vscode"         2>nul
rmdir /s /q "MDK-ARM\.eide"           2>nul
del /f /q "MDK-ARM\startup_stm32h723xx.lst"         2>nul
del /f /q "MDK-ARM\Chassis.uvguix.*"               2>nul
del /f /q "MDK-ARM\Chassis.code-workspace"          2>nul
del /f /q "MDK-ARM\JLinkLog.txt"                    2>nul
del /f /q "MDK-ARM\JLinkSettings.ini"               2>nul
del /f /q "MDK-ARM\CLAUDE.md"                       2>nul
del /f /q "MDK-ARM\.gitignore"                      2>nul

echo Done.
pause
