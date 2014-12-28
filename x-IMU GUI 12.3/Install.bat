@echo off
if exist "x-IMU GUI\x-IMU GUI\publish\setup.exe" (
"x-IMU GUI\x-IMU GUI\publish\setup.exe"
exit
) else (
echo Cannot find setup.exe.  Please ensure that you have extracted all files from the .zip folder before attempting to run install.bat.
pause
)