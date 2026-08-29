@echo off
rem PIX capture of an M5c frame: the swe flux/height substep chain + derive, the FFT, churn,
rem and the tessellated draw, all marked. pixtool quoting only survives cmd, not PowerShell.
cd /d C:\Users\lordc\source\repos\GAGAME
"C:\Program Files\Microsoft PIX\2603.25\pixtool.exe" launch C:\Users\lordc\source\repos\GAGAME\build\bin\gagame.exe --working-directory=C:\Users\lordc\source\repos\GAGAME --command-line="--sea --swe-spinup 0.1 --start 2026-08-28T18:30:00 --storm 3,11,95 --frames 40" take-capture save-capture C:\Users\lordc\source\repos\GAGAME\swe_frame.wpix
