@echo off
rem PIX capture of an M6 globe frame: the CDLOD node draw under the `globe` marker.
cd /d C:\Users\lordc\source\repos\GAGAME
"C:\Program Files\Microsoft PIX\2603.25\pixtool.exe" launch C:\Users\lordc\source\repos\GAGAME\build\bin\gagame.exe --working-directory=C:\Users\lordc\source\repos\GAGAME --command-line="--globe --globe-cam 41,-68,1800 --frames 40" take-capture save-capture C:\Users\lordc\source\repos\GAGAME\globe_frame.wpix
