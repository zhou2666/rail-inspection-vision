@echo off
if "%~1"=="" (
    echo Drag .yuv file onto this bat
    pause
    exit /b
)
set /p FPS=Enter fps (from recording output):
echo Converting: %~nx1 at %FPS% fps
"%~dp0ffmpeg.exe" -y -f rawvideo -pix_fmt nv12 -s 800x480 -r %FPS% -i "%~1" -c:v libx264 "%~dpn1.mp4"
echo Done! Output: %~dpn1.mp4
pause
