@echo off

if [%2]==[] goto usage

set dataset=%1
set map=%2

@echo Generating video for %dataset%/%map%...

ffmpeg -framerate 24 -i %dataset%/images/%map%/annotation/%map%_%%06d_annotation.png %map%_annotation.mp4
ffmpeg -framerate 24 -i %dataset%/images/%map%/camera/%map%_%%06d.png %map%_camera.mp4
ffmpeg -framerate 24 -i %dataset%/images/%map%/depth/%map%_%%06d_depth.png %map%_depth.mp4
ffmpeg -i %map%_annotation.mp4 -i %map%_camera.mp4 -i %map%_depth.mp4 -filter_complex vstack=inputs=3 %map%.mp4

goto :eof
:usage
@echo Usage: %0 dataset_path map
exit /B 1