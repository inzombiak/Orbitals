@echo off

mkdir ..\build
pushd ..\build
dir

cl -Zi /I"C:\Development Libraries\glm" /I"C:\Development Libraries\glew-2.0.0\include" ..\code\main.cpp /link /LIBPATH:"C:\Development Libraries\glew-2.0.0\lib\Release\x64" glew32s.lib user32.lib gdi32.lib opengl32.lib 
popd