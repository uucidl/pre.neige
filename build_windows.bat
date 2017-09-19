@echo off
setlocal
set Defines=-DOS=OS_WINDOWS -DCPU=CPU_X86_64
set ModularityCFlags=-Zs -W4 -wd4505
REM -- Modularity tests
cl %ModularityCFlags% %Defines% -nologo aabb2.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo algorithms.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo allocators.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo arithmetics.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo array2.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo circular_ordinate.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo cpu.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo ds4.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo errors.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo floats.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo integers.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo os.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo perlin_noise2.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo pointers.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo vec3.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
cl %ModularityCFlags% %Defines% -nologo vine_effect.cpp
if %ERRORLEVEL% neq 0 goto in_error_exit
REM -- Build objects
mkdir output
set HIDAPIDir=hidapi
set HIDAPIProducts=output\hid.obj Setupapi.lib
cl -c -Fo:output\hid.obj -I %HIDAPIDir%\hidapi %HIDAPIDir%\windows\hid.c
if %ERRORLEVEL% neq 0 goto in_error_exit
set Defines=%Defines% -DNEIGE_SLOW
set CommonCLFlags=-O2 -Zi
set MicrosDir=uu.micros
set MicrosCLFlags=-I%MicrosDir%/include/ -I%MicrosDir%/libs/glew/include/
set MicrosCLFlags=%MicrosDir%/libs/glew/src/glew.c -DGLEW_STATIC opengl32.lib %MicrosCLFlags%
set MicrosCLFlags=-I%MicrosDir%/libs %MicrosDir%/libs/NT_x64/glfw3.lib gdi32.lib user32.lib shell32.lib %MicrosCLFlags%
set MicrosCLFLags=ole32.lib %MicrosCLFlags%
set MicrosCLFLags=-MD %MicrosCLFlags%

set CLWarnings=-W4
set CLWarnings=-wd4312 -wd4100 -wd4459 -wd4838 -wd4456 -wd4068 %CLWarnings%
set CLWarnings=-wd4457 -wd4244 -wd4305 -wd4267 -wd4996 -wd4505 %CLWarnings%
set CLWarnings=-wd4800 %CLWarnings%
cl %CommonCLFlags% -Fo:output\ -Fe:output\neige.exe %HIDAPIProducts% -nologo -EHsc -FC %Defines% %CLWarnings% ^
  neige_unit.cpp %MicrosCLFlags% ^
  -link -NODEFAULTLIB:libcmt.lib -NODEFAULTLIB:libc.lib -NODEFAULTLIB:libcd.lib ^
        -NODEFAULTLIB:libcmtd.lib -NODEFAULTLIB:msvcrtd.lib ^
        legacy_stdio_definitions.lib
if %ERRORLEVEL% neq 0 goto in_error_exit
:in_success_exit
endlocal
exit /b 0
:in_error_exit
endlocal
exit /b 1
