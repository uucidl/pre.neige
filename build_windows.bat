@set CommonCLFlags=-O2 -Zi
@set MicrosDir=uu.micros
@set MicrosCLFlags=-I%MicrosDir%/include/ -I%MicrosDir%/libs/glew/include/
@set MicrosCLFlags=%MicrosDir%/libs/glew/src/glew.c -DGLEW_STATIC opengl32.lib %MicrosCLFlags%
@set MicrosCLFlags=-I%MicrosDir%/libs %MicrosDir%/libs/NT_x64/glfw3.lib gdi32.lib user32.lib shell32.lib %MicrosCLFlags%
@set MicrosCLFLags=ole32.lib %MicrosCLFlags%
@set MicrosCLFLags=-MD %MicrosCLFlags%
@set HIDAPIDir=hidapi
mkdir output
cl -c -Fo:output\hid.obj -I %HIDAPIDir%\hidapi %HIDAPIDir%\windows\hid.c
@set HIDAPIProducts=output\hid.obj Setupapi.lib
@set CLWarnings=-W4
@set CLWarnings=-wd4312 -wd4100 -wd4459 -wd4838 -wd4456 -wd4068 %CLWarnings%
@set CLWarnings=-wd4457 -wd4244 -wd4305 -wd4267 -wd4996 -wd4505 %CLWarnings%
@set CLWarnings=-wd4800 %CLWarnings%
@set Defines=-DOS=OS_WINDOWS -DCPU=CPU_X86_64 -DNEIGE_SLOW
cl %CommonCLFlags% -Fo:output\ -Fe:neige neige_unit.cpp %MicrosCLFlags% %HIDAPIProducts% -nologo -EHsc -FC %Defines% %CLWarnings% ^
  -link -NODEFAULTLIB:libcmt.lib -NODEFAULTLIB:libc.lib -NODEFAULTLIB:libcd.lib ^
        -NODEFAULTLIB:libcmtd.lib -NODEFAULTLIB:msvcrtd.lib ^
        legacy_stdio_definitions.lib
