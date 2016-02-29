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
@set CLWarnings= -W4 -wd4312 -wd4100 -wd4459 -wd4838 -wd4456 -wd4068 -wd4457 -wd4244 -wd4305 -wd4267 -wd4996
cl -Fo:output\ -Fe:neige neige.cpp %MicrosCLFlags% %HIDAPIProducts% -nologo -EHsc -FC -DOS=OS_WINDOWS -DCPU=CPU_IA64 -DNEIGE_SLOW %CLWarnings%