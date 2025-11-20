###################################################################################
# Ripple APIs Demo makefile
###################################################################################
all: Demo

Demo: Demo.o ../HAL-Lib/TI_Ripple_HAL.a
	gcc -o Demo Demo.o -L. ../HAL-Lib/TI_Ripple_HAL.a

Demo.o: Demo.c
	gcc -c Demo.c -o Demo.o

../HAL-Lib/TI_Ripple_HAL.a: ../HAL-Lib/RippleHAL.o ../HAL-Lib/CLI_Write.o
	ar rcs ../HAL-Lib/TI_Ripple_HAL.a ../HAL-Lib/RippleHAL.o ../HAL-Lib/CLI_Write.o

../HAL-Lib/RippleHAL.o: ../HAL-Lib/RippleHAL.c
	gcc -c ../HAL-Lib/RippleHAL.c -o ../HAL-Lib/RippleHAL.o

../HAL-Lib/CLI_Write.o: ../HAL-Lib/CLI_Write.c
	gcc -c ../HAL-Lib/CLI_Write.c -o ../HAL-Lib/CLI_Write.o