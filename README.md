JoyMegaAvr3

Sega Mega Drive joypad to MSX adapter based on AVR MCU

- Modes:
  
	1. Default mode
		- MD buttons A and B mapped to MSX triggers 1 and 2
		- X and Y buttons switch autofire for trigger 1 and 2
		- Z button reset both autofires
		- MODE button switches alternate mode (clocked MSX_OUT pin) between Nyyrikki Mouse (RED), MegaDrive passthrough (BLUE) and OFF (purple)

	2. Alternate mode Nyyrikki Mouse (RED/D1)
		- Direction buttons move mouse cursor
		- Mouse buttons A, B, X, Start, Mode
		- Mouse scroll buttons Y, Z
		- Mouse slow motion - hold C button or 8bitdo M30 right bumper
	
	3. Alternate mode Mega Drive passthrough mode (BLUE/D2)
		- Buttons passed 1:1 to MSX
	
- Bill Of Materials:
	- PCB - use file from "Gerber" folder when ordering pcb
	- 3D printed case - print case with included STL file - translucent PETG prefferred
	- MCU Atmega328PB-M
	- D1 - LED SMD RED 0805 ex. IN-S85CS5R
	- D2 - LED SMD BLUE 0805 ex. IN-S85CS5B
	- Capacitors 100nF 0805 - 3 pcs.
	- Resistors 1 KOhm 0805 - 2 pcs.
	- Resistor 10 KOhm 0805 - 1 pc.
	- Resistor 470 Ohm 0805 - 2 pcs.
	- Male and female 9 pin DE9 connectors
 
 - !IMPORTANT! Please remember to solder 3rd 100nF capacitor between V and G pads of ISP connector as it's decoupling main MCU power.
 - AVR fuses - default, no change needed
 - Program ATmega329pb-m with "firmware.hex" using onboard ISP connector. Recommend using common AVR programmer like STK500v2 compatible hardware and avrdude+avrdudess software. Temporarily soldering wires to the ISP pads and proper connection to programmer is required for programming.
