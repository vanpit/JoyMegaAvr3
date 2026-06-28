JoyMegaAvr3

Sega Mega Drive joypad to MSX adapter based on AVR MCU

- Modes:
  
1. Default mode
	- MD buttons A and B mapped to MSX triggers 1 and 2
	- X and Y buttons switches autofire for trigger 1 and 2
	- Z button reset both autofires
	- MODE button switches alternate mode (clocked MSX_OUT pin) between Nyyrikki Mouse (RED), MegaDrive passthrough (BLUE) and OFF (purple)

2. Alternate mode Nyyrikki Mouse (RED)
	- Direction button move mouse cursor
	- Mouse buttons A, B, X, Start, Mode
	- Mouse scroll buttons Y, Z
	- Mouse slow motion - hold C button or 8bitdo M30 right bumper
	
3. Alternate mode Mega Drive passthrough mode
	- Buttons passed 1:1 to MSX
	
- Bill Of Materials:
	- PCB
	- 3D printed case - translucent PETG preffered
	- MCU Atmega328PB-M
	- LED SMD RED 0805 ex. IN-S85CS5R
	- LED SMD BLUE 0805 ex. IN-S85CS5B
	- Capacitors 100nF 0805 - 3 pcs.
	- Resistors 1 KOhm 0805 - 2 pcs.
	- Resistor 10 KOhm 0805 - 1 pc.
	- Resistor 470 Ohm 0805 - 2 pcs.
	- Male and female 9 pin DE9 connectors
 
 - Please remember to solder 3rd 100nF capacitor between V and G pads of ISP connector as it's decoupling main MCU power.
