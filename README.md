# NESRGB-IGR
here you find PCBs and Codes for the NESRGB-IGR :)

If you successfully add the NESRGB-IGR you have the following functionalities available:

Controller: Start+Select+ ...
    - ... A+B pressed for <2s: short reset
    - ... A+B pressed for >2s: long reset (going back to main menu if a power pak is used or in EDN8 reset to game is enabled)
    - ... D-Pad right: Forward switching through the palette and RGB off (RGB off only available if set by jumper)
    - ... D-Pad left: Backward switching through palette
Reset:
    - short push <~750ms: reset
    - second punsh while PIC resets the NES/FC: reset as long long as button is held
    - keeping pressed: forward switching through the palette and RGB off with delay of ~750ms for each palette 
	
	
Please note that one is not allowed to connect the PIC directly to the NESRGB.
If you do so, the CPLD on the NESRGB will be destroyed because the PIC running with NESRGB-IGR-code is designed to run with +5V.
I recommend to use a 74LVX125 running with 3.3V in between - this IC allows inputs up to +5V.