NES-I/O
========

an idea by Arcade-TV from circuit-board.de :) Thank you very much!!!

The board replaces the silver box in NES frontloader. On this PCB the following components are included:

- AC input jack
- AC to DC conversion
- MultiAV (SNES and GC(*) are supported) 
- PIC16F684 microcontroller to control the NESRGB color palette and reset the console using your controller :D

All components you need are stated on the silkscreen. An complete overview as well as an English installation guide is coming soon ;)

Grab the firmware for the PIC16F684 here: https://github.com/borti4938/Switchless-Mods/tree/master/NES/IGR_for_NESRGB/
- current version: use the 'normal version' and flash the nesrgb_igr_684_var_led.hex
- oldest PCB version (with three 10k resistors next to the 74*125): use the 'risky version' and flash nesrgb_igr_684_risky_var_led.hex

(*) 3d-printed Helder/Buffalowing connector on assemblergames (http://assemblergames.com/l/threads/f-s-nes-top-loader-snes-style-multi-a-v-socket.50494/) also fits  
    thanks for figuring out, splits (shmups forum)