#include <p16f684.inc>

; -----------------------------------------------------------------------
;   NES "In-game reset" (IGR) controller with extended control
;   for Viletims NESRGB-PCB (color switch)
;
;   Copyright (C) 2014 by Peter Bartmann <peter.bartmann@gmx.de>
;
;   This program is free software; you can redistribute it and/or modify
;   it under the terms of the GNU General Public License as published by
;   the Free Software Foundation; version 2 of the License only.
;
;   This program is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;   GNU General Public License for more details.
;
;   You should have received a copy of the GNU General Public License
;   along with this program; if not, write to the Free Software
;   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
;
; -----------------------------------------------------------------------
;
;   This program is designed to run on a PIC 16F630 microcontroller connected
;   to the controller port and NES main board. It allows an NES to be reset
;   via a standard controller.
;
;   pin configuration: (controller port pin) [Mainboard pin/pad]
;
;                                  ,-----_-----.
;             +5V (7) [CIC Pin 16] |1        14| GND (1) [CIC Pin 15]
;      Reset - out (-) [CIC Pin 7] |2  A5 A0 13| serial data in (4) [U7 74HC368 Pin 2]
;   Reset -  in (-) [Reset button] |3  A4 A1 12| latch in (3) [CPU Pin 39]
;       Number of Modes in (*) [*] |4  A3 A2 11| clk in (2) [CPU Pin 36]
;                 (green) LED1 out |5  C5 C0 10| rgb-mode: garish   out [NESRGB Pad 1]
;                   (red) LED2 out |6  C4 C1  9| rgb-mode: improved out [NESRGB Pad 2]
;                RGB_indicator out |7  C3 C2  8| rgb-mode: natural  out [NESRGB Pad 3]
;                                  `-----------'
;
;   As the internal oscillator is used, you should connect a capacitor of about 100nF between
;   Pin 1 (Vdd/+5V) and Pin 8 (Vss/GND) as close as possible to the PIC. This esures best
;   operation.
;
;   One have to use a 74LVX125 inbetween of the PIC and the NESRGB. Otherwise the CPLD on
;   the NESRGB will be destroyed :'(.
;
;   The voltage level on pin 4 gives the number of modes available.
;   - pin 4 connected to GND: four modes available  (improved, natural, garish, off)
;   - pin 4 connected to +5V: three modes available (improved, natural, garish)
;
;   Pin 7 (RGB_indicator) sets the output mode:
;      low  = RGB is off
;      high = RGB is on
;
;
;   preparation of the Resetline
;   ============================
;
;   The wire from the reset button to the CIC Pin 7 (NES frontloader) or to the CPU Pin 3
;   (Famicom) has to be cutted. Please note, that there is a capacitor anywhere at this line.
;   Please remove that cap completely or make sure that this cap stays at the CIC-/CPU-end.
;   Add a weak resistor (around 10kOhm) between Pin 3 of the PIC and ...
;   ... +5V (low-active reset, Pin 2 IGR goes to CPU Pin 3)
;   ... GND (high-active reset, Pin 2 IGR goes to CIC Pin 7)
;
;
;   controller pin numbering
;   ========================
;
;        _______________
;       |               \     (1) - GND          (5) - not connected
;       | (5) (6) (7)    \    (2) - clk          (6) - not connected
;       | (4) (3) (2) (1) |   (3) - latch        (7) - Power, +5V 
;       |_________________|   (4) - serial data
;
;   key mapping: Start + Select +                            (stream data)
;   =============================
;   A + B       Reset                                           0x0f
;   (nothing)   Pressed for ~2s -> perform Longreset (3s)       0xcf
;   Left        previous mode (had to be pressed for ~0.6s)     0xcd
;   Right       next mode     (had to be pressed for ~0.6s)     0xce
;
;  switching through the modes:
;  ============================
;
;  cycle (three modes): ,->natural--->improved--->garish->.
;                       `-------<--------<-------<--------´
;  cycle (four modes): ,->natural--->improved--->garish--->off->.
;                      `-------<--------<-------<-------<-------´
;
;  There are three possibilities to switch through the modes:
;    1. using the reset-button by holding it
;    2. using the controller with Start+Select+Right
;    3. using the controller with Start+Select+Left (reverse order of the cycle)
;
;    Holding the Reset-button or the appropriate button combination at the controller
;    lets the LED fade through the colors red, green, yellow and off (RG-LED, off
;    only if four modes available). Releasing the button(s) chooses the mode according
;    to the color shown:
;
;       red LED: RGB on, natural
;     green LED: RGB on, improved
;    yellow LED: RGB on, garish
;       LED off: RGB off (only available if Pin 4 is connected to GND)
;
;  The console needs a reset or a restart (hard-reset) if the RGB-mode is switched from
;  off to on or vise versa.
;
; -----------------------------------------------------------------------
; Palette-Switch Info
; 0 = shorted to ground. 1 = unconnected.
;
; V1.0,1.3
; 123 - PALETTE
; 000 - off         <- used by this code
; 001 - off
; 010 - off
; 011 - garish      <- used by this code
; 100 - off
; 101 - improved    <- used by this code
; 110 - natural     <- used by this code
; 111 - off
;
; V1.4+
; 123 - PALETTE
; 000 - off         <- used by this code
; 001 - off
; 010 - garish
; 011 - garish      <- used by this code
; 100 - improved
; 101 - improved    <- used by this code
; 110 - natural     <- used by this code
; 111 - off         
; 
;
; -----------------------------------------------------------------------

; -----------------------------------------------------------------------
; Configuration bits: adapt to your setup and needs

    __CONFIG _INTOSCIO & _IESO_OFF & _WDT_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOD_OFF

CA_LED   set 0 ; 0 = LED with common cathode, 1 = LED with common anode

; -----------------------------------------------------------------------
; macros and definitions

M_movff macro   fromReg, toReg  ; move filereg to filereg
        movfw   fromReg
        movwf   toReg
        endm

M_movpf macro   fromPORT, toReg ; move PORTx to filereg
        movfw   fromPORT
        andlw   0x3f
        movwf   toReg
        endm

M_movlf macro   literal, toReg  ; move literal to filereg
        movlw   literal
        movwf   toReg
        endm

M_beff  macro   compReg1, compReg2, branch  ; branch if two fileregs are equal
        movfw   compReg1
        xorwf   compReg2, w
        btfsc   STATUS, Z
        goto    branch
        endm

M_bepf  macro   compPORT, compReg, branch   ; brach if PORTx equals compReg (ignoring bit 6 and 7)
        movfw   compPORT
        xorwf   compReg, w
        andlw   0x3f
        btfsc   STATUS, Z
        goto    branch
        endm

M_belf  macro   literal, compReg, branch  ; branch if a literal is stored in filereg
        movlw   literal
        xorwf   compReg, w
        btfsc   STATUS, Z
        goto    branch
        endm

M_celf  macro   literal, compReg, call_func  ; call if a literal is stored in filereg
        movlw   literal
        xorwf   compReg, w
        btfsc   STATUS, Z
        call    call_func
        endm

M_delay_x05ms   macro   literal ; delay about literal x 05ms
                movlw   literal
                movwf   reg_repetition_cnt
                call    delay_x05ms
                endm

#define skipnext_for_lowreset    btfsc   reg_reset_type, bit_reset_type

; -----------------------------------------------------------------------

CTRL_DATA   EQU 0
CTRL_LATCH  EQU 1
CTRL_CLK    EQU 2
NUM_MODES   EQU 3
RESET_IN    EQU 4
RESET_OUT   EQU 5

LED_RED     EQU 4
LED_GREEN   EQU 5

; reg_port_buffer     EQU 0x20
reg_ctrl_data       EQU 0x21
reg_overflow_cnt    EQU 0x22
reg_repetition_cnt  EQU 0x23
reg_current_mode    EQU 0x30
reg_previous_mode   EQU 0x31
reg_reset_type      EQU 0x40
reg_ctrl_reset      EQU 0x41

RGB_off             EQU 0x00    ; code for RGB off
RGB_natural         EQU 0x01    ; code for RGB on natural
RGB_improved        EQU 0x02    ; code for RGB on improved
RGB_garish          EQU 0x03    ; code for RGB on garish
code_mode_overflow  EQU 0x04    ; code for a non available mode (last mode + 1)

code_RGB_off        EQU 0x00
code_RGB_natural    EQU 0x0b
code_RGB_improved   EQU 0x0d
code_RGB_garish     EQU 0x0e

code_led_off    EQU 0x00
code_led_red    EQU (1<<LED_RED)
code_led_green  EQU (1<<LED_GREEN)
code_led_yellow EQU (1<<LED_RED) ^ (1<<LED_GREEN)

bit_reset_type          EQU RESET_OUT
code_reset_low_active   EQU (0<<bit_reset_type)  ; 0x00
code_reset_high_active  EQU (1<<bit_reset_type)  ; 0x20

bit_ctrl_reset_perform_long EQU 5
bit_ctrl_reset_flag         EQU 7

delay_05ms_t0_overflows EQU 0x14    ; prescaler T0 set to 1:2 @ 8MHz
repetitions_045ms       EQU 0x09
repetitions_200ms       EQU 0x28
repetitions_300ms       EQU 0x3c
repetitions_480ms       EQU 0x60
repetitions_1000ms      EQU 0xc8
repetitions_mode_delay  EQU 0x94    ; around 740ms

; -----------------------------------------------------------------------
; buttons

BUTTON_A    EQU 7
BUTTON_B    EQU 6
BUTTON_Sl   EQU 5
BUTTON_St   EQU 4
BUTTON_Up   EQU 3
BUTTON_Dw   EQU 2
BUTTON_Le   EQU 1
BUTTON_Ri   EQU 0

; -----------------------------------------------------------------------

; code memory
 org    0x0000
    clrf    STATUS      ; 00h Page 0, Bank 0
    nop                 ; 01h
    nop                 ; 02h
    goto    start       ; 03h begin program / Initializing

 org    0x0004  ; jump here on interrupt with GIE set (should not appear)
    return      ; return with GIE unset

 org    0x0005
idle
    clrf    reg_ctrl_data
    btfsc   PORTA, CTRL_LATCH
    goto    wait_ctrl_read      ; go go go
    bcf     INTCON, RAIF
    skipnext_for_lowreset
    goto    idle_loop_reset_high

idle_loop_reset_low
    btfsc	INTCON, RAIF            ; data latch changed?
    goto    wait_ctrl_read          ; yes
    btfss   PORTA, RESET_IN         ; reset pressed?
    goto    check_reset             ; yes
    btfsc	INTCON, RAIF            ; data latch changed?
    goto    wait_ctrl_read          ; yes
    goto    idle_loop_reset_low     ; no -> repeat loop

idle_loop_reset_high
    btfsc	INTCON, RAIF            ; data latch changed?
    goto    wait_ctrl_read          ; yes
    btfsc   PORTA, RESET_IN         ; reset pressed?
    goto    check_reset             ; yes
    btfsc	INTCON, RAIF            ; data latch changed?
    goto    wait_ctrl_read          ; yes
    goto    idle_loop_reset_high    ; no -> repeat loop


wait_ctrl_read
    btfsc   PORTA, CTRL_LATCH
    goto    wait_ctrl_read

read_Button_A
    bcf     INTCON, INTF
    btfsc   PORTA, CTRL_DATA
    bsf     reg_ctrl_data, BUTTON_A
postwait_Button_A
    btfss   INTCON, INTF
    goto    postwait_Button_A
    bcf     INTCON, RAIF        ; from now on, no IOC at the data latch shall appear

    bcf     INTCON, INTF
    movfw   PORTA

read_Button_B
    btfss   INTCON, INTF
    movfw   PORTA
    andlw   (1 << CTRL_DATA)
    btfss   INTCON, INTF
    goto    read_Button_B
store_Button_B
    btfss   STATUS, Z
    bsf     reg_ctrl_data, BUTTON_B

    bcf     INTCON, INTF
    movfw   PORTA

read_Button_Sl
    btfss   INTCON, INTF
    movfw   PORTA
    andlw   (1 << CTRL_DATA)
    btfss   INTCON, INTF
    goto    read_Button_Sl
store_Button_Sl
    btfss   STATUS, Z
    bsf     reg_ctrl_data, BUTTON_Sl

    bcf     INTCON, INTF
    movfw   PORTA

read_Button_St
    btfss   INTCON, INTF
    movfw   PORTA
    andlw   (1 << CTRL_DATA)
    btfss   INTCON, INTF
    goto    read_Button_St
store_Button_St
    btfss   STATUS, Z
    bsf     reg_ctrl_data, BUTTON_St

    bcf     INTCON, INTF
    movfw   PORTA

read_Button_Up
    btfss   INTCON, INTF
    movfw   PORTA
    andlw   (1 << CTRL_DATA)
    btfss   INTCON, INTF
    goto    read_Button_Up
store_Button_Up
    btfss   STATUS, Z
    bsf     reg_ctrl_data, BUTTON_Up

    bcf     INTCON, INTF
    movfw   PORTA

read_Button_Dw
    btfss   INTCON, INTF
    movfw   PORTA
    andlw   (1 << CTRL_DATA)
    btfss   INTCON, INTF
    goto    read_Button_Dw
store_Button_Dw
    btfss   STATUS, Z
    bsf     reg_ctrl_data, BUTTON_Dw

    bcf     INTCON, INTF
    movfw   PORTA

read_Button_Le
    btfss   INTCON, INTF
    movfw   PORTA
    andlw   (1 << CTRL_DATA)
    btfss   INTCON, INTF
    goto    read_Button_Le
store_Button_Le
    btfss   STATUS, Z
    bsf     reg_ctrl_data, BUTTON_Le

    bcf     INTCON, INTF
    movfw   PORTA

read_Button_Ri
    btfss   INTCON, INTF
    movfw   PORTA
    andlw   (1 << CTRL_DATA)
    btfss   INTCON, INTF
    goto    read_Button_Ri
store_Button_Ri
    btfss   STATUS, Z
    bsf     reg_ctrl_data, BUTTON_Ri

    btfsc   INTCON, RAIF
    goto    idle            ; another IOC on data latch appeared -> invalid read
	

checkkeys
    M_belf  0x0f, reg_ctrl_data, ctrl_reset                 ; Start+Select+A+B
    btfsc   reg_ctrl_reset, bit_ctrl_reset_flag             ; Start+Select+A+B previously detected?
    goto    doreset                                         ; if yes, perform a reset
    M_belf  0xcd, reg_ctrl_data, domode_prev                ; Start+Select+D-Pad Left
    M_belf  0xce, reg_ctrl_data, domode_next                ; Start+Select+D-Pad Right
	goto	idle

ctrl_reset
    btfss           reg_ctrl_reset, bit_ctrl_reset_flag
    goto            flash_led_rst                               ; first loop -> confirm combo-detection and set ctrl_reset_flag
    btfsc           reg_ctrl_reset, bit_ctrl_reset_perform_long
    goto            dolongreset
    M_delay_x05ms   repetitions_045ms
    incf            reg_ctrl_reset, 1
    goto            idle

doreset
    banksel         TRISA                   ; Bank 1
    bcf             TRISA, RESET_OUT
    banksel         PORTA                   ; Bank 0
    M_movff         reg_reset_type, PORTA
    call            set_reg_current_mode
    M_delay_x05ms   repetitions_480ms
    skipnext_for_lowreset
    goto            releasereset_wait_high

releasereset_wait_low
    btfss   PORTA, RESET_IN         ; reset pressed?
    goto    releasereset_wait_low   ; yes -> still waitin'
    goto    release_reset

releasereset_wait_high
    btfsc   PORTA, RESET_IN         ; reset pressed?
    goto    releasereset_wait_high  ; yes -> still waitin'
    goto    release_reset

dolongreset
    banksel         TRISA                   ; Bank 1
    bcf             TRISA, RESET_OUT
    banksel         PORTA                   ; Bank 0
    M_movff         reg_reset_type, PORTA
    call            set_reg_current_mode
    M_delay_x05ms   repetitions_1000ms
    M_delay_x05ms   repetitions_1000ms
    M_delay_x05ms   repetitions_1000ms

release_reset
    movfw   reg_reset_type
    xorlw   (1<<RESET_OUT)                      ; invert to release
    movwf   PORTA
    banksel TRISA                               ; Bank 1
    bsf     TRISA, RESET_OUT
    banksel PORTA                               ; Bank 0
    M_movff reg_current_mode, reg_previous_mode
    clrf    reg_ctrl_reset
    goto    idle

domode_prev
    btfsc   PORTA, NUM_MODES                        ; four modes available?
    goto    domode_prev_3                           ; if no, check only 3 modes

domode_prev_4
    M_celf  RGB_off, reg_current_mode, modepreset  ; preset mode if current mode is RGB_off
    goto    domode_prev_fin                         ; shortcut to set new mode

domode_prev_3
    M_celf  RGB_natural, reg_current_mode, modepreset ; preset mode if current mode is RGB_natural

domode_prev_fin
    decf    reg_current_mode, 1
    call    set_reg_current_mode_pre    ; change mode during runtime
    call    mode_change_delay
    goto    idle

domode_next
    incf    reg_current_mode, 1
    M_celf  code_mode_overflow, reg_current_mode, modereset
    call    set_reg_current_mode_pre                        ; change mode during runtime
    M_delay_x05ms   repetitions_mode_delay
    goto            idle

; --------check reset routines--------

check_reset
    call    delay_05ms                      ; software debounce
    call    delay_05ms                      ; software debounce
    call    delay_05ms                      ; software debounce
    skipnext_for_lowreset
    goto    check_reset_deb_high

check_reset_deb_low
    btfsc   PORTA, RESET_IN         ; reset still pressed?
    goto    idle                    ; no -> goto idle
    goto    check_reset_continue
    
check_reset_deb_high
    btfss   PORTA, RESET_IN ; reset still pressed?
    goto    idle            ; no -> goto idle

check_reset_continue
    M_movlf repetitions_mode_delay , reg_repetition_cnt  ; repeat delay_05ms x-times
    skipnext_for_lowreset
    goto    check_reset_high

check_reset_low
    call    delay_05ms
    btfsc   PORTA, RESET_IN         ; reset still pressed?
    goto    doreset                 ; no -> perform a reset
    decfsz  reg_repetition_cnt, 1   ; delay_05ms repeated x-times?
    goto    check_reset_low         ; no -> stay in this loop
    goto    reset_next_mode         ; yes -> next mode

check_reset_high
    call    delay_05ms
    btfss   PORTA, RESET_IN         ; reset still pressed?
    goto    doreset                 ; no -> perform a reset
    decfsz  reg_repetition_cnt, 1   ; delay_05ms repeated x-times?
    goto    check_reset_high        ; no -> stay in this loop
;    goto    reset_next_mode         ; yes -> next mode

reset_next_mode
    incf    reg_current_mode, 1                             ; next mode
    M_celf  code_mode_overflow, reg_current_mode, modereset
    call    set_reg_current_mode_pre                        ; change mode during runtime
;    call    setleds                                         ; change mode once reset is released

    M_movlf repetitions_mode_delay , reg_repetition_cnt  ; repeat delay_05ms x-times
    skipnext_for_lowreset
    goto    reset_high_mode_change_loop

reset_low_mode_change_loop
    call    delay_05ms
    btfsc   PORTA, RESET_IN             ; reset still pressed?
    goto    idle                        ; no -> end procedure (runtime change)
    decfsz  reg_repetition_cnt, 1       ; delay_05ms repeated x-times?
    goto    reset_low_mode_change_loop  ; no -> stay in this loop
    goto    reset_next_mode             ; yes -> next mode

reset_high_mode_change_loop
    call    delay_05ms
    btfss   PORTA, RESET_IN             ; reset still pressed?
    goto    idle                        ; no -> end procedure (runtime change)
    decfsz  reg_repetition_cnt, 1       ; delay_05ms repeated x-times?
    goto    reset_high_mode_change_loop ; no -> stay in this loop
    goto    reset_next_mode             ; yes -> next mode

; --------mode, led, delay and save_mode calls--------

set_reg_current_mode_pre
    M_belf  RGB_off, reg_previous_mode, setleds ; reg_prev_mode == 0?
                                                ; -> cannot change mode in runtime anyway

set_reg_current_mode_pre_nz ; reg_prev_mode |= 0
                            ; -> can change mode in runtime if reg_current_mode is non zero too
    M_belf  RGB_off, reg_current_mode, setleds ; reg_current_mode is zero

set_reg_current_mode ; includes setting pin 7 and LED
    M_belf  RGB_natural, reg_current_mode, modeset_natural
    M_belf  RGB_improved, reg_current_mode, modeset_improved
    M_belf  RGB_garish, reg_current_mode, modeset_garish
    M_belf  RGB_off, reg_current_mode, modeset_off
    ; the following should not appear
    goto    modeset_default

modeset_off
    btfsc   PORTA, NUM_MODES
    goto    modeset_default
    movlw   code_RGB_off ^ code_led_off
    if CA_LED       ; if common anode:
      xorlw   0x30  ; invert output
    endif
    movwf	PORTC
    goto    save_mode

modeset_default
    M_movlf RGB_natural, reg_current_mode
    goto    set_reg_current_mode_pre

modeset_natural
    movlw   code_RGB_natural ^ code_led_red
    if CA_LED       ; if common anode:
      xorlw   0x30  ; invert output
    endif
    movwf	PORTC
    goto    save_mode

modeset_improved
    movlw   code_RGB_improved ^ code_led_green
    if CA_LED       ; if common anode:
      xorlw   0x30  ; invert output
    endif
    movwf	PORTC
    goto    save_mode

modeset_garish
    movlw   code_RGB_garish ^ code_led_yellow
    if CA_LED       ; if common anode:
      xorlw   0x30  ; invert output
    endif
    movwf	PORTC
    goto    save_mode

setleds
    M_belf  RGB_off, reg_current_mode, setleds_off
    M_belf  RGB_natural, reg_current_mode, setleds_red
    M_belf  RGB_improved, reg_current_mode, setleds_green
    M_belf  RGB_garish, reg_current_mode, setleds_yellow
    ; should not appear - set to a valid mode
    goto    modeset_default

setleds_off
    movfw   PORTC
    andlw   0x0f            ; save RGB mode
    iorlw   code_led_off    ; set LED
    if CA_LED               ; if common anode:
      xorlw   0x30          ; invert output
    endif
    movwf	PORTC
    goto    save_mode

setleds_red
    movfw   PORTC
    andlw   0x0f            ; save RGB mode
    iorlw   code_led_red    ; set LED
    if CA_LED               ; if common anode:
      xorlw   0x30          ; invert output
    endif
    movwf	PORTC
    goto    save_mode

setleds_green
    movfw   PORTC
    andlw   0x0f            ; save RGB mode
    iorlw   code_led_green  ; set LED
    if CA_LED               ; if common anode:
      xorlw   0x30          ; invert output
    endif
    movwf	PORTC
    goto    save_mode

setleds_yellow
    movfw   PORTC
    andlw   0x0f            ; save RGB mode
    iorlw   code_led_yellow ; set LED
    if CA_LED               ; if common anode:
      xorlw   0x30          ; invert output
    endif
    movwf	PORTC
;    goto    save_mode

save_mode
    movfw   reg_current_mode    ; load current mode to work
    banksel EEADR               ; save to EEPROM. note: banksels take two cycles each!
    movwf   EEDAT
    bsf     EECON1, WREN
    M_movlf 0x55, EECON2
    M_movlf 0xaa, EECON2
    bsf     EECON1, WR
    banksel	PORTA		; two cycles again
    return

flash_led_rst
    movlw   PORTC
    if CA_LED               ; if common anode:
      xorlw   0x30          ; invert output
    endif
    andlw   0x30
    btfsc   STATUS, Z
    goto    flash_led_rst_on_off_on

flash_led_rst_off_on_off
    M_movpf         PORTC, reg_ctrl_reset
    movlw           code_led_red            ; set LED
    if CA_LED                               ; if common anode:
      xorlw           0x30                  ; invert output
    endif
    movwf           PORTC
    M_delay_x05ms   repetitions_300ms
    goto            flash_led_rst_end

flash_led_rst_on_off_on
    M_movpf         PORTC, reg_ctrl_reset
    movlw           code_led_off            ; set LED
    if CA_LED                               ; if common anode:
      xorlw           0x30                  ; invert output
    endif
    movwf           PORTC
    M_delay_x05ms   repetitions_300ms
;    goto            flash_led_rst_end

flash_led_rst_end
    M_movff reg_ctrl_reset, PORTC
    clrf    reg_ctrl_reset
    bsf     reg_ctrl_reset, bit_ctrl_reset_flag
    goto    idle


modepreset
    M_movlf code_mode_overflow, reg_current_mode
    return


modereset
    btfsc   PORTA, NUM_MODES
    goto    modereset_1

modereset_0
    M_movlf RGB_off, reg_current_mode
    return

modereset_1
    M_movlf RGB_natural, reg_current_mode
    return


delay_05ms
    clrf    TMR0                ; start timer (operation clears prescaler of T0)
    banksel TRISA
    movfw   OPTION_REG
    andlw   0xf0
    movwf   OPTION_REG
    banksel PORTA
    M_movlf delay_05ms_t0_overflows, reg_overflow_cnt
    bsf     INTCON, T0IE        ; enable timer interrupt

delay_05ms_loop_pre
    bcf     INTCON, T0IF

delay_05ms_loop
    btfss   INTCON, T0IF
    goto    delay_05ms_loop
    decfsz  reg_overflow_cnt, 1
    goto    delay_05ms_loop_pre
    bcf     INTCON, T0IE        ; disable timer interrupt
    return

delay_x05ms
    call    delay_05ms
    decfsz  reg_repetition_cnt, 1
    goto    delay_x05ms
    return
    

mode_change_delay
    M_movlf repetitions_mode_delay , reg_repetition_cnt

mode_change_delay_loop
    call    delay_05ms
    decfsz  reg_repetition_cnt, 1
    goto    mode_change_delay_loop
    return

; --------initialization--------

start
    clrf    PORTA
    clrf    PORTC
    M_movlf 0x07, CMCON0            ; GPIO2..0 are digital I/O (not connected to comparator)
    M_movlf 0x18, INTCON            ; enable interrupts: INTE (interrupt on rising/falling edge on A2) and RAIE
    banksel TRISA                   ; Bank 1
    M_movlf 0x70, OSCCON            ; use 8MHz internal clock (internal clock set on config)
    clrf    ANSEL
    M_movlf 0x3f, TRISA             ; in in in in in in
    clrf    TRISC                   ; out out out out out out
    M_movlf (1<<CTRL_LATCH), IOCA   ; IOC on CTRL_LATCH
    M_movlf 0x80, OPTION_REG        ; global pullup disable, use falling edge on A2, prescaler T0 1:2
    banksel	PORTA                   ; Bank 0

load_mode
    clrf	reg_current_mode
    clrf    reg_previous_mode
    bcf     STATUS, C           ; clear carry
    banksel EEADR               ; fetch current mode from EEPROM
    clrf    EEADR               ; address 0
    bsf     EECON1, RD
    movfw   EEDAT
    banksel PORTA
    movwf	reg_current_mode    ; last mode saved
    movwf   reg_previous_mode   ; last mode saved to compare
    call    set_reg_current_mode ; set mode as fast as possible

detect_reset_type
    clrf    reg_reset_type
    movlw   code_reset_high_active
    btfss   PORTA, RESET_IN         ; jump next instruction for low-active reset
    movwf   reg_reset_type

set_led_type
    call    setleds ; set LEDs according to teh LED type

init_end
    clrf    reg_ctrl_reset  ; clear this reg here just in case
    goto    idle

; -----------------------------------------------------------------------
; eeprom data
DEEPROM	CODE
	de	RGB_natural		;default mode (default: RGB on, natural)

theend
    END
; ------------------------------------------------------------------------