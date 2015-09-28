#include <p16f684.inc>

; -----------------------------------------------------------------------
;   NES "In-game reset" (IGR) controller with extended control
;   for Viletims RGB-PCB (color switch)
;
;   Copyright (C) 2014 by Peter Bartmann <peter.bartmann@gmx.de>
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
;                  n.c. in (*) [*] |4  A3 A2 11| clk in (2) [CPU Pin 36]
;                 (green) LED1 out |5  C5 C0 10| rgb-mode: garish   out [NESRGB Pad 1]
;                   (red) LED2 out |6  C4 C1  9| rgb-mode: improved out [NESRGB Pad 2]
;                      LED_TYPE in |7  C3 C2  8| rgb-mode: natural  out [NESRGB Pad 3]
;                                  `-----------'
;
;   As the internal oscillator is used, you should connect a capacitor of about 100nF between
;   Pin 1 (Vdd/+5V) and Pin 8 (Vss/GND) as close as possible to the PIC. This esures best
;   operation.
;
;   One have to use a 74LVX125 inbetween of the PIC and the NESRGB. Otherwise the CPLD on
;   the NESRGB will be destroyed :'(.
;
;   - pin 4 connected to +5V or GND: three modes available (improved, natural, garish)
;
;   Pin 7 (LED_TYPE) sets the output mode for the LED pins
;   (must be tied to either level):
;      low  = common cathode
;      high = common anode   (output inverted)
;
;   preparation of the Resetline
;   ============================
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
; 000 - off
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
; 000 - off
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

; -----------------------------------------------------------------------

CTRL_DATA   EQU 0
CTRL_LATCH  EQU 1
CTRL_CLK    EQU 2
NUM_MODES   EQU 3
RESET_IN    EQU 4
RESET_OUT   EQU 5

LED_TYPE    EQU 3
LED_RED     EQU 4
LED_GREEN   EQU 5

; reg_port_buffer     EQU 0x20
reg_ctrl_data       EQU 0x21
reg_overflow_cnt    EQU 0x22
reg_repetition_cnt  EQU 0x23
reg_current_mode    EQU 0x30
reg_ctrl_reset      EQU 0x40

RGB_off             EQU 0x00    ; code for RGB off
RGB_natural         EQU 0x01    ; code for RGB on natural
RGB_improved        EQU 0x02    ; code for RGB on improved
RGB_garish          EQU 0x03    ; code for RGB on garish
code_mode_overflow  EQU 0x04    ; code for a non available mode (last mode + 1)

code_RGB_natural    EQU 0x03
code_RGB_improved   EQU 0x05
code_RGB_garish     EQU 0x06

code_led_off    EQU 0x00
code_led_red    EQU (1<<LED_RED)
code_led_green  EQU (1<<LED_GREEN)
code_led_yellow EQU (1<<LED_RED) ^ (1<<LED_GREEN)

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
BUTTON_SL   EQU 5
BUTTON_ST   EQU 4
DPAD_UP     EQU 3
DPAD_DW     EQU 2
DPAD_LE     EQU 1
DPAD_RI     EQU 0

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

idle_loop
    btfsc	INTCON, RAIF    ; data latch changed?
    goto    wait_ctrl_read  ; yes
    btfsc   PORTA, RESET_IN ; reset button pressed?
    goto    check_reset     ; yes
    btfsc	INTCON, RAIF    ; data latch changed?
    goto    wait_ctrl_read  ; yes
    goto    idle_loop       ; no -> repeat loop


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

prewait_Button_B
    btfss   PORTA, CTRL_CLK
    goto    prewait_Button_B
    bcf     INTCON, INTF
    bcf     INTCON, RAIF        ; from now on, no IOC at the data latch shall appear
store_Button_B
    btfsc   PORTA, CTRL_DATA
    bsf     reg_ctrl_data, BUTTON_B
postwait_Button_B
    btfss   INTCON, INTF
    goto    postwait_Button_B

prewait_Button_Sl
    btfss   PORTA, CTRL_CLK
    goto    prewait_Button_Sl
    bcf     INTCON, INTF
    nop
store_Button_Sl
    btfsc   PORTA, CTRL_DATA
    bsf     reg_ctrl_data, BUTTON_SL
postwait_Button_Sl
    btfss   INTCON, INTF
    goto    postwait_Button_Sl

prewait_Button_St
    btfss   PORTA, CTRL_CLK
    goto    prewait_Button_St
    bcf     INTCON, INTF
    nop
store_Button_St
    btfsc   PORTA, CTRL_DATA
    bsf     reg_ctrl_data, BUTTON_ST
postwait_Button_St
    btfss   INTCON, INTF
    goto    postwait_Button_St

prewait_DPad_Up
    btfss   PORTA, CTRL_CLK
    goto    prewait_DPad_Up
    bcf     INTCON, INTF
    nop
store_DPad_Up
    btfsc   PORTA, CTRL_DATA
    bsf     reg_ctrl_data, DPAD_UP
postwait_DPad_Up
    btfss   INTCON, INTF
    goto    postwait_DPad_Up

prewait_DPad_Dw
    bcf     INTCON, INTF
    nop
    btfss   PORTA, CTRL_CLK
    goto    prewait_DPad_Dw
store_Button_DW
    btfsc   PORTA, CTRL_DATA
    bsf     reg_ctrl_data, DPAD_DW
postwait_DPad_Dw
    btfss   INTCON, INTF
    goto    postwait_DPad_Dw

prewait_DPad_Le
    btfss   PORTA, CTRL_CLK
    goto    prewait_DPad_Le
    bcf     INTCON, INTF
    nop
store_DPad_Le
    btfsc   PORTA, CTRL_DATA
    bsf     reg_ctrl_data, DPAD_LE
postwait_DPad_Le
    btfss   INTCON, INTF
    goto    postwait_DPad_Le

prewait_DPad_Ri
    btfss   PORTA, CTRL_CLK
    goto    prewait_DPad_Ri
    bcf     INTCON, INTF
    nop
store_DPad_Ri
    btfsc   PORTA, CTRL_DATA
    bsf     reg_ctrl_data, DPAD_RI
postwait_DPad_Ri
    btfss   INTCON, INTF
    goto    postwait_DPad_Ri

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
    bsf             PORTA, RESET_OUT
    M_delay_x05ms   repetitions_480ms

releasereset_wait_high
    btfsc   PORTA, RESET_IN         ; reset pressed?
    goto    releasereset_wait_high  ; yes -> still waitin'
    goto    release_reset

dolongreset
    banksel         TRISA                   ; Bank 1
    bcf             TRISA, RESET_OUT
    banksel         PORTA                   ; Bank 0
    bsf             PORTA, RESET_OUT
    M_delay_x05ms   repetitions_1000ms
    M_delay_x05ms   repetitions_1000ms
    M_delay_x05ms   repetitions_1000ms

release_reset    ; Bank 0
    bcf     PORTA, RESET_OUT
    banksel TRISA                               ; Bank 1
    bsf     TRISA, RESET_OUT
    banksel PORTA                               ; Bank 0
    clrf    reg_ctrl_reset
    goto    idle

domode_prev
    M_celf  RGB_natural, reg_current_mode, modepreset ; preset mode if current mode is RGB_natural
    decf    reg_current_mode, 1
    call    set_reg_current_mode    ; change mode during runtime
    call    mode_change_delay
    goto    idle

domode_next
    incf            reg_current_mode, 1
    M_celf          code_mode_overflow, reg_current_mode, modereset
    call            set_reg_current_mode                        ; change mode during runtime
    M_delay_x05ms   repetitions_mode_delay
    goto            idle

; --------check reset routines--------

check_reset
    call    delay_05ms      ; software debounce
    call    delay_05ms      ; software debounce
    call    delay_05ms      ; software debounce
    btfss   PORTA, RESET_IN ; reset still pressed?
    goto    idle            ; no -> goto idle

    M_movlf repetitions_mode_delay , reg_repetition_cnt  ; repeat delay_05ms x-times

check_reset_loop
    call    delay_05ms
    btfss   PORTA, RESET_IN         ; reset still pressed?
    goto    doreset                 ; no -> perform a reset
    decfsz  reg_repetition_cnt, 1   ; delay_05ms repeated x-times?
    goto    check_reset_loop        ; no -> stay in this loop

reset_next_mode
    incf    reg_current_mode, 1                             ; next mode
    M_celf  code_mode_overflow, reg_current_mode, modereset
    call    set_reg_current_mode                            ; change mode during runtime

    M_movlf repetitions_mode_delay , reg_repetition_cnt  ; repeat delay_05ms x-times

reset_mode_change_loop
    call    delay_05ms
    btfss   PORTA, RESET_IN         ; reset still pressed?
    goto    idle                    ; no -> end procedure (runtime change)
    decfsz  reg_repetition_cnt, 1   ; delay_05ms repeated x-times?
    goto    reset_mode_change_loop  ; no -> stay in this loop
    goto    reset_next_mode         ; yes -> next mode

; --------mode, led, delay and save_mode calls--------

set_reg_current_mode ; setting the RGB mode includes setting the LED
    M_belf  RGB_natural, reg_current_mode, modeset_natural
    M_belf  RGB_improved, reg_current_mode, modeset_improved
    M_belf  RGB_garish, reg_current_mode, modeset_garish
    M_movlf RGB_natural, reg_current_mode   ; should not appear

modeset_natural
    movlw   code_RGB_natural ^ code_led_red
    btfsc	PORTC, LED_TYPE ; if common anode:
    xorlw	0x30            ; invert output
    movwf	PORTC
    goto    save_mode

modeset_improved
    movlw   code_RGB_improved ^ code_led_green
    btfsc	PORTC, LED_TYPE ; if common anode:
    xorlw	0x30            ; invert output
    movwf	PORTC
    goto    save_mode

modeset_garish
    movlw   code_RGB_garish ^ code_led_yellow
    btfsc	PORTC, LED_TYPE ; if common anode:
    xorlw	0x30            ; invert output
    movwf	PORTC
    goto    save_mode

setleds
    M_belf  RGB_natural, reg_current_mode, setleds_red
    M_belf  RGB_improved, reg_current_mode, setleds_green
    M_belf  RGB_garish, reg_current_mode, setleds_yellow
    M_movlf RGB_natural, reg_current_mode  ; should not appear - set to a valid mode
    goto    setleds_red

setleds_off
    movfw   PORTC
    andlw   0x07            ; save RGB mode
    xorlw   code_led_off    ; set LED
    btfsc	PORTC, LED_TYPE ; if common anode:
    xorlw	0x30            ; invert output
    movwf	PORTC
    goto    save_mode

setleds_red
    movfw   PORTC
    andlw   0x07            ; save RGB mode
    xorlw   code_led_red    ; set LED
    btfsc	PORTC, LED_TYPE ; if common anode:
    xorlw	0x30            ; invert output
    movwf	PORTC
    goto    save_mode

setleds_green
    movfw   PORTC
    andlw   0x07            ; save RGB mode
    xorlw   code_led_green  ; set LED
    btfsc	PORTC, LED_TYPE ; if common anode:
    xorlw	0x30            ; invert output
    movwf	PORTC
    goto    save_mode

setleds_yellow
    movfw   PORTC
    andlw   0x07            ; save RGB mode
    xorlw   code_led_yellow ; set LED
    btfsc	PORTC, LED_TYPE ; if common anode:
    xorlw	0x30            ; invert output
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
    M_movpf         PORTC, reg_ctrl_reset
    andlw           0x07                    ; save RGB mode
    xorlw           code_led_off            ; set LED
    btfsc           PORTC, LED_TYPE         ; if common anode:
    xorlw           0x30                    ; invert output
    movwf           PORTC
    M_delay_x05ms   repetitions_300ms
    M_movff         reg_ctrl_reset, PORTC
    clrf            reg_ctrl_reset
    bsf             reg_ctrl_reset, bit_ctrl_reset_flag
    goto            idle


modepreset
    M_movlf code_mode_overflow, reg_current_mode
    return


modereset
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
    M_movlf 0x08, TRISC             ; out out in out out out
    M_movlf (1<<CTRL_LATCH), IOCA   ; IOC on CTRL_LATCH
    M_movlf 0x80, OPTION_REG        ; global pullup disable, use falling edge on A2, prescaler T0 1:2
    banksel	PORTA                   ; Bank 0

load_mode
    clrf	reg_current_mode
    bcf     STATUS, C           ; clear carry
    banksel EEADR               ; fetch current mode from EEPROM
    clrf    EEADR               ; address 0
    bsf     EECON1, RD
    movfw   EEDAT
    banksel PORTA
    movwf	reg_current_mode    ; last mode saved
    call    set_reg_current_mode

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