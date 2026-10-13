;******************************************************************
;*                      Maze-Solving Robot                        *
;*                                                                *
;* This program is for the mc9s12c32 microcontroller. It runs on  *
;* the eebot and allows it to navigate a maze using a state       *
;* machine and sensors (LEDS and photoresistors).                 *
;******************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc'

; equates section
LCD_DAT         EQU   PORTB                     ; LCD data port, bits - PB7,...,PB0
LCD_CNTR        EQU   PTJ                       ; LCD control port, bits - PJ6(RS),PJ7(E)
LCD_E           EQU   $80                       ; LCD E-signal pin
LCD_RS          EQU   $40                       ; LCD RS-signal pin

A_PTH_INT       EQU   $C0                       ; detection intervals
B_PTH_INT       EQU   $CA                       ;      -"-
C_PTH_INT       EQU   $CA                       ;      -"-
D_PTH_INT       EQU   $CA                       ;      -"-
E_PTH_INT       EQU   $70                       ; bot shift right condition
F_PTH_INT       EQU   $72                       ; bot shift left condition

INC_DIS         EQU   300                       ; increment travel distance
FWD_DIS         EQU   2000                      ; forward travel distance
REV_DIS         EQU   3000                      ; reverse travel distance
STR_DIS         EQU   1000                      ; starboard right travel distance
TRN_DIS         EQU   6000                      ; turn travel distance
UTRN_DIS        EQU   13000                     ; u-turn travel distance

PRI_PTH_INT     EQU   0                         ; primary path value
SEC_PTH_INT     EQU   1                         ; secondary path value

START           EQU   0                         ; start state
FWD             EQU   1                         ; forward state
REV             EQU   2                         ; reverse state
RT_TRN          EQU   3                         ; rightturn state
LT_TRN          EQU   4                         ; leftturn state
BK_TRK          EQU   5                         ; backtrack state
SBY             EQU   6                         ; standby state

; variable section
                ORG   $3850
CRNT_STATE      dc.b  6                         ; state register

COUNT1          DC.W  0                         ; initialize 2bytes
COUNT2          DC.W  0                         ; initialize 2bytes

A_DETN          dc.b  0                         ; sensor A register
B_DETN          dc.b  0                         ; sensor B register
C_DETN          dc.b  0                         ; sensor C register
D_DETN          dc.b  0                         ; sensor D register
E_DETN          dc.b  0                         ; sensor E register
F_DETN          dc.b  0                         ; sensor F register

RETURN          dc.b  0                         ; boolean return
NEXT_DIRECTION  dc.b  1                         ; Next direction instruction

TEN_THOUS       ds.b  1                         ; 10,000 digit
THOUSANDS       ds.b  1                         ;  1,000 digit
HUNDREDS        ds.b  1                         ;    100 digit
TENS            ds.b  1                         ;     10 digit
UNITS           ds.b  1                         ;      1 digit
NO_BLANK        ds.b  1                         ; Used in 'leading zero' blanking by BCD2ASC
BCD_SPARE       ds.b  10                        ; Extra space for decimal point and string terminator

SENSOR_LINE     dc.b  $0
SENSOR_BOW      dc.b  $0
SENSOR_PORT     dc.b  $0
SENSOR_MID      dc.b  $0
SENSOR_STBD     dc.b  $0

SENSOR_NUM      ds.b  1
TEMP            ds.b  1

; data section
msg1:           dc.b  "S:",0                    ; Current state
msg2:           dc.b  "R:",0                    ; Sensor readings
msg3:           dc.b  "V:",0                    ; Battery voltage
msg4:           dc.b  "B:",0                    ; Bumper status

tab:            dc.b  "START  ",0               ; States
                dc.b  "FWD    ",0               ;  -"-
                dc.b  "REV    ",0               ;  -"-
                dc.b  "RHTTRN ",0               ;  -"-
                dc.b  "LFTTRN ",0               ;  -"-
                dc.b  "RETURN ",0               ;  -"-
                dc.b  "STANDBY",0               ;  -"-

; code section
                ORG   $4000                     ; code starts
Entry:
_Startup:
                LDS   #$4000                    ; Initialize the stack pointer

                JSR   initPORTS                 ; Initialize PORTAD, PORTA, PORTT
                JSR   initAD                    ; Initialize ATD converter
                JSR   initLCD                   ; Initialize the LCD
                JSR   clrLCD                    ; Clear LCD & home cursor
                JSR   initTCNT                  ; Initialize timer

                CLI

                LDX   #msg1                     ; Display msg1
                JSR   putsLCD                   ;      -"-

                LDAA  #$8A                      ; move LCD cursor
                JSR   cmd2LCD                   ;      -"-
                LDX   #msg2                     ; Display msg2
                JSR   putsLCD                   ;      -"-

                LDAA  #$C0                      ; move LCD cursor
                JSR   cmd2LCD                   ;      -"-
                LDX   #msg3                     ; Display msg3
                JSR   putsLCD                   ;      -"-

                LDAA  #$C7                      ; move LCD cursor
                JSR   cmd2LCD                   ;      -"-
                LDX   #msg4                     ; Display msg4
                JSR   putsLCD                   ;      -"-

MAIN:           JSR   UPDT_READING
                JSR   UPDT_DISPL
                LDAA  CRNT_STATE
                JSR   DISPATCHER
                BRA   MAIN

; subroutine section
STARON          BSET  PTT,%00100000
                RTS

STAROFF         BCLR  PTT,%00100000
                RTS

STARFWD         BCLR  PORTA,%00000010
                RTS

STARREV         BSET  PORTA,%00000010
                RTS

PORTON          BSET  PTT,%00010000
                RTS

PORTOFF         BCLR  PTT,%00010000
                RTS

PORTFWD         BCLR  PORTA,%00000001
                RTS

PORTREV         BSET  PORTA,%00000001
                RTS

; states section
DISPATCHER      CMPA  #START                    ; If it is the START state
                BNE   NOT_START                 ;
                JSR   START_ST                  ;  then call THE START_ST routine 
                RTS                             ;  and exit 

NOT_START       CMPA  #FWD                      ; Else if it's the FWD state  
                BNE   NOT_FORWARD               ;
                JMP   FWD_ST                    ;  then call the FWD_ST routine 

NOT_FORWARD     CMPA  #RT_TRN                   ; Else if it's the RT_TRN state
                BNE   NOT_RT_TRN                ;
                JSR   RT_TRN_ST                 ;  then call the RT_TRN routine
                RTS

NOT_RT_TRN      CMPA  #LT_TRN                   ; Else if it's the LT_TRN state
                BNE   NOT_LT_TRN                ;
                JSR   LT_TRN_ST                 ;  then call the LT_TRN routine
                RTS                             ;  and exit

NOT_LT_TRN      CMPA  #REV                      ; Else if it's the REV state
                BNE   NOT_REVERSE               ;
                JSR   REV_ST                    ;  then call the REV_ST routine
                RTS                             ;  and exit

NOT_REVERSE     CMPA  #BK_TRK                   ; Else if it's the BK_TRK state
                BNE   NOT_BK_TRK                ;
                JMP   BK_TRK_ST                 ;  then call the BK_TRK routine

NOT_BK_TRK      CMPA  #SBY                      ; Else if it's the SBY state
                BNE   NOT_SBY                   ;
                JSR   SBY_ST                    ;  then call the SBY routine
                RTS                             ;  and exit

NOT_SBY         NOP
DISP_EXIT       RTS                             ; Exit from the state dispatcher

;********************************************************************************************
START_ST        BRCLR PORTAD0,$04,NO_FWD        ; If /FWD_BUMP
                JSR   INIT_FWD                  ; Initialize the FORWARD state
                MOVB  #FWD, CRNT_STATE          ; Go into the FORWARD state
                BRA   START_EXIT

NO_FWD          NOP                             ; Else
START_EXIT      RTS                             ;  return to the MAIN routine

;********************************************************************************************
FWD_ST          PULD                            
                BRSET PORTAD0,$04,NO_FWD_BUMP   ; If foward bump then correct the next direction to the alternate path
                LDAA  SEC_PTH_INT
                STAA  NEXT_DIRECTION
                JSR   INIT_REV                  ;  initialize the REVERSE routine
                MOVB  #REV,CRNT_STATE           ;  set the state to REVERSE
                JMP   FWD_EXIT

NO_FWD_BUMP     BRSET PORTAD0,$08,NO_REV_BUMP   ; If reverse bump, then initalize backtrack state and set current state as backtrack
                JMP   INIT_BK_TRK               ;  initialize the BK_TRK routine
                MOVB  #BK_TRK,CRNT_STATE        ;  set the state to BK_TRK
                JMP   FWD_EXIT

NO_REV_BUMP     LDAA  D_DETN
                BEQ   NO_RT_INTXN               ; If sensor d equals 1 then robot should turn right
                LDAA  NEXT_DIRECTION            ; 
                PSHA                            ;  previous intersection direction pushed to stack
                LDAA  PRI_PTH_INT               ; 
                STAA  NEXT_DIRECTION            ;  current direction stored
                JSR   INIT_RT_TRN               ;  initialize the RT_TRN routine
                MOVB  #RT_TRN,CRNT_STATE        ;  set the state to RT_TRN
                JMP   FWD_EXIT

NO_RT_INTXN     LDAA  B_DETN                    ; If sensor b equals 1 and a equals 1 a forward path exists at intersection and the bot should continue forward
                BEQ   NO_LT_INTXN               ;
                LDAA  A_DETN                    ;
                BEQ   LT_TURN                   ;
                LDAA  NEXT_DIRECTION            ;
                PSHA                            ;  previous intersection direction pushed to stack
                LDAA  PRI_PTH_INT               ;
                STAA  NEXT_DIRECTION            ;  current direction stored
                BRA   NO_SHFT_LT                ; If sensor b equals 1 and a equals 0 bot should turn left
LT_TURN         LDAA  NEXT_DIRECTION            ;
                PSHA                            ;  previous intersection direction pushed to stack
                LDAA  SEC_PTH_INT               ;
                STAA  NEXT_DIRECTION            ;  the next direction stored
                JSR   INIT_LT_TRN               ;  initialize the LT_TRN routine
                MOVB  #LT_TRN,CRNT_STATE        ;  set the state to LT_TRN
                JMP   FWD_EXIT

NO_LT_INTXN     LDAA  F_DETN
                BEQ   NO_SHFT_RT                ; If sensor f equals 1 robot should do a right shift
                JSR   PORTON                    ;  the left motor is turned on for a set count and then turned off
RT_FWD_DIS      LDD   COUNT2                    ;
                CPD   #INC_DIS                  ;
                BLO   RT_FWD_DIS                ;  If AccD is lower (COUNT2 < INC_DIS), then branch
                JSR   INIT_FWD                  ;  Else, initialize the FWD routine
                JMP   FWD_EXIT

NO_SHFT_RT      LDAA  E_DETN
                BEQ   NO_SHFT_LT                ; If sensor e equals 1 robot should do a left shift
                JSR   STARON                    ;  the right motor is turned on for a set count and then turned off
LT_FWD_DIS      LDD   COUNT1                    ;
                CPD   #INC_DIS                  ;
                BLO   LT_FWD_DIS                ;  If AccD is lower (COUNT1 < INC_DIS), then branch
                JSR   INIT_FWD                  ;  Else, initialize the FWD routine
                JMP   FWD_EXIT

NO_SHFT_LT      JSR   STARON                    ; Turn motors on
                JSR   PORTON                    ;      -"-
FWD_STR_DIS     LDD   COUNT1
                CPD   #FWD_DIS                  ; Compare
                BLO   FWD_STR_DIS               ; If AccD is lower (COUNT1 < INC_DIS), then branch
                JSR   INIT_FWD                  ; Else, initialize the FWD routine (turn motors off)

FWD_EXIT        JMP   MAIN                      ; return to the MAIN routine

;********************************************************************************************   
REV_ST          LDD   COUNT1
                CPD   #REV_DIS                  ; Compare
                BLO   REV_ST                    ; Make a u-turn by turning on the starboard motor until the reverse count is reached
                JSR   STARFWD                   ;
                LDD   #0                        ; Reset timer
                STD   COUNT1                    ;      -"-

REV_U_TRN       LDD   COUNT1
                CPD   #UTRN_DIS
                BLO   REV_U_TRN                 ; If u-turn count is reached the bot should stop and go into forward
                JSR   INIT_FWD                  ;  initialize the FWD routine
                LDAA  RETURN                    ;
                BNE   BK_TRK_REV                ; If RETURN equals 1 the current state is set to forward
                MOVB  #FWD,CRNT_STATE           ;  set the current state is to forward
                BRA   REV_EXIT                  ;  and exit
BK_TRK_REV      JSR   INIT_FWD                  ;
                MOVB  #BK_TRK,CRNT_STATE        ; Else set the current state to backtrack

REV_EXIT        RTS                             ; return to the MAIN routine

;********************************************************************************************   
RT_TRN_ST       LDD   COUNT2                    ; If Dc>Dfwd then the robot should make a turn
                CPD   #STR_DIS                  ; 
                BLO   RT_TRN_ST                 ; 
                JSR   STAROFF                   ;  turn off starboard motor
                LDD   #0                        ; Reset timer
                STD   COUNT2                    ;      -"-

RT_TURN_LOOP    LDD   COUNT2                    ; If Dc>Dfwdturn the robot should stop
                CPD   #TRN_DIS                  ;
                BLO   RT_TURN_LOOP              ;
                JSR   INIT_FWD                  ; initialize the FWD routine
                LDAA  RETURN                    ; If RETURN equals 1
                BNE   BK_TRK_RT_TRN             ;
                MOVB  #FWD,CRNT_STATE           ;  set current state to forward
                BRA   RT_TRN_EXIT               ;  and exit
BK_TRK_RT_TRN   MOVB  #BK_TRK,CRNT_STATE        ; Else set current state to backtrack

RT_TRN_EXIT     RTS                             ; return to the MAIN routine

;********************************************************************************************
LT_TRN_ST       LDD   COUNT1                    ; If Dc>Dfwd then the robot should make a turn
                CPD   #STR_DIS                  ;
                BLO   LT_TRN_ST                 ;
                JSR   PORTOFF                   ;  turn off port motor
                LDD   #0                        ; Reset timer
                STD   COUNT1                    ;      -"-

LT_TURN_LOOP    LDD   COUNT1                    ; If Dc>Dfwdturn then the robot should stop
                CPD   #TRN_DIS                  ;
                BLO   LT_TURN_LOOP              ;
                JSR   INIT_FWD                  ;  Initialize the forward state
                LDAA  RETURN                    ; If RETURN equals 1
                BNE   BK_TRK_LT_TRN             ;
                MOVB  #FWD,CRNT_STATE           ;  set current state to forward
                BRA   LT_TRN_EXIT               ;  and exit
BK_TRK_LT_TRN   MOVB  #BK_TRK,CRNT_STATE        ; Else set current state to backtrack

LT_TRN_EXIT     RTS                             ; return to the MAIN routine

;********************************************************************************************
BK_TRK_ST       PULD
                BRSET PORTAD0,$08,NO_BK_BUMP    ; If reverse bumper is 0, then we should stop
                JSR   INIT_SBY                  ;  initialize the standby state
                MOVB  #SBY,CRNT_STATE           ;  set the state to standby
                JMP   BK_TRK_EXIT               ;  and exit

NO_BK_BUMP      LDAA  NEXT_DIRECTION
                BEQ   REG_PATHING               ; If next direction equals 0, use regular pathing mode
                BNE   IRREG_PATHING             ; Else use irregular pathing mode
               
REG_PATHING     LDAA  D_DETN
                BEQ   NO_RT_TRN                 ; If sensor d equals 1, the bot should make a right turn
                PULA                            ;
                PULA                            ;  pull the next direction value from the stack
                STAA  NEXT_DIRECTION            ;  and store it in next direction
                JSR   INIT_RT_TRN               ;  initialize the right turn state
                MOVB  #RT_TRN,CRNT_STATE        ;  set current state to right turn
                JMP   BK_TRK_EXIT               ;  exit

NO_RT_TRN       LDAA  B_DETN                    ; If sensor b equals 1
                BEQ   RT_LINE_S                 ; Check if sensor a equals 1
                LDAA  A_DETN                    ; If sensor a equals 1 a forward path exists
                BEQ   LEFT_TURN                 ; The robot should continue forward
                PULA                            ; Pull the next direction value from the stack
                PULA                            ; and store it in next direction
                STAA  NEXT_DIRECTION            ;      -"-
                BRA   NO_LINE_S                 ; Else if sensor d equals 0
LEFT_TURN       PULA                            ; The robot should make a left turn
                PULA                            ; Pull the next direction value from the stack
                STAA  NEXT_DIRECTION            ; and store it in next direction
                JSR   INIT_LT_TRN               ; Initialize the left turn state
                MOVB  #LT_TRN,CRNT_STATE        ; Set current state to left turn
                JMP   BK_TRK_EXIT               ; and exit

IRREG_PATHING   LDAA  B_DETN                    ; If sensor b equals 1
                BEQ   NO_LT_TRN                 ; The robot should make a left turn
                PULA                            ; Pull the next direction value from the stack
                STAA  NEXT_DIRECTION            ; and store it in next direction
                JSR   INIT_LT_TRN               ; Initialize the left turn state
                MOVB  #LT_TRN,CRNT_STATE        ; Set current state to left turn
                JMP   BK_TRK_EXIT               ; and exit

NO_LT_TRN       LDAA  D_DETN                    ; If sensor d equals 1
                BEQ   RT_LINE_S                 ; Check if sensor a equals 1
                LDAA  A_DETN                    ; If sensor a equals 1 a forward path exists
                BEQ   RIGHT_TURN                ; The robot should continue forward
                PULA                            ; Pull the next direction value from the stack
                STAA  NEXT_DIRECTION            ; and store it in next direction
                BRA   NO_LINE_S                 ; Else if sensor a equals 0
RIGHT_TURN      PULA                            ; The robot should make a right turn
                STAA  NEXT_DIRECTION            ; Pull the next direction value from the stack
                JSR   INIT_RT_TRN               ; Initialize the right state
                MOVB  #RT_TRN,CRNT_STATE        ; Set current state to right turn
                JMP   BK_TRK_EXIT               ; Then exit

RT_LINE_S       LDAA  F_DETN                    ; Else if sensor f equals 1
                BEQ   LT_LINE_S                 ; The robot should shift right
                JSR   PORTON                    ; and turn on the left motor
RT_FWD_D        LDD   COUNT2                    ;
                CPD   #INC_DIS                  ;
                BLO   RT_FWD_D                  ; If Dc>Dfwd then
                JSR   INIT_FWD                  ; Turn motors off 
                JMP   BK_TRK_EXIT               ; and exit  

LT_LINE_S       LDAA  E_DETN                    ; Else if sensor e equals 1
                BEQ   NO_LINE_S                 ; The robot should shift left
                JSR   STARON                    ; and turn on the right motor
LT_FWD_D        LDD   COUNT1                    ;
                CPD   #INC_DIS                  ;
                BLO   LT_FWD_D                  ; If Dc>Dfwd then
                JSR   INIT_FWD                  ; Turn motors off
                JMP   BK_TRK_EXIT               ; and exit

NO_LINE_S       JSR   STARON                    ; Turn motors on
                JSR   PORTON                    ;      -"-
FWD_STR_D       LDD   COUNT1                    ;
                CPD   #FWD_DIS                  ;
                BLO   FWD_STR_D                 ; If Dc>Dfwd then
                JSR   INIT_FWD                  ; Turn motors off

BK_TRK_EXIT     JMP   MAIN                      ; return to the MAIN routine

;********************************************************************************************   
SBY_ST          BRSET PORTAD0,$04,NO_START      ; If forward bumper = 0
                BCLR  PTT,%00110000             ;  then initialize the start state
                MOVB  #START,CRNT_STATE         ;  set current state to start
                BRA   SBY_EXIT                  ;  and exit
NO_START        NOP                             ; Else  
SBY_EXIT        RTS                             ;  return to the MAIN routine 

;********************************************************************************************
; state initialization section
INIT_FWD        BCLR  PTT,%00110000             ; Turn off the drive motors
                LDD   #0                        ; Reset timer
                STD   COUNT1                    ;      -"-
                STD   COUNT2                    ;      -"-
                BCLR  PORTA,%00000011           ; Set FWD direction for both motors
                RTS

;********************************************************************************************
INIT_REV        BSET  PORTA,%00000011           ; Set REV direction for both motors
                LDD   #0                        ; Reset timer
                STD   COUNT1                    ;      -"-
                BSET  PTT,%00110000             ; Turn on the drive motors
                RTS

;********************************************************************************************
INIT_RT_TRN     BCLR  PORTA,%00000011           ; Set FWD direction for both motors
                LDD   #0                        ; Reset timer
                STD   COUNT2                    ;      -"-
                BSET  PTT,%00110000             ; Turn on the drive motors
                RTS

;********************************************************************************************
INIT_LT_TRN     BCLR  PORTA,%00000011           ; Set FWD direction for both motors
                LDD   #0                        ; Reset timer
                STD   COUNT1                    ;      -"-
                BSET  PTT,%00110000             ; Turn on the drive motors
                RTS

;********************************************************************************************
INIT_BK_TRK     INC   RETURN                    ; Change RETURN value to 1
                PULA                            ; Pull the next direction value from the stack
                STAA  NEXT_DIRECTION            ; and store it in next direction
                JSR   INIT_REV                  ; Initialize the reverse routine
                JSR   REV_ST                    ; Jump to reverse state
                JMP   MAIN

;********************************************************************************************   
INIT_SBY        BCLR  PTT,%00110000             ; Turn off the drive motors
                RTS

; sensors section
UPDT_READING    JSR   G_LEDS_ON                 ; Turn ON LEDS
                JSR   READ_SENSORS              ; Take readings from sensors
                JSR   G_LEDS_OFF                ; Turn OFF LEDS

                LDAA  #0
                STAA  A_DETN                    ; Clear sensor A detection value  
                STAA  B_DETN                    ; Clear sensor B detection value  
                STAA  C_DETN                    ; Clear sensor C detection value  
                STAA  D_DETN                    ; Clear sensor D detection value  
                STAA  E_DETN                    ; Clear sensor E detection value  
                STAA  F_DETN                    ; Clear sensor F detection value  

CHECK_A         LDAA  SENSOR_BOW
                CMPA  #A_PTH_INT
                BLO   CHECK_B                   ; If bow sensor is lower than A_PTH_INT, then branch
                INC   A_DETN                    ; Increment A_DETN

CHECK_B         LDAA  SENSOR_PORT 
                CMPA  #B_PTH_INT 
                BLO   CHECK_C                   ; If port sensor is lower than B_PTH_INT, then branch
                INC   B_DETN                    ; Increment B_DETN

CHECK_C         LDAA  SENSOR_MID
                CMPA  #C_PTH_INT
                BLO   CHECK_D                   ; If mid sensor is lower than C_PTH_INT, then branch
                INC   C_DETN                    ; Increment C_DETN

CHECK_D         LDAA  SENSOR_STBD
                CMPA  #D_PTH_INT
                BLO   CHECK_E                   ; If starboard sensor is lower than D_PTH_INT, then branch
                INC   D_DETN                    ; Increment D_DETN

CHECK_E         LDAA  SENSOR_LINE
                CMPA  #E_PTH_INT
                BHI   CHECK_F                   ; If line sensor is greater than E_PTH_INT, then branch
                INC   E_DETN                    ; Increment E_DETN

CHECK_F         LDAA  SENSOR_LINE
                CMPA  #F_PTH_INT
                BLO   UPDT_EXIT                 ; If line sensor is lower than F_PTH_INT, then branch
                INC   F_DETN                    ; Increment F_DETN
               
UPDT_EXIT       RTS

;********************************************************************************************
;* The support routines from here to the end of the file (guider sensors, delay, LCD
;* driver, number conversion, display update, hardware initialization and interrupt
;* service routines) are adapted from the eebot reference routines by Peter Hiscocks.
;********************************************************************************************

;********************************************************************************************
;*                              Guider LEDs ON
;*
;* This routine enables the guider LEDs so that readings of the sensor
;* correspond to the ’illuminated’ situation.
;*
;* Passed: Nothing
;* Returns: Nothing
;* Side: PORTA bit 5 is change

G_LEDS_ON       BSET  PORTA,%00100000           ; Set bit 5
                RTS

;********************************************************************************************
;*                              Guider LEDs OFF
;*
;* This routine disables the guider LEDs. Readings of the sensor
;* correspond to the ’ambient lighting’ situation.
;*
;* Passed: Nothing
;* Returns: Nothing
;* Side: PORTA bit 5 is changed

G_LEDS_OFF      BCLR  PORTA,%00100000           ; Clear bit 5
                RTS

;********************************************************************************************
;*                          Read Sensors
;*
;* This routine reads the eebot guider sensors and puts the results in RAM
;*  registers.
;* Note: Do not confuse the analog multiplexer on the Guider board with the
;*  multiplexer in the HCS12. The guider board mux must be set to the
;*  appropriate channel using the SELECT_SENSOR routine. The HCS12 always
;*  reads the selected sensor on the HCS12 A/D channel AN1.
;*
;* The A/D conversion mode used in this routine is to read the A/D channel
;*  AN1 four times into HCS12 data registers ATDDR0,1,2,3. The only result
;*  used in this routine is the value from AN1, read from ATDDR0. However,
;*  other routines may wish to use the results in ATDDR1, 2 and 3.
;* Consequently, Scan=0, Mult=0 and Channel=001 for the ATDCTL5 control word.
;*
;* Passed:  None
;* Returns: Sensor readings in:
;*          SENSOR_LINE (0) (Sensor E/F)
;*          SENSOR_BOW  (1) (Sensor A)
;*          SENSOR_PORT (2) (Sensor B)
;*          SENSOR_MID  (3) (Sensor C)
;*          SENSOR_STBD (4) (Sensor D)
;* Note:
;*  The sensor number is shown in brackets
;*
;* Algorithm:
;*        Initialize the sensor number to 0
;*        Initialize a pointer into the RAM at the start of the Sensor Array storage
;* Loop   Store %10000001 to the ATDCTL5 (to select AN1 and start a conversion)
;*        Repeat
;*            Read ATDSTAT0
;*        Until Bit SCF of ATDSTAT0 == 1 (at which time the conversion is complete)
;*        Store the contents of ATDDR0L at the pointer
;*        If the pointer is at the last entry in Sensor Array, then
;*            Exit
;*        Else
;*            Increment the sensor number
;*            Increment the pointer
;*        Loop again.

READ_SENSORS    CLR   SENSOR_NUM                ; Select sensor number 0  
                LDX   #SENSOR_LINE              ; Point at the start of the sensor array  

RS_MAIN_LOOP    LDAA  SENSOR_NUM                ; Select the correct sensor input 
                JSR   SELECT_SENSOR             ;  on the hardware
                LDY   #400                      ; 20 ms delay to allow the
                JSR   del_50us                  ;  sensor to stabilize

                LDAA  #%10000001                ; Start A/D conversion on AN1
                STAA  ATDCTL5
                BRCLR ATDSTAT0,$80,*            ; Repeat until A/D signals done

                LDAA  ATDDR0L                   ; A/D conversion is complete in ATDDR0L
                STAA  0,X                       ;  so copy it to the sensor register
                CPX   #SENSOR_STBD              ; If this is the last reading
                BEQ   RS_EXIT                   ; Then exit

                INC   SENSOR_NUM                ; Else, increment the sensor number
                INX                             ;  and the pointer into the sensor array
                BRA   RS_MAIN_LOOP              ;  and do it again

RS_EXIT:        RTS

;********************************************************************************************
;*                          Select Sensor
;* This routine selects the sensor number passed in ACCA. The motor direction
;*  bits 0, 1, the guider sensor select bit 5 and the unused bits 6,7 in the
;*  same machine register PORTA are not affected.
;* Bits PA2,PA3,PA4 are connected to a 74HC4051 analog mux on the guider board,
;*  which selects the guider sensor to be connected to AN1.
;* 
;* Passed: Sensor Number in ACCA
;* Returns: Nothing
;* Side Effects: ACCA is changed
;* 
;* Algorithm:
;* First, copy the contents of PORTA into a temporary location TEMP and clear
;*        the sensor bits 2,3,4 in the TEMP to zeros by ANDing it with the mask
;*        11100011. The zeros in the mask clear the corresponding bits in the
;*        TEMP. The 1’s have no effect.
;* Next, move the sensor selection number left two positions to align it
;*        with the correct bit positions for sensor selection.
;* Clear all the bits around the (shifted) sensor number by ANDing it with
;*  the mask 00011100. The zeros in the mask clear everything except
;*        the sensor number.
;* Now we can combine the sensor number with the TEMP using logical OR.
;*  The effect is that only bits 2,3,4 are changed in the TEMP, and these
;*  bits now correspond to the sensor number.
;* Finally, save the TEMP to the hardware.

SELECT_SENSOR   PSHA                            ; Save the sensor number for the moment

                LDAA  PORTA                     ; Clear the sensor selection bits to zeros
                ANDA  #%11100011                
                STAA  TEMP                      ; and save it into TEMP

                PULA                            ; Get the sensor number
                ASLA                            ; Shift the selection number left, twice
                ASLA
                ANDA  #%00011100                ; Clear irrelevant bit positions

                ORAA  TEMP                      ; OR it into the sensor bit positions
                STAA  PORTA                     ; Update the hardware
                RTS


; utilities section
;*******************************************************************
;* ([Y] x 50us)-delay subroutine. E-clk=41,67ns.                   *
;*******************************************************************
del_50us:       PSHX                            ; 2 E-clk
eloop:          LDX   #300                      ; 2 E-clk
iloop:          NOP                             ; 1 E-clk
                DBNE  X,iloop                   ; 3 E-clk
                DBNE  Y,eloop                   ; 3 E-clk
                PULX                            ; 3 E-clk
                RTS                             ; 5 E-clk

;*******************************************************************
;* This function sends a command in accumulator A to the LCD       *
;*******************************************************************
cmd2LCD:        BCLR  LCD_CNTR,LCD_RS           ; select the LCD Instruction Register (IR)
                JSR   dataMov                   ; send data to IR
                RTS

;*******************************************************************
;* This function outputs a NULL-terminated string pointed to by X  *
;*******************************************************************
putsLCD         LDAA  1,X+                      ; get one character from the string
                BEQ   donePS                    ; reach NULL character?
                JSR   putcLCD
                BRA   putsLCD
donePS          RTS

;*******************************************************************
;* This function outputs the character in accumulator in A to LCD  *
;*******************************************************************
putcLCD         BSET  LCD_CNTR,LCD_RS           ; select the LCD Data register (DR)
                JSR   dataMov                   ; send data to DR
                RTS

;*******************************************************************
;* This function sends data to the LCD IR or DR depening on RS     *
;*******************************************************************
dataMov         BSET  LCD_CNTR,LCD_E            ; pull the LCD E-signal high
                STAA  LCD_DAT                   ; send the upper 4 bits of data to LCD
                BCLR  LCD_CNTR,LCD_E            ; pull the LCD E-signal low to complete the write oper.

                LSLA                            ; match the lower 4 bits with the LCD data pins
                LSLA                            ;      -"-
                LSLA                            ;      -"-
                LSLA                            ;      -"-

                BSET  LCD_CNTR,LCD_E            ; pull the LCD E signal high
                STAA  LCD_DAT                   ; send the lower 4 bits of data to LCD
                BCLR  LCD_CNTR,LCD_E            ; pull the LCD E-signal low to complete the write oper.

                LDY   #1                        ; adding this delay will complete the internal
                JSR   del_50us                  ; operation for most instructions
                RTS   

;*****************************************************************
;* Integer to BCD Conversion Routine
;*
;* This routine converts a 16 bit binary number in .D into
;* BCD digits in BCD_BUFFER.
;* Peter Hiscocks
;*
;* Algorithm:
;*    Because the IDIV (Integer Division) instruction is available on
;*    the HCS12, we can determine the decimal digits by repeatedly
;*    dividing the binary number by ten: the remainder each time is
;*    a decimal digit. Conceptually, what we are doing is shifting
;*    the decimal number one place to the right past the decimal
;*    point with each divide operation. The remainder must be
;*    a decimal digit between 0 and 9, because we divided by 10.
;*    The algorithm terminates when the quotient has become zero.
;*
;* Bug note: XGDX does not set any condition codes, so test for
;*    quotient zero must be done explicitly with CPX.
;*
;* Data structure:
;*    BCD_BUFFER    EQU * The following registers are the BCD buffer area
;*    TEN_THOUS     RMB 1 10,000 digit, max size for 16 bit binary
;*    THOUSANDS     RMB 1 1,000 digit
;*    HUNDREDS      RMB 1 100 digit
;*    TENS          RMB 1 10 digit
;*    UNITS         RMB 1 1 digit
;*    BCD_SPARE     RMB 2 Extra space for decimal point and string terminator

int2BCD         XGDX                            ; Save the binary number into .X, X <=> D
                LDAA  #0                        ; Clear the BCD_BUFFER
                STAA  TEN_THOUS
                STAA  THOUSANDS
                STAA  HUNDREDS
                STAA  TENS
                STAA  UNITS
                STAA  BCD_SPARE
                STAA  BCD_SPARE+1

                CPX   #0                        ; Check for a zero input
                BEQ   CON_EXIT                  ; and if so, exit

                XGDX                            ; Not zero, get the binary number back to .D as dividend
                LDX   #10                       ; Setup 10 (Decimal!) as the divisor
                IDIV                            ; Divide: Quotient is now in .X, remainder in .D
                STAB  UNITS                     ; Store remainder, lower half of AccD is AccB
                CPX   #0                        ; If quotient is zero,
                BEQ   CON_EXIT                  ; then exit

                XGDX                            ; swap first quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  TENS
                CPX   #0
                BEQ   CON_EXIT

                XGDX                            ; Swap quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  HUNDREDS
                CPX   #0
                BEQ   CON_EXIT

                XGDX                            ; Swap quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  THOUSANDS
                CPX   #0  
                BEQ   CON_EXIT

                XGDX                            ; Swap quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  TEN_THOUS 

CON_EXIT:       RTS                             ; We're done the conversion

;****************************************************************
;* BCD to ASCII Conversion Routine
;* This routine converts the BCD number in the BCD_BUFFER
;* into ascii format, with leading zero suppression.
;* Leading zeros are converted into space characters.
;* The flag 'NO_BLANK' starts cleared and is set once a non-zero
;* digit has been detected.
;* The 'units' digit is never blanked, even if it and all the
;* preceding digits are zero.
;* Peter Hiscocks

BCD2ASC         LDAA  #0                        ; Initialize the blanking flag  
                STAA  NO_BLANK

C_TTHOU         LDAA  TEN_THOUS                 ; Check the ten_thousands digit
                ORAA  NO_BLANK                  ; ORAA with 0 to potentially set Z-flag
                BNE   NOT_BLANK1                ; If resultant byte is not 0, branch to NOT_BLANK1  

ISBLANK1:       LDAA  #' '                      ; It's blank
                STAA  TEN_THOUS                 ; so store a space  
                BRA   C_THOU                    ; and check the thousands digit

NOT_BLANK1:     LDAA  TEN_THOUS                 ; Get the ten thousands digit
                ORAA  #$30                      ; Convert to ascii
                STAA  TEN_THOUS
                LDAA  #$1                       ; Signal that we have seen a non-blank digit
                STAA  NO_BLANK

C_THOU:         LDAA  THOUSANDS                 ; Check the thousands digit for blankness 
                ORAA  NO_BLANK                  ; If it's blank and no-blank is still zero
                BNE   NOT_BLANK2
                     
ISBLANK2:       LDAA  #' '                      ; Thousands digit is blank  
                STAA  THOUSANDS                 ; so store a space  
                BRA   C_HUNS                    ; and check the hundreds digit

NOT_BLANK2:     LDAA  THOUSANDS                 ; (similar to ten_thousands case)
                ORAA  #$30
                STAA  THOUSANDS
                LDAA  #$1 
                STAA  NO_BLANK

C_HUNS:         LDAA  HUNDREDS                  ; Check the hundreds digit for blankness
                ORAA  NO_BLANK                  ; If it's blank and no-blank is still zero
                BNE   NOT_BLANK3

ISBLANK3:       LDAA  #' '                      ; Hundreds digit is blank
                STAA  HUNDREDS                  ; so store a space
                BRA   C_TENS                    ; and check the tens digit
                     
NOT_BLANK3:     LDAA  HUNDREDS                  ; (similar to ten_thousands case)
                ORAA  #$30
                STAA  HUNDREDS
                LDAA  #$1
                STAA  NO_BLANK

C_TENS:         LDAA  TENS                      ; Check the tens digit for blankness
                ORAA  NO_BLANK                  ; If it's blank and no-blank is still zero
                BNE   NOT_BLANK4

ISBLANK4:       LDAA  #' '                      ; Tens digit is blank
                STAA  TENS                      ; so store a space
                BRA   C_UNITS                   ; and check the units digit

NOT_BLANK4:     LDAA  TENS                      ; (similar to ten_thousands case)
                ORAA  #$30  
                STAA  TENS  

C_UNITS:        LDAA  UNITS                     ; No blank check necessary, convert to ascii.
                ORAA  #$30
                STAA  UNITS

                RTS                             ; We're done

;********************************************************************************************
;*                                   Binary to ASCII
;*
;* Converts an 8 bit binary value in ACCA to the equivalent ASCII character 2
;* character string in accumulator D
;* Uses a table-driven method rather than various tricks.
;*
;* Passed: Binary value in ACCA
;* Returns: ASCII Character string in D
;* Side Fx: ACCB is destroyed

HEX_TABLE       FCC '0123456789ABCDEF'          ; Table for converting values

BIN2ASC         PSHA                            ; Save a copy of the input number on the stack      
                TAB                             ;  and copy it into ACCB
                ANDB #%00001111                 ; Strip off the upper nibble of ACCB
                CLRA                            ; D now contains 000n where n is the LSnibble
                ADDD #HEX_TABLE                 ; Set up for indexed load
                XGDX
                LDAA 0,x                        ; Get the LSnibble character

                PULB                            ; Retrieve the input number into ACCB
                PSHA                            ; and push the LSnibble character in its place
                RORB                            ; Move the upper nibble of the input number
                RORB                            ;  into the lower nibble position.
                RORB
                RORB
                ANDB #%00001111                 ; Strip off the upper nibble
                CLRA                            ; D now contains 000n where n is the MSnibble
                ADDD #HEX_TABLE                 ; Set up for indexed load
                XGDX                                                              
                LDAA 0,X                        ; Get the MSnibble character into ACCA
                PULB                            ; Retrieve the LSnibble character into ACCB
                RTS 

;*******************************************************************
;* Update Display (Current State + Bumper Switches                 *
;* + Battery Voltage + Sensor Readings)                            *
;******************************************************************* 
UPDT_DISPL      LDAA  #$82                      ; Move LCD cursor to the end of msg1
                JSR   cmd2LCD
               
                LDAB  CRNT_STATE                ; Display current state
                LSLB                            ;      -"-
                LSLB                            ;      -"-
                LSLB                            ;      -"-
                LDX   #tab                      ;      -"-
                ABX                             ;      -"-
                JSR   putsLCD                   ;      -"-
             
                LDAA  #$8F                      ; Move LCD cursor to the 1st row, end of msg2
                JSR   cmd2LCD
                LDAA  SENSOR_BOW                ; Convert value from bow sensor to a two digit hexidecimal value
                JSR   BIN2ASC                   ;      -"-
                JSR   putcLCD 
                EXG   A,B 
                JSR   putcLCD 

                LDAA  #$92                      ; Move LCD cursor to Line position    
                JSR   cmd2LCD
                LDAA  SENSOR_LINE               ; Convert value from line sensor to a t
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD 
                EXG   A,B 
                JSR   putcLCD

                LDAA  #$CC                      ; Move LCD cursor to Port position on 2nd row   
                JSR   cmd2LCD
                LDAA  SENSOR_PORT               ; Convert value from port sensor to a two digit hexidecimal value
                JSR   BIN2ASC                   ;      -"-
                JSR   putcLCD
                EXG   A,B
                JSR   putcLCD

                LDAA  #$CF                      ; Move LCD cursor to Mid position on 2nd row    
                JSR   cmd2LCD
                LDAA  SENSOR_MID                ; Convert value from mid sensor to a two digit hexidecimal value
                JSR   BIN2ASC                   ;      -"-
                JSR   putcLCD
                EXG   A,B
                JSR   putcLCD

                LDAA  #$D2                      ; Move LCD cursor to Starboard position on 2nd row
                JSR   cmd2LCD
                LDAA  SENSOR_STBD               ; Convert value from starboard sensor to a two digit hexidecimal value
                JSR   BIN2ASC                   ;      -"-
                JSR   putcLCD 
                EXG   A,B 
                JSR   putcLCD
         
                MOVB  #$90,ATDCTL5              ; r.just., unsign., sing.conv., mult., ch0, start conv.
                BRCLR ATDSTAT0,$80,*            ; wait until the conversion sequence is complete

                LDAA  ATDDR0L                   ; load the ch0 result into AccA
                LDAB  #39                       ; AccB = 39
                MUL                             ; AccD = AccA x AccB = 1st result x 39
                ADDD  #600                      ; AccD = 1st result x 39 + 600 = 1000V_batt

                JSR   int2BCD
                JSR   BCD2ASC

                LDAA  #$C2                      ; move LCD cursor to the end of msg3
                JSR   cmd2LCD                   ;      -"-
                LDAA  TEN_THOUS                 ; output the TEN_THOUS ASCII character
                JSR   putcLCD                   ;      -"-
                LDAA  THOUSANDS                 ; output the THOUSANDS ASCII character
                JSR   putcLCD                   ;      -"-
                LDAA  #$2E                      ; output the .
                JSR   putcLCD                   ;      -"-  
                LDAA  HUNDREDS                  ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ;      -"-

                LDAA  #$C9                      ; Move LCD cursor to the end of msg4
                JSR   cmd2LCD 

                BRCLR PORTAD0,#%00000100,bowON  ; If FWD_BUMP, then
                LDAA  #$20
                JSR   putcLCD
                BRA   stern_bump                ; Display 'B' on LCD
bowON:          LDAA  #$42
                JSR   putcLCD 

stern_bump:     BRCLR PORTAD0,#%00001000,sternON; If REV_BUMP, then
                LDAA  #$20
                JSR   putcLCD
                BRA   UPDT_DISPL_EXIT           ; Display 'S' on LCD
sternON:        LDAA  #$53
                JSR   putcLCD

UPDT_DISPL_EXIT RTS                             ; exit

; initializtions section
initPORTS       BCLR  DDRAD,$FF                 ; Make PORTAD an input
                BSET  DDRA, $FF                 ; Make PORTA an output
                BSET  DDRT, $30                 ; Set channels 4 & 5 of PORTT as output
                RTS

;********************************************************************************************
initAD          MOVB  #$C0,ATDCTL2              ; power up AD, select fast flag clear
                JSR   del_50us                  ; wait for 50 us
                MOVB  #$00,ATDCTL3              ; 8 conversions in a sequence
                MOVB  #$85,ATDCTL4              ; res=8, conv-clks=2, prescal=12
                BSET  ATDDIEN,$0C               ; configure pins AN03,AN02 as digital inputs
                RTS

;*******************************************************************
;* Initialization of the LCD: 4-bit data width, 2-line display,    *
;* turn on display, cursor and blinking off. Shift cursor right.   *
;*******************************************************************
initLCD         BSET  DDRB,%11111111            ; configure pins PB7,PB6,...,PB0 for output
                BSET  DDRJ,%11000000            ; configure pins PJ7,PJ6 for output
                LDY   #2000                     ; wait for LCD to be ready
                JSR   del_50us                  ;      -"-
                LDAA  #$28                      ; set 4-bit data, 2-line display
                JSR   cmd2LCD                   ;      -"-
                LDAA  #$0C                      ; display on, cursor off, blinking off
                JSR   cmd2LCD                   ;      -"-
                LDAA  #$06                      ; move cursor right after entering a character
                JSR   cmd2LCD                   ;      -"-
                RTS

;********************************************************************************************
clrLCD          LDAA  #$01                      ; clear cursor and return to home position
                JSR   cmd2LCD                   ;      -"-
                LDY   #40                       ; wait until "clear cursor" command is complete
                JSR   del_50us                  ;      -"-
                RTS

;********************************************************************************************
initTCNT        MOVB  #$80,TSCR1                ; Enable TCNT by setting bit 7
                MOVB  #$00,TSCR2                ; disable TCNT OVF interrupt, set prescaler to 1
                MOVB  #$FC,TIOS                 ; channels PT1/IC1,PT0/IC0 are input captures
                MOVB  #$05,TCTL4                ; capture on rising edges of IC1,IC0 signals
                MOVB  #$03,TFLG1                ; clear the C1F,C0F input capture flags
                MOVB  #$03,TIE                  ; enable interrupts for channels IC1,IC0
                RTS


; interrupt service routines
ISR1            MOVB  #$01,TFLG1                ; clear the C0F input capture flag
                INC   COUNT1                    ; increment COUNT1
                RTI

ISR2            MOVB  #$02,TFLG1                ; clear the C1F input capture flag
                INC   COUNT2                    ; increment COUNT2
                RTI

;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
                ORG   $FFFE
                DC.W  Entry                     ; Reset Vector
                ORG   $FFEE
                DC.W  ISR1                      ; COUNT1 INT
                ORG   $FFEC
                DC.W  ISR2                      ; COUNT2 INT