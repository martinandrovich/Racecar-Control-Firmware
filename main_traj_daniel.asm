; ###################################################################################################################################################
; Racecar Control Firmware
; Version 1.2.0
; 
; Sequential Flag Architecture

; ___________________________________________________________________________________________________________________________________________________
; >> VECTORS:

.ORG	0x00																	; Reset Vector
	JMP		INIT																; ^

.ORG 	0x02																	; INT0 Interrupt (PD2)
	JMP		INT0_HANDLER														; ^
	 
.ORG	0x04																	; INT1 Interrupt (PD3)
	JMP 	INT1_HANDLER														; ^

.ORG	0x14																	; TIMER1 Compare Match Interrupt
	JMP		TMR1_HANDLER														; ^

.ORG	0x20																	; ADC Conversion Complete Interrupt (PAO)
	JMP		ADC_HANDLER															; ^

; ____________________________________________________________________________________________________________________________________________________
; >> DEFINITIONS

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > INCLUDES

.ORG	0x28

	.INCLUDE	"macros.inc"													; Include Macros
	.INCLUDE	"ram_table.inc"													; Include RAM Table
	.INCLUDE	"command_table.inc"												; Include Command Table
	.INCLUDE	"break_table.inc"												; Include Brake Offset Table

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > CONSTANTS

	.EQU	BAUDRATE	= 0x00CF												; Baudrate configuration (default = 0xCF)

	.EQU	TMR1FREQ	= 976 - 1												; Timer1 configuration

																				; 62500 - 1		= 4Hz
																				; 31250 - 1		= 8Hz
																				; 15625 - 1		= 16Hz
																				;  7812 - 1		= 32Hz
																				;  1953 - 1		= 128Hz
																				;   976 - 1		= 256Hz [DEFAULT]
																				;   488 - 1		= 512Hz
																				;	244 - 1		= 1024Hz
																				;	122 - 1		= 2048Hz
																				;	 61 - 1		= 4096Hz

	; Moving Average Filter
	
	.EQU	MOVAVG_SIZE					= 64									; Size (bytes) of Moving Average Filter
	.EQU	MOVAVG_DIVS					= 6										; Number of division to perform (i.e. 2^5 = 32)
	.EQU	MOVAVG_TABLE_END			= MOVAVG_TABLE + MOVAVG_SIZE			;

	; Mapping & Turn Detection Constants

	.EQU	TURN_TH_IN_LEFT				= 122
	.EQU	TURN_TH_IN_RIGHT			= 115
	.EQU	TURN_TH_OUT_LEFT			= 122
	.EQU	TURN_TH_OUT_RIGHT			= 115

	.EQU	MAPPING_SEEK_PWM			= 58									; Mapping Seek PWM in BYTES (0-255)
	.EQU	MAPPING_PWM					= 90									; Mapping PWM in BYTES (0-255)
	.EQU	MAPPING_DEBOUNCE_VAL		= 10									; Mapping Debounce in TICKS
	.EQU	MAPPING_OFFSET_IN			= 8										; Mapping Offset In in TICKS
	.EQU	MAPPING_OFFSET_OUT			= 11									; Mapping Offset Out in TICKS

	; Trajectory Constants
	
	.EQU	TRAJECTORY_ACCLR_OFFSET		= 0										;
	.EQU	TRAJECTORY_BRAKE_OFFSET		= 0										;

	.EQU	TRAJECTORY_ACCLR_PWM		= 255									;
	.EQU	TRAJECTORY_TURN_PWM			= 101									;

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > REGISTERS

	.DEF	MDFLG		= R0													; Mode Flags
	.DEF	FNFLG		= R1													; Function Flags for Interrupts
	.DEF	MTFLG		= R2													; Mapping & Trajectory Flags
	
	.DEF	TEMP1		= R16													; Temporary Register #1
	.DEF	TEMP2		= R17													; Temporary Register #2
	.DEF	TEMP3		= R18													; Temporary Register #3
	.DEF	TEMPI		= R19													; Temporary Interrupts Register
	
	.DEF	TEMPWH		= R25													; Temporary Register (Word) Pair
	.DEF	TEMPWL		= R24													; ^
		
	.DEF	RXREG		= R20													; USART Reception Register
	.DEF	TXREG		= R21													; USART Transmission Register

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > FLAGS

	; GLOBAL VARS

	.EQU	MSB			= 7;

	; MDFLG | Mode Flags 

	.EQU	AUTO		= 7														; Autonomous Mode
	.EQU	MAP			= 6														; Mapping Mode
	.EQU	BROD2		= 5														; Broadcast Mode
	.EQU	BROD1		= 4														; ^
	.EQU	BROD0		= 3														; ^

	; FNFLG | Function Flags

	.EQU	TACHO		= 7														; Tachometer Ready
	.EQU	FNLNE		= 6														; Finishline Ready
	.EQU	ACCLR		= 5														; Accelerometer Ready
	.EQU	TMR1		= 4														; Timer1 Ready
	.EQU	CMDPD		= 3														; Command Pending

	; MTFLG | Mapping & Trajectory Flags

	.EQU	ISMAP		= 7														; Program currently mapping
	.EQU	INTURN		= 6														; In turn
	.EQU	TURNDIR		= 5														; Direction of turn (0 = L & 1 = R)
	.EQU	TJRDY		= 4														; Trajectory ready
	.EQU	ISBRAKE		= 3														;


; ____________________________________________________________________________________________________________________________________________________
; >> INITIALIZATION

INIT:

	; Stack Pointer

	LDI 	TEMP1, HIGH(RAMEND)													; Initialize Stack Pointer
	OUT 	SPH, TEMP1															; ^
	LDI 	TEMP1,  LOW(RAMEND)													; ^
	OUT 	SPL, TEMP1															; ^

	; SRAM Initialization

	CLR		TEMP1																; Initialize allocated SRAM to NULL
	STS		RECENT_DAT, TEMP1													; ^
	STS		TEL_STEP, TEMP1														; ^
	STS		MODE_FLG, TEMP1														; ^
	STS		FUNC_FLG, TEMP1														; ^
	STS		TACHOMETER_H, TEMP1													; ^
	STS		TACHOMETER_L, TEMP1													; ^
	STS		TACHOMETER_L_PREV, TEMP1											; ^
	STS		TURN_MIN_TACHOMETER_H, TEMP1										; ^
	STS		TURN_MIN_TACHOMETER_L, TEMP1										; ^
	STS		ACCELEROMETER, TEMP1												; ^
	STS		ADC_H, TEMP1														; ^
	STS		ADC_L, TEMP1														; ^
	STS		FINISHLINE, TEMP1

	CALL	MOVAVG_POINTER_RESET												; Reset Moving Average pointer
	CALL	MOVAVG_SRAM_SETUP													; Initialize allocated Moving Average SRAM to default

	; Flags Initialization

	CLR		MDFLG																; Clear Flag Registers
	CLR		FNFLG																; ^
	CLR		MTFLG																; ^

	; USART Config

	LDI		TEMP1, HIGH(BAUDRATE)												; Set Transmission Rate
	OUT		UBRRH, TEMP1														; ^
	LDI		TEMP1,  LOW(BAUDRATE)												; ^
	OUT		UBRRL, TEMP1														; ^

	LDI		TEMP1, 0x02															; Clear all Error Flags + Enable DoubleMode
	OUT		UCSRA, TEMP1														; ^

	LDI		TEMP1, (1<<RXEN)|(1<<TXEN)											; Enable Transmission & Reception
	OUT		UCSRB, TEMP1														; ^

	LDI		TEMP1, (1<<URSEL)|(3<<UCSZ0)										; Set Frame Format (8, N, 1)
	OUT		UCSRC, TEMP1														; ^

	CLR		RXREG																; Reset Reception Register
	CLR		TXREG																; Reset Transmission Register

	; ADC Config	

	LDI		TEMP1, (1<<ADLAR)													; Choose -> ADC0 and AVCC. Vcc = 5V
	OUT		ADMUX, TEMP1														; AUTOTRIGGER ENABLED (ADATE) otherwise it doesnt work?

	LDI		TEMP1, (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)			; ADEN: ENABLE ADC, ADSC: START CONVERSATION ; (1<<ADPS2)
	OUT		ADCSRA, TEMP1														; ADFR: Activate Free Running Select, Prescaler: 128 // 125kHz ADC clock

	SBI		ADCSR, ADSC															; Start ADC Conversion	

	; I/O (Port) Setup

	SBI		DDRD, PD4															; Set PD4 on PORTD as Output
	SBI		DDRD, PD7															; Set PD7 on PORTD as Output
	SBI		PORTD, PD2															; Set PD2 on PORTD as Pullup Input

	CBI		PORTD, PD4															; Disable MOSFET Brake

	; Timer1 Setup

	LDI		TEMP1, (1<<OCIE1A)													; Enable Timer1 Compare Match Interrupt
	OUT		TIMSK, TEMP1														; ^

	LDI		TEMP1, 0x00															; Set Default
	OUT		TCCR1A, TEMP1														; ^

	LDI		TEMP1, (1<<CS11)|(1<<CS10)|(1<<WGM12)								; Set 64 Prescelar, CTC-MODE
	OUT		TCCR1B, TEMP1														; ^

	LDI		TEMP1, HIGH(TMR1FREQ)												; Set timer offset
	OUT		OCR1AH, TEMP1														; ^
	LDI		TEMP1,  LOW(TMR1FREQ)												; ^
	OUT		OCR1AL, TEMP1														; ^

	LDI		TEMP1, (1<<TOV1)													; Enable Timer1
	OUT		TIFR, TEMP1															; ^
	
	; Waveform Generator (Timer2)

	LDI		TEMP1, 0x00															; Reset Timer2
	OUT		OCR2, TEMP1															; ^

	; External Interrupt Setup
	
	LDI		TEMP1, (1<<ISC01)|(1<<ISC00) | (1<<ISC10)|(1<<ISC11)				; Set INT0 & INT1 to rising edge
	OUT		MCUCR, TEMP1														; ^

	LDI 	TEMP1, (1<<INT0)|(1<<INT1)											; Enable external interrupts
	OUT 	GICR, TEMP1															; ^

	SEI																			; Set Global Interrupt Flag

	RJMP	MAIN																; Start MAIN Program

; ____________________________________________________________________________________________________________________________________________________
; >> MAIN PROGRAM

MAIN:
	
	CALL	LOAD_FLAGS															; Load Flags

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > FUNCTIONS
	
	CALL	TELEGRAM_CHECK														; Check for Telegrams

	SBRC	FNFLG, CMDPD														; Command Pending
	CALL	EXECUTE_COMMAND														; ^

	SBRC	FNFLG, FNLNE														; Finishline Ready
	CALL	LOG_FINISHLINE														; ^
	
	SBRC	FNFLG, TACHO														; Tachometer Ready
	CALL	LOG_TACHOMETER														; ^

	SBRC	FNFLG, ACCLR														; Accelerometer Ready
	CALL	LOG_ACCELEROMETER													; ^

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > MODES

	SBRC	MDFLG, AUTO															; Autonomous Mode
	NOP																			; ^

	SBRC	MDFLG, MAP															; Mapping Mode
	CALL	MAPPING																; ^

	SBRC	MDFLG, BROD0														; Broadcast Mode
	CALL	BROADCAST															; ^

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > CLOCK (TIMER1)

	SBRC	FNFLG, TMR1															; Timer1 Ready
	CALL	CLOCK																; ^

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > REPEAT LOOP

	RJMP	MAIN																; Loop forever

; ____________________________________________________________________________________________________________________________________________________
; >> SENSOR PROCESSING & LOGGING

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > TACHOMETER

LOG_TACHOMETER:
	
	// Tachometer value is not allowed to exceed 32.768
	
	LDS		TEMPWH, TACHOMETER_H												; Load previous values from SRAM
	LDS		TEMPWL, TACHOMETER_L												; into WORD registers

	ADIW	TEMPWH:TEMPWL, 1													; Increment data

	STS		TACHOMETER_H, TEMPWH												; Store new values into SRAM
	STS		TACHOMETER_L, TEMPWL												; ^

	CFLG	FNFLG, TACHO														; Clear TACHO flag in FNFLG

	;MOV		TEMP1, FNFLG														; Clear TACHO Flag
	;CBR		TEMP1, (1<<TACHO)													; ^
	;MOV		FNFLG, TEMP1														; ^

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > FINISH LINE

LOG_FINISHLINE:
	
	LDS		TEMP1, FINISHLINE													; Load, increment & store Finishline value
	INC		TEMP1																; ^
	STS		FINISHLINE, TEMP1													; ^

	CLR		TEMP1																;
	STS		TACHOMETER_H, TEMP1													;
	STS		TACHOMETER_L, TEMP1													;

	SBRC	MDFLG, MAP	 														; Skip clearing flag if mapping mode enabled
	RET																			; ^

	;MOV		TEMP1, FNFLG														; Clear FNLNE Flag
	;CBR		TEMP1, (1<<FNLNE)													; ^
	;MOV		FNFLG, TEMP1														; ^

	CFLG	FNFLG, FNLNE														; Set FNLNE flag in FNFLG

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > ACCELEROMETER

LOG_ACCELEROMETER:

	SBRS	FNFLG, TMR1															; Check if logging is synchronized with CLOCK (Timer1)
	RET

	IN		TEMP1, ADCL															; Read LOW of ADC
	NOP																			; ^
	STS		ADC_L, TEMP1														; ^

	IN		TEMP1, ADCH															; Read HIGH of ADC
	NOP																			; ^
	STS		ADC_H, TEMP1														; ^

	STS		ACCELEROMETER, TEMP1

	CALL	MOVAVG																; Apply Moving Average Filter

	SBI		ADCSR, ADSC															; Start ADC Conversion

	CFLG	FNFLG, ACCLR														; Set ACCLR flag in FNFLG

	;MOV		TEMP1, FNFLG														; Clear ACCLR Flag
	;CBR		TEMP1, (1<<ACCLR)													; ^
	;MOV		FNFLG, TEMP1														; ^

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> MAPPING

MAPPING:
	
	SBRC	FNFLG, FNLNE														; Check FNLNE flag
	RJMP	MAPPING_ISMAP_CHECK													; If SET then check ISMAP

	SBRC	MTFLG, ISMAP														; Check ISMAP flag
	RJMP	MAPPING_CHECK_DEBOUNCE												; If SET then continue mapping

	RJMP	MAPPING_ESC															; Return

MAPPING_ISMAP_CHECK:

	// If  ISMAP -> CLR ISMAP(MTFLG) && CLR MAP(MDFLG)							= End Mapping
	// If ~ISMAP -> SET ISMAP													= Begin Mapping
	
	;MOV		TEMP1, FNFLG														; Clear Finishline Flag
	;CBR		TEMP1, (1<<FNLNE)													; ^
	;MOV		FNFLG, TEMP1														; ^

	CFLG	FNFLG, FNLNE														; Set FNFNE flag in FNFLG
	
	SBRC	MTFLG, ISMAP														; Check ISMAP flag
	RJMP	MAPPING_END															; If SET then end mapping

	SBRS	MTFLG, ISMAP														; Check ISMAP flag
	RJMP	MAPPING_BEGIN														; If CLR then begin mapping

	
MAPPING_BEGIN:

	;MOV		TEMP1, MTFLG														; Set ISMAP Flag
	;SBR		TEMP1, (1<<ISMAP)													; ^
	;MOV		MTFLG, TEMP1														; ^

	SFLG	MTFLG, ISMAP														; Set ISMAP flag in MTFLG

	RCALL	MAPPING_RESET_DEBOUNCE												; Reset Tachometer Debounce											

	LDI		TEMP1, MAPPING_PWM													; Start vehicle with mapping PWM
	STS		RECENT_DAT, TEMP1													; ^	
	CALL	SET_MOTOR_PWM														; ^

	LDI		YH, HIGH(MAPP_TABLE)												; Initialize Y Pointer
	LDI		YL,  LOW(MAPP_TABLE)												; ^

	CLR		TEMP1																; Store 0x0000 into mapping in SRAM
	ST		Y+, TEMP1															; ^
	ST		Y+, TEMP1															; ^

	RJMP	MAPPING_ESC															; Return

MAPPING_END:
	
	;MOV		TEMP1, MDFLG														; Clear MAP flag
	;CBR		TEMP1, (1<<MAP)														; ^
	;MOV		MDFLG, TEMP1														; ^

	CFLG	MDFLG, MAP															; Clear MAP flag in MDFLG
	STS		MODE_FLG, MDFLG														; Store new mode flags to SRAM

	;MOV		TEMP1, MTFLG														; Clear ISMAP Flag
	;CBR		TEMP1, (1<<ISMAP)													; ^
	;MOV		MTFLG, TEMP1														; ^

	CFLG	MTFLG, ISMAP														; Clear ISMAP flag in MTFLG

	LDS		TEMP2, TACHOMETER_H													; Load current Tachometer values
	LDS		TEMP3, TACHOMETER_L													; ^

	STS		TRACK_LENGTH_H, TEMP2 												; Store Track Length (Finishline Tachometer value) into SRAM
	STS		TRACK_LENGTH_L, TEMP3 												; ^

	;ST		Y+, TEMP2															; Save finishline tachometer data
	;ST		Y+, TEMP3															;

	SER		TEMP1																; Store 0xFFFF into mapping in SRAM
	ST		Y+, TEMP1															; ^
	ST		Y, TEMP1															; ^

	CALL	SET_MOTOR_BREAK														; Break vehicle

	RJMP	MAPPING_ESC															; Return

MAPPING_CHECK_DEBOUNCE:
	
	LDS		TEMPWH, TACHOMETER_H												; Load current Tachometer value
	LDS		TEMPWL, TACHOMETER_L												; ^

	LDS		TEMP2, MAPPING_DEBOUNCE_H											; Load Debounce Tachometer value
	LDS		TEMP3, MAPPING_DEBOUNCE_L											; ^

	CP		TEMPWL, TEMP3														; Check if debounce is reached
	CPC		TEMPWH, TEMP2 														; ^
	BRLO	MAPPING_ESC															; Escape if FALSE
	RJMP	MAPPING_CHECK_TURN													; Continue if TRUE

MAPPING_RESET_DEBOUNCE:

	LDS		TEMPWH, TACHOMETER_H												; Load current Tachometer value
	LDS		TEMPWL, TACHOMETER_L												; ^
	
	ADIW	TEMPWH:TEMPWL, MAPPING_DEBOUNCE_VAL									; Update Minumum Tachometer Detection value (current Tachometer + Constant)

	STS		MAPPING_DEBOUNCE_H, TEMPWH											; Store values to SRAM
	STS		MAPPING_DEBOUNCE_L, TEMPWL											; ^

	RET																			; Return

MAPPING_CHECK_TURN:

	LDS		TEMP2, ACCELEROMETER												; Load Accelerometer value

	SBRC	MTFLG, INTURN														; Check INTURN flag
	RJMP	MAPPING_CHECK_TURN_OUT												; If SET then check if a turn has been exited
	RJMP	MAPPING_CHECK_TURN_IN												; If CLR then check if a turn has been detected

MAPPING_CHECK_TURN_IN:

	; Check Right Turn
	
	;MOV		TEMP1, MTFLG														; Set TURNDIR Flag (checking right turn)
	;SBR		TEMP1, (1<<TURNDIR)													; ^
	;MOV		MTFLG, TEMP1														; ^

	SFLG	MTFLG, TURNDIR														; Set TURNDIR flag in MTFLG

	CPI		TEMP2, TURN_TH_IN_RIGHT												; Check Right Turn In
	BRLO	MAPPING_ADD															; Create mapping entry if true

	; Check Left Turn

	;MOV		TEMP1, MTFLG														; Clear TURNDIR Flag (checking left turn)
	;CBR		TEMP1, (1<<TURNDIR)													; ^
	;MOV		MTFLG, TEMP1														; ^

	CFLG	MTFLG, TURNDIR														; Clear TURNDIR flag in MTFLG

	CPI		TEMP2, TURN_TH_IN_LEFT												; Check Left Turn In
	BRSH	MAPPING_ADD															; Create mapping entry if true

	RJMP	MAPPING_ESC															; Return

MAPPING_CHECK_TURN_OUT:

	SBRC	MTFLG, TURNDIR														; Check TURNDIR flag
	RJMP	MAPPING_CHECK_TURN_OUT_RIGHT										; If SET then check RIGHT turn out
	RJMP	MAPPING_CHECK_TURN_OUT_LEFT											; If CLR then check LEFT turn out

MAPPING_CHECK_TURN_OUT_RIGHT:

	CPI		TEMP2, TURN_TH_OUT_RIGHT											; Check Right Turn Out
	BRSH	MAPPING_ADD															; Create mapping entry if true

	RJMP	MAPPING_ESC															; Return

MAPPING_CHECK_TURN_OUT_LEFT:

	CPI		TEMP2, TURN_TH_OUT_LEFT												; Check Left Turn Out
	BRLO	MAPPING_ADD															; Create mapping entry if true

	RJMP	MAPPING_ESC															; Return

MAPPING_ADD:

	MOV		TEMP1, MTFLG														; Toggle INTURN flag
	LDI		TEMP2, (1<<INTURN)													; ^
	EOR		TEMP1, TEMP2														; ^
	MOV		MTFLG, TEMP1														; ^

	RCALL	MAPPING_RESET_DEBOUNCE												; Reset Tachometer Debounce

	LDS		TEMPWH, TACHOMETER_H												; Load current Tachometer values
	LDS		TEMPWL, TACHOMETER_L												; ^

	SBRC	MTFLG, INTURN														; 
	SBIW	TEMPWH:TEMPWL, MAPPING_OFFSET_IN									;
	SBRS	MTFLG, INTURN														;
	SBIW	TEMPWH:TEMPWL, MAPPING_OFFSET_OUT									;

	// Tachometer value is not allowed to exceed 32.768

	SBRC	TEMP1, INTURN														; Set MSB of Tachometer (HIGH) to value of INTURN bit
	ORI		TEMPWH, (1<<7)														; ^
	
	ST		Y+, TEMPWH															; Store mapping into SRAM
	ST		Y+, TEMPWL															; ^

MAPPING_ESC:

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> TRAJECTORY

TRAJECTORY:
	
	// BRANCH ACCORDING TO TJRDY FLAG

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > COMPILER

TRAJECTORY_COMPILER_SETUP:

	LDI		XH, HIGH(TRAJ_TABLE)												; Load X Pointer to Trajectory Table
	LDI		XL,  LOW(TRAJ_TABLE)												;

	LDI		YH, HIGH(MAPP_TABLE)												; Load Y Pointer to Mapping Table
	LDI		YL,  LOW(MAPP_TABLE)												; 

	CFLG	MTFLG, TJRDY

	LD		TEMP1, Y+															; ^
	LD		TEMP2, Y+															; 

	CPI		TEMP1, 0															; ^

	BREQ	TRAJECTORY_COMPILER_RUNUP											; Branch if FinishLine 00_00

	//Should start mapping, if error occured

	RET																			;

TRAJECTORY_COMPILER_RUNUP:

	LD		TEMP1, Y+															; Load Mapping Table TachoH & L
	LD		TEMP2, Y+															; ^

	CPI		TEMP1, 0xFF															; Check for 0xFF_FF
	BRNE	TRAJECTORY_COMPILER_RUNUP											; ^

	SBIW	Y, 4																; Offset the mapping with 4, must be last OutSwing

	LD		TEMP1, Y+															; Load Last Swing
	LD		TEMP2, Y															; 	

	ST		X+, TEMP1															; Save Last Swing for Run_Up
	ST		X+, TEMP2															;

	LDS		TEMPWH, TRACK_LENGTH_H 												; Load_Circuit_Length
	LDS		TEMPWL, TRACK_LENGTH_L												;

	SUB		TEMP2, TEMPWL														; Subtract Total circuit length with Last Swing value
	SBC		TEMP1, TEMPWH														; 

	STS		LATEST_STRAIGHT_L, TEMP2											; Save latest Straight
	STS		LATEST_STRAIGHT_H, TEMP1											;

	;STS		LATEST_STRAIGHT, TEMP2

	LDI		YH, HIGH(MAPP_TABLE+2)												; Load Y Pointer to (offset) Mapping Table
	LDI		YL,  LOW(MAPP_TABLE+2)												; 

TRAJECTORY_COMPILER_LOOP:

	LD		TEMP2, Y+															; TEMP2 first because MOVW is used, remember this!
	LD		TEMP1, Y+															;

	CPI		TEMP2, 0xFF															; Check EoT
	BREQ	TRAJECTORY_COMPILER_END												; 

	SBRC	TEMP2, 7															; CHECK HIGHBIT TACHO FOR BREAK OR ACCELEROMETER
	RJMP	TRAJECTORY_COMPILER_BREAK											;
	RJMP	TRAJECTORY_COMPILER_ACCELERATE										; 
	

TRAJECTORY_COMPILER_ACCELERATE:
	
	MOVW	TEMPWH:TEMPWL, TEMP2:TEMP1											;
	SBIW	TEMPWH:TEMPWL, TRAJECTORY_ACCLR_OFFSET								;

	ST		X+, TEMPWH															;
	ST		X+, TEMPWL															;

	STS		LATEST_STRAIGHT_H, TEMP2											;
	STS		LATEST_STRAIGHT_L, TEMP1											;

	RJMP	TRAJECTORY_COMPILER_LOOP											; Loop

TRAJECTORY_COMPILER_BREAK:
	
	LDS		TEMPWH, LATEST_STRAIGHT_H											;
	LDS		TEMPWL,	LATEST_STRAIGHT_L											;

	CBR		TEMP2, (1<<7)
	
	SUB		TEMP1, TEMPWL														; Calculate latest straight
	SBC		TEMP2, TEMPWH														;

	SBR		TEMP2, (1<<7)

	STS		LATEST_STRAIGHT, TEMP1												;

	;MOV		TXREG, TEMP1
	;CALL	SERIAL_WRITE

	RCALL	TRAJECTORY_COMPILER_BREAK_OFFSET									;

	SBIW	YH:YL, 2															; Offset back

	LD		TEMP2, Y+															; Read values again
	LD		TEMP1, Y+															;

	CBR		TEMP2, (1<<7)														; Clear MSB for sub

	SUB		TEMP1, TEMP3														;
	SBCI	TEMP2, 0															;

	SBR		TEMP2, (1<<7)														; Set MSB for sub

	ST		X+, TEMP2															; Store Tachometer values for break.
	ST		X+,	TEMP1															;

	RJMP	TRAJECTORY_COMPILER_LOOP											; Do it again

TRAJECTORY_COMPILER_BREAK_OFFSET:
	
	LDI		ZH, HIGH(BREAK_OFFSET_TABLE*2)										; Load Z Pointer to Break Offset Table
	LDI		ZL,  LOW(BREAK_OFFSET_TABLE*2)										; ^

	LDS		TEMP1, LATEST_STRAIGHT												; Load value of Latest Straight Distance

TRAJECTORY_COMPILER_BREAK_OFFSET_LOOP:

	LPM		TEMP2, Z															; Load value of Z Pointer from Table and increment by 2
	ADIW	ZH:ZL, 2

	CP		TEMP2, TEMP1														; Find matching Table value
	BRSH	TRAJECTORY_COMPILER_BREAK_OFFSET_END								; ^

	RJMP	TRAJECTORY_COMPILER_BREAK_OFFSET_LOOP

TRAJECTORY_COMPILER_BREAK_OFFSET_END:

	SBIW	ZH:ZL, 1															; Decrement Z Pointer
	LPM		TEMP3, Z															; Load value of Z Pointer															

	CALL	TELEGRAM_RESET														; Reset Telegram due to Z Pointer

	RET																			; Return

TRAJECTORY_COMPILER_END:

	SER		TEMP1																; Store 0xFFFF into trajectory in SRAM
	ST		X+, TEMP1															; ^
	ST		X, TEMP1															; ^
	
	SFLG	MTFLG, TJRDY

	RJMP	TRAJECTORY_ESC	

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > RUN

TRAJECTORY_RUNNER:

	SBRS MTFLG, TJRDY														; If Set Skip
	// RJMP TRAJECTORY_ESC													;
	RET																		; Should be checked for before...

	LD		TEMP1, X														; Check EoT
	CPI		TEMP1, 0xFF														;
	BREQ	TRAJECTORY_RUNNER_END											;

	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! SHOULD JUST LOAD FIRST TIME !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// LDI		XH, HIGH(TRAJ_TABLE)											; Load X Pointer to Trajectory Table
	// LDI		XL,  LOW(TRAJ_TABLE)											; -> SHOULD NOT DO THIS EVERYTIME 
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! SHOULD JUST LOAD FIRST TIME !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// LDI		YH, HIGH(MAPP_TABLE)											; Load Y Pointer to Mapping Table
	// LDI		YL,  LOW(MAPP_TABLE)											; -> SHOULD NOT DO THIS EVERYTIME
																				
	LDS		TEMP1, TACHOMETER_H													; Load Tachometer Vals
	LDS		TEMP2, TACHOMETER_L													; 

	SBRC	MTFLG, ISBRAKE														; Check Isbrake FLAG
	RJMP	TRAJECTORY_ISBRAKE_COMPARE											;

	LD		TEMPWH, X+															; Load Trajectory
	LD		TEMPWL, X+															; ^
	SBIW	XH:XL, 2															; Reset X-pointer to old pos

	LDI		TEMP3, (1<<MSB)														; Load MSB
	AND		TEMP3, TEMPWH														; Mask MSB
	CBR		TEMPWH, (1<<MSB)													; Remove high-bit

	CP		TEMPWL, TEMP2														; Comparing 16 bit- used to decide if action should happend or not
	CPC		TEMPWH, TEMP2														;
	BRSH	TRAJECTORY_RUNNER_BRK_ACCLR											; BRSH incase we skipped a tachometer tick

	// RJMP TRAJECTORY_ESC
	RET																			;

TRAJECTORY_RUNNER_BRK_ACCLR:
												
	SBRC	TEMP3, MSB															;Check if anything equals
	RJMP	TRAJECTORY_RUNNER_BRAKE												;Default vaLue should be 101 or something
	RJMP	TRAJECTORY_RUNNER_ACCELERATE										;Default value should be 255

TRAJECTORY_RUNNER_ACCELERATE:

	LDI		TEMP3, PWM_VELOCITY_VALUE															;Trajectory speed at first
	STS		RECENT_DAT, TEMP3													;
	CALL	SET_MOTOR_PWM														;
	ADIW	XH:XL, 2															;Offset Pointer
	ADIW	YH:YL, 2															;^
	// RJMP TRAJECTORY_ESC
	RET																			;Return

TRAJECTORY_RUNNER_BRAKE:

	CALL	SET_MOTOR_BREAK														;
	SFLG	MTFLAG, ISBRAKE														;
	// RJMP TRAJECTORY_ESC
	RET																			;

TRAJECTORY_ISBRAKE_COMPARE	
	
	LD		TEMPWH, Y+															;
	LD		TEMPWL, Y+															;
	SBIW	YH:YL, 2															;

	SBIW	TEMPWH:TEMPWL, BRAKE_OFFSET_VALUE											;

	CBR		TEMPWH, (1<<MSB)													;

	CP		TEMPWL, TEMP2														;
	CPC		TEMPWH, TEMP1														;
	BRSH	TRAJECTORY_ESC														;
	
	CFLG	MTFLAG, ISBRAKE														;

	LDI		TEMP3, PWM_VELOCITY_VALUE_ISBRAKE											;Trajectory speed at first
	STS		RECENT_DAT, TEMP3													;
	CALL	SET_MOTOR_PWM														; 

	ADIW	XH:XL, 2															;Offset Pointer
	ADIW	YH:YL, 2															;^

	// RJMP TRAJECTORY_ESC														;
	RET


TRAJECTORY_RUNNER_END:

	CFLG	######MTFLG######, (1<<STOP_RUNNER)
	// RJMP TRAJECTORY_ESC
	RET


;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

TRAJECTORY_ESC:

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> AUTONOMOUS MODE

	// Placeholder
	// ...

; ____________________________________________________________________________________________________________________________________________________
; >> BROADCAST

BROADCAST:
	
	SBRS	FNFLG, TMR1															; Check if broadcast is synchronized with CLOCK (Timer1)
	RET																			; ^
	
	LDI		TEMP1, (1<<BROD2)|(1<<BROD1)										; Mask Broadcast Modes
	AND		TEMP1, MDFLG														; ^

	CPI		TEMP1, (1<<BROD1)													; Tachometer Mode		= (011)
	BREQ	BROADCAST_TACHOMETER												; ^
	
	CPI		TEMP1, (1<<BROD2)|(1<<BROD1)										; Finishline Mode		= (111)
	BREQ	BROADCAST_FINISHLINE												; ^

	CPI		TEMP1, (1<<BROD2)													; Accelerometer Mode	= (101)
	BREQ	BROADCAST_ACCELEROMETER												; ^

	RCALL	BROADCAST_ALL														; Broadcast All			= (001)

	RET																			; Return

BROADCAST_TACHOMETER:

	LDS		TXREG, TACHOMETER_H													; Load & transmit HIGH byte of Tachometer data
	CALL	SERIAL_WRITE														; ^

	LDS		TXREG, TACHOMETER_L													; Load & transmit HIGH byte of Tachometer data
	CALL	SERIAL_WRITE														; ^

	RET																			; Return

BROADCAST_ACCELEROMETER:

	LDS		TXREG, ACCELEROMETER												; Load & transmit Accelerometer data
	CALL	SERIAL_WRITE														; ^

	RET																			; Return

BROADCAST_FINISHLINE:
	
	LDS		TXREG, FINISHLINE													; Load & transmit Finishline data
	CALL	SERIAL_WRITE														; ^

	RET																			; Return

BROADCAST_ALL:

	LDS		TEMP1, TACHOMETER_L													; Load current & recent Tachometer (LOW) value
	LDS		TEMP2, TACHOMETER_L_PREV											;

	CP		TEMP1, TEMP2														; Compare and only transmit if value has changed
	BRNE	BROADCAST_ALL_SEND													; ^

	RET																			; Return

BROADCAST_ALL_SEND:

	STS		TACHOMETER_L_PREV, TEMP1											; Update recent Tachometer (LOW) value

	RCALL	BROADCAST_TACHOMETER												; Broadcast Tachometer
	RCALL	BROADCAST_ACCELEROMETER												; Broadcast Accelerometer

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> COMMUNICATION PROTOCOL

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > USART

SERIAL_READ:
	
	SBIS	UCSRA, RXC															; Wait for Recieve (RXC) Flag
	RJMP	SERIAL_READ															; ^

	IN		RXREG, UDR															; Read data into Reception Register
	STS		SERIAL_RX, RXREG													; Store data in SRAM

	RET																			; Return

SERIAL_WRITE:
	
	SBIS	UCSRA, UDRE															; Wait for Empty Transmit Buffer (UDRE) Flag
	RJMP	SERIAL_WRITE														; ^

	OUT		UDR, TXREG															; Write data from Transmission Register

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > TELEGRAM PARSER

	// !#!#!#!
	// Needs to be changed to support recent Z pointer from SRAM!

TELEGRAM_CHECK:

	SBIS	UCSRA, RXC															; Return if no data in USART reception buffer
	RET																			; ^

	IN		RXREG, UDR															; Read data into Reception Register
	STS		SERIAL_RX, RXREG													; Store data in SRAM

TELEGRAM_PARSER:
	
	LDS		TEMP1, TEL_STEP														; Load, increment & store Telegram Step Counter
	INC		TEMP1																; ^
	STS		TEL_STEP, TEMP1														; ^

	CPI		TEMP1, 1															; Setup Telegram Parser if Step = 1
	BREQ	TELEGRAM_PARSE_SETUP												; ^

	CPI		TEMP1, 4															; Execute Telegram if Step = 4
	BREQ	TELEGRAM_EXECUTE													; ^

	CPI		TEMP1, 2															; Parse (TYPE) if Step = 2
	BREQ	TELEGRAM_JUMP														; ^

	DEC		ZL																	; Offset Z pointer (-1) to parse TYPE (0x00_XX)

	CPI		TEMP1, 3															; Parse (COMMAND) if Step = 3
	BREQ	TELEGRAM_JUMP														; ^

TELEGRAM_PARSER_ESC:
	
	RET

TELEGRAM_PARSE_SETUP:

	LDI		ZH, HIGH(COMMANDS*2)												; Reset Z Pointer to COMMANDS jump table
	LDI 	ZL,  LOW(COMMANDS*2)												; ^

	INC		ZL																	; Offset Z pointer (+1) to parse COMMAND (0xXX_00)

	RJMP	TELEGRAM_PARSER														; Return

TELEGRAM_JUMP:

	ADIW	ZH:ZL, 4															; Increment Z Pointer (by 4)

	LPM		TEMP1, Z															; Load the matching duty cycle

	CPI		TEMP1, 0xEE															; Reset everything if out of table bounds
	BREQ	TELEGRAM_ERROR														; ^

	CP		TEMP1, RXREG														; Find match in jump table
	BREQ	TELEGRAM_PARSER_ESC													; ^

	RJMP	TELEGRAM_JUMP														; Repeat

TELEGRAM_EXECUTE:

	ADIW	ZH:ZL, 2															; Point at & read LOW of Table address
	LPM		TEMP1, Z															; ^

	ADIW	ZH:ZL, 1															; Point at & read HIGH of Table address
	LPM		TEMP2, Z															; ^

	MOV		ZL, TEMP1															; Load Z Pointer
	MOV		ZH, TEMP2															; ^

	STS		RECENT_DAT, RXREG													; Store recieved data in SRAM

	;MOV		TEMP1, FNFLG														; Set CMDPD flag
	;SBR		TEMP1, (1<<CMDPD)													; ^
	;MOV		FNFLG, TEMP1														; ^

	SFLG	FNFLG, CMDPD														; Set CMDPD flag in FNFLG

TELEGRAM_RESET:
	
	CLR		TEMP1																; Reset & store parse step counter
	STS		TEL_STEP, TEMP1														; ^

	RET																			; Return

TELEGRAM_CLRBUFFER:

	IN		TEMP1, UDR															; Empty buffer
	SBIC	UCSRA, RXC															; ^
	RJMP	TELEGRAM_CLRBUFFER													; ^

	RET																			; Return

TELEGRAM_ERROR:

	CLR		RXREG																; Clear reception register

	;MOV		TEMP1, FNFLG														; Clear CMDPD flag
	;CBR		TEMP1, (1<<CMDPD)													; ^
	;MOV		FNFLG, TEMP1														; ^

	CFLG	FNFLG, CMDPD														; Clear CMDPD flag in FNFLG

	RCALL	TELEGRAM_RESET														; Reset parse step counter
	RCALL	TELEGRAM_CLRBUFFER													; Clear reception buffer
	
	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > COMMANDS

EXECUTE_COMMAND:
	
	;MOV		TEMP1, FNFLG														; Clear CMDPD flag
	;CBR		TEMP1, (1<<CMDPD)													; ^
	;MOV		FNFLG, TEMP1														; ^

	CFLG	FNFLG, CMDPD														; Clear CMDPD flag in FNFLG

	ICALL																		; Call function (address) of Z-pointer

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

BROADCAST_SET:

	MOV		TEMP1, MDFLG														; Load & reset current Broadcast Flags in Mode Register
	ANDI	TEMP1, 0b11000111													; ^

	LDS		TEMP2, RECENT_DAT													; Load & apply recieved Broadcast Flags
	OR		TEMP1, TEMP2														; ^

	MOV		MDFLG, TEMP1														; Save new mode flags
	STS		MODE_FLG, MDFLG														; Store new mode flags to SRAM

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

MAPPING_SET:

	SFLG	MDFLG, MAP															; Set MAP flag in MDFLG

	STS		MODE_FLG, MDFLG														; Store new mode flags to SRAM

	LDI		TEMP1, MAPPING_SEEK_PWM												; Start vehicle with mapping seek PWM
	STS		RECENT_DAT, TEMP1													; ^	
	CALL	SET_MOTOR_PWM														; ^

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

TRAJECTORY_SET:

	CALL	TRAJECTORY_COMPILER_SETUP											; Compile Trajectory

	RET

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

MAPPING_GET:

	// If MAPP > 512 bytes, then buffer will overflow in MatLab
	
	LDI		YH, HIGH(MAPP_TABLE)												; Initialize Y Pointer
	LDI		YL,  LOW(MAPP_TABLE)												; ^

MAPPING_GET_LOOP:

	LD		TEMP1, Y+															; Load mapping values (HIGH & LOW)
	LD		TEMP2, Y+															; ^

	MOV		TXREG, TEMP1														; Transmit HIGH byte of mapping
	CALL	SERIAL_WRITE														; ^

	MOV		TXREG, TEMP2														; Transmit LOW byte of mapping
	CALL	SERIAL_WRITE														; ^

	CPI		TEMP1, 0xFF															; Escape if EoT has been reached
	BREQ	MAPPING_GET_ESC														; ^

	RJMP	MAPPING_GET_LOOP													; Loop

MAPPING_GET_ESC:

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

TRAJECTORY_GET:

	// If TRAJS > 512 bytes, then buffer will overflow in MatLab
	
	CALL	TRAJECTORY_COMPILER_SETUP											; Compile Trajectory
	
	LDI		YH, HIGH(TRAJ_TABLE)												; Initialize Y Pointer
	LDI		YL,  LOW(TRAJ_TABLE)												; ^

TRAJECTORY_GET_LOOP:

	LD		TEMP1, Y+															; Load mapping values (HIGH & LOW)
	LD		TEMP2, Y+															; ^

	MOV		TXREG, TEMP1														; Transmit HIGH byte of mapping
	CALL	SERIAL_WRITE														; ^

	MOV		TXREG, TEMP2														; Transmit LOW byte of mapping
	CALL	SERIAL_WRITE														; ^

	CPI		TEMP1, 0xFF															; Escape if EoT has been reached
	BREQ	TRAJECTORY_GET_ESC													; ^

	RJMP	TRAJECTORY_GET_LOOP													; Loop

TRAJECTORY_GET_ESC:

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> DEVICE (RACECAR) CONTROL

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > MOTOR CONTROL

SET_MOTOR_PWM:

	LDI		TEMP1, 0x6A															; Initialize Waveform Generator (Timer2) (0110_1010)
	OUT		TCCR2, TEMP1														; ^

	LDS		TEMP3, RECENT_DAT													; Load recent recieved telegram data from SRAM
	STS		DUTY_CYCLE, TEMP3													; Store loaded Duty Cycle in SRAM
	
	TST		TEMP3																; Check if recieved Duty Cycle is 0
	BREQ	SET_MOTOR_BREAK														; Break vehicle if true

	CBI		PORTD, PD4															; Disable MOSFET Brake

	CALL	DELAY_100uS															; Wait for 100 ?s

	OUT		OCR2, TEMP3															; Set Duty Cycle (PWM) (0-255) on Timer2

SET_MOTOR_PWM_ESC:

	RET																			; Return

SET_MOTOR_BREAK:

	LDI		TEMP1, 0x00															; Disable Timer2 (PWM)
	OUT		TCCR2, TEMP1														; ^

	CBI 	PORTD, PD7															; Clear PD7 of PORTD
		
	CALL	DELAY_100uS															; Wait for 100 us

	SBI		PORTD, PD4															; Enable MOSFET Brake

	RJMP	SET_MOTOR_PWM_ESC													; Return

SET_MOTOR_MAX:

	SBI 	PORTD, PD7															; Set PD7 of PORTD
	RJMP	SET_MOTOR_PWM_ESC													; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > MOVING AVERAGE

	// !#!#!#!
	// Needs to be changed to support an arbitrary SRAM Location!

MOVAVG:

	LDS		XH, MOVAVG_RECENT_XH												; Load recent MOVAVG X pointer
	LDS		XL, MOVAVG_RECENT_XL												; ^

	LDS		TEMP1, ADC_H														; Insert ADC_val from high because of ADLAR
	ST		X+, TEMP1															; Save ADC_H to SRAM pointer X

	CPI		XL, MOVAVG_SIZE														;
	//CPI		XL, LOW(MOVAVG_TABLE_END)										; Check if Pointer should be reset
	BRNE	MOVAVG_SKIP_RESET													; ^
	//CPI		XH, HIGH(MOVAVG_TABLE_END)										; Check if Pointer should be reset
	//BRNE	MOVAVG_SKIP_RESET													; ^
	RCALL	MOVAVG_POINTER_RESET												; ^

MOVAVG_SKIP_RESET:
	
	//STS		MOVAVG_RECENT_XH, XH												; - skal med
	STS		MOVAVG_RECENT_XL, XL												; -

	RCALL	MOVAVG_ADD															; Do Moving Average Addition
	RCALL	MOVAVG_DIVIDE														; Do Moving Average Division

	RET																			; Return

MOVAVG_POINTER_RESET:

	LDI		XH, HIGH(MOVAVG_TABLE)												; Load reset values into X Pointer
	LDI		XL,  LOW(MOVAVG_TABLE)												; ^

	RET																			; Return

MOVAVG_SRAM_SETUP:

	LDI		TEMP1, MOVAVG_SIZE													; Load size of Moving Average filter into register

	STS		MOVAVG_RECENT_XH, XH												; Store location of X pointer into SRAM
	STS		MOVAVG_RECENT_XL, XL												; ^

MOVAVG_SRAM_SETUP_LOOP:
	
	ST		X+, TEMP1															; Set all values of SRAM to default value

	CPI		XL, LOW(MOVAVG_TABLE_END)											; Check if reached end of table.
	BRNE	MOVAVG_SRAM_SETUP_LOOP												; ^
	//CPI	XH, HIGH(MOVAVG_TABLE_END)						16BIT COMPARE!! CPC	; Check if reached end of table.
	//BRNE	MOVAVG_SRAM_SETUP_LOOP												; ^

	RET																			; Return

MOVAVG_ADD:
	
	CLR		TEMP1																; Reset Temporary Register
	CLR		TEMP2																; ^
	CLR		TEMP3																; ^

	RCALL	MOVAVG_POINTER_RESET												; Reset X Pointer

MOVAVG_ADD_LOOP:
	
	LD		TEMP1, X+ 															; Load value from X pointer location.
	ADD		TEMP2, TEMP1														; Add values

	// Can be changed to SBIC, SREG ..
	
	//SBRC	SREG, 0
	BRCC	MOVAVG_ADD_SKIP_CARRY												; Branch if carry is not set - skal slettes
	INC		TEMP3																; ^

MOVAVG_ADD_SKIP_CARRY:															;skal slettes!
	
	CPI		XL, LOW(MOVAVG_TABLE_END)											; Check if reached end of table
	BRNE	MOVAVG_ADD_LOOP														; 
	//CPI		XH, HIGH(MOVAVG_TABLE_END) THIS IS WRONG SHOULD BE CPC 16bit if ever moved!
	//BRNE	MOVAVG_ADD_LOOP

	RET																			; Return

MOVAVG_DIVIDE:

	LDI		TEMP1, MOVAVG_DIVS													; Load number of Divisions

MOVAVG_DIVIDE_LOOP:

	ASR		TEMP3																; TEMP3 IS HIGH
	ROR		TEMP2																; TEMP2 IS LOW

	DEC		TEMP1																; Perform 16 bit divison until done
	BRNE	MOVAVG_DIVIDE_LOOP													; ^

	STS		ACCELEROMETER, TEMP2												; Save value of division into SRAM

	RET

; ____________________________________________________________________________________________________________________________________________________
; >> CONTROL UNIT

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > FLAGS

LOAD_FLAGS:
	
	CLI																			; Disable Interrupts

	LDS		TEMP1, FUNC_FLG														; Merge SRAM Function Flags from SRAM with Register Function Flags
	OR		FNFLG, TEMP1														; ^

	CLR		TEMP1																; Reset SRAM Function Flags
	STS		FUNC_FLG, TEMP1														; ^

	SEI																			; Enable Interrupts

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > CLOCK

CLOCK:

	NOP																			; Do Something

	;MOV		TEMP1, FNFLG														; Clear TMR1 flag
	;CBR		TEMP1,  (1<<TMR1)													; ^
	;MOV		FNFLG, TEMP1														; ^

	CFLG	FNFLG, TMR1															; Clear TMR1 flag in FNFLG

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > DEBUGGING

EMPTY:
	NOP
	RET

TEST:

	LDI		TEMP1, 60
	STS		RECENT_DAT, TEMP1

	CALL	SET_MOTOR_PWM
	CALL	DELAY
	CALL	SET_MOTOR_BREAK
	CALL	DELAY

	RET

TEST35:
	
	LDI		TXREG, 0x35
	CALL	SERIAL_WRITE
	
	RET

TEST40:
	
	LDI		TXREG, 0x40
	CALL	SERIAL_WRITE
	
	RET

TEST45:
	
	LDI		TXREG, 0x45
	CALL	SERIAL_WRITE
	
	RET

DELAY:
    LDI		TEMP1, 100
LOOP3:
	LDI		TEMP2, 100
LOOP2:
	LDI		TEMP3, 100
LOOP1:
    DEC		TEMP3
    BRNE	LOOP1
    DEC		TEMP2
    BRNE	LOOP2
    DEC		TEMP1
    BRNE	LOOP3

    RET

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > 10ms DELAY

DELAY_10MS:

    LDI		TEMP1, 208
    LDI		TEMP2, 202

DELAY_10MS_LOOP:

	DEC		TEMP2
    BRNE	DELAY_10MS_LOOP
    DEC		TEMP1
    BRNE	DELAY_10MS_LOOP
    NOP

	RET

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > 100us DELAY

DELAY_100uS:

    LDI  TEMP1, 3
    LDI  TEMP2, 19

DELAY_100uS_LOOP:
	
	DEC  TEMP2
    BRNE DELAY_100US_LOOP
    DEC  TEMP1
    BRNE DELAY_100US_LOOP

	RET

; ____________________________________________________________________________________________________________________________________________________
; >> INTERRUPT HANDLERS

INT0_HANDLER:
	
	LDS		TEMPI, FUNC_FLG														; Load Function Flags from SRAM into Temporary Interrupt Register
	SET																			; Set T flag
	BLD		TEMPI, TACHO														; Set BIT in Temporary Interrupt Register
	STS		FUNC_FLG, TEMPI														; Store Function Flags to SRAM

	RETI																		; Return

INT1_HANDLER:
	
	LDS		TEMPI, FUNC_FLG														; Load Function Flags from SRAM into Temporary Interrupt Register
	SET																			; Set T flag
	BLD		TEMPI, FNLNE														; Set BIT in Temporary Interrupt Register
	STS		FUNC_FLG, TEMPI														; Store Function Flags to SRAM

	RETI																		; Return

TMR1_HANDLER:
	
	LDS		TEMPI, FUNC_FLG														; Load Function Flags from SRAM into Temporary Interrupt Register
	SET																			; Set T flag
	BLD		TEMPI, TMR1															; Set BIT in Temporary Interrupt Register
	STS		FUNC_FLG, TEMPI														; Store Function Flags to SRAM

	RETI																		; Return

ADC_HANDLER:

	LDS		TEMPI, FUNC_FLG														; Load Function Flags from SRAM into Temporary Interrupt Register
	SET																			; Set T flag
	BLD		TEMPI, ACCLR														; Set BIT in Temporary Interrupt Register
	STS		FUNC_FLG, TEMPI														; Store Function Flags to SRAM		

	RETI																		; Return