; ###################################################################################################################################################
; Racecar Control Firmware
; Version 1.1.2
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

.ORG	0x20																	; ADC Conversion Complete Interrupt
	JMP		ADC_HANDLER															; ^

; ____________________________________________________________________________________________________________________________________________________
; >> DEFINITIONS

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > INCLUDES

.ORG	0x28

	.INCLUDE	"ram_table.inc"
	.INCLUDE	"command_table.inc"

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

	; Turn Detection Thresholds

	.EQU	TURN_THRESHOLD_RIGHT		= 115
	.EQU	TURN_THRESHOLD_LEFT			= 122
	.EQU	TURN_THRESHOLD_DELAY		= 10
	.EQU	TURN_THRESHOLD_OFFSET		= 0

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > REGISTERS

	.DEF	MDFLG		= R0													; Mode Flags for Interrupts
	.DEF	FNFLG		= R1													; Function Flags for Interrupts
	
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

	; Mode Flags

	.EQU	AUTO		= 7														; Autonomous Mode
	.EQU	MAP			= 6														; Mapping Mode
	.EQU	BROD2		= 5														; Broadcast Mode
	.EQU	BROD1		= 4														; ^
	.EQU	BROD0		= 3														; ^

	; Function Flags

	.EQU	TACHO		= 7														; Tachometer Ready
	.EQU	FNLNE		= 6														; Finishline Ready
	.EQU	ACCLR		= 5														; Accelerometer Ready
	.EQU	TMR1		= 4														; Timer1 Ready
	.EQU	CMDPD		= 3														; Command Pending
	.EQU	INTURN		= 2														; Currently in a turn

; ____________________________________________________________________________________________________________________________________________________
; >> INITIALIZATION

INIT:

	; Stack Pointer

	LDI 	TEMP1, HIGH(RAMEND)
	OUT 	SPH, TEMP1
	LDI 	TEMP1,  LOW(RAMEND)
	OUT 	SPL, TEMP1

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

	CLR		MDFLG																; Clear Flags
	CLR		FNFLG																; ^

	; USART Config

	LDI		TEMP1, HIGH(BAUDRATE)												; Set Transmission Rate
	OUT		UBRRH, TEMP1														; ^
	LDI		TEMP1, LOW(BAUDRATE)												; ^
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

	SBI		DDRD, PD7															; Set PD7 on PORTD as Output
	SBI		PORTD, PD2															; Set PD2 on PORTD as Pullup Input

	LDI		TEMP1, 0x00															; Set Port A as Input (is this needed?)
	OUT		DDRA, TEMP1															; ^

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

	SEI																			; Set global interrupt flag

	RJMP	MAIN																; Start MAIN Program

; ____________________________________________________________________________________________________________________________________________________
; >> MAIN PROGRAM

MAIN:
	
	CALL	LOAD_FLAGS															; Load Flags

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > FUNCTIONS
	
	SBRC	FNFLG, TACHO														; Tachometer Ready
	CALL	LOG_TACHOMETER														; ^

	SBRC	FNFLG, FNLNE														; Finishline Ready
	CALL	LOG_FINISHLINE														; ^

	SBRC	FNFLG, ACCLR														; Accelerometer Ready
	CALL	LOG_ACCELEROMETER													; ^

	CALL	TELEGRAM_CHECK														; Check for Telegrams

	SBRC	FNFLG, CMDPD														; Command Pending
	CALL	EXECUTE_COMMAND														; ^

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > MODES

	SBRC	MDFLG, AUTO															; Autonomous Mode
	NOP																			; ^

	SBRC	MDFLG, MAP															; Mapping Mode
	NOP																			; ^

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

	LDS		TEMPWH, TACHOMETER_H												; Load previous values from SRAM
	LDS		TEMPWL, TACHOMETER_L												; into WORD registers

	ADIW	TEMPWH:TEMPWL, 1													; Increment data

	STS		TACHOMETER_H, TEMPWH												; Store new values into SRAM
	STS		TACHOMETER_L, TEMPWL												; ^

	MOV		TEMP1, FNFLG														; Clear Tachometer Flag
	CBR		TEMP1, (1<<TACHO)													; ^
	MOV		FNFLG, TEMP1														; ^

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > FINISH LINE

LOG_FINISHLINE:

	LDS		TEMP1, FINISHLINE													; Test purposes, send how many time it detects finishline

	INC		TEMP1																;

	STS		FINISHLINE, TEMP1													;

	MOV		TEMP1, FNFLG														; Clear Finishline Flag
	CBR		TEMP1, (1<<FNLNE)													; ^
	MOV		FNFLG, TEMP1														; ^

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
	CALL	TURN_CHECK															; Replace ACCLR data with Turn Detection

	SBI		ADCSR, ADSC															; Start ADC Conversion

	MOV		TEMP1, FNFLG														; Clear Accelerometer Flag
	CBR		TEMP1, (1<<ACCLR)													; ^
	MOV		FNFLG, TEMP1														; ^

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> MAPPING

	// PLACEHOLDER
	// ...
	//
	// FOR MAPPING

	//.EQU	MAPPING_RECENT_XH				= 0x0076
	//.EQU	MAPPING_RECENT_XL				= 0x0077
	
	//MAPPING_POINTER_RESET:
	//	LDI		XH, HIGH(MAPP)												; Load reset values into X Pointer
	//	LDI		XL,  LOW(MAPP)												; ^
	//	STS		MAPPING_RECENT_XH, XH
	//	STS		MAPPING_RECENT_XL, XL
	//	RET																	; Return
	
	//MAPPING_SAVE_VALUE_X:
	//	STS		MAPPING_RECENT_XH, XH
	//	STS		MAPPING_RECENT_XL, XL
	//	RET
	
	//MAPPING_LOAD_VALUE_X:
	//	LDS		XH, MAPPING_RECENT_XH
	//	LDS		XL, MAPPING_RECENT_XL
	//	RET
	
	//	CHECKIFSWING:
	//	LDS		TEMP1,	FINISHLINE
	//	CPI		TEMP1,	1
	//	BREQ	((DO SOMETHING HAVENT FIGURED OUT WHAT YET))
	//	LDS		TEMP1,	ACCELEROMETER
	//	CPI		TEMP1,	0
	//	BREQ	NOSWING_RETURN	//should use pointer here so pointer points to 0 -> 1 -> 0 -> 1 -> 0 -> 1 -> 0 -> 1 ...
	//	LDS		TEMP1,	TACHOMETER_H
	//	ST		X+, TEMP1
	//	LDS		TEMP1,	TACHOMETER_L
	//	ST		X+, TEMP1

	//	RET
	
	// 1. Compare previous Tachometer value with new -> add entry if necessary
	// 2. Check for swings
	// 3. Check for finishline

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

BROADCAST_FINISHLINE:															;

	// A bit retarded.
	
	LDS		TXREG, FINISHLINE													;
	CALL	SERIAL_WRITE														;

	RET																			; Return

BROADCAST_ALL:

	LDS		TEMP1, TACHOMETER_L													;
	LDS		TEMP2, TACHOMETER_L_PREV											;

	CP		TEMP1, TEMP2														; makes sure to not send the same TACHOMETER twice
	BRNE	BROADCAST_ALL_SEND													;

	RET																			;

BROADCAST_ALL_SEND:

	STS		TACHOMETER_L_PREV, TEMP1											; Update recent Tachometer (LOW) value

	RCALL	BROADCAST_TACHOMETER												; Broadcast Tachometer
	RCALL	BROADCAST_ACCELEROMETER												; Broadcast Accelerometer

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

	RCALL	SET_COMMAND_FLG														; Set command pending flag

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

	RCALL	CLR_COMMAND_FLG														; Clear command pending flag
	RCALL	TELEGRAM_RESET														; Reset parse step counter
	RCALL	TELEGRAM_CLRBUFFER													; Clear reception buffer
	
	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > COMMANDS

SET_COMMAND_FLG:

	MOV		TEMP1, FNFLG														; Set CMDPD bit
	SBR		TEMP1, (1<<CMDPD)													; ^
	MOV		FNFLG, TEMP1														; ^
	
	RET

CLR_COMMAND_FLG:
	
	MOV		TEMP1, FNFLG														; Clear CMDPD bit
	CBR		TEMP1, (1<<CMDPD)													; ^
	MOV		FNFLG, TEMP1														; ^
	
	RET

EXECUTE_COMMAND:
	
	RCALL	CLR_COMMAND_FLG														; Clear Command Pending Flag

	ICALL																		; Call function (address) of Z-pointer

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> DEVICE (RACECAR) CONTROL

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > MOTOR CONTROL

SET_MOTOR_PWM:

	LDI		TEMP1, 0x6A															; Initialize Waveform Generator (Timer2) (0110_1010)
	OUT		TCCR2, TEMP1														; ^

	LDS		TEMP1, RECENT_DAT													; Load recent recieved telegram data from SRAM
	STS		DUTY_CYCLE, TEMP1													; Store loaded Duty Cycle in SRAM

	TST		TEMP1																; Check if recieved Duty Cycle is 0
	BREQ	SET_MOTOR_MIN														; Stop vehicle if true (or BRAKE)

	OUT		OCR2, TEMP1															; Set Duty Cycle (PWM) (0-255) on Timer2

SET_MOTOR_PWM_ESC:

	RET																			; Return

SET_MOTOR_MIN:

	LDI		TEMP1, 0x00															; Disable Timer2 (PWM)
	OUT		TCCR2, TEMP1														; ^

	CBI 	PORTD, PD7															; Clear BIT on PIN7 of PORTD
	RJMP	SET_MOTOR_PWM_ESC													; Return

	RJMP	SET_MOTOR_PWM_ESC													; Return

SET_MOTOR_MAX:

	SBI 	PORTD, PD7															; Set BIT on PIN7 of PORTD
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

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > TURN DETECTION (ACCELEROMETER THRESHOLD)

TURN_CHECK:

	// Replaces accelerometer data with value of swing direction
	// 1 = LEFT, 0 = STRAIGHT, 2 = RIGHT
	
	// !#!#!#!
	// Tachometer should travel a minimum distance (delay) after exiting a turn,
	// before a turn may be detected and Accelerometer value changed again.

	LDS		TEMPWH, TACHOMETER_H												; Load current Tachometer value
	LDS		TEMPWL, TACHOMETER_L												; ^

	LDS		TEMP2, TURN_MIN_TACHOMETER_H										; Load Minumum Tachometer Detection value
	LDS		TEMP3, TURN_MIN_TACHOMETER_L										; ^

	CP		TEMPWL, TEMP3														; Comapre to check if delay is reached
	CPC		TEMPWH, TEMP2 														; ^
	BRLO	TURN_CHECK_STRAIGHT													; Branch to STRAIGHT if not

	LDS		TEMP1, ACCELEROMETER												; Load Accelerometer value

	CPI		TEMP1, TURN_THRESHOLD_RIGHT											; Check Right Turn
	BRLO	TURN_CHECK_RIGHT													; ^

	CPI		TEMP1, TURN_THRESHOLD_LEFT											; Check Left Turn
	BRSH	TURN_CHECK_LEFT														; ^

	SBRC	FNFLG, INTURN														; If no turns detected, check if INTURN is SET
	RCALL	TURN_CHECK_SETDELAY													; Update Minumum Tachometer Detection value if turn has recently been exited

	MOV		TEMP1, FNFLG														; Clear TURNIN bit
	CBR		TEMP1, (1<<INTURN)													; ^
	MOV		FNFLG, TEMP1														; ^

TURN_CHECK_STRAIGHT:

	CLR		TEMP1																; Set Accelerometer value to 0 (STRAIGHT)
	STS		ACCELEROMETER, TEMP1												; ^

	RET																			; Return
 
TURN_CHECK_RIGHT:

	LDI		TEMP1, 2															; Set Accelerometer value to 2 (RIGHT)
	STS		ACCELEROMETER, TEMP1												; ^

	MOV		TEMP1, FNFLG														; Set TURNIN bit
	SBR		TEMP1, (1<<INTURN)													; ^
	MOV		FNFLG, TEMP1														; ^

	RET																			; Return

TURN_CHECK_LEFT:

	LDI		TEMP1, 1															; Set Accelerometer value to 2 (LEFT)
	STS		ACCELEROMETER, TEMP1												;

	MOV		TEMP1, FNFLG														; Set TURNIN bit
	SBR		TEMP1, (1<<INTURN)													; ^
	MOV		FNFLG, TEMP1														; ^

	RET																			; Return																			

TURN_CHECK_SETDELAY:

	LDS		TEMPWH, TACHOMETER_H												; Load current Tachometer value
	LDS		TEMPWL, TACHOMETER_L												; ^
	
	ADIW	TEMPWH:TEMPWL, TURN_THRESHOLD_DELAY									; Update Minumum Tachometer Detection value (current Tachometer + constant)

	STS		TURN_MIN_TACHOMETER_H, TEMPWH										; Store values to SRAM
	STS		TURN_MIN_TACHOMETER_L, TEMPWL										; ^

	RET																			; Return

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

	MOV		TEMP1, FNFLG														; Clear Timer1 Flag
	CBR		TEMP1,  (1<<TMR1)													; ^
	MOV		FNFLG, TEMP1														; ^

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > PURE TESTING

EMPTY:
	NOP
	RET

TEST:

	LDI		TEMP1, 60
	STS		RECENT_DAT, TEMP1

	CALL	SET_MOTOR_PWM
	CALL	DELAY
	CALL	SET_MOTOR_MIN
	CALL	DELAY

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