; ###################################################################################################################################################
; Racecar Control Firmware
; Version 1.1.3
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

	.EQU	TMR1FREQ	= 62500 - 1												; Timer1 configuration

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

	.EQU	TURN_TH_IN_LEFT				= 122
	.EQU	TURN_TH_IN_RIGHT			= 122
	.EQU	TURN_TH_OUT_LEFT			= 115
	.EQU	TURN_TH_OUT_RIGHT			= 115

	.EQU	MAPPING_SEEK_PWM			= 56									; Mapping Seek PWM in BYTES (0-255)
	.EQU	MAPPING_PWM					= 90									; Mapping PWM in BYTES (0-255)
	.EQU	MAPPING_DEBOUNCE_VAL		= 10									; Mapping Debounce in TICKS
	.EQU	MAPPING_OFFSET_IN			= 8										; Mapping Offset In in TICKS
	.EQU	MAPPING_OFFSET_OUT			= 11									; Mapping Offset Out in TICKS

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

	; Mapping SRAM Initialization TEST

	LDI		TEMP1, 0x01
	STS		MAPP, TEMP1

	LDI		TEMP1, 0x02
	STS		MAPP+1, TEMP1

	LDI		TEMP1, 0xFF
	STS		MAPP+2, TEMP1

	; Flags Initialization

	CLR		MDFLG																; Clear Flag Registers
	CLR		FNFLG																; ^
	CLR		MTFLG																; ^

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

	SBRC	MDFLG, MAP	 														; Skip clearing flag if mapping mode enabled
	RET																			; ^

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

	SBI		ADCSR, ADSC															; Start ADC Conversion

	MOV		TEMP1, FNFLG														; Clear Accelerometer Flag
	CBR		TEMP1, (1<<ACCLR)													; ^
	MOV		FNFLG, TEMP1														; ^

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> MAPPING

MAPPING:
	
	SBRC	FNFLG, FNLNE														; Check FNLNE flag
	RJMP	MAPPING_ISMAP_CHECK													; If SET then check ISMAP

	SBRC	MTFLG, ISMAP														; Check ISMAP flag
	RJMP	MAPPING_CHECK_DEBOUNCE												; If SET then continue mapping

	RET

MAPPING_ISMAP_CHECK:

	// If  ISMAP -> CLR ISMAP(MTFLG) && CLR MAP(MDFLG)							= End Mapping
	// If ~ISMAP -> SET ISMAP													= Begin Mapping
	
	CALL	TEST35
	
	MOV		TEMP1, FNFLG														; Clear Finishline Flag
	CBR		TEMP1, (1<<FNLNE)													; ^
	MOV		FNFLG, TEMP1														; ^
	
	SBRC	MTFLG, ISMAP														; Check ISMAP flag
	RCALL	MAPPING_END															; If SET then end mapping

	SBRS	MTFLG, ISMAP														; Check ISMAP flag
	RCALL	MAPPING_BEGIN														; If CLR then begin mapping

	RET																			; Return
	
MAPPING_BEGIN:
	
	CALL	TEST40
	
	MOV		TEMP1, MTFLG														; SET ISMAP Flag
	SBR		TEMP1, (1<<ISMAP)													; ^
	MOV		MTFLG, TEMP1														; ^

	LDI		TEMP1, MAPPING_PWM													; Start vehicle with mapping PWM
	STS		RECENT_DAT, TEMP1													; ^	
	CALL	SET_MOTOR_PWM														; ^

	LDI		YH, HIGH(MAPP)														; Initialize Y Pointer
	LDI		YL,  LOW(MAPP)														; ^

	CLR		TEMP1																; Store 0x0000 into mapping in SRAM
	ST		Y+, TEMP1															; ^
	ST		Y+, TEMP1															; ^

	RET																			; Return

MAPPING_END:

	CALL	TEST45
	
	MOV		TEMP1, MDFLG														; CLR MAP Flag
	CBR		TEMP1, (1<<MAP)														; ^
	MOV		MDFLG, TEMP1														; ^

	MOV		TEMP1, MTFLG														; CLR ISMAP Flag
	CBR		TEMP1, (1<<ISMAP)													; ^
	MOV		MTFLG, TEMP1														; ^

	SER		TEMP1																; Store 0xFFFF into mapping in SRAM
	ST		Y+, TEMP1															; ^
	ST		Y, TEMP1															; ^

	CALL	SET_MOTOR_MIN														; Break vehicle

	RET																			; Return

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
	
	ADIW	TEMPWH:TEMPWL, MAPPING_DEBOUNCE_VAL									; Update Minumum Tachometer Detection value (current Tachometer + constant)

	STS		MAPPING_DEBOUNCE_H, TEMPWH											; Store values to SRAM
	STS		MAPPING_DEBOUNCE_L, TEMPWL											; ^

	RET																			; Return

MAPPING_CHECK_TURN:

	LDS		TEMP1, ACCELEROMETER												; Load Accelerometer value

	SBRC	MTFLG, INTURN														; Check INTURN flag
	RJMP	MAPPING_CHECK_TURN_OUT												; If SET then check if a turn has been exited
	RJMP	MAPPING_CHECK_TURN_IN												; If CLR then check if a turn has been detected

MAPPING_CHECK_TURN_IN:

	; Check Right Turn
	
	MOV		TEMP1, MTFLG														; SET TURNDIR Flag (checking right turn)
	SBR		TEMP1, (1<<TURNDIR)													; ^
	MOV		MTFLG, TEMP1														; ^

	CPI		TEMP1, TURN_TH_IN_RIGHT												; Check Right Turn In
	BRLO	MAPPING_ADD															; Create mapping entry if true

	; Check Left Turn

	MOV		TEMP1, MTFLG														; CLR TURNDIR Flag (checking left turn)
	CBR		TEMP1, (1<<TURNDIR)													; ^
	MOV		MTFLG, TEMP1														; ^

	CPI		TEMP1, TURN_TH_IN_LEFT												; Check Left Turn In
	BRSH	MAPPING_ADD															; Create mapping entry if true

	RJMP	MAPPING_ESC															; Return

MAPPING_CHECK_TURN_OUT:

	SBRC	MTFLG, TURNDIR														; Check TURNDIR flag
	RCALL	MAPPING_CHECK_TURN_OUT_RIGHT										; If SET then check RIGHT turn out
	RCALL	MAPPING_CHECK_TURN_OUT_LEFT											; If CLR then check LEFT turn out

	RJMP	MAPPING_ESC															; Return

MAPPING_CHECK_TURN_OUT_RIGHT:

	CPI		TEMP1, TURN_TH_OUT_RIGHT											; Check Right Turn Out
	BRSH	MAPPING_ADD															; Create mapping entry if true

	RET

MAPPING_CHECK_TURN_OUT_LEFT:

	CPI		TEMP1, TURN_TH_OUT_LEFT												; Check Left Turn Out
	BRLO	MAPPING_ADD															; Create mapping entry if true

	RET																			; Return

MAPPING_ADD:

	MOV		TEMP1, MTFLG														; Toggle INTURN flag
	LDI		TEMP2, (1<<INTURN)													; ^
	EOR		TEMP1, TEMP2														; ^
	MOV		FNFLG, TEMP1														; ^

	LDS		TEMP2, TACHOMETER_H													; Load current Tachometer values
	LDS		TEMP3, TACHOMETER_L													; ^

	SBRC	TEMP1, INTURN														; Set MSB of Tachometer (HIGH) to value of INTURN bit
	ORI		TEMP2, (1<<7)														; ^

	RCALL	MAPPING_RESET_DEBOUNCE												; Reset Tachometer Debounce

	// Store pointers?
	
	ST		Y+, TEMP2															; Store Mapping into SRAM
	ST		Y+, TEMP3															; ^

MAPPING_ESC:

	RET																			; Return

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

	MOV		TEMP1, MDFLG														; Set MAP flag in MDFLG
	SBR		TEMP1, (1<<MAP)														; ^
	MOV		MDFLG, TEMP1														; ^

	STS		MODE_FLG, MDFLG														; Store new mode flags to SRAM

	LDI		TEMP1, MAPPING_SEEK_PWM												; Start vehicle with mapping seek PWM
	STS		RECENT_DAT, TEMP1													; ^	
	CALL	SET_MOTOR_PWM														; ^

	RET

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

MAPPING_GET:

	// If MAPP > 512 bytes, then buffer will overflow in MatLab
	
	LDI		YH, HIGH(MAPP)
	LDI		YL,  LOW(MAPP)

MAPPING_GET_LOOP:

	LD		TEMP1, Y+

	CPI		TEMP1, 0xFF
	BREQ	MAPPING_GET_ESC

	MOV		TXREG, TEMP1
	CALL	SERIAL_WRITE

	RJMP	MAPPING_GET_LOOP

MAPPING_GET_ESC:

	RET

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

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