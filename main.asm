; ###################################################################################################################################################
; Racecar Control Firmware
; Version 1.1.1
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
																				;   976 - 1		= 256Hz
																				;   488 - 1		= 512Hz
																				;	244 - 1		= 1024Hz
																				;	122 - 1		= 2048Hz
																				;	 61 - 1		= 4096Hz

	.EQU	AVGSIZE		= 128													; Size (bytes) of Moving Average Filter
	.EQU	AVGDIV		= 7														; 2^5 = 32
	.EQU	MOVAVG_END	= MOVAVG + AVGSIZE										;

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

; ____________________________________________________________________________________________________________________________________________________
; >> INITIALIZATION

INIT:

	; Stack Pointer

	LDI 	TEMP1, HIGH(RAMEND)
	OUT 	SPH, TEMP1
	LDI 	TEMP1, LOW(RAMEND)
	OUT 	SPL, TEMP1

	; SRAM Initialization

	CLR		TEMP1																; Set all allocated SRAM to NULL
	STS		RECENT_DAT, TEMP1													; ^
	STS		TEL_STEP, TEMP1														; ^
	STS		MODE_FLG, TEMP1														; ^
	STS		FUNC_FLG, TEMP1														; ^
	STS		TACHOMETER_H, TEMP1													; ^
	STS		TACHOMETER_L, TEMP1													; ^
	STS		ACCELEROMETER, TEMP1
	STS		ADC_H, TEMP1
	STS		ADC_L, TEMP1

	CALL	SET_POINTERAVG_X
	CALL	SETUP_SRAM

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

	LDI		TEMP1, (1<<ADEN)|(1<<ADIE)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADPS2)			; ADEN: ENABLE ADC, ADSC: START CONVERSATION ; (1<<ADPS2)
	OUT		ADCSRA, TEMP1														; ADFR: Activate Free Running Select, Prescaler: 128 // 125kHz ADC clock

	SBI		ADCSR, ADSC															; Start ADC Conversion	

	; I/O (Port) Setup

	SBI		DDRD, PD7															; Set PD7 on PORTD as Output
	SBI		PORTD, PD2															; Set PD2 on PORTD as Pullup Input

	LDI		TEMP1, 0x00															; Set Port A as Input (is this needed?)
	OUT		DDRA, TEMP1															; ^

	NOP

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

	LDI		TEMP1, (1<<ISC01)|(1<<ISC00)										; Set INT0 to rising edge
	OUT		MCUCR, TEMP1														; ^

	LDI 	TEMP1, (1<<INT0)													; Enable external interrupts
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
	
	SBRC	FNFLG, TMR1															; Timer1 Ready
	CALL	CLOCK																; ^

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
;  > REPEAT LOOP

	;CLR		FNFLG																; Clear Functions Flags

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

	MOV		TEMP1, FNFLG														; Clear Tachometer functional flag
	CBR		TEMP1, (1<<TACHO)													; ^
	MOV		FNFLG, TEMP1														; ^

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > FINISH LINE

LOG_FINISHLINE:

	NOP																			; Do Someting

	MOV		TEMP1, FNFLG														; Clear Finishline functional flag
	CBR		TEMP1, (1<<FNLNE)													; ^
	MOV		FNFLG, TEMP1														; ^

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > ACCELEROMETER

LOG_ACCELEROMETER:

	SBRS	FNFLG, TMR1															; Check if broadcast is synchronized with frequency (Timer1)
	RET																			; ^

	IN		TEMP1, ADCL															; Read LOW of ADC
	NOP																			; ^
	STS		ADC_L, TEMP1														; ^

	IN		TEMP1, ADCH															; Read HIGH of ADC
	NOP																			; ^
	STS		ADC_H, TEMP1														; ^

	CALL	ADC_MOVING

	SBI		ADCSR, ADSC															; Start ADC Conversion

	MOV		TEMP1, FNFLG														; Clear Accelerometer functional flag
	CBR		TEMP1, (1<<ACCLR)													; ^
	MOV		FNFLG, TEMP1														; ^

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> MAPPING

	// PLACEHOLDER
	// ...

; ____________________________________________________________________________________________________________________________________________________
; >> BROADCAST

BROADCAST:
	
	SBRS	FNFLG, TMR1															; Check if broadcast is synchronized with frequency (Timer1)
	RET																			; ^

	MOV		TEMP1, FNFLG														; Clear TIMER1 functional flag
	CBR		TEMP1, TMR1															; ^
	MOV		FNFLG, TEMP1														; ^
	
	LDI		TEMP1, (1<<BROD2)|(1<<BROD1)										; Mask Broadcast Modes
	AND		TEMP1, MDFLG														; ^

	CPI		TEMP1, (1<<BROD1)													; Tachometer Mode (011)
	BREQ	BROADCAST_TACHOMETER												; ^
	
	CPI		TEMP1, (1<<BROD2)|(1<<BROD1)										; Finishline Mode (111)
	BREQ	BROADCAST_FINISHLINE												; ^

	CPI		TEMP1, (1<<BROD2)													; Accelerometer Mode (101)
	BREQ	BROADCAST_ACCELEROMETER												; ^

	RCALL	BROADCAST_TACHOMETER												; Broadcast All
	RCALL	BROADCAST_ACCELEROMETER												; ^
	RCALL	BROADCAST_FINISHLINE												; ^

	RET																			; Return

BROADCAST_TACHOMETER:

	LDS		TXREG, TACHOMETER_H													; Load & transmit HIGH byte of Tachometer data
	CALL	SERIAL_WRITE														; ^

	LDS		TXREG, TACHOMETER_L													; Load & transmit HIGH byte of Tachometer data
	CALL	SERIAL_WRITE														; ^

	RET																			; Return

BROADCAST_ACCELEROMETER:

	;LDS		TXREG, ADC_H														; Load & transmit HIGH byte of ADC (accelerometer) data
	LDS		TXREG, ACCELEROMETER												; Load & transmit Accelerometer data
	CALL	SERIAL_WRITE														; ^

	RET																			; Return

BROADCAST_FINISHLINE:

	// A bit retarded.
	
	LDS		TXREG, FINISHLINE
	CALL	SERIAL_WRITE

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

	RCALL	SET_COMMAND															; Set command pending flag

TELEGRAM_RESET:
	
	CLR		TEMP1																; Reset parse counter
	STS		TEL_STEP, TEMP1

	RET																			; Return

TELEGRAM_CLRBUFFER:

	IN		TEMP1, UDR															; Empty buffer
	SBIC	UCSRA, RXC															; ^
	RJMP	TELEGRAM_CLRBUFFER													; ^

	RET																			; Return

TELEGRAM_ERROR:

	CLR		RXREG																; Clear reception register

	RCALL	CLR_COMMAND															; Clear command pending flag
	RCALL	TELEGRAM_RESET
	RCALL	TELEGRAM_CLRBUFFER													; Clear reception buffer
	
	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > COMMANDS

SET_COMMAND:

	MOV		TEMP1, FNFLG														; Load current Flags Register and set CMDPD bit
	SBR		TEMP1, (1<<CMDPD)													; ^
	MOV		FNFLG, TEMP1														; ^
	
	RET

CLR_COMMAND:
	
	MOV		TEMP1, FNFLG														; Load current Flags Register and clear CMDPD bit
	CBR		TEMP1, (1<<CMDPD)													; ^
	MOV		FNFLG, TEMP1														; ^
	
	RET

EXECUTE_COMMAND:
	
	RCALL	CLR_COMMAND															; Clear command pending flag

	ICALL																		; Call function of Z-pointer

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> DEVICE CONTROL

EMPTY:
	NOP
	RET

CLOCK:
	NOP

	MOV		TEMP1, FNFLG														; Clear Timer1 functional flag
	CBR		TEMP1, TMR1															; ^
	MOV		FNFLG, TEMP1														; ^

	RET

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > PURE TESTING

TEST:

	LDI		TEMP1, 60
	STS		RECENT_DAT, TEMP1

	CALL	SET_MOTOR_PWM
	CALL	DELAY
	CALL	SET_MOTOR_MIN
	CALL	DELAY

	RET

DELAY:
    LDI		R27, 100
LOOP3:
	LDI		R26, 100
LOOP2:
	LDI		R25, 100
LOOP1:
    DEC		R25
    BRNE	LOOP1
    DEC		R26
    BRNE	LOOP2
    DEC		R27
    BRNE	LOOP3

    RET

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > MOTOR CONTROL

SET_MOTOR_PWM:

	LDI		TEMP1, 0x6A															; Initialize Waveform Generator (Timer2) (0110_1010)
	OUT		TCCR2, TEMP1														; ^

	LDS		TEMP1, RECENT_DAT													; Load recent recieved telegram data from SRAM
	STS		DUTY_CYCLE, TEMP1													; Store loaded Duty Cycle in SRAM

	TST		TEMP1																; Check if PWM is Zero
	BREQ	SET_MOTOR_MIN														; Stop vehicle if true (or BRAKE)

	OUT		OCR2, TEMP1															; Set Duty Cycle (0-255) on Timer2

SET_MOTOR_PWM_ESC:

	RET																			; Return

SET_MOTOR_MIN:

	LDI		TEMP1, 0x00															; Disable Timer2
	OUT		TCCR2, TEMP1														; ^

	CBI 	PORTD, PD7															; Clear BIT on PIN7 of PORTD
	RJMP	SET_MOTOR_PWM_ESC													; Return

	RJMP	SET_MOTOR_PWM_ESC													; Return

SET_MOTOR_MAX:

	SBI 	PORTD, PD7															; Set BIT on PIN7 of PORTD
	RJMP	SET_MOTOR_PWM_ESC													; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > MOVING AVERAGE

ADC_MOVING:

	LDS		XH, RECENT_XAVG_H							;
	LDS		XL, RECENT_XAVG_L							;

	
	LDS		TEMP1, ADC_H
	ST		X+, TEMP1									;insert ADC_val from high because of ADLAR

	CPI		XL, AVGSIZE									;check for reset
	BRNE	SKIP_THIS_RESET								;

	RCALL	SET_POINTERAVG_X							;	

SKIP_THIS_RESET:										;

	STS		RECENT_XAVG_L, XL							;

	RCALL	ADD_LOOP									;add loop
	RCALL	DIV_LOOP									;

	RET

SET_POINTERAVG_X:


	LDI		XH, HIGH(MOVAVG)							;
	LDI		XL, LOW(MOVAVG)								;

	RET

SETUP_SRAM:												;
	LDI TEMP1, AVGSIZE									;

	STS RECENT_XAVG_H, XH
	STS RECENT_XAVG_L, XL

CONTINUE_NULL:											;
	ST	X+, TEMP1 										;
	CPI XL, LOW(MOVAVG_END)								;
	BRNE CONTINUE_NULL									;
	RET													;

ADD_LOOP:
	CLR		TEMP1										;
	CLR		TEMP2										;
	CLR		TEMP3										;
	RCALL	SET_POINTERAVG_X

ADDER_LOOP:												;
	LD		TEMP1, X+ 									;load from pointer
	ADD		TEMP2, TEMP1								;

	BRCC	SKIP_INCREMENT_H							;branch if carry is not set %% can change to SBIC, SREG <- 
	INC		TEMP3										;

SKIP_INCREMENT_H:									;

	CPI		XL, LOW(MOVAVG_END)							;remember to compare ZH aswell.
	BRNE	ADDER_LOOP									;

	RET													;

DIV_LOOP:
	LDI		TEMP1, AVGDIV								;

DIVIDE_LOOP_CONTINUE:								;

	ASR		TEMP3										;TEMP3 IS HIGH
	ROR		TEMP2										;TEMP2 IS LOW

	DEC		TEMP1										;
	BRNE	DIVIDE_LOOP_CONTINUE						;

	STS		ACCELEROMETER, TEMP2						;save val

	RET													;

; ____________________________________________________________________________________________________________________________________________________
; >> FLAGS MANAGEMENT

LOAD_FLAGS:
	
	CLI																			; Disable Interrupts

	LDS		TEMP1, FUNC_FLG														; Load Function Flags from SRAM into Register
	OR		FNFLG, TEMP1

	CLR		TEMP1
	STS		FUNC_FLG, TEMP1														; ^

	SEI																			; Enable Interrupts

	RET																			; Return

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

	RETI