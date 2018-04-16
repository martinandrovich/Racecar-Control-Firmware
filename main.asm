; ###################################################################################################################################################
; Racecar Control Firmware
; Version 1.1.0
; 
; Sequential Flag Architecture

; ___________________________________________________________________________________________________________________________________________________
; >> VECTORS:

.ORG	0x00																	; Reset Vector
	JMP		INIT																; ^

.ORG 	0x02																	; INT0 Interrupt (PD2)
	JMP		INT0_ISR															; ^

.ORG	0x04																	; INT1 Interrupt (PD3)
	JMP 	INT1_ISR															; ^

.ORG	0x06																	; INT2 Interrupt (PB2)
	JMP		INT2_ISR															; ^

.ORG	0x14																	; TIMER1 Compare Match Interrupt
	JMP		TMR1_ISR															; ^

.ORG	0x20																	; ADC Conversion Complete Interrupt
	JMP		ADC_ISR																; ^

; ____________________________________________________________________________________________________________________________________________________
; >> DEFINITIONS

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > CONSTANTS

	.EQU	BAUDRATE	= 0x00CF												; Baudrate settings

	.EQU	TMR1FREQ	= 1953 - 1												; Settings for Timer1

																				; 62500 - 1		= 4Hz
																				; 31250 - 1		= 8Hz
																				; 15625 - 1		= 16Hz
																				;  7812 - 1		= 32Hz
																				;  1953 - 1		= 128Hz
																				;   976 - 1		= 256Hz

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > REGISTERS

	.DEF	MDFLG		= R0													; Mode Flags for Interrupts
	.DEF	FNFLG		= R1													; Function Flags for Interrupts
	
	.DEF	TEMP1		= R16													; Temporary Register #1
	.DEF	TEMP2		= R17													; Temporary Register #2
	.DEF	TEMPI		= R18													; Temporary Interrupts Register

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

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > INCLUDES

.ORG	0x28

	.INCLUDE	"ram_table.inc"
	.INCLUDE	"command_table.inc"

; ____________________________________________________________________________________________________________________________________________________
; >> INITIALIZATION

INIT:

	; Stack Pointer

	LDI 	TEMP1, HIGH(RAMEND)
	OUT 	SPH, TEMP1
	LDI 	TEMP1, LOW(RAMEND)
	OUT 	SPL, TEMP1

	; SRAM Initialization

	CLR		TEMP1
	STS		RECENT_DAT, TEMP1
	STS		MODE_FLG, TEMP1
	STS		FUNC_FLG, TEMP1
	STS		TEL_STEP, TEMP1

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

	; !!! Needs revision.

	LDI		TEMP1, (1<<ADLAR)													; Choose -> ADC0 and AVCC. Vcc = 5V
	OUT		ADMUX, TEMP1														; AUTOTRIGGER ENABLED (ADATE) otherwise it doesnt work?

	LDI		TEMP1, (1<<ADEN)|(1<<ADIE)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADPS2)			; ADEN: ENABLE ADC, ADSC: START CONVERSATION ; (1<<ADPS2)
	OUT		ADCSRA, TEMP1														; ADFR: Activate Free Running Select, Prescaler: 128 // 125kHz ADC clock 

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
;  > MODES:

	SBRC	MDFLG, AUTO															; Autonomous Mode
	NOP																			; ^

	SBRC	MDFLG, MAP															; Mapping Mode
	NOP																			; ^

	SBRC	MDFLG, BROD0														; Broadcast Mode
	NOP																			; ^

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > FUNCTIONS

	SBRC	FNFLG, TACHO														; Tachometer Ready
	NOP																			; ^

	SBRC	FNFLG, FNLNE														; Finishline Ready
	NOP																			; ^

	SBRC	FNFLG, ACCLR														; Accelerometer Ready
	NOP																			; ^
	
	SBRC	FNFLG, TMR1															; Timer1 Ready
	NOP																			; ^

	CALL	TELEGRAM_CHECK														; Check for Telegrams

	SBRC	FNFLG, CMDPD														; Command Pending
	CALL	EXECUTE_COMMAND														; ^

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > REPEAT LOOP
	
	CLR		MDFLG																; Clear Flags
	CLR		FNFLG																; ^

	RJMP	MAIN																; Loop forever

; ____________________________________________________________________________________________________________________________________________________
; >> SENSOR PROCESSING & LOGGING

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > GENERAL (ALL)

LOG:

	NOP																			; Do Someting

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > TACHOMETER

LOG_TACHOMETER:

	NOP																			; Do Someting

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > FINISH LINE

LOG_FINISHLINE:

	NOP																			; Do Someting

	RET																			; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > ACCELEROMETER

LOG_ACCELEROMETER:

	NOP																			; Do Someting

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> MAPPING

	// PLACEHOLDER
	// ...

; ____________________________________________________________________________________________________________________________________________________
; >> BROADCAST

	// PLACEHOLDER
	// ...

	// Check if Timer1 ready before broadcasting?

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

; ____________________________________________________________________________________________________________________________________________________
; >> FLAGS MANAGEMENT

LOAD_FLAGS:
	
	CLI																			; Disable Interrupts
	
	LDS		TEMPI, MODE_FLG														; Load Mode Flags from SRAM into Register
	MOV		MDFLG, TEMPI														; ^

	LDS		TEMPI, FUNC_FLG														; Load Function Flags from SRAM into Register
	MOV		FNFLG, TEMPI														; ^

	CLR		TEMPI																; Clear stored flags
	STS		MODE_FLG, TEMPI														; ^
	STS		FUNC_FLG, TEMPI														; ^

	SEI																			; Enable Interrupts

	RET																			; Return

; ____________________________________________________________________________________________________________________________________________________
; >> INTERRUPT SERVICE ROUTINES

INT0_ISR:
	
	LDS		TEMPI, FUNC_FLG														; Load Function Flags from SRAM into Temporary Interrupt Register
	SET																			; Set T flag
	BLD		TEMPI, TACHO														; Set BIT in Temporary Interrupt Register
	STS		FUNC_FLG, TEMPI														; Store Function Flags to SRAM

	RETI																		; Return

INT1_ISR:
	
	NOP

	RETI																		; Return

INT2_ISR:
	
	NOP

	RETI																		; Return

TMR1_ISR:
	
	NOP

	RETI																		; Return

ADC_ISR:
	
	NOP

	RETI																		; Return