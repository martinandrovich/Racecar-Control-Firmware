; Racecar Control Firmware
; Version 1.0.2
;
; RCF.asm
;
; Created: 08-03-2018 21:13:20
; Author : Martin Androvich, Daniel Tofte & Sina P. Soltani
;

; ________________________________________________________________________________________________
; >> VECTORS:

.ORG	0x00											; Reset Vector
	RJMP	INIT										; ^

.ORG 	0x02											; INT0 Interrupt (PD2) Vector
	JMP		INT0_ISR									; ^

; ________________________________________________________________________________________________
; >> DEFINITIONS

	.EQU	BAUDRATE	= 0xCF							; Baudrate settings for BAUDRATE of 9600

	.EQU	BOOL1		= 0x01							; Boolean #1
	.EQU	BOOL2		= 0x02							; Boolean #1
	.EQU	BOOL3		= 0x04							; Boolean #1

	.DEF	TEMP1		= R16							; Temporary Register #1
	.DEF	TEMP2		= R17							; Temporary Register #2
	.DEF	BOOLS		= R18							; Variable Booleans Register

	.DEF	RXREG		= R20							; USART Reception Register
	.DEF	TXREG		= R21							; USART Transmission Register
	.DEF	MTSPD		= R22							; Motor speed (duty cycle)
	.DEF	MTCNT		= R23							; Motor rotation counter

; ________________________________________________________________________________________________
; >> INITIALIZATION:

.ORG	0x28
INIT:

	; Stack Pointer

	LDI 	TEMP1, HIGH(RAMEND)
	OUT 	SPH, TEMP1
	LDI 	TEMP1, LOW(RAMEND)
	OUT 	SPL, TEMP1

	; USART Config

	LDI		TEMP1, 0xCF									; Set Transmission Rate
	OUT		UBRRL, TEMP1								; ^
	LDI		TEMP1, 0x00									; ^
	OUT		UBRRH, TEMP1								; ^

	LDI		TEMP1, 0x02									; Clear all Error Flags + Enable DoubleMode
	OUT		UCSRA, TEMP1								; ^

	LDI		TEMP1, (1<<RXEN) | (1<<TXEN)				; Enable Transmission & Reception
	OUT		UCSRB, TEMP1								; ^

	LDI		TEMP1, (1<<URSEL) | (3<<UCSZ0)				; Set Frame Format (8, N, 1)
	OUT		UCSRC, TEMP1								; ^

	LDI		RXREG, 0x00									; Reset Reception Register
	LDI		TXREG, 0x00									; Reset Transmission Register

	; Port setup

	SBI		DDRD, PD7									; Set PIN7 on PORTD as Output
	SBI		PORTD, PD2									; Set PIN2 on PORTD as Pullup Input

	; Interrupt setup

	LDI		TEMP1, (1<<ISC01) | (1<<ISC00)				; Set INT0 to rising edge
	OUT		MCUCR, TEMP1								; ^

	LDI 	TEMP1, (1<<INT0)							; Enable external interrupts
	OUT 	GICR, TEMP1									; ^

	SEI													; Set global interrupt flag

	; Waveform Generator (Timer2)

	LDI		TEMP1, 0x00									; Reset Timer2
	OUT		OCR2, TEMP1									; ^

	; !!! Works, but needs further analysis.

	LDI		TEMP1, 0x6A									; Initialize Timer2 with 0110_1010
	OUT		TCCR2, TEMP1								; ^

	RJMP	MAIN										; Goto MAIN

; ________________________________________________________________________________________________
; >> MAIN PROGRAM:

MAIN:

	RCALL	SERIAL_READ									; Begin reading

	CPI		RXREG, 0x00									; Enable motor if RXREG != 0
	BRNE	ENABLE_MOTOR								; ^

	RJMP	MAIN										; Loop forever


; ________________________________________________________________________________________________
; >> INTERRUPTS:

INT0_ISR:
	LDI		TXREG, 0x35									; Load 0x35 into transmission register
	RCALL	SERIAL_WRITE								; Write to USART

	RETI												; Return


/*TURN_ISR:

	CALL	 SERIAL_WRITE

	RETI


GOALDETECT_ISR:

	LDI		TXREG, 0x35									; Load 0x35 into transmission register
	RCALL	SERIAL_WRITE								; Write

	RETI*/



; ________________________________________________________________________________________________
; >> SERIAL COMMUNICATION MODULE:

SERIAL_READ:
	SBIS	UCSRA, RXC									; Wait for Recieve (RXC) flag
	RJMP	SERIAL_READ									; ^

	IN		RXREG, UDR									; Load data from serial to register

	RET													; Return


SERIAL_WRITE:
	SBIS	UCSRA, UDRE									; Wait for Empty Transmit Buffer (UDRE) flag
	RJMP	SERIAL_WRITE								; ^

	OUT		UDR, TXREG									; Load transmission data from register to serial

	RET													; Return


; ________________________________________________________________________________________________
; >> COMMUNICATION PROTOCOL MODULE:

PARSE_TELEGRAM:

	CPI		RXREG, 0x00									; Enable motor if RXREG != 0
	BRNE	ENABLE_MOTOR								; ^

	RET													; Return

PARSE_TELEGRAM_TYPE:
	NOP													; No code yet
	RET													; Return

PARSE_TELEGRAM_COMMAND:
	NOP													; No code yet
	RET													; Return

PARSE_TELEGRAM_DATA:
	NOP													; No code yet
	RET													; Return


; ________________________________________________________________________________________________
; >> MOTOR CONTROL MODULE:

ENABLE_MOTOR_MAX:
	SBI 	PORTD, PD7									; Enable BIT on PIN7 of PORTD
	RJMP	MAIN										; Return


ENABLE_MOTOR:
	LDI		TEMP1, 0x6A									; Initialize Waveform Generator (Timer2) (0110_1010)
	OUT		TCCR2, TEMP1								; ^

	; !!! Would be good with some error catching of RXREG

	LDI		ZH, HIGH(DUTY_CYCLES*2)						; Initialize Address Pointer
	LDI 	ZL, LOW(DUTY_CYCLES*2)						; ^

	ADD		ZL, RXREG									; Set pointer to RXREG value
	LPM		MTSPD, Z									; Load the matching duty cycle

	OUT		OCR2, MTSPD									; Set Duty Cycle (0-255)

	RJMP	MAIN										; Return


; ________________________________________________________________________________________________
; >> SPEED VALUES TABLE:

	; !!! Should be moved to EEPROM

DUTY_CYCLES:
	.DB		0, 1    	; 0% and 1%
	.DB		4, 6    	; 2% and 3%
	.DB		9, 11       ; 4% and 5%
	.DB		14, 16      ; 6% and 7%
	.DB		19, 22      ; 8% and 9%
	.DB		24, 27      ; 10% and 11%
	.DB		29, 32      ; 12% and 13%
	.DB		34, 37      ; 14% and 15%
	.DB		39, 42      ; 16% and 17%
	.DB		45, 47      ; 18% and 19%
	.DB		50, 52      ; 20% and 21%
	.DB		55, 57      ; 22% and 23%
	.DB		60, 63      ; 24% and 25%
	.DB		65, 68      ; 26% and 27%
	.DB		70, 73      ; 28% and 29%
	.DB		75, 78      ; 30% and 31%
	.DB		80, 83      ; 32% and 33%
	.DB		86, 88      ; 34% and 35%
	.DB		91, 93      ; 36% and 37%
	.DB		96, 98    	; 38% and 39%
	.DB		101, 103    ; 40% and 41%
	.DB		106, 109    ; 42% and 43%
	.DB		111, 114    ; 44% and 45%
	.DB		116, 119    ; 46% and 47%
	.DB		121, 124    ; 48% and 49%
	.DB		127, 129    ; 50% and 51%
	.DB		132, 134    ; 52% and 53%
	.DB		137, 139    ; 54% and 55%
	.DB		142, 144    ; 56% and 57%
	.DB		147, 150    ; 58% and 59%
	.DB		152, 155    ; 60% and 61%
	.DB		157, 160    ; 62% and 63%
	.DB		162, 165    ; 64% and 65%
	.DB		167, 170    ; 66% and 67%
	.DB		173, 175    ; 68% and 69%
	.DB		178, 180    ; 70% and 71%
	.DB		183, 185    ; 72% and 73%
	.DB		188, 191    ; 74% and 75%
	.DB		193, 196    ; 76% and 77%
	.DB		198, 201    ; 78% and 79%
	.DB		203, 206    ; 80% and 81%
	.DB		208, 211    ; 82% and 83%
	.DB		214, 216    ; 84% and 85%
	.DB		219, 221    ; 86% and 87%
	.DB		224, 226    ; 88% and 89%
	.DB		229, 231    ; 90% and 91%
	.DB		234, 237    ; 92% and 93%
	.DB		239, 242    ; 94% and 95%
	.DB		244, 247    ; 96% and 97%
	.DB		249, 252    ; 98% and 99%
	.DB		255, 0		; 100%
