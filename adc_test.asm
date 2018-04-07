; Racecar Control Firmware
; Version 1.0.2
;
; RCF.asm
;
; Created: 08-03-2018 21:13:20
; Authors : Martin Androvich, Daniel Tofte & Sina P. Soltani
;

; ________________________________________________________________________________________________
; >> VECTORS:

.ORG	0x00											; Reset Vector
	RJMP	INIT										; ^

.ORG 	0x02											; INT0 Interrupt (PD2) Vector
	RJMP	INT0_ISR									; ^

.ORG	0x014											; TIMER1 COMPARE MATCH INT
	RJMP	INT1_COMPARE_MATCH							; ^

.ORG 0x020
	RJMP	ADC_ISR										;ADC Conv Complete Handler

; ________________________________________________________________________________________________
; >> DEFINITIONS

	.EQU	HERTZTIMER1	= 62500-1						; Settings for Timer
	.EQU	BAUDRATE	= 0xCF							; Baudrate settings for BAUDRATE of 9600

	.EQU	BOOL1		= 0x01							; Boolean #1
	.EQU	BOOL2		= 0x02							; Boolean #1
	.EQU	BOOL3		= 0x04							; Boolean #1

	.DEF	TEMP1		= R16							; Temporary Register #1
	.DEF	TEMP2		= R17							; Temporary Register #2

	.DEF	FLAGS		= R18							; Variable Flags Register [COMMAND RECIEVED | Boolean 6 | ... | Boolean 0]

	.DEF	TELSC		= R19							; Telegram Parser Step Counter

	.DEF	RXREG		= R20							; USART Reception Register
	.DEF	TXREG		= R21							; USART Transmission Register

	.DEF	MTSPD		= R23							; Motor speed (duty cycle)

	.DEF	ADC_L		= R24							;
	.DEF	ADC_H		= R25							;

; ________________________________________________________________________________________________
; >> INITIALIZATION:

.ORG	0x28
INIT:

	; Stack Pointer

	LDI 	TEMP1, HIGH(RAMEND)
	OUT 	SPH, TEMP1
	LDI 	TEMP1, LOW(RAMEND)
	OUT 	SPL, TEMP1

	; ADC Config
	LDI		TEMP1, 0x00
	OUT		DDRA, TEMP1

	LDI		TEMP1, 0																			; Choose -> ADC0 and AVCC. Vcc = 5V
	OUT		ADMUX, TEMP1																		; AUTOTRIGGER ENABLED (ADATE) otherwise it doesnt work?
	LDI		TEMP1, (1<<ADEN)|(1<<ADIE)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADPS2)							;(1<<ADATE)	; ADEN: ENABLE ADC, ADSC: START CONVERSATION ; (1<<ADPS2)
	OUT		ADCSRA, TEMP1																		; ADFR: Activate Free Running Select, Prescalar: 128 // 125kHz ADC clock // *might be divided by 2*

	; USART Config

	LDI		TEMP1, BAUDRATE								; Set Transmission Rate
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

	; TIFR REGISTER

	LDI		TEMP1, (1<<OCIE1A)							;Enable Timer1 Compare Match INterrupt
	OUT		TIMSK, TEMP1								;

	LDI		TEMP1,	00									;Set Default
	OUT		TCCR1A, TEMP1								;

	LDI		TEMP1, (1<<CS11)|(1<<CS10)|(1<<WGM12)		;Set 64 Prescalar, CTC-MODE, 62.500 TIMER, since it is very close
	OUT		TCCR1B, TEMP1								;

	LDI		TEMP1, HIGH(HERTZTIMER1)					;Set this to 62500-1, since 16.000.000 / ( 4 * 64) 4 Hertz
	OUT		OCR1AH, TEMP2								;

	LDI		TEMP1, LOW(HERTZTIMER1)						;Set this to 62500-1, since 16.000.000 / ( 4 * 64) 4 Hertz
	OUT		OCR1AL, TEMP2								;

	LDI		TEMP1, (1<<TOV1)							; Enable Timer1
	OUT		TIFR, TEMP1									;^

	; Waveform Generator (Timer2)

	LDI		TEMP1, 0x00									; Reset Timer2
	OUT		OCR2, TEMP1									; ^

	; !!! Works, but needs further analysis.

	LDI		TEMP1, 0x6A									; Initialize Timer2 with 0110_1010
	OUT		TCCR2, TEMP1								; ^

	SEI													; Set global interrupt flag

	RJMP	MAIN										; Goto MAIN

; ________________________________________________________________________________________________
; >> MAIN PROGRAM:



MAIN:

	CALL	SERIAL_READ

	CPI		RXREG, 0x00									; Set motor if RXREG != 0
	BRNE	SET_MOTOR									; ^

	; !!! Maybe reset telegram step counter + clear buffer after a short delay ?

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
; >> TIMER1 (INTERRUPT VERSION):
INT1_COMPARE_MATCH:

	;LDI TXREG, 'Y'										;If you want to make sure it works!
	;CALL SERIAL_WRITE

	SBI ADCSRA, ADSC									;turn on ADC

	RETI

; ________________________________________________________________________________________________
; >> ADC (INTERRUPT VERSION):
ADC_ISR:

	SBIS	PINA, PA1										;This is the digital value of PINA1
	RJMP	WAIT_UNTIL_ACTUAL								;REPEATUNTILSET

	IN		ADC_L, ADCL										;when ADCL is read, the ADC Data Register is not updated until ADCH is read.
	NOP;
	IN		ADC_H, ADCH										;get the last ADC result, low byte first

	;while (ADCH != 0)

	LDI		TEMP1, 2

REPEAT_ROTATE:

	ROR		ADC_H											;ror takes care of the carry
	ROR		ADC_L
	DEC		TEMP1
	BRNE	REPEAT_ROTATE

SEND256RESO:

	SBIS	UCSRA, UDRE
	RJMP	SEND256RESO
	OUT		UDR, ADC_L

	RETI

WAIT_UNTIL_ACTUAL:
	SBI		ADCSRA, ADSC
	RETI
; ________________________________________________________________________________________________
; >> COMMUNICATION PROTOCOL MODULE (INTERRUPT VERSION):

	// Placeholder
	// ...
; ________________________________________________________________________________________________
; >> CONTROL MODULE:

SET_MOTOR:

	LDI		TEMP1, 0x6A									; Initialize Waveform Generator (Timer2) (0110_1010)
	OUT		TCCR2, TEMP1								; ^

	; !!! Would be good with some error catching of RXREG

	LDI		ZH, HIGH(DUTY_CYCLES*2)						; Initialize Address Pointer
	LDI 	ZL, LOW(DUTY_CYCLES*2)						; ^

	ADD		ZL, RXREG									; Set pointer to RXREG value (with carry)
	CLR		TEMP1										; ^
	ADC		ZH, TEMP1									; ^

	LPM		MTSPD, Z									; Load the matching duty cycle
	OUT		OCR2, MTSPD									; Set Duty Cycle (0-255)

	RJMP	MAIN									; Return

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
