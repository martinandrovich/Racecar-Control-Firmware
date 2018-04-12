; Racecar Control Firmware
; Version 1.0.2

; ________________________________________________________________________________________________
; >> VECTORS:

.ORG	0x00											; Reset Vector
	JMP		INIT										; ^

.ORG 	0x02											; INT0 Interrupt (PD2)
	JMP		INT0_ISR									; ^

.ORG	0x04											; INT1 Interrupt (PD3)
	JMP 	INT1_ISR									; ^

.ORG	0x06											; INT2 Interrupt (PB2)
	JMP		INT2_ISR									; ^

.ORG	0x14											; TIMER1 Compare Match Interrupt
	JMP		TMR1_ISR									; ^

.ORG	0x20											; ADC Conversion Complete Interrupt
	JMP		ADC_ISR										; ^


; ________________________________________________________________________________________________
; >> DEFINITIONS

	.EQU	BAUDRATE	= 0x00CF						; Baudrate settings for BAUDRATE of 9600

	.EQU	TMR1FREQ	= 15625 - 1						; Settings for Timer1

														; 62500 - 1 = 4Hz
														; 31250 - 1 = 8Hz
														; 15625 - 1 = 16Hz
														; 7812 - 1	= 32Hz
														; 1953 - 1  = 128Hz
														; 976 - 1   = 256Hz

	.DEF	TEMP1		= R16							; Temporary Register #1
	.DEF	TEMP2		= R17							; Temporary Register #2

	.DEF	FLAGS		= R18							; Variable Flags Register
														; [CMDPD = 7] | [AUTMD = 6] | [DATLG = 5] | X (4) | X (3) | X (2) | X (1) | X (0)]
	
	.EQU	DATLG		= 5								; Datalogging Mode
	.EQU	AUTMD		= 6								; Autonomous Mode
	.EQU	CMDPD		= 7								; Command Pending Flag
	
	
	.DEF	TELSC		= R19							; Telegram Parser Step Counter

	.DEF	RXREG		= R20							; USART Reception Register
	.DEF	TXREG		= R21							; USART Transmission Register

	.DEF	ADC_L		= R24							; ADC Registers
	.DEF	ADC_H		= R25							; ^

; ________________________________________________________________________________________________
; >> RAM ALLOCATION

	.EQU	TCLKH		= 0x0300						; Data memory location for Tachometer Clicks
	.EQU	TCLKL		= 0x0301						; ^

; ________________________________________________________________________________________________
; >> INITIALIZATION:

.ORG	0x28
INIT:

	; Stack Pointer

	LDI 	TEMP1, HIGH(RAMEND)
	OUT 	SPH, TEMP1
	LDI 	TEMP1, LOW(RAMEND)
	OUT 	SPL, TEMP1

	; RAM Initialization

	CLR		TEMP1										; Clear Tachometer Data
	STS		TCLKH, TEMP1								; ^
	STS		TCLKL, TEMP1								; ^

	; USART Config

	LDI		TEMP1, HIGH(BAUDRATE)						; Set Transmission Rate
	OUT		UBRRH, TEMP1								; ^
	LDI		TEMP1, LOW(BAUDRATE)						; ^
	OUT		UBRRL, TEMP1								; ^

	LDI		TEMP1, 0x02									; Clear all Error Flags + Enable DoubleMode
	OUT		UCSRA, TEMP1								; ^

	LDI		TEMP1, (1<<RXEN)|(1<<TXEN)					; Enable Transmission & Reception
	OUT		UCSRB, TEMP1								; ^

	LDI		TEMP1, (1<<URSEL)|(3<<UCSZ0)				; Set Frame Format (8, N, 1)
	OUT		UCSRC, TEMP1								; ^

	CLR		RXREG										; Reset Reception Register
	CLR		TXREG										; Reset Transmission Register

	; ADC Config	

	; !!! Needs revision.

	LDI		TEMP1, 0x00									; Choose -> ADC0 and AVCC. Vcc = 5V
	OUT		ADMUX, TEMP1								; AUTOTRIGGER ENABLED (ADATE) otherwise it doesnt work?

	LDI		TEMP1, (1<<ADEN)|(1<<ADIE)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADPS2)		; ADEN: ENABLE ADC, ADSC: START CONVERSATION ; (1<<ADPS2)
	OUT		ADCSRA, TEMP1								; ADFR: Activate Free Running Select, Prescalar: 128 // 125kHz ADC clock 

	; Port Setup

	SBI		DDRD, PD7									; Set PIN7 on PORTD as Output
	SBI		PORTD, PD2									; Set PIN2 on PORTD as Pullup Input

	LDI		TEMP1, 0x00									; Set Port A as Input (is this needed?)
	OUT		DDRA, TEMP1									; ^

	; Timer1 Setup

	LDI		TEMP1, (1<<OCIE1A)							; Enable Timer1 Compare Match INterrupt
	OUT		TIMSK, TEMP1								; ^

	LDI		TEMP1, 0x00									; Set Default
	OUT		TCCR1A, TEMP1								; ^

	LDI		TEMP1, (1<<CS11)|(1<<CS10)|(1<<WGM12)		; Set 64 Prescelar, CTC-MODE
	OUT		TCCR1B, TEMP1								; ^

	LDI		TEMP1, HIGH(TMR1FREQ)						; Set timer offset
	OUT		OCR1AH, TEMP1								; ^
	LDI		TEMP1, LOW(TMR1FREQ)						; ^
	OUT		OCR1AL, TEMP1								; ^

	LDI		TEMP1, (1<<TOV1)							; Enable Timer1
	OUT		TIFR, TEMP1									; ^
	
	; Waveform Generator (Timer2)

	LDI		TEMP1, 0x00									; Reset Timer2
	OUT		OCR2, TEMP1									; ^

	; !!! Works, but needs further analysis & revision.

	LDI		TEMP1, 0x6A									; Initialize Timer2 with 0110_1010
	OUT		TCCR2, TEMP1								; ^

	; Interrupt Setup

	LDI		TEMP1, (1<<ISC01)|(1<<ISC00)				; Set INT0 to rising edge
	OUT		MCUCR, TEMP1								; ^

	LDI 	TEMP1, (1<<INT0)							; Enable external interrupts
	OUT 	GICR, TEMP1									; ^

	SEI													; Set global interrupt flag

	RJMP	MAIN										; Goto MAIN


; ________________________________________________________________________________________________
; >> MAIN PROGRAM:

MAIN:

	; !!! Maybe include authentication for security measures?

	RCALL	SERIAL_READ									; Begin reading

	CPI		RXREG, 0x00									; Set motor if RXREG != 0
	BRNE	SET_MOTOR									; ^

	; !!! Maybe reset telegram step counter + clear buffer after a short delay ?

	RJMP	MAIN										; Loop forever


; ________________________________________________________________________________________________
; >> INTERRUPTS:

INT0_ISR:												; INT0 Interrupt Handler

	; !!! CONFLICTS WITH ADC DATA
	; !!! X POINTER & "LD" SHOULD BE USED INSTEAD

	LDS		R25, TCLKH									; Load previous values from RAM
	LDS		R24, TCLKL									; ^

	ADIW	R25:R24, 1									; Increment data

	STS		TCLKH, R25									; Store values into RAM
	STS		TCLKL, R24									; ^

	CPI		R24, 100
	BRNE	INT0_ISR_ESC

	LDI		TXREG, 0x30									; Load 0x30 into transmission register
	RCALL	SERIAL_WRITE								; Write to USART

INT0_ISR_ESC:

	RETI												; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

INT1_ISR:

	LDI		TXREG, 0x31									; Load 0x31 into transmission register
	RCALL	SERIAL_WRITE								; Write to USART

	RETI												; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

INT2_ISR:

	LDI		TXREG, 0x32									; Load 0x32 into transmission register
	RCALL	SERIAL_WRITE								; Write to USART

	RETI												; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

TMR1_ISR:												; TIMER1 Interrupt Handler

	SBI		ADCSRA, ADSC								; Turn on ADC

	RETI												; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

ADC_ISR:												; ADC Interrupt Handler

	; !!! Better to load into RAM.

	IN		ADC_L, ADCL									; Load ADC data into designated  registers
	NOP													; ^
	IN		ADC_H, ADCH									; ^

	LDI		TEMP1, 2

ADC_ISR_ROR:

	ROR		ADC_H										; ROR takes care of the carry
	ROR		ADC_L										; ^
	DEC		TEMP1										; ^
	BRNE	ADC_ISR_ROR									; ^

	MOV		TXREG, ADC_L								; Transmit ADC data
	RCALL	SERIAL_WRITE								; ^

	RETI												; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _


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
; >> COMMUNICATION PROTOCOL MODULE (INTERRUPT VERSION):

	// Placeholder
	// ...

; ________________________________________________________________________________________________
; >> CONTROL MODULE:

SET_MOTOR_MAX:
	SBI 	PORTD, PD7									; Enable BIT on PIN7 of PORTD
	RJMP	MAIN										; Return

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

SET_MOTOR:
	LDI		TEMP1, 0x6A									; Initialize Waveform Generator (Timer2) (0110_1010)
	OUT		TCCR2, TEMP1								; ^

	; !!! Would be good with some error catching of RXREG (Bounds Checking)
	; i.e. if >100, then abort

	; !!! Maybe disable interrupts while doing this?

	; !!! Disable if 0.

	LDI		ZH, HIGH(DUTY_CYCLES*2)						; Initialize Address Pointer
	LDI 	ZL, LOW(DUTY_CYCLES*2)						; ^

	ADD		ZL, RXREG									; Set pointer to RXREG value (with carry)
	CLR		TEMP1										; ^
	ADC		ZH, TEMP1									; ^

	LPM		TEMP1, Z									; Load the matching duty cycle

	OUT		OCR2, TEMP1									; Set Duty Cycle (0-255)

	RJMP	MAIN										; Return

; ________________________________________________________________________________________________
; >> SPEED VALUES TABLE:

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
