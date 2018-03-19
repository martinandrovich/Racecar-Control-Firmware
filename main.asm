; Racecar Control Firmware
; Version 1.0.0
;
; RCF.asm
;
; Created: 08-03-2018 21:13:20
; Author : Martin Androvich
;

; ________________________________________________
; >> RESET VECTOR:

.ORG	0x00
	RJMP	INIT

; ________________________________________________
; >> DEFINITIONS

.ORG	0x60
	;.EQU	BAUD = 9600
	.EQU	BAUDRATE = 0xCF

	.DEF	RXREG = R20
	.DEF	TXREG = R21
	.DEF	MTSPD = R22

; ________________________________________________
; >> INITIALIZATION:

INIT:

	; Stack Pointer

	LDI 	R16, HIGH(RAMEND) 	
	OUT 	SPH, R16 			
	LDI 	R16, LOW(RAMEND)		
	OUT 	SPL, R16 
	
	; USART Config

	LDI		R16, BAUDRATE					; Set Transmission Rate
	OUT		UBRRL, R16						; ^
	LDI		R16, 0x00						; ^
	OUT		UBRRH, R16						; ^

	LDI		R16, 0x02						; Clear all Error Flags + Enable DoubleMode
	OUT		UCSRA, R16						; ^

	LDI		R16, (1<<RXEN) | (1<<TXEN)		; Enable Transmission & Reception
	OUT		UCSRB, R16						; ^

	LDI		R16, (1<<URSEL) | (3<<UCSZ0)	; Set Frame Format (8, N, 1)
	OUT		UCSRC, R16						; ^	

	; Init Port D

	LDI		R16, 0xFF						; Set PORTD as Output
	SBI		DDRD, PD7						; ^

	LDI		RXREG, 0x00						; Reset Reception Register
	LDI		TXREG, 0x00						; Reset Transmission Register

	; Waveform Generator (Timer2)

	LDI		R16, 0x00						; Reset Timer2
	OUT		OCR2, R16						; ^

	; !!! Works, but needs further analysis.

	(1<<URSEL) | (3<<UCSZ0)
	
	LDI		R16, 0x6A						; Initialize Timer2 with 0110_1010
	OUT		TCCR2, R16						; ^

	RJMP	MAIN							; Goto MAIN

; ________________________________________________
; >> MAIN PROGRAM:

MAIN:

	RCALL	SERIAL_READ						; Begin reading

	CPI		RXREG, 0x00						; Enable motor if command recieved
	BRNE	ENABLE_MOTOR					; ^
	
	;RCALL	SERIAL_WRITE

	RJMP	MAIN

SERIAL_READ:
	SBIS	UCSRA, RXC						; Wait for Recieve (RXC) flag
	RJMP	SERIAL_READ						; ^

	IN		RXREG, UDR						; Load data from serial to register

	RET										; Return

SERIAL_WRITE:
	SBIS	UCSRA, UDRE						; Wait for Empty Transmit Buffer (UDRE) flag
	RJMP	SERIAL_WRITE					; ^
	
	LDI		R16, 0x35
	OUT		UDR, R16						; Load data from register to serial

	RET										; Return

ENABLE_MOTOR_MAX:
	SBI 	PORTD, PD7						; Enable BIT on PIN7 of PORTD
	RJMP	MAIN							; Return

ENABLE_MOTOR:
	LDI		R16, 0x6A						; Initialize Waveform Generator (Timer2) (0110_1010)
	OUT		TCCR2, r16						; ^

	; !!! Would be good with some error catching of RXREG

	LDI		ZH, HIGH(DUTY_CYCLES*2)			; Initialize Address Pointer
	LDI 	ZL, LOW(DUTY_CYCLES*2)			; ^

	ADD		ZL, RXREG						; Set pointer to RXREG value
	LPM		MTSPD, Z						; Load the matching duty cycle

	OUT		OCR2, MTSPD						; Set Duty Cycle (0-255)

	RJMP	MAIN							; Return
	

; ________________________________________________
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
