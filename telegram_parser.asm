.ORG 0x00
	RJMP INIT
	
	; DEFINITIONS:
	
	.DEF	TEMP1		= R16							; Temporary Register #1
	.DEF	TEMP2		= R17							; Temporary Register #2

	.DEF	FLAGS		= R18							; Variable Flags Register
														; [COMMAND PENDING (CMDPD) (7) | X (6) | X (5) | X (4) | X (3) | X (2) | X (1) | X (0)]
	
	.EQU	CMDPD		= 7								; Command Pending Flag

	.DEF	TELSC		= R19							; Telegram Parser Step Counter
	.DEF	RECMD		= R24							; Telegam Recent COMMAND

	; !!! Bad use of registers (DATAH:DATAL) -> needs better solution!

	.DEF	DATAH		= R25							; Telegram DATA HIGH
	.DEF	DATAL		= R26							; Telegram DATA LOW

	.DEF	RXREG		= R20							; USART Reception Register
	.DEF	TXREG		= R21							; USART Transmission Register

	.DEF	SWREG		= R23							; Switch Register

.ORG URXCaddr
	RJMP TELEGRAM_ISR

.ORG 40
INIT:
	; Stack Pointer

	LDI 	R16, HIGH(RAMEND)
	OUT 	SPH, R16
	LDI 	R16, LOW(RAMEND)
	OUT 	SPL, R16

	; USART Config

	LDI		R16, 0x0C									; Set Transmission Rate
	OUT		UBRRL, R16									; ^
	LDI		R16, 0x00									; ^
	OUT		UBRRH, R16									; ^

	LDI		R16, 0x02									; Clear all Error Flags + Enable DoubleMode
	OUT		UCSRA, R16									; ^

	LDI		R16, (1<<URSEL) | (3<<UCSZ0)				; Set Frame Format (8, N, 1)
	OUT		UCSRC, R16									; ^

	; Register Config
	
	CLR		RXREG										; Clear registers
	CLR		TXREG										; ^
	CLR		TELSC										; ^
	CLR		RECMD										; ^
	CLR		DATAH										; ^
	CLR		DATAL										; ^

	; Port Config

	LDI		R16, 0xFF									; Load HIGH into R16
	OUT		DDRB, R16									; Set PortB as Output
	LDI		R16, 0xFF									; Turn off Display
	OUT		PORTB, R16									; ^

	LDI		R16, 0x00									; Load LOW into R16
	OUT		DDRC, R16									; Set PortC as Input
	LDI		R16, 0xFF									; Load HIGH into R16
	OUT		PORTC, R16									; Use PortC with Intern Pull-Up Resistor

	; Interrupts

	LDI		R16, (1<<RXEN) | (1<<RXCIE)	| (1<<TXEN)		; Enable Interrupts Reception
	OUT		UCSRB, R16									; ^

	SEI													; Enable interrupts
	
	RCALL	LED_OFF										; Turn off LED

MAIN:
	
	;RCALL	TELEGRAM_ISR
	
	SBRC	FLAGS, CMDPD								; Execute any pending commands
	RCALL	EXECUTE_COMMAND								; ^

	CPI		TELSC, 0									; Halt if Telegram parsing (interrupt) in progress.
	BRNE	MAIN										; ^

	; !!! SWITCH_READ CAUSES BUGS
	; Potentional fix -> disable RX interrupts while running rest of MAIN.
	
	RJMP	SWITCH_READ
	
	RJMP	MAIN										; Loop

SWITCH_READ:
	
	IN		R16, PINC
	COM		R16
	MOV		R17, R16

	EOR		R17, SWREG

	CP		R16, SWREG
	BRNE	SWITCH_CALL

	RJMP	MAIN

SWITCH_CALL:

	MOV		SWREG, R16

	SBRC	SWREG, 0									; SWITCH 0 (LSB)
	RCALL	LED_OFF										; Turn off Display
	
	SBRC	SWREG, 1									; SWITCH 1
	RCALL	LED_INC										; Increment Display value

	SBRC	SWREG, 2									; SWITCH 2
	RCALL	SEND_TEST_BYTE								; Send a single byte.

	SBRC	SWREG, 3									; SWITCH 3
	RCALL	SEND_TEST_TELEGRAM							; Send test telegram (4 bytes)

	RCALL	DELAY										; Call DELAY

	RJMP	MAIN										; Return to Main

DELAY:
    LDI		R27, 5
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

    RET													; Return

LED_INC:
	IN		R16, PORTB
	INC		R16
	OUT		PORTB, R16

	RET													; Return

LED_ON:
	LDI		R16, 0x00									; Turn on Display
	OUT		PORTB, R16									; ^

	RET													; Return
	
LED_OFF:
	LDI		R16, 0xFF									; Turn off Display
	OUT		PORTB, R16									; ^

	RET													; Return

LED_E:
	LDI		R16, 0b10001100								; Show "E" on Display
	OUT		PORTB, R16									; ^

	RET													; Return

SEND_TEST_BYTE:
	LDI		TXREG, 0x35									; Load 0x35 into transmission register
	RCALL	SERIAL_WRITE								; Write to USART

	RET

SEND_TEST_TELEGRAM:
	LDI		TXREG, 0xBB									; Send TYPE REPLY (0xBB)
	RCALL	SERIAL_WRITE								; ^

	LDI		TXREG, 0x12									; Send COMMAND VAR1 (0x12)
	RCALL	SERIAL_WRITE								; ^

	LDI		TXREG, 0xFF									; Send DATAH (0xFF)
	RCALL	SERIAL_WRITE								; ^

	LDI		TXREG, 0x35									; Send DATAL (0x35)
	RCALL	SERIAL_WRITE								; ^

	RET													; Return

REPLY_TELEGRAM:
	LDI		TXREG, 0xBB									; Send TYPE REPLY (0xBB)
	RCALL	SERIAL_WRITE								; ^

	MOV		TXREG, RECMD								; Send Most Recent COMMAND
	RCALL	SERIAL_WRITE								; ^

	MOV		TXREG, DATAH								; Send DATAH
	RCALL	SERIAL_WRITE								; ^

	MOV		TXREG, DATAL								; Send DATAL
	RCALL	SERIAL_WRITE								; ^

	RET	

SET_LED:
	LDI		R16, 0x00									; Set display to recieved byte value
	OUT		PORTB, RXREG								; ^

	RET													; Return

DO_ECHO:
	LDI		TXREG, 0x35									; Load 0x35 into transmission register
	RCALL	SERIAL_WRITE								; Write to USART

	RET

GET_LED:
	CLR		DATAH
	CLR		DATAL
	LDI		RECMD, 0x03
	
	IN		DATAL, PORTB								; Get byte value of LED

	RCALL	REPLY_TELEGRAM								; ^

	RET													; Return

EXECUTE_COMMAND:

	; !!! Maybe use ICALL and disable RX interrupts while performing call?

	CBR		FLAGS, (1<<CMDPD)							; CLEAR command execution flag
	ICALL												; CALL function of Z-pointer

	RET													; Return
	

TELEGRAM_ISR:

	; !!! Maybe use PRCMD / TELCMD instead and not count steps but commands, so that 0 = setup, 1 = parse, 2 = read data, 3 = execute.

	; !!! Maybe check if a command is pending (flag) and execute that first?

	; !!! Maybe add small delay?

	; !!! Maybe some ERROR CHECKING using PARITY?

	INC		TELSC										; Increment Telegram Step Counter

	CPI		TELSC, 1									; Setup Telegram Parser if Step = 1
	BREQ	TELEGRAM_PARSE_SETUP						; ^

	IN		RXREG, UDR									; Read UDR register

	CPI		TELSC, 4									; Execute Telegram if Step = 4
	BREQ	TELEGRAM_EXECUTE							; ^

	CPI		TELSC, 2									; Parse (TYPE) if Step = 2
	BREQ	TELEGRAM_PARSE								; ^

	DEC		ZL											; Offset Z pointer (-1) to parse TYPE (0x00_XX)

	CPI		TELSC, 3									; Parse (COMMAND) if Step = 3
	BREQ	TELEGRAM_PARSE								; ^

TELEGRAM_ISR_ESC:
	
	RETI

TELEGRAM_PARSE_SETUP:

	LDI		ZH, HIGH(COMMANDS*2)						; Reset Z Pointer to COMMANDS jump table
	LDI 	ZL, LOW(COMMANDS*2)							; ^

	INC		ZL											; Offset Z pointer (+1) to parse COMMAND (0xXX_00)

	RJMP	TELEGRAM_ISR								; Return

TELEGRAM_PARSE:

	ADIW	ZH:ZL, 4									; Increment Z Pointer (by 4)

	LPM		R16, Z										; Load the matching duty cycle

	CPI		R16, 0xEE									; Reset everything if out of table bounds
	BREQ	TELEGRAM_ERROR								; ^

	CP		R16, RXREG									; Find match in jump table
	BREQ	TELEGRAM_ISR_ESC							; ^

	RJMP	TELEGRAM_PARSE								; Repeat

TELEGRAM_EXECUTE:

	ADIW	ZH:ZL, 2									; Point at & read LOW of Table address
	LPM		R16, Z										; ^

	ADIW	ZH:ZL, 1									; Point at & read HIGH of Table address
	LPM		R17, Z										; ^

	MOV		ZL, R16										; Load Z Pointer
	MOV		ZH, R17										; ^

	SBR		FLAGS, (1<<CMDPD)							; Enable command execution flag

TELEGRAM_RESET:
	
	CLR		TELSC										; Reset parse counter
	RETI												; Return

TELEGRAM_CLRBUFFER:
	IN		TEMP1, UDR									; Empty buffer
	SBIC	UCSRA, RXC									; ^
	RJMP	TELEGRAM_CLRBUFFER							; ^

	RET													; Return

TELEGRAM_ERROR:

	CLR		TELSC										; Clear parse counter
	CLR		RXREG										; Clear reception register
	CBR		FLAGS, (1<<CMDPD)							; CLEAR command execution flag

	RCALL	LED_E

	RCALL	TELEGRAM_CLRBUFFER							; Clear reception buffer

	; !!! Maybe write error byte to master?
	
	LDI		R16, 0b00001100
	OUT		PORTB, R16
	
	RETI												; Return

SERIAL_WRITE:
	SBIS	UCSRA, UDRE									; Wait for Empty Transmit Buffer (UDRE) flag
	RJMP	SERIAL_WRITE								; ^

	OUT		UDR, TXREG									; Load transmission data from register to serial

	RET													; Return

COMMANDS:
	;DW		0xTYP_00, 0										; Explanation
	;  DW	  0x00_CMD, FUNC_ADDRESS						; ^

	.DW		0, 0											; SoT

	.DW		0xAA_00, 0										; >> GET
	  .DW	  0x00_03, GET_LED									; DATA3
	  .DW	  0x00_12, SEND_TEST_TELEGRAM						; VAR1

	.DW		0x55_00, 0										; >> SET
	  .DW	  0x00_01, LED_OFF									; DATA1
	  .DW	  0x00_02, LED_ON									; DATA2
	  .DW	  0x00_03, SET_LED									; DATA3
	  .DW	  0x00_12, DO_ECHO									; VAR1

	.DW		0xEE_EE, 0										; EoT