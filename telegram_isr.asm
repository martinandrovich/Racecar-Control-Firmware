/*
 * telegram_int.asm
 *
 *  Created: 30-03-2018 16:42:10
 *   Author: Martin Androvich
 */ 

 ; ________________________________________________________________________________________________
; >> COMMUNICATION PROTOCOL MODULE (INTERRUPT VERSION):

TELEGRAM_ISR:

	; !!! Maybe use PRCMD / TELCMD instead and not count steps but commands, so that 0 = setup, 1 = parse, 2 = read data, 3 = execute.

	; !!! Maybe check if a command is pending (flag) and excetute that first?

	INC		PRCNT

	CPI		PRCNT, 1
	BREQ	TELEGRAM_PARSE_SETUP

	CPI		PRCNT, 2
	BREQ	TELEGRAM_PARSE

	CPI		PRCNT, 3
	BREQ	TELEGRAM_PARSE


	; !!! GET DATA + EXECUTE

	CPI		PRCNT, 4
	BREQ	TELEGRAM_EXECUTE

TELEGRAM_ISR_ESC:
	
	RETI

TELEGRAM_PARSE_SETUP:

	CLR		TEMP1
	CLR		TEMP2

	LDI		ZH, HIGH(COMMANDS*2)							; Set SEQUENCE_1
	LDI 	ZL, LOW(COMMANDS*2)							; ^

	RJMP	TELEGRAM_ISR

TELEGRAM_PARSE:

	;RCALL	SERIAL_READ

	LPM		R16, Z										; Load the matching duty cycle

	CP		R16, RXREG
	BREQ	TELEGRAM_ISR_ESC
	
	CPI		R16, 0xEE									; Reset everything if out of table bounds.
	BREQ	TELEGRAM_RESET								; ^

	ADIW	ZH:ZL, 4

	RJMP	TELEGRAM_PARSE

TELEGRAM_EXECUTE:

	ADIW	ZH:ZL, 2									; Point at & read LOW of Table address
	LPM		R16, Z										; ^

	ADIW	ZH:ZL, 1									; Point at & read HIGH of Table address
	LPM		R17, Z										; ^

	MOV		ZL, R16										; Load Z Pointer
	MOV		ZH, R17										; ^

	SBR		FLAGS, (1<<7)								; Enable execute flag

TELEGRAM_RESET:
	
	CLR		PRCNT										; Reset parse counter
	RETI												; Return

TELEGRAM_ERROR:
	
	; !!! Should send error byte and clear buffer.