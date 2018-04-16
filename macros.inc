.MACRO	SWRITE	
	
	LDS		TXREG, @0															;
	STS		SERIAL_TX, TXREG													;
	SBIS	UCSRA, UDRE															; Wait for Empty Transmit Buffer (UDRE) Flag
	RJMP	SERIAL_WRITE														; ^

	OUT	UDR, TXREG																; Write data from Transmission Register
	
	RET	
.ENDMACRO	