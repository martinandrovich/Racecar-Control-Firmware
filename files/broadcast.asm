; ____________________________________________________________________________________________________________________________________________________
; >> BROADCAST

	// PLACEHOLDER
	LDI		TEMP1, (1<<BROD2)|(1<<BROD1)										;MASK SETTINGS
	AND		TEMP1, MODFLAG			

	CPI		TEMP1, (1<<BROD1)													;TACHOMETER
	BREQ	TACHOMETER

	CPI		TEMP1, (1<<BROD2)													;ACCELEROMETER
	BREQ	ACCELEROMETER
	
	CPI		TEMP1, (1<<BROD2)|(1<<BROD1)										;FINISHLINE
	BREQ	FINISHLINE

	RCALL TACHOMETER															;ALL
	RCALL ACCELEROMETER
	RCALL FINISHLINE
RET

; ____________________________________________________________________________________________________________________________________________________
; >> TACHOMETER
TACHOMETER:
	LDS	   TXREG, TACHOMETER_H
	RCALL  SERIAL_WRITE
	LDS	   TXREG, TACHOMETER_L
	RCALL  SERIAL_WRITE
	RET

; ____________________________________________________________________________________________________________________________________________________
; >> ACCELEROMETER
ACCELEROMETER:
	LDS	   TXREG, ADC_L
	RCALL  SERIAL_WRITE
	RET
; ____________________________________________________________________________________________________________________________________________________
; >> FINISHLINE
FINISHLINE:
	LDS	   TXREG, FINISHLINE
	RCALL  SERIAL_WRITE
	RET