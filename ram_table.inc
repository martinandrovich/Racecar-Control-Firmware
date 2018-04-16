; ________________________________________________________________________________________________
; >> RAM TABLE

.DSEG
.ORG	0x0060

.EQU	TACHOMETER_H					= 0x0060
.EQU	TACHOMETER_L					= 0x0061

.EQU	COMDATA_H						= 0x0062
.EQU	COMDATA_L						= 0x0063

.EQU	ADC_H							= 0x0064
.EQU	ADC_L							= 0x0065

.EQU	TEL_STEP						= 0x0066

.EQU	RECENT_CMD						= 0x0067
.EQU	RECENT_DATA						= 0x0068

.EQU	DATA_H							= 0x0069
.EQU	DATA_L							= 0x006A

.EQU	SERIAL_RX						= 0x006B
.EQU	SERIAL_TX						= 0x006C

.EQU	DUTY_CYCLE						= 0x006D

.ORG	0x0100
.Byte	NAVN,1

.CSEG