; ________________________________________________________________________________________________
; >> RAM TABLE :

.EQU	TACHOMETER_H					= 0x0060
.EQU	TACHOMETER_L					= 0x0061

.EQU	COMDATA_H						= 0x0062
.EQU	COMDATA_L						= 0x0063

.EQU	ADC_H							= 0x0064
.EQU	ADC_L							= 0x0065

.EQU	TEL_STEP						= 0x0066

.EQU	RECENT_CMD						= 0x0067
.EQU	RECENT_DAT						= 0x0068

.EQU	DATA_H							= 0x0069
.EQU	DATA_L							= 0x006A

.EQU	SERIAL_RX						= 0x006B
.EQU	SERIAL_TX						= 0x006C

.EQU	DUTY_CYCLE						= 0x006D

.EQU	FUNC_FLG						= 0x006E
.EQU	MODE_FLG						= 0x006F

.EQU	FINISHLINE						= 0x0070

.EQU	MOVAVG_RECENT_XH				= 0x0071
.EQU	MOVAVG_RECENT_XL				= 0x0071

.EQU	ACCELEROMETER					= 0x0073

; ___________________________________________________________________________________________________________________________________________________
; >> MOVAVG		:					; 32 values in the moving avg filter

.EQU	MOVAVG_TABLE					= 0x0100

; ___________________________________________________________________________________________________________________________________________________
; >> MAPPING_START	:					; 128 values in the moving avg filter, does not have to take care of HIGH bit on ZH when first passed to ZH

.EQU	MAPP							= 0x0200

; ___________________________________________________________________________________________________________________________________________________
; >> Tracjectory	:					; 1024 values in the moving avg filter

.EQU	TRAC							= 0x0300