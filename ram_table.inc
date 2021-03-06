; ____________________________________________________________________________________________________________________________________________________
; >> RAM TABLE

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > Variables [32B]

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
.EQU	MOVAVG_RECENT_XL				= 0x0072

.EQU	ACCELEROMETER					= 0x0073

.EQU	TURN_MIN_TACHOMETER_H			= 0x0074		; Old
.EQU	TURN_MIN_TACHOMETER_L			= 0x0075		; ^

.EQU	MAPPING_DEBOUNCE_H				= 0x0074
.EQU	MAPPING_DEBOUNCE_L				= 0x0075

.EQU	TACHOMETER_L_PREV				= 0x0076

.EQU	TRACK_LENGTH_H					= 0x0078
.EQU	TRACK_LENGTH_L					= 0x0079

.EQU	LATEST_STRAIGHT					= 0x007A

.EQU	LATEST_STRAIGHT_H				= 0x007B
.EQU	LATEST_STRAIGHT_L				= 0x007C

.EQU	LAST_TURN_H						= 0x007D
.EQU	LAST_TURN_L						= 0x007E

.EQU	VELOCITY_COUNTER				= 0x007F

.EQU	VELOCITY_H						= 0x0080
.EQU	VELOCITY_L						= 0x0081

.EQU	VELOCITY_PREV_H					= 0x0082
.EQU	VELOCITY_PREV_L					= 0x0083

.EQU	BREAK_TEST_VAL					= 0x0084

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > Moving Average Filter [MOVAVG | 128B]

.EQU	MOVAVG_TABLE					= 0x0100

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > Mapping [128B]

.EQU	MAPP_TABLE						= 0x0200

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
;  > Trajectory [1024B]

.EQU	TRAJ_TABLE						= 0x0300

;  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _