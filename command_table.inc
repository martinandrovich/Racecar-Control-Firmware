; ____________________________________________________________________________________________________________________________________________________
; >> COMMUNICATION PROTOCOL COMMANDS

COMMANDS:
	;DW		0xTYP_00, 0										; Explanation
	;  DW	  0x00_CMD, FUNC_ADDRESS						; ^

	.DW		0x00_00, 0										; SoT

	.DW		0xAA_00, 0										; >> GET
	  .DW	  0x00_03, EMPTY									; DATA3
	  .DW	  0x00_12, EMPTY									; VAR1
	  .DW	  0x00_13, MAPPING_GET								; MAPP

	.DW		0x55_00, 0										; >> SET
	  .DW	  0x00_10, SET_MOTOR_PWM							; START
	  .DW	  0x00_11, EMPTY									; STOP
	  .DW	  0x00_12, EMPTY									; AUTO
	  .DW	  0x00_13, MAPPING_SET								; MAPP
	  .DW	  0x00_14, BROADCAST_SET							; BROD

	.DW		0xEE_EE, 0										; EoT