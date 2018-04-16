COMMANDS:
	;DW		0xTYP_00, 0										; Explanation
	;  DW	  0x00_CMD, FUNC_ADDRESS						; ^

	.DW		0, 0											; SoT

	.DW		0xAA_00, 0										; >> GET
	  .DW	  0x00_03, EMPTY									; DATA3
	  .DW	  0x00_12, EMPTY						; VAR1

	.DW		0x55_00, 0										; >> SET
	  .DW	  0x00_01, EMPTY									; DATA1
	  .DW	  0x00_02, EMPTY									; DATA2
	  .DW	  0x00_03, EMPTY									; DATA3
	  .DW	  0x00_12, EMPTY									; VAR1

	.DW		0xEE_EE, 0										; EoT