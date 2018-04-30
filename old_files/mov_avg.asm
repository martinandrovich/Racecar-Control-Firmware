//MOVING-MEAN HOW DOES IT WORK? NEED ATLEAST TWO POINTS TO START
.def addREGL = R4;
.def addREGH = R5;
.def TEMP1 = R20;
.def TEMP2 = R21;
.def TEMP3 = R22;

//CONSTANTS ->
.equ WHILELOOP_ADDER_CMP = 0x40							;
.equ AMOUNT_OF_DIVIDES = 0x06								; => 2^5 = 32

.equ DATARAM_SPOT = 0x500								;
.equ DATA_RAM_MAX_ADC = 0x540							;

.equ baudrate4800 = 0x0C								;

//ADMUX SETUP

SETUP:

//LOAD POINTER THAT IS TO BE USED

LDI TEMP1, HIGH(RAMEND)
OUT SPH, TEMP1
LDI TEMP1, LOW(RAMEND)
OUT SPL, TEMP1

LDI TEMP1, 0xFF
OUT DDRB, TEMP1

LDI TEMP1, (1<<ADLAR)							//SETUP ADC
OUT ADMUX, TEMP1

LDI TEMP1, (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0)		//REMEMBER TO START ADC, BY TYPING ONE TO ADSC
OUT ADCSR, TEMP1

LDI TEMP1, LOW(baudrate4800);
OUT	UBRRL, TEMP1

LDI TEMP1, HIGH(baudrate4800);
OUT UBRRH, TEMP1

LDI TEMP1, (1<<TXEN)
OUT UCR, TEMP1 

RCALL SET_POINTER_Y
RCALL SETUP_SRAM
RCALL SET_POINTER_Y

RJMP MAIN
;____________ADC IS FIRST________________
;------------------------------------------------------------------------------;
MAIN:
;------------------------------------------------------------------------------;
SBI ADCSR, ADSC					//ADC POLLING
;------------------------------------------------------------------------------;
REPEAT_UNTIL_SET:				//
SBIS ADCSR, ADIF				//
RJMP REPEAT_UNTIL_SET			//^REPEATS POLLING UNTIL ADC SET
;------------------------------------------------------------------------------;
CLI							//INSERT A STOP INTERRUPTS HERE
;------------------------------------------------------------------------------;
IN TEMP2, ADCL					//READ LOW FIRST
NOP;
IN TEMP2, ADCH					//ADLAR IS TURNED ON, SO D9-D2 IS HERE 256 RESO
NOP;
;------------------------------------------------------------------------------;
SEI								//TURN INTERRUPT ON AGAIN
;------------------------------------------------------------------------------;
;____________AVG_FILTER IS SECOND_______
;------------------------------------------------------------------------------;
;->>>>>>>OUT PORTB, TEMP2 //test WORK Stil HERE
;->>>>>>>RJMP MAIN

ST	Y+, TEMP2									//INSERT ADC_VALUE
;------------------------------------------------------------------------------;
CPI YL, WHILELOOP_ADDER_CMP						//IF IT HAS TO RESET FOR NEXT TIME ->
BRNE SKIP_THIS_RESET							//
RCALL SKIP_RESET_POINTER						//	
SKIP_THIS_RESET:								//
MOV TEMP1, YL									//SAVE THE VALUE FOR LATER
;------------------------------------------------------------------------------;
;________START_TO_ADD_VALUES__________
;------------------------------------------------------------------------------;
CALL ADD_LOOP
;------------------------------------------------------------------------------;
;_______DO_THE_DIVISION
;------------------------------------------------------------------------------;
CALL DIV_LOOP
;------------------------------------------------------------------------------;
MOV YL, TEMP1
OUT PORTB, addREGL

POLL_IT:
SBIS USR, UDRE
RJMP POLL_IT 

OUT UDR, addREGL

JMP MAIN
;------------------------------------------------------------------------------;
;____________POINTER_FOR_MEANFILTER_______
;------------------------------------------------------------------------------;
SET_POINTER_Y:
SKIP_RESET_POINTER:

LDI YH, HIGH(DATARAM_SPOT);
LDI YL, LOW(DATARAM_SPOT);

RET
;____________ADD_LOOP____________
;------------------------------------------------------------------------------;
ADD_LOOP:
LDI TEMP2, 0									//set all values to 0
MOV addREGL, TEMP2								//
MOV addREGH, TEMP2								//
LDI YL, LOW(DATARAM_SPOT)						//reset value;

ADDER_LOOP:										//
LD	TEMP2, Y+ 									//load from pointer
ADD addREGL, TEMP2								//

BRCC SKIP_INCREMENT_H							//branch if carry is not set
INC addREGH
SKIP_INCREMENT_H:

CPI YL, LOW(DATA_RAM_MAX_ADC)					//
BRNE ADDER_LOOP
RET												//
;____________DIV_LOOP____________
;------------------------------------------------------------------------------;
DIV_LOOP:
LDI TEMP2, AMOUNT_OF_DIVIDES
DIVIDE_LOOP_CONTINUE:
CLC
ASR addREGH
ROR addREGL
DEC TEMP2
BRNE DIVIDE_LOOP_CONTINUE
RET
;____________SETUP_SRAM__________
;------------------------------------------------------------------------------;
SETUP_SRAM:
LDI TEMP2, 128
CONTINUE_NULL:									//
ST	Y+, TEMP2 									//
CPI YL, LOW(DATA_RAM_MAX_ADC)					//
BRNE CONTINUE_NULL								//
RET												//
;------------------------------------------------------------------------------;