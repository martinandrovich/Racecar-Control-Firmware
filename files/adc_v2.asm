//CONSTANTS ->
.equ AVGSIZE = 128								;
.equ AVGDIV	 = 7								; => 2^5 = 32

.equ MOVAVG = 0x500								;
.equ MOVAVG_END = MOVAVG+AVGSIZE				;


RCALL SET_POINTERAVG_X

RCALL SETUP_SRAM

;;make recent X_AVG_H

;____________ADC IS FIRST________________
;------------------------------------------------------------------------------;

ADC_MOVING:

LDS XH, RECENT_XAVG_H							;
LDS XL, RECENT_XAVG_L							;

ST	X+, ADC_H									;insert ADC_val from high because of ADLAR

CPI XL, AVGSIZE									;check for reset

BRNE SKIP_THIS_RESET							;

RCALL SET_POINTERAVG_X							;	

SKIP_THIS_RESET:								;

STS RECENT_XAVG_L, XL							;

RCALL ADD_LOOP									;add loop

RCALL DIV_LOOP									;


RET

;------------------------------------------------------------------------------;
;____________POINTER_FOR_MEANFILTER_______
;------------------------------------------------------------------------------;

SET_POINTERAVG_X:


LDI XH, HIGH(MOVAVG)							;
LDI XL, LOW(MOVAVG)								;


RET

;____________ADD_LOOP____________
;------------------------------------------------------------------------------;


ADD_LOOP:
CLR		TEMP1										;
CLR		TEMP2										;
CLR		TEMP3										;
RCALL	SET_POINTERAVG_X

ADDER_LOOP:											;
LD		TEMP1, X+ 									;load from pointer
ADD		TEMP2, TEMP1								;

BRCC	SKIP_INCREMENT_H							;branch if carry is not set %% can change to SBIC, SREG <- 
INC		TEMP3										;
SKIP_INCREMENT_H:									;

CPI		XL, LOW(MOVAVG_END)							;remember to compare ZH aswell.
BRNE	ADDER_LOOP									;

RET													;

;____________DIV_LOOP____________
;------------------------------------------------------------------------------;

DIV_LOOP:
LDI		TEMP1, AVGDIV								;

DIVIDE_LOOP_CONTINUE:								;

ASR		TEMP3										;TEMP3 IS HIGH
ROR		TEMP2										;TEMP2 IS LOW

DEC		TEMP1										;
BRNE	DIVIDE_LOOP_CONTINUE						;

STS		ACCELEROMETER, TEMP2						;save val

RET													;

;____________SETUP_SRAM__________
;------------------------------------------------------------------------------;
SETUP_SRAM:											;
LDI TEMP1, AVGSIZE									;

STS RECENT_XAVG_H, XH
STS RECENT_XAVG_L, XL


CONTINUE_NULL:										;
ST	X+, TEMP1 										;
CPI XL, LOW(MOVAVG_END)								;
BRNE CONTINUE_NULL									;
RET													;