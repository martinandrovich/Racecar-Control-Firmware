.MACRO SUBI16 ; Start macro definition 
subi @1,low(@0) ; Subtract low byte 
sbci @2,high(@0) ; Subtract high byte 
.ENDMACRO ; End macro definition