#include "p16f877a.inc"

#define XINPUT PORTA,1 ; ������ X
#define YINPUT PORTA,4 ; ������ Y
; ������ ��������� X    
#define TRX    PORTB,0
#define BLX    PORTB,1
#define BRX    PORTB,2
#define TLX    PORTB,3
; ������ ��������� Y    
#define TRY    PORTD,4
#define BLY    PORTD,5
#define BRY    PORTD,6
#define TLY    PORTD,7
; ������ ���    
#define DACON  PORTE,0 ; ����� !SYNC
#define DACCLK PORTE,1 ; ����� SCLK
#define DACDIN PORTE,2 ; ����� DIN
    
#define CHANGE_LASER PORTC,0 ;��������� ������ ������
; CONFIG
; __config 0xFF3A
    __CONFIG _FOSC_HS & _WDTE_OFF & _PWRTE_OFF & _BOREN_OFF & _LVP_OFF & _CPD_OFF & _WRT_OFF & _CP_OFF 
    
    org	    0x00
    goto    start	;����� ������������ ����������
    org	    0x04
    goto    Interrupt

start:
    call    init    
    ;TR + = +90
    ;BL + = +180
    ;BR + = +270
    ;TL + = +360
    
    ;BR + = -90
    ;BL + = -180
    ;TR + = -270
    ;TL + = -360
    
    clrf    PORTB
    clrf    tmr
calibration:
    bsf	    cal,0	;��������� ������ ����������
    bsf	    cal,1
    bsf	    cal,2
    bsf	    cal,3
    movlw   b'10001000' ;��������� ���������� �������� ��� ��
    movwf   PORTD
    movwf   PORTB
    clrf    DAC_buffer+0 ;������� ������ ��� ���
    clrf    DAC_buffer+1
    bsf	    INTCON,	PEIE ;�������� ����������
    bsf	    INTCON, GIE
    bsf	    RCSTA,	CREN
    bsf	    RCSTA,  SPEN
    bsf	    ADCON0,2
    goto    USART_ready
;-------------------------------------------------------------------------------
;    
;   ���� ����� 00
;    
;-------------------------------------------------------------------------------    
USART_ready:		;����� �������������, ���� ������ �� USART

    errorlevel -302
    movlw   0x24	;�������� ����������� �� ���� ������
    movwf   TXREG
    banksel TXSTA
    bsf	    TXSTA,TXEN
    banksel TXREG
    errorlevel +302
    bcf	    cal,3
    
USART_cycle:		;���� �������� ������ ����� �� USART
    btfss   Flags, 2
    goto    USART_cycle
    btfsc   Flags, 0
    goto    Two_bytes
One_byte:		;��� ������ ���� ����
    ;0x20 - ������
    ;0xD  - enter
    bcf	    STATUS,Z
    movf    buffer,W
    xorlw   0xD		;Enter �������� ��������-������������ ��� �������
    btfsc   STATUS, Z
    goto    Got_command ;������� Enter, ���� ��������� ������ ���������� ������
    ;���������� ������� �� �������� Enter'��, �������� ����� �� ����� ���� ������
    bcf	    STATUS,Z    
    movf    counter,W
    xorlw   .1
    btfsc   STATUS,Z
    goto    Got_XYZF	;������ ���� - ����� ���������
    
    xorlw   .1 ^ .2
    btfsc   STATUS,Z
    goto    Got_move1	;������ ���� - ��������� ������ ���������� ����� (X000)
    
    xorlw   .2 ^ .3
    btfsc   STATUS,Z
    goto    Got_move2	;������ ���� - ������ ������ ���������� ����� (0X00)
    
    xorlw   .3 ^ .4
    btfsc   STATUS,Z
    goto    Got_move3   ;��������� ���� -  ������ ������ ���������� ����� (00X0)
    
    xorlw   .4 ^ .5
    btfsc   STATUS,Z
    goto    Got_move4   ;����� ���� - ������ ������ ���������� ����� (000X)
    
    xorlw   .5 ^ .6
    btfsc   STATUS,Z
    goto    Got_dir	;������ ���� - ����� ����������� + ��� -
    
    xorlw   .6 ^ .7
    btfsc   STATUS,Z
    goto    Got_laser
    goto    Got_command	 ;������� ���� - �������� ��� ��������� �����
    
Got_XYZF:   ;������ ���� ������� �� ������ � ������� ������ "��� ����" 
    movf    buffer+0,W
    movwf   XYZFcmd
    goto    Next_byte
Got_move1:	;��������� ������ ����� ���������� ������� ASCII ��� �������
    movlw   0x30  ;����� �������� ����� �� 0 �� 9 � ������� ����� �� ��������� �������
    subwf   buffer+0,W
    movwf   var+3
    goto    Next_byte
Got_move2:
    movlw   0x30
    subwf   buffer+0,W
    movwf   var+2
    goto    Next_byte
Got_move3:
    movlw   0x30
    subwf   buffer+0,W
    movwf   var+1
    goto    Next_byte
Got_move4:
    movlw   0x30
    subwf   buffer+0,W
    movwf   var+0
    goto    Next_byte
Got_dir:    ;������ ���� ������� "��� ����"
    movf    buffer+0,W
    movwf   DIRcmd
    goto    Next_byte
Got_laser:  ;������� ���� ������� "��� ����"
    movf    buffer+0,W
    movwf   LASERcmd
    goto    Next_byte
Next_byte:  ;������� ����� � ����� ��� ������ ���������� �����
    clrf    buffer+0
    clrf    buffer+1
    bcf	    Flags,0
    bcf	    Flags,1
    bcf	    Flags,2
    goto    USART_cycle
Two_bytes:
    clrf    buffer+0
    clrf    buffer+1
    bcf	    Flags,0
    bcf	    Flags,1
    bcf	    Flags,2
    goto    USART_cycle
Got_command: ;��� ������� �������� (������ Enter), ������� ����� � 
	     ;������������ � ����������� ������� ��� ����������
    clrf    buffer+0
    clrf    buffer+1
    clrf    counter
    ;************************************************************************************************
    ;��������� ���������� �����. ���� ����������� ���������� �� ��������� � ������� ������ � ��������
    ;������� ������� � �������� ��� ��������� �� 1000. 4 � �������� �������
    ; 4*1000 ���������� ���:
    ; 4<<9 = 2048
    ; 4<<8 = 1024
    ; 4<<7 = 512
    ; 4<<6 = 256
    ; 4<<5 = 128
    ; 4<<3 = 32
    ; 2048+1024+512+256+128+32 = 4000 
    ;************************************************************************************************
    movlw   .5	;���������� �������� �����
    movwf   temp
Step_counter1:  ;��������� �� 1000
    movf    temp,W
    addlw   .4	;����������� ������� ����� ����������� �������� ����� � �������� ������
    movwf   counter
    movf    var+3,W
    movwf   buffer+0
    call    Calc ;������� ������� ������ � ��������
    decfsz  temp,f
    goto    Step_counter1
    
    movlw   .3
    movwf   counter
    movf    var+3,W
    movwf   buffer+0
    call    Calc
    ;************************************************************************************************
    ; ������� ������� � �������� ��� ��������� �� 100. 4 � �������� �������
    ; 4*100 ���������� ���:
    ; 4<<6 = 256 
    ; 4<<5 = 128
    ; 4<<2 = 16
    ; 256+128+16 = 400
    ;************************************************************************************************
Step_counter2:	;��������� �� 100
    movlw   .6
    movwf   counter
    movf    var+2,W
    movwf   buffer+0
    call    Calc
    
    movlw   .5
    movwf   counter
    movf    var+2,W
    movwf   buffer+0
    call    Calc
    
    movlw   .2
    movwf   counter
    movf    var+2,W
    movwf   buffer+0
    call    Calc
    ;************************************************************************************************
    ;������� ������� � �������� ��� ��������� �� 10. 4 � �������� �������
    ;4*10 ���������� ���:
    ; 4<<5 = 32
    ; 4<<1 = 8
    ; 32 + 4 = 40
    ;************************************************************************************************
Step_counter3:	;��������� �� 10
    movlw   .3
    movwf   counter
    movf    var+1,W
    movwf   buffer+0
    call    Calc
    
    movlw   .1
    movwf   counter
    movf    var+1,W
    movwf   buffer+0
    call    Calc
Step_counter4:	; �������� ������� ������
    bcf	    STATUS,C
    movf    var+0,W
    addwf   MOVEcmd+0,f
    btfsc   STATUS,C
    incf    MOVEcmd+1,f

Work:
    ;************************************************************************************************
    ;�������� ������ ���������� ������ ��� ����������� ����������� �������� ����������
    ;X �����������
    ;Y ���������
    ;Z ��������� ����� ������� /
    ;F ��������� ������ ������ \
    ;+ ��� - ���������� ����������� ��������. ��� X/Y �������� �� ������� ������� 
    ;���� ������ ������� �������
    ;��� Z/F �������� ����� ����� ���� ������ ����
    ;************************************************************************************************
    clrf    Flags
    bcf     STATUS,Z
    movf    XYZFcmd,W
    xorlw   'X'
    btfsc   STATUS, Z
    goto    X
    
    xorlw   'X' ^ 'Y'
    btfsc   STATUS,Z
    goto    Y
    
    xorlw   'Y' ^ 'Z'
    btfsc   STATUS,Z
    goto    Z_symbol
    
    xorlw   'Z' ^ 'F'
    btfsc   STATUS,Z
    goto    F_symbol
X:
    bcf	    Flags,3
    goto    Detect_direction
Y:
    bsf	    Flags,3
    goto    Detect_direction
Z_symbol:  
    bcf	    Flags,3
    bsf	    Flags,4
    goto    Detect_direction
F_symbol:  
    bsf	    Flags,3
    bsf	    Flags,4
Detect_direction:	;����������� + ��� -
    bcf	    STATUS,Z
    movlw   '+'
    xorwf   DIRcmd,W
    btfsc   STATUS,Z
    goto    Plus_setup	;������� +, �������� �������

Minus_setup:		; ��� Z(10) 11 / ��� F(11) 01 \
    bsf	    Flags,5
    btfss   Flags,3
    bsf	    Flags,6
    goto    Detect_laser
Plus_setup:		;��� Z(10) 00 / ��� F(11) 10 \ 
    btfsc   Flags,3
    bsf	    Flags,6
Detect_laser:		;���� ������� Y - ����� ��������, ���� ���-���� ������ - ����� ���������
    bcf	    STATUS,Z
    movlw   'Y'
    xorwf   LASERcmd,W
    btfsc   STATUS,Z
    bsf	    Flags,7	;��������� ����� ����� ���
    bsf	    Flags,1	;��������� ����� ���������� ��������� �������
   
Wait:			;���� �������� ��������� �������
    btfss   work_done,0
    goto    Wait
    bcf	    PORTA,5
    errorlevel -302
    banksel TXSTA
    bsf	    TXSTA,TXEN
    banksel TXREG
    movlw   0x24	;������� ���������, ������� � �������� �����
    movwf   TXREG
    errorlevel +302
			;������� ��� ����� � ������ ��� ������ ����� �������
    bcf	    work_done,0
    clrf    Flags
    clrf    XYZFcmd
    clrf    MOVEcmd+0
    clrf    MOVEcmd+1
    clrf    DIRcmd
    clrf    LASERcmd
    clrf    var+0
    clrf    var+1
    clrf    var+2
    clrf    var+3
    clrf    counter
    clrf    buffer+0
    clrf    buffer+1
    
    goto    USART_cycle
    
;-------------------------------------------------------------------------------
;    
;  ����� ���� ����� 00
;    
;-------------------------------------------------------------------------------    
;*******************************************************************************
;
;       �������
;
;*******************************************************************************    
    ORG     0x0400   

;*******************************************************************************
;       Calc
;-------------------------------------------------------------------------------
; ��������� ����� � �������� �����
; �� �����:
;       buffer+0 - ����� �� 0 �� 9
;	counter  - ������� �������
; �� ������:
;       MOVEcmd - ��������� �������� (Word)
;-------------------------------------------------------------------------------
Calc:
    bcf	    STATUS,C
    rlf	    buffer+0,f
    rlf	    buffer+1,f
    decfsz  counter,f
    goto    Calc

    bcf	    STATUS,C
    movf    buffer+0,W
    addwf   MOVEcmd+0,f
    movf    buffer+1,W
    btfsc   STATUS,C
    incfsz  buffer+1,W
    addwf   MOVEcmd+1,f
    clrf    buffer+0
    clrf    buffer+1
    return
#include "Init.inc"
#include "Interrupt.inc"
#include "Memory.inc"
    end