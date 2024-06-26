;*******************************************************************************
;
;       ������������ ��������� ����������
;
;*******************************************************************************

Interrupt:
    movwf       W_TEMP_          ;������ ����������� ������������ W � 
                                 ;������� W_TEMP
    swapf       STATUS,W         ;�������� ������� ��������� �������� STATUS �
                                 ;� ������� � W
    clrf        STATUS           ;������� �������� STATUS, ������� 0-� ����
    movwf       STATUS_TEMP      ;������ ����������� ������������ W �
                                 ;������� W_TEMP

                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                                 ;� ������ ���� ������ ���� ��������� 2048 ����:
    movf        PCLATH,W         ;������ ����������� �������� PCLATH �
    movwf       PCLATH_TEMP      ;������� PCLATH_TEMP
    clrf        PCLATH           ;������� �������� PCLATH
                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                                 ;��� ������������� ��������� ���������:
    movf        FSR,W            ;������ ����������� �������� FSR �
    movwf       FSR_osn          ;������� FSR_osn
    movf        FSR_prer,W       ;������ ����������� �������� FSR_prer �
    movwf       FSR              ;������� FSR
                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
				 
;-------------------------------------------------------------------------------
TMR0_check:
    btfsc   INTCON,T0IF		;���� �� ���������� �� TMR0
    btfss   INTCON,T0IE		;��������� �� ����������
    goto    TMR0_skip
    btfsc   Flags,1
    goto    Command_move        ;���� �������, ��������� � ���������� ��
    btfsc   cal,0
    goto    calX		;������� �� ���������� ��������� X
    btfsc   cal,1
    goto    calY		;������� �� ���������� ��������� Y
    btfsc   cal,2
    goto    calLaser		;������� �� ���������� ������
    btfsc   tmr,1
    bsf	    tmr,0
    bcf	    INTCON,T0IF		;������� ���� ����������
    btfsc   tmr,1
    goto    calLaser
TMR0_skip:
TMR1_check:
    btfss   PIR1,TMR1IF         ;���� �� ���������� �� TMR1
    goto    TMR1_skip
    btfsc   tmr,1
    bsf	    tmr,0
    bcf	    PIR1,TMR1IF
    btfsc   tmr,1
    goto    calLaser
TMR1_skip:
ADC_check:
    btfss   PIR1,ADIF		;���� �� ���������� �� ���
    goto    TX_check
    btfsc   ADCON0,2		; ���������� �� ��� GO/-DONE
    goto    TX_check
    
    btfsc   cal,2		;���� ������� �� ���������� ������, ��������� 25% ��������
    goto    V1_24
    
    btfss   Flags,1
    goto    calLaser		;���� ������� �� ������ ���, �������� ����� ��������������
V5_0:				;�������� �� ��������� 100% ��� ������� ������
    
    movf    ADRESH,W
    addlw   -.244
    addlw   -(.249 - .244 + 1)
    btfss   STATUS,C
    goto    calLaser		;���� 90 < W� < 95, �������� ����� ��������������
    
    movf    ADRESH,W		;�������� ���� W� < 89%
    addlw   -.0
    addlw   -(.243 - .0 + 1)
    btfss   STATUS,C
    goto    DAC_inc		;W� < 89%, �������� ���������� �� ���
    
    ;�������� 95 < W� < 100, �������� �� �������� ������
    goto    DAC_dec
    
V1_24: 
    ;�������� �� ��������� W� = 25% ��� ����������
    movlw   b'00111111'		;1.24V (W� = 25%)
    xorwf   ADRESH,W
    btfsc   STATUS,Z
    goto    calEnd	      ;���������� ������������� �����, ��������� ����������
    
    ;�������� �� W� > 25%
    movf    ADRESH,W
    addlw   -.64	      ;��������� ��������
    addlw   -(.255 - .64 + 1) ;�������� �� ��������� � �������� 26% <= W� <= 100%
    btfss   STATUS,C
    goto    DAC_dec	      ;������, ����� ��������� ����������
    ;�������� �� �� ����� ��� ��������� 0% <= W� <=24%
    movf    ADRESH,W
    addlw   -.0
    addlw   -(.62 - .0 + 1)
    btfss   STATUS,C
    goto    DAC_inc
    
    ;�������� �� ������ �� � ���� ��������, �������� ���������� ��������������
    bcf	    PIR1,ADIF
    bcf	    INTCON,T0IF
    bsf	    ADCON0,2
    goto    rollback

DAC_inc:		    ;������� ������ ��� ��� �� 1
    incf    DAC_buffer+0,f
    movf    DAC_buffer+0,W
    btfsc   STATUS,Z
    incf    DAC_buffer+1,f
    btfsc   DAC_buffer+1,4
    goto    DAC_dec
    goto    DAC_write
    
DAC_dec:		    ;�������� ����� ��� ��� �� 1
    bcf	    CHANGE_LASER
    btfsc   DAC_buffer+1,4
    bsf	    CHANGE_LASER
    decf    DAC_buffer+0,f
    movlw   0xFF
    xorwf   DAC_buffer+0,W
    btfss   STATUS,Z
    goto    DAC_write
    decf    DAC_buffer+1,f
DAC_write:		    ;���������� ����� � ������ � ���
    movlw   b'00001111'	    ;����� ��� ��������� ������������ 
    andwf   DAC_buffer+1,f
    movlw   .4
    movwf   temp
    movf    DAC_buffer+0,W
    movwf   DAC_temp+0
    movf    DAC_buffer+1,W
    movwf   DAC_temp+1
    
    
DAC_rlf: ;�������� ����� xxxx0000 00000000 � ���� 00000000 0000xxxx, ��� ������� ����������
    rlf	    DAC_temp+0,f
    rlf	    DAC_temp+1,f
    decfsz  temp,f
    goto    DAC_rlf
    movlw   .12
    movwf   temp
    bcf	    DACON
DAC_Write_cycle:	    ;������� ������������ ����� � ��� AD5310 
    bcf	    DACDIN
    btfsc   DAC_temp+1,7
    bsf	    DACDIN
    bsf	    DACCLK
    bcf	    DACCLK
    rlf	    DAC_temp+0,f
    rlf	    DAC_temp+1,f
    decfsz  temp,f
    goto    DAC_Write_cycle
    
    bsf	    DACON
    bcf	    PIR1,ADIF
    bcf	    INTCON,T0IF
    bsf	    ADCON0,2 ;�������� �������������� ��� �������� ������������� �������� �� ������ ���
    goto    rollback
    

calEnd: ;���������� ������ ���������, ������� ��� ���������� ������,�������� ����� �������������� � ������ �� ����������
    bcf	    cal,2
    bcf	    PIR1,ADIF
    bsf	    tmr,1
    goto    rollback
TX_check:		    ;�������� ���������� �� ����������� USART
    btfss   PIR1,TXIF
    goto    RX_check
    btfsc   work_done,0	    ;���� ������� ���������, �������� Welcome - ������
    goto    TX_send_ready   
    btfsc   cal,3	    ;���� ������������� ���������, �������� Welcome- ������
    goto    TX_send_ready
    errorlevel -302
    banksel TXSTA
    bcf	    TXSTA,TXEN	    ;���� ���������, �������� ��������
    banksel 0 
    goto    rollback
    
TX_send_ready:		    ;�������� Welcome-������� ($)
    movlw   0x24    
    movwf   TXREG
    goto    rollback
;-------------------------------------------------------------------------------
;    
;   ���� ����� 01
;    
;-------------------------------------------------------------------------------    
calX:			    ;���������� ��������� X ���� �� ��������� ������ X = 0
    btfss   XINPUT
    bcf	    cal,0 
    bcf	    STATUS,C
    rlf	    PORTB,f
    btfsc   STATUS,C
    bsf	    PORTB,0
    bcf	    INTCON,T0IF
    goto    rollback

calY:			    ;���������� ��������� Y ���� �� ��������� ������ Y = 0
    btfss   YINPUT
    bcf	    cal,1 
    bcf	    STATUS,C
    rlf	    PORTD,f
    btfsc   STATUS,C
    bsf	    PORTD,0
    bcf	    INTCON,T0IF
    goto    rollback
;-------------------------------------------------------------------------------
;    
;   ����� ���� 01
;    
;------------------------------------------------------------------------------- 
;-------------------------------------------------------------------------------
;    
;   ���� ����� 03
;    
;-------------------------------------------------------------------------------
calLaser:		  ;������ �������������� ���
    bcf	    PIR1,ADIF
    bsf	    tmr,1
    btfss   tmr,0         ;���� �� ���������� �� TMR1
    goto    rollback
    bcf	    tmr,0
    btfss   ADCON0,2
    bsf	    ADCON0,2
    bcf	    PIR1,TMR1IF
    goto    rollback
;-------------------------------------------------------------------------------
;    
;   ����� ���� 03
;    
;-------------------------------------------------------------------------------    
RX_check:		;�������� ���������� �� ��������� USART
    btfsc   PIR1,RCIF	;���� �� ���������� �� ��������� USART
    btfss   PIE1,RCIE   ;��������� �� ����������
    goto    RX_skip
    btfss   Flags,1	;����������� �� ������
    goto    RX_read	;�� �����������, ������ ����
    ;������ �����������, ������ �����
    errorlevel -302
    banksel TXSTA
    bsf	    TXSTA,TXEN
    banksel TXREG
    movlw   0x21
    movwf   TXREG
    movwf   TXREG
    errorlevel +302
    movf    RCREG,W
    goto    rollback
    ;�������� ���������� ���������� RCSTA,CREN � 1
    ;���� RCIF ������������ ��� ������ �� RCREG
    ;��� ������������ ������ ��� RCSTA,OERR (1) ��������������� � 1
    ;��� OERR ��������� ������� ���� CREN (������ ������)
    ;��� ������ ����� RCSTA,FERR ��������������� ���� �� ��������� �������� ���
    ;������������� ������� ������ RCSTA ����� RCREG ����� �� �������� 9 ��� � FERR
RX_read:        ;�������� ������ USART
    btfss   RCSTA,OERR
    goto    NO_overflow_err
    bcf	    RCSTA,CREN
    bsf	    Flags,0
    movf    RCREG,W
    movwf   buffer+0
    movf    RCREG,W
    movwf   buffer+1
    bsf	    Flags,2
    bsf	    RCSTA,CREN
    incf    counter,f
    incf    counter,f
    goto    rollback
NO_overflow_err: ;�� ���� ������������ ������ USART, ������ ���� �� ��������� ������
    movf    RCREG,W
    movwf   buffer+0
    bcf	    STATUS,Z
    xorlw   0x20	;���� ������ ������ (0x20) - ����������
    btfsc   STATUS,Z
    goto    delimitter
    bsf	    Flags,2
    incf    counter,f
    goto    rollback
delimitter:		;������ ������, ������� �����
    clrf    buffer+0
    goto    rollback
RX_skip:
    goto    rollback
;-------------------------------------------------------------------------------
;    
;   ���� ����� 02
;    
;-------------------------------------------------------------------------------    
Command_move:		;��������� ������ ��
    btfsc   Flags,4
    goto    Both	;����� ������� ��� ���������
    btfss   Flags,5
    goto    Solo_Plus	;����� ������� ���� ��������� �� �������
Solo_minus:		;������� ��������� X ������ ������� �������, ���� �������� ���� �� ���� �� ������� ���� �� ���������
    btfsc   Flags,3	;���� ���� ��������� Y ����������, �������� �������, ����� ������������ �������� �� X 
    goto    Solo_minusY	
    bcf	    STATUS,C
    rrf	    PORTB,f
    btfsc   STATUS,C
    bsf	    PORTB,7
    goto    Count
    
Solo_minusY:		;��������� �������� �� Y ������ ������� �������
    bcf	    STATUS,C
    rrf	    PORTD,f
    btfsc   STATUS,C
    bsf	    PORTD,7
    goto    Count
    
    
Solo_Plus:		;������� ��������� X �� ������� �������
    btfsc   Flags,3
    goto    SoloPlusY
    bcf	    STATUS,C
    rlf	    PORTB,f
    btfsc   STATUS,C
    bsf	    PORTB,0
    goto    Count
SoloPlusY:		;������� ��������� Y �� ������� �������
    bcf	    STATUS,C
    rlf	    PORTD,f
    btfsc   STATUS,C
    bsf	    PORTD,0
    goto    Count
Both:			;������� ���������� X � Y
    movf    Flags,W
    andlw   b'01100000' ;����� ��� ���������� ������ ���
    bcf	    STATUS,Z
    xorlw   b'00000000' ;���� ������� Z +
    btfsc   STATUS,Z
    goto    Plus_both
    xorlw   b'00000000' ^ b'01100000' ;���� ������� Z -
    btfsc   STATUS,Z
    goto    Minus_both
    xorlw   b'01100000' ^ b'00100000' ;���� ������� F +
    btfsc   STATUS,Z
    goto    XPlusYMinus
    xorlw   b'00100000' ^ b'01000000' ;���� ������� F - 
    btfsc   STATUS,Z
    goto    YPlusXMinus
    
Plus_both:		;Z+
    bcf	    STATUS,C
    rlf	    PORTB,f
    btfsc   STATUS,C
    bsf	    PORTB,0
    bcf	    STATUS,C
    rlf	    PORTD,f
    btfsc   STATUS,C
    bsf	    PORTD,0
    goto    Count
    
Minus_both:		;Z-
    bcf	    STATUS,C
    rrf	    PORTB,f
    btfsc   STATUS,C
    bsf	    PORTB,7
    bcf	    STATUS,C
    rrf	    PORTD,f
    btfsc   STATUS,C
    bsf	    PORTD,7
    goto    Count
    
XPlusYMinus:		;F+
    bcf	    STATUS,C
    rlf	    PORTB,f
    btfsc   STATUS,C
    bsf	    PORTB,0
    bcf	    STATUS,C
    rrf	    PORTD,f
    btfsc   STATUS,C
    bsf	    PORTD,7
    goto    Count
    
YPlusXMinus:		;F-
    bcf	    STATUS,C
    rrf	    PORTB,f
    btfsc   STATUS,C
    bsf	    PORTB,7
    bcf	    STATUS,C
    bcf	    STATUS,C
    rlf	    PORTD,f
    btfsc   STATUS,C
    bsf	    PORTD,0
    goto    Count
    
Count:			;����������� ������� ���������� �����
    
    movlw   0x00 
    bcf	    STATUS,Z
    xorwf   MOVEcmd+0,W
    btfss   STATUS,Z
    goto    _NEXT_LOOP0
    ;comf  MOVEcmd+0,f
dec_H:			;���� ������� ���� ��������� � 0xFF, ������ �� �������� �������
    movlw   0x0
    bcf	    STATUS,Z
    xorwf   MOVEcmd+1,W
    btfss   STATUS,Z
    goto    LoadLowByte
    bsf	    work_done,0 ;���� � ������� ����� 0, ���� ��������. ������� ������� � ��������� ����
    clrf     MOVEcmd+0
    clrf    MOVEcmd+1
    bcf	    INTCON,T0IF
    bcf	    Flags, 1
    goto    rollback
LoadLowByte:		;���� � ������� ����� ����� �������� �� 0, ������ �� ���� 1
    decf    MOVEcmd+1,f
    decf    MOVEcmd+0,f
    
_NEXT_LOOP0:
    decf    MOVEcmd+0,f
_NEXT_LOOP:		;��������� ����, �������/�������� ����� �� ����������� � ������� ���� ���������� TMR1
    
    
    btfsc   Flags,7
    bsf	    PORTA,5
    btfss   Flags,7
    bcf	    PORTA,5
    bcf	    INTCON,T0IF
    goto    rollback
;-------------------------------------------------------------------------------
;    
;   ���� ����� 00
;    
;-------------------------------------------------------------------------------    			 
;------------------------------------------------------------------------------- 
rollback:			; ����� � ���������� � �������������� ���������
    

    
				 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                                 ;��� ������� ������������� ��������� ���������:
    movf        FSR,W            ;������ ����������� �������� FSR �
    movwf       FSR_prer         ;������� FSR_prer
    movf        FSR_osn,W        ;������ ����������� �������� FSR_osn �
    movwf       FSR              ;������� FSR
                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                                 ;��� ������� ���������� �������� PCLATH:
    movf        PCLATH_TEMP,W    ;�������������� ����������� �������� PCLATH
    movwf       PCLATH           ;
                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    swapf       STATUS_TEMP,W    ;�������������� ����������� �������� STATUS
    movwf       STATUS

    swapf       W_TEMP_,F         ;�������������� ����������� ������������ W
    swapf       W_TEMP_,W
    
    retfie
    #include "Memory.inc"