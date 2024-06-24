;*******************************************************************************
;
;       Подпрограмма обработки прерываний
;
;*******************************************************************************

Interrupt:
    movwf       W_TEMP_          ;запись содержимого аккумулятора W в 
                                 ;регистр W_TEMP
    swapf       STATUS,W         ;обменять местами полубайты регистра STATUS и
                                 ;с записью в W
    clrf        STATUS           ;очистка регистра STATUS, выбрать 0-й банк
    movwf       STATUS_TEMP      ;запись содержимого аккумулятора W в
                                 ;регистр W_TEMP

                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                                 ;в случае если размер кода превышает 2048 слов:
    movf        PCLATH,W         ;запись содержимого регистра PCLATH в
    movwf       PCLATH_TEMP      ;регистр PCLATH_TEMP
    clrf        PCLATH           ;очистка регистра PCLATH
                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                                 ;при использовании косвенной адресации:
    movf        FSR,W            ;запись содержимого регистра FSR в
    movwf       FSR_osn          ;регистр FSR_osn
    movf        FSR_prer,W       ;запись содержимого регистра FSR_prer в
    movwf       FSR              ;регистр FSR
                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
				 
;-------------------------------------------------------------------------------
TMR0_check:
    btfsc   INTCON,T0IF		;было ли прерывание от TMR0
    btfss   INTCON,T0IE		;разрешено ли прерывание
    goto    TMR0_skip
    btfsc   Flags,1
    goto    Command_move        ;есть задание, переходим к управлению ШД
    btfsc   cal,0
    goto    calX		;задание на калибровку двигателя X
    btfsc   cal,1
    goto    calY		;задание на калибровку двигателя Y
    btfsc   cal,2
    goto    calLaser		;задание на калибровку лазера
    btfsc   tmr,1
    bsf	    tmr,0
    bcf	    INTCON,T0IF		;сбросим флаг прерывания
    btfsc   tmr,1
    goto    calLaser
TMR0_skip:
TMR1_check:
    btfss   PIR1,TMR1IF         ;было ли прерывание от TMR1
    goto    TMR1_skip
    btfsc   tmr,1
    bsf	    tmr,0
    bcf	    PIR1,TMR1IF
    btfsc   tmr,1
    goto    calLaser
TMR1_skip:
ADC_check:
    btfss   PIR1,ADIF		;было ли прерывание от АЦП
    goto    TX_check
    btfsc   ADCON0,2		; установлен ли бит GO/-DONE
    goto    TX_check
    
    btfsc   cal,2		;если задание на калибровку лазера, установим 25% мощности
    goto    V1_24
    
    btfss   Flags,1
    goto    calLaser		;если задания на работу нет, запустим новое преобразование
V5_0:				;проверка на равенство 100% при рабочем режиме
    
    movf    ADRESH,W
    addlw   -.244
    addlw   -(.249 - .244 + 1)
    btfss   STATUS,C
    goto    calLaser		;если 90 < Wл < 95, запустим новое преобразование
    
    movf    ADRESH,W		;проверка если Wл < 89%
    addlw   -.0
    addlw   -(.243 - .0 + 1)
    btfss   STATUS,C
    goto    DAC_inc		;Wл < 89%, увеличим напряжение на ЦАП
    
    ;значение 95 < Wл < 100, уменьшим до рабочего режима
    goto    DAC_dec
    
V1_24: 
    ;проверка на равенство Wл = 25% при калибровке
    movlw   b'00111111'		;1.24V (Wл = 25%)
    xorwf   ADRESH,W
    btfsc   STATUS,Z
    goto    calEnd	      ;напряжение соответствует норме, завершаем калибровку
    
    ;проверка на Wл > 25%
    movf    ADRESH,W
    addlw   -.64	      ;нормируем значение
    addlw   -(.255 - .64 + 1) ;проверка на попадание в диапазон 26% <= Wл <= 100%
    btfss   STATUS,C
    goto    DAC_dec	      ;попали, нужно уменьшить напряжение
    ;повторим то же самое для диапазона 0% <= Wл <=24%
    movf    ADRESH,W
    addlw   -.0
    addlw   -(.62 - .0 + 1)
    btfss   STATUS,C
    goto    DAC_inc
    
    ;значение не попало ни в один диапазон, подождем следующего преобразования
    bcf	    PIR1,ADIF
    bcf	    INTCON,T0IF
    bsf	    ADCON0,2
    goto    rollback

DAC_inc:		    ;увелиим буффер для ЦАП на 1
    incf    DAC_buffer+0,f
    movf    DAC_buffer+0,W
    btfsc   STATUS,Z
    incf    DAC_buffer+1,f
    btfsc   DAC_buffer+1,4
    goto    DAC_dec
    goto    DAC_write
    
DAC_dec:		    ;уменьшим буфер для ЦАП на 1
    bcf	    CHANGE_LASER
    btfsc   DAC_buffer+1,4
    bsf	    CHANGE_LASER
    decf    DAC_buffer+0,f
    movlw   0xFF
    xorwf   DAC_buffer+0,W
    btfss   STATUS,Z
    goto    DAC_write
    decf    DAC_buffer+1,f
DAC_write:		    ;подготовим буфер к записи в ЦАП
    movlw   b'00001111'	    ;маска для избежания переполнения 
    andwf   DAC_buffer+1,f
    movlw   .4
    movwf   temp
    movf    DAC_buffer+0,W
    movwf   DAC_temp+0
    movf    DAC_buffer+1,W
    movwf   DAC_temp+1
    
    
DAC_rlf: ;приведем число xxxx0000 00000000 к виду 00000000 0000xxxx, так удобнее записывать
    rlf	    DAC_temp+0,f
    rlf	    DAC_temp+1,f
    decfsz  temp,f
    goto    DAC_rlf
    movlw   .12
    movwf   temp
    bcf	    DACON
DAC_Write_cycle:	    ;запишем получившееся число в ЦАП AD5310 
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
    bsf	    ADCON0,2 ;повторим преобразование для проверки получившегося значения на выходе ЦАП
    goto    rollback
    

calEnd: ;калибровка лазера завершена, обнулим бит калибровки лазера,запустим новое преобразование и выйдем из прерывания
    bcf	    cal,2
    bcf	    PIR1,ADIF
    bsf	    tmr,1
    goto    rollback
TX_check:		    ;проверка прерывания от передатчика USART
    btfss   PIR1,TXIF
    goto    RX_check
    btfsc   work_done,0	    ;если задание выполнено, отправим Welcome - символ
    goto    TX_send_ready   
    btfsc   cal,3	    ;если инициализация завершена, отправим Welcome- символ
    goto    TX_send_ready
    errorlevel -302
    banksel TXSTA
    bcf	    TXSTA,TXEN	    ;байт отправлен, закончим передачу
    banksel 0 
    goto    rollback
    
TX_send_ready:		    ;отправка Welcome-символа ($)
    movlw   0x24    
    movwf   TXREG
    goto    rollback
;-------------------------------------------------------------------------------
;    
;   Узел графа 01
;    
;-------------------------------------------------------------------------------    
calX:			    ;калибровка двигателя X пока не сработает датчик X = 0
    btfss   XINPUT
    bcf	    cal,0 
    bcf	    STATUS,C
    rlf	    PORTB,f
    btfsc   STATUS,C
    bsf	    PORTB,0
    bcf	    INTCON,T0IF
    goto    rollback

calY:			    ;калибровка двигателя Y пока не сработает датчик Y = 0
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
;   Конец узла 01
;    
;------------------------------------------------------------------------------- 
;-------------------------------------------------------------------------------
;    
;   Узел графа 03
;    
;-------------------------------------------------------------------------------
calLaser:		  ;запуск преобразования АЦП
    bcf	    PIR1,ADIF
    bsf	    tmr,1
    btfss   tmr,0         ;было ли прерывание от TMR1
    goto    rollback
    bcf	    tmr,0
    btfss   ADCON0,2
    bsf	    ADCON0,2
    bcf	    PIR1,TMR1IF
    goto    rollback
;-------------------------------------------------------------------------------
;    
;   Конец узла 03
;    
;-------------------------------------------------------------------------------    
RX_check:		;проверка прерывания от приемника USART
    btfsc   PIR1,RCIF	;было ли прерывание от приемника USART
    btfss   PIE1,RCIE   ;разрешено ли прерывание
    goto    RX_skip
    btfss   Flags,1	;выполняется ли работа
    goto    RX_read	;не выполняется, примем байт
    ;работа выполняется, пошлем отказ
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
    ;приемник включается установкой RCSTA,CREN в 1
    ;флаг RCIF сбрасывается при чтении из RCREG
    ;при переполнении буфера бит RCSTA,OERR (1) устанавливается в 1
    ;бит OERR очищается сбросом бита CREN (запрет приема)
    ;бит ошибки кадра RCSTA,FERR устанавливается если не обнаружен стоповый бит
    ;рекомендуется сначала читать RCSTA затем RCREG чтобы не потерять 9 бит и FERR
RX_read:        ;проверка флагов USART
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
NO_overflow_err: ;не было переполнения буфера USART, читаем байт во временный буффер
    movf    RCREG,W
    movwf   buffer+0
    bcf	    STATUS,Z
    xorlw   0x20	;если принят пробел (0x20) - игнорируем
    btfsc   STATUS,Z
    goto    delimitter
    bsf	    Flags,2
    incf    counter,f
    goto    rollback
delimitter:		;принят пробел, очистим буфер
    clrf    buffer+0
    goto    rollback
RX_skip:
    goto    rollback
;-------------------------------------------------------------------------------
;    
;   Узел графа 02
;    
;-------------------------------------------------------------------------------    
Command_move:		;отработка команд ШД
    btfsc   Flags,4
    goto    Both	;нужно двигать оба двигателя
    btfss   Flags,5
    goto    Solo_Plus	;Нужно двигать один двигатель по часовой
Solo_minus:		;поворот двигателя X против часовой стрелки, сюда попадаем если ни одно из условий выше не выполнено
    btfsc   Flags,3	;если Флаг двигателя Y установлен, выполним переход, иначе обрабатываем вращение ШД X 
    goto    Solo_minusY	
    bcf	    STATUS,C
    rrf	    PORTB,f
    btfsc   STATUS,C
    bsf	    PORTB,7
    goto    Count
    
Solo_minusY:		;обработка движения ШД Y против часовой стрелки
    bcf	    STATUS,C
    rrf	    PORTD,f
    btfsc   STATUS,C
    bsf	    PORTD,7
    goto    Count
    
    
Solo_Plus:		;поворот двигателя X по часовой стрелке
    btfsc   Flags,3
    goto    SoloPlusY
    bcf	    STATUS,C
    rlf	    PORTB,f
    btfsc   STATUS,C
    bsf	    PORTB,0
    goto    Count
SoloPlusY:		;поворот двигателя Y по часовой стрелке
    bcf	    STATUS,C
    rlf	    PORTD,f
    btfsc   STATUS,C
    bsf	    PORTD,0
    goto    Count
Both:			;поворот двигателей X и Y
    movf    Flags,W
    andlw   b'01100000' ;маска для исключения лишних бит
    bcf	    STATUS,Z
    xorlw   b'00000000' ;если задание Z +
    btfsc   STATUS,Z
    goto    Plus_both
    xorlw   b'00000000' ^ b'01100000' ;если задание Z -
    btfsc   STATUS,Z
    goto    Minus_both
    xorlw   b'01100000' ^ b'00100000' ;если задание F +
    btfsc   STATUS,Z
    goto    XPlusYMinus
    xorlw   b'00100000' ^ b'01000000' ;если задание F - 
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
    
Count:			;двухбайтный счетчик оставшихся шагов
    
    movlw   0x00 
    bcf	    STATUS,Z
    xorwf   MOVEcmd+0,W
    btfss   STATUS,Z
    goto    _NEXT_LOOP0
    ;comf  MOVEcmd+0,f
dec_H:			;если младший байт обратился в 0xFF, вычтем из старшего единицу
    movlw   0x0
    bcf	    STATUS,Z
    xorwf   MOVEcmd+1,W
    btfss   STATUS,Z
    goto    LoadLowByte
    bsf	    work_done,0 ;если в старшем байте 0, счет закончен. очистим счетчик и установим флаг
    clrf     MOVEcmd+0
    clrf    MOVEcmd+1
    bcf	    INTCON,T0IF
    bcf	    Flags, 1
    goto    rollback
LoadLowByte:		;если в старшем байте число отличное от 0, вычтем из него 1
    decf    MOVEcmd+1,f
    decf    MOVEcmd+0,f
    
_NEXT_LOOP0:
    decf    MOVEcmd+0,f
_NEXT_LOOP:		;следующий цикл, включим/выключим лазер по потребности и сбросим флаг прерывания TMR1
    
    
    btfsc   Flags,7
    bsf	    PORTA,5
    btfss   Flags,7
    bcf	    PORTA,5
    bcf	    INTCON,T0IF
    goto    rollback
;-------------------------------------------------------------------------------
;    
;   Узел графа 00
;    
;-------------------------------------------------------------------------------    			 
;------------------------------------------------------------------------------- 
rollback:			; выход и прерывания и восстановление контекста
    

    
				 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                                 ;при условии использования косвенной адресации:
    movf        FSR,W            ;запись содержимого регистра FSR в
    movwf       FSR_prer         ;регистр FSR_prer
    movf        FSR_osn,W        ;запись содержимого регистра FSR_osn в
    movwf       FSR              ;регистр FSR
                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                                 ;при условии сохранения регистра PCLATH:
    movf        PCLATH_TEMP,W    ;восстановление содержимого регистра PCLATH
    movwf       PCLATH           ;
                                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    swapf       STATUS_TEMP,W    ;восстановление содержимого регистра STATUS
    movwf       STATUS

    swapf       W_TEMP_,F         ;восстановление содержимого аккумулятора W
    swapf       W_TEMP_,W
    
    retfie
    #include "Memory.inc"