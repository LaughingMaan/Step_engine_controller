#include "p16f877a.inc"

#define XINPUT PORTA,1 ; датчик X
#define YINPUT PORTA,4 ; датчик Y
; выводы двигателя X    
#define TRX    PORTB,0
#define BLX    PORTB,1
#define BRX    PORTB,2
#define TLX    PORTB,3
; выводы двигателя Y    
#define TRY    PORTD,4
#define BLY    PORTD,5
#define BRY    PORTD,6
#define TLY    PORTD,7
; выводы ЦАП    
#define DACON  PORTE,0 ; вывод !SYNC
#define DACCLK PORTE,1 ; вывод SCLK
#define DACDIN PORTE,2 ; вывод DIN
    
#define CHANGE_LASER PORTC,0 ;светодиод замены лазера
; CONFIG
; __config 0xFF3A
    __CONFIG _FOSC_HS & _WDTE_OFF & _PWRTE_OFF & _BOREN_OFF & _LVP_OFF & _CPD_OFF & _WRT_OFF & _CP_OFF 
    
    org	    0x00
    goto    start	;обход подпрограммы прерывания
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
    bsf	    cal,0	;установка флагов калибровки
    bsf	    cal,1
    bsf	    cal,2
    bsf	    cal,3
    movlw   b'10001000' ;установка начального значения для ШД
    movwf   PORTD
    movwf   PORTB
    clrf    DAC_buffer+0 ;очистка буфера для ЦАП
    clrf    DAC_buffer+1
    bsf	    INTCON,	PEIE ;разрешим прерывания
    bsf	    INTCON, GIE
    bsf	    RCSTA,	CREN
    bsf	    RCSTA,  SPEN
    bsf	    ADCON0,2
    goto    USART_ready
;-------------------------------------------------------------------------------
;    
;   Узел графа 00
;    
;-------------------------------------------------------------------------------    
USART_ready:		;конец инициализации, ждем команд по USART

    errorlevel -302
    movlw   0x24	;отправим приглашение на ввод команд
    movwf   TXREG
    banksel TXSTA
    bsf	    TXSTA,TXEN
    banksel TXREG
    errorlevel +302
    bcf	    cal,3
    
USART_cycle:		;цикл ожидания приема байта по USART
    btfss   Flags, 2
    goto    USART_cycle
    btfsc   Flags, 0
    goto    Two_bytes
One_byte:		;был принят один байт
    ;0x20 - пробел
    ;0xD  - enter
    bcf	    STATUS,Z
    movf    buffer,W
    xorlw   0xD		;Enter является символом-терминатором для команды
    btfsc   STATUS, Z
    goto    Got_command ;Получен Enter, идем выполнять разбор полученных данных
    ;полученная команда не является Enter'ом, проверим какой по счету байт пришел
    bcf	    STATUS,Z    
    movf    counter,W
    xorlw   .1
    btfsc   STATUS,Z
    goto    Got_XYZF	;первый байт - выбор двигателя
    
    xorlw   .1 ^ .2
    btfsc   STATUS,Z
    goto    Got_move1	;второй байт - четвертый разряд количества шагов (X000)
    
    xorlw   .2 ^ .3
    btfsc   STATUS,Z
    goto    Got_move2	;третий байт - третий разряд количества шагов (0X00)
    
    xorlw   .3 ^ .4
    btfsc   STATUS,Z
    goto    Got_move3   ;четвертый байт -  второй разряд количества шагов (00X0)
    
    xorlw   .4 ^ .5
    btfsc   STATUS,Z
    goto    Got_move4   ;пятый байт - первый разряд количества шагов (000X)
    
    xorlw   .5 ^ .6
    btfsc   STATUS,Z
    goto    Got_dir	;шестой байт - выбор направления + или -
    
    xorlw   .6 ^ .7
    btfsc   STATUS,Z
    goto    Got_laser
    goto    Got_command	 ;седьмой байт - включить или выключить лазер
    
Got_XYZF:   ;первый байт запишем из буфера в регистр команд "как есть" 
    movf    buffer+0,W
    movwf   XYZFcmd
    goto    Next_byte
Got_move1:	;следующие четыре байта обработаем вычитая ASCII код символа
    movlw   0x30  ;чтобы получить число от 0 до 9 и запишем число во временный регистр
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
Got_dir:    ;шестой байт запишем "как есть"
    movf    buffer+0,W
    movwf   DIRcmd
    goto    Next_byte
Got_laser:  ;седьмой байт запишем "как есть"
    movf    buffer+0,W
    movwf   LASERcmd
    goto    Next_byte
Next_byte:  ;очистим буфер и флаги для приема следующего байта
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
Got_command: ;все команды получены (пришел Enter), очистим буфер и 
	     ;подготовимся к составлению задания для прерывания
    clrf    buffer+0
    clrf    buffer+1
    clrf    counter
    ;************************************************************************************************
    ;посчитаем количество шагов. счет выполняется умножением на константу с помощью сдвига и сложения
    ;рассчет сдвигов и сложений для умножения на 1000. 4 в качестве примера
    ; 4*1000 представим как:
    ; 4<<9 = 2048
    ; 4<<8 = 1024
    ; 4<<7 = 512
    ; 4<<6 = 256
    ; 4<<5 = 128
    ; 4<<3 = 32
    ; 2048+1024+512+256+128+32 = 4000 
    ;************************************************************************************************
    movlw   .5	;количество прогонов цикла
    movwf   temp
Step_counter1:  ;умножение на 1000
    movf    temp,W
    addlw   .4	;компенсация разницы между количеством прогонов цикла и степенью двойки
    movwf   counter
    movf    var+3,W
    movwf   buffer+0
    call    Calc ;вызовем функцию сдвига и сложения
    decfsz  temp,f
    goto    Step_counter1
    
    movlw   .3
    movwf   counter
    movf    var+3,W
    movwf   buffer+0
    call    Calc
    ;************************************************************************************************
    ; рассчет сдвигов и сложений для умножения на 100. 4 в качестве примера
    ; 4*100 представим как:
    ; 4<<6 = 256 
    ; 4<<5 = 128
    ; 4<<2 = 16
    ; 256+128+16 = 400
    ;************************************************************************************************
Step_counter2:	;умножение на 100
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
    ;рассчет сдвигов и сложений для умножения на 10. 4 в качестве примера
    ;4*10 представим как:
    ; 4<<5 = 32
    ; 4<<1 = 8
    ; 32 + 4 = 40
    ;************************************************************************************************
Step_counter3:	;умножение на 10
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
Step_counter4:	; сложение разряда единиц
    bcf	    STATUS,C
    movf    var+0,W
    addwf   MOVEcmd+0,f
    btfsc   STATUS,C
    incf    MOVEcmd+1,f

Work:
    ;************************************************************************************************
    ;выполним разбор полученных команд для определения направления движения двигателей
    ;X горизонталь
    ;Y вертикаль
    ;Z диагональ слева направо /
    ;F диагональ справа налево \
    ;+ или - определяют направление движения. для X/Y движение по часовой стрелке 
    ;либо против часовой стрелки
    ;для Z/F движение снизу вверх либо сверху вниз
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
Detect_direction:	;определение + или -
    bcf	    STATUS,Z
    movlw   '+'
    xorwf   DIRcmd,W
    btfsc   STATUS,Z
    goto    Plus_setup	;получен +, выполним переход

Minus_setup:		; при Z(10) 11 / при F(11) 01 \
    bsf	    Flags,5
    btfss   Flags,3
    bsf	    Flags,6
    goto    Detect_laser
Plus_setup:		;при Z(10) 00 / при F(11) 10 \ 
    btfsc   Flags,3
    bsf	    Flags,6
Detect_laser:		;если получен Y - лазер включить, если что-либо другое - лазер выключить
    bcf	    STATUS,Z
    movlw   'Y'
    xorwf   LASERcmd,W
    btfsc   STATUS,Z
    bsf	    Flags,7	;установка флага лазер ВКЛ
    bsf	    Flags,1	;установка флага разрешения отработки команды
   
Wait:			;цикл ожидания отработки задания
    btfss   work_done,0
    goto    Wait
    bcf	    PORTA,5
    errorlevel -302
    banksel TXSTA
    bsf	    TXSTA,TXEN
    banksel TXREG
    movlw   0x24	;команда выполнена, сообщим о ожидании новой
    movwf   TXREG
    errorlevel +302
			;очистим все флаги и буферы для приема новой команды
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
;  Конец узла графа 00
;    
;-------------------------------------------------------------------------------    
;*******************************************************************************
;
;       ФУНКЦИИ
;
;*******************************************************************************    
    ORG     0x0400   

;*******************************************************************************
;       Calc
;-------------------------------------------------------------------------------
; Выполняет сдвиг и сложение чисел
; На входе:
;       buffer+0 - число от 0 до 9
;	counter  - счетчик сдвигов
; На выходе:
;       MOVEcmd - результат операций (Word)
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