
_interrupt:
	MOVWF      R15+0
	SWAPF      STATUS+0, 0
	CLRF       STATUS+0
	MOVWF      ___saveSTATUS+0
	MOVF       PCLATH+0, 0
	MOVWF      ___savePCLATH+0
	CLRF       PCLATH+0

;MyProject.c,17 :: 		void interrupt(void){
;MyProject.c,18 :: 		if(INTCON & 0x04){  // Check Timer0 overflow
	BTFSS      INTCON+0, 2
	GOTO       L_interrupt0
;MyProject.c,19 :: 		TMR0 = 231;     // Reload Timer0 for 100 us
	MOVLW      231
	MOVWF      TMR0+0
;MyProject.c,22 :: 		if (delayCounter > 0) { delayCounter--; }
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      _delayCounter+1, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__interrupt101
	MOVF       _delayCounter+0, 0
	SUBLW      0
L__interrupt101:
	BTFSC      STATUS+0, 0
	GOTO       L_interrupt1
	MOVLW      1
	SUBWF      _delayCounter+0, 1
	BTFSS      STATUS+0, 0
	DECF       _delayCounter+1, 1
L_interrupt1:
;MyProject.c,25 :: 		timerTick++;
	INCF       _timerTick+0, 1
	BTFSC      STATUS+0, 2
	INCF       _timerTick+1, 1
;MyProject.c,26 :: 		if (timerTick < servoPulseWidth){
	MOVLW      128
	XORWF      _timerTick+1, 0
	MOVWF      R0+0
	MOVLW      128
	XORWF      _servoPulseWidth+1, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__interrupt102
	MOVF       _servoPulseWidth+0, 0
	SUBWF      _timerTick+0, 0
L__interrupt102:
	BTFSC      STATUS+0, 0
	GOTO       L_interrupt2
;MyProject.c,27 :: 		PORTD = PORTD | 0x01;    // Set servo HIGH
	BSF        PORTD+0, 0
;MyProject.c,28 :: 		} else{
	GOTO       L_interrupt3
L_interrupt2:
;MyProject.c,29 :: 		PORTD = PORTD & 0b11111110; // Set servo LOW
	MOVLW      254
	ANDWF      PORTD+0, 1
;MyProject.c,30 :: 		}
L_interrupt3:
;MyProject.c,33 :: 		if (timerTick >= 200){
	MOVLW      128
	XORWF      _timerTick+1, 0
	MOVWF      R0+0
	MOVLW      128
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__interrupt103
	MOVLW      200
	SUBWF      _timerTick+0, 0
L__interrupt103:
	BTFSS      STATUS+0, 0
	GOTO       L_interrupt4
;MyProject.c,34 :: 		timerTick = 0;
	CLRF       _timerTick+0
	CLRF       _timerTick+1
;MyProject.c,35 :: 		}
L_interrupt4:
;MyProject.c,37 :: 		INTCON &= 0xFB; // Clear Timer0 interrupt flag
	MOVLW      251
	ANDWF      INTCON+0, 1
;MyProject.c,38 :: 		}
L_interrupt0:
;MyProject.c,39 :: 		}
L_end_interrupt:
L__interrupt100:
	MOVF       ___savePCLATH+0, 0
	MOVWF      PCLATH+0
	SWAPF      ___saveSTATUS+0, 0
	MOVWF      STATUS+0
	SWAPF      R15+0, 1
	SWAPF      R15+0, 0
	RETFIE
; end of _interrupt

_delay_ms_T0:

;MyProject.c,43 :: 		void delay_ms_T0(int ms){
;MyProject.c,44 :: 		delayCounter = ms * 10;
	MOVF       FARG_delay_ms_T0_ms+0, 0
	MOVWF      R0+0
	MOVF       FARG_delay_ms_T0_ms+1, 0
	MOVWF      R0+1
	MOVLW      10
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	CALL       _Mul_16X16_U+0
	MOVF       R0+0, 0
	MOVWF      _delayCounter+0
	MOVF       R0+1, 0
	MOVWF      _delayCounter+1
;MyProject.c,45 :: 		while(delayCounter > 0);
L_delay_ms_T05:
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      _delayCounter+1, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__delay_ms_T0105
	MOVF       _delayCounter+0, 0
	SUBLW      0
L__delay_ms_T0105:
	BTFSC      STATUS+0, 0
	GOTO       L_delay_ms_T06
	GOTO       L_delay_ms_T05
L_delay_ms_T06:
;MyProject.c,46 :: 		}
L_end_delay_ms_T0:
	RETURN
; end of _delay_ms_T0

_delay_us_T0:

;MyProject.c,48 :: 		void delay_us_T0(int us){
;MyProject.c,49 :: 		delayCounter = us / 100;
	MOVLW      100
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVF       FARG_delay_us_T0_us+0, 0
	MOVWF      R0+0
	MOVF       FARG_delay_us_T0_us+1, 0
	MOVWF      R0+1
	CALL       _Div_16x16_S+0
	MOVF       R0+0, 0
	MOVWF      _delayCounter+0
	MOVF       R0+1, 0
	MOVWF      _delayCounter+1
;MyProject.c,50 :: 		while(delayCounter > 0);
L_delay_us_T07:
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      _delayCounter+1, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__delay_us_T0107
	MOVF       _delayCounter+0, 0
	SUBLW      0
L__delay_us_T0107:
	BTFSC      STATUS+0, 0
	GOTO       L_delay_us_T08
	GOTO       L_delay_us_T07
L_delay_us_T08:
;MyProject.c,51 :: 		}
L_end_delay_us_T0:
	RETURN
; end of _delay_us_T0

_setServoAngle:

;MyProject.c,56 :: 		void setServoAngle(int angle){
;MyProject.c,57 :: 		if(angle > 180) angle = 180;
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      FARG_setServoAngle_angle+1, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__setServoAngle109
	MOVF       FARG_setServoAngle_angle+0, 0
	SUBLW      180
L__setServoAngle109:
	BTFSC      STATUS+0, 0
	GOTO       L_setServoAngle9
	MOVLW      180
	MOVWF      FARG_setServoAngle_angle+0
	CLRF       FARG_setServoAngle_angle+1
L_setServoAngle9:
;MyProject.c,58 :: 		servoPulseWidth = 10 + (angle * 10) / 180;
	MOVF       FARG_setServoAngle_angle+0, 0
	MOVWF      R0+0
	MOVF       FARG_setServoAngle_angle+1, 0
	MOVWF      R0+1
	MOVLW      10
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	CALL       _Mul_16X16_U+0
	MOVLW      180
	MOVWF      R4+0
	CLRF       R4+1
	CALL       _Div_16x16_S+0
	MOVF       R0+0, 0
	ADDLW      10
	MOVWF      _servoPulseWidth+0
	MOVLW      0
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDWF      R0+1, 0
	MOVWF      _servoPulseWidth+1
;MyProject.c,59 :: 		}
L_end_setServoAngle:
	RETURN
; end of _setServoAngle

_readDistanceCM:

;MyProject.c,69 :: 		int readDistanceCM(){
;MyProject.c,70 :: 		int distance = 0;
;MyProject.c,71 :: 		TMR1H = 0; TMR1L = 0;                  // Reset Timer1
	CLRF       TMR1H+0
	CLRF       TMR1L+0
;MyProject.c,72 :: 		PORTC |= 0b00010000;                    // Trigger HIGH
	BSF        PORTC+0, 4
;MyProject.c,73 :: 		delay_us_T0(10);                        // 10 µs pulse
	MOVLW      10
	MOVWF      FARG_delay_us_T0_us+0
	MOVLW      0
	MOVWF      FARG_delay_us_T0_us+1
	CALL       _delay_us_T0+0
;MyProject.c,74 :: 		PORTC &= 0b11101111;                    // Trigger LOW
	MOVLW      239
	ANDWF      PORTC+0, 1
;MyProject.c,76 :: 		while(!(PORTC & 0b00100000));          // Wait for echo start
L_readDistanceCM10:
	BTFSC      PORTC+0, 5
	GOTO       L_readDistanceCM11
	GOTO       L_readDistanceCM10
L_readDistanceCM11:
;MyProject.c,77 :: 		T1CON |= 0b00000001;                    // Start Timer
	BSF        T1CON+0, 0
;MyProject.c,78 :: 		while(PORTC & 0b00100000);             // Wait for echo end
L_readDistanceCM12:
	BTFSS      PORTC+0, 5
	GOTO       L_readDistanceCM13
	GOTO       L_readDistanceCM12
L_readDistanceCM13:
;MyProject.c,79 :: 		T1CON &= 0b11111110;                    // Stop Timer
	MOVLW      254
	ANDWF      T1CON+0, 1
;MyProject.c,81 :: 		distance = (TMR1L | (TMR1H << 8));
	MOVF       TMR1H+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       TMR1L+0, 0
	IORWF      R0+0, 1
	MOVLW      0
	IORWF      R0+1, 1
;MyProject.c,82 :: 		distance = distance / 58.82;           // Convert to cm
	CALL       _int2double+0
	MOVLW      174
	MOVWF      R4+0
	MOVLW      71
	MOVWF      R4+1
	MOVLW      107
	MOVWF      R4+2
	MOVLW      132
	MOVWF      R4+3
	CALL       _Div_32x32_FP+0
	CALL       _double2int+0
;MyProject.c,83 :: 		return distance;
;MyProject.c,84 :: 		}
L_end_readDistanceCM:
	RETURN
; end of _readDistanceCM

_PWM_init:

;MyProject.c,88 :: 		void PWM_init(){
;MyProject.c,89 :: 		T2CON = 0x07;      // Timer2 prescaler 1:16
	MOVLW      7
	MOVWF      T2CON+0
;MyProject.c,90 :: 		CCP1CON = 0x0C;    // PWM enable CCP1
	MOVLW      12
	MOVWF      CCP1CON+0
;MyProject.c,91 :: 		CCP2CON = 0x0C;    // PWM enable CCP2
	MOVLW      12
	MOVWF      CCP2CON+0
;MyProject.c,92 :: 		PR2 = 250;         // 2ms period
	MOVLW      250
	MOVWF      PR2+0
;MyProject.c,93 :: 		}
L_end_PWM_init:
	RETURN
; end of _PWM_init

_ADC_init:

;MyProject.c,96 :: 		void ADC_init(){
;MyProject.c,97 :: 		ADCON0 = 0x41; // ADC ON, channel 0, Fosc/16
	MOVLW      65
	MOVWF      ADCON0+0
;MyProject.c,98 :: 		ADCON1 = 0xCE; // RA0 analog, others digital, right aligned
	MOVLW      206
	MOVWF      ADCON1+0
;MyProject.c,99 :: 		}
L_end_ADC_init:
	RETURN
; end of _ADC_init

_ADC_read:

;MyProject.c,101 :: 		int ADC_read(){
;MyProject.c,102 :: 		ADCON0 |= 0x04;          // Start conversion
	BSF        ADCON0+0, 2
;MyProject.c,103 :: 		while(ADCON0 & 0x04);    // Wait until done
L_ADC_read14:
	BTFSS      ADCON0+0, 2
	GOTO       L_ADC_read15
	GOTO       L_ADC_read14
L_ADC_read15:
;MyProject.c,104 :: 		return (ADRESH << 8) | ADRESL;
	MOVF       ADRESH+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       ADRESL+0, 0
	IORWF      R0+0, 1
	MOVLW      0
	IORWF      R0+1, 1
;MyProject.c,105 :: 		}
L_end_ADC_read:
	RETURN
; end of _ADC_read

_setLeftMotorSpeed:

;MyProject.c,108 :: 		void setLeftMotorSpeed(int speed){
;MyProject.c,109 :: 		CCPR1L = speed; }
	MOVF       FARG_setLeftMotorSpeed_speed+0, 0
	MOVWF      CCPR1L+0
L_end_setLeftMotorSpeed:
	RETURN
; end of _setLeftMotorSpeed

_setRightMotorSpeed:

;MyProject.c,110 :: 		void setRightMotorSpeed(int speed){
;MyProject.c,111 :: 		CCPR2L = speed; }
	MOVF       FARG_setRightMotorSpeed_speed+0, 0
	MOVWF      CCPR2L+0
L_end_setRightMotorSpeed:
	RETURN
; end of _setRightMotorSpeed

_moveForward:

;MyProject.c,114 :: 		void moveForward(){
;MyProject.c,115 :: 		PORTB = (PORTB & 0b01011111) | 0b01010000;
	MOVLW      95
	ANDWF      PORTB+0, 0
	MOVWF      R0+0
	MOVLW      80
	IORWF      R0+0, 0
	MOVWF      PORTB+0
;MyProject.c,116 :: 		}
L_end_moveForward:
	RETURN
; end of _moveForward

_moveBackward:

;MyProject.c,117 :: 		void moveBackward(){
;MyProject.c,118 :: 		PORTB = (PORTB & 0b10101111) | 0b10100000;
	MOVLW      175
	ANDWF      PORTB+0, 0
	MOVWF      R0+0
	MOVLW      160
	IORWF      R0+0, 0
	MOVWF      PORTB+0
;MyProject.c,119 :: 		}
L_end_moveBackward:
	RETURN
; end of _moveBackward

_turnRight:

;MyProject.c,120 :: 		void turnRight(){
;MyProject.c,121 :: 		PORTB = (PORTB & 0b01101111) | 0b01100000;
	MOVLW      111
	ANDWF      PORTB+0, 0
	MOVWF      R0+0
	MOVLW      96
	IORWF      R0+0, 0
	MOVWF      PORTB+0
;MyProject.c,122 :: 		}
L_end_turnRight:
	RETURN
; end of _turnRight

_turnLeft:

;MyProject.c,123 :: 		void turnLeft(){
;MyProject.c,124 :: 		PORTB = (PORTB & 0b10011111) | 0b10010000;
	MOVLW      159
	ANDWF      PORTB+0, 0
	MOVWF      R0+0
	MOVLW      144
	IORWF      R0+0, 0
	MOVWF      PORTB+0
;MyProject.c,125 :: 		}
L_end_turnLeft:
	RETURN
; end of _turnLeft

_stopMotors:

;MyProject.c,126 :: 		void stopMotors(){
;MyProject.c,127 :: 		PORTB &= 0b00001111;
	MOVLW      15
	ANDWF      PORTB+0, 1
;MyProject.c,128 :: 		}
L_end_stopMotors:
	RETURN
; end of _stopMotors

_main:

;MyProject.c,132 :: 		void main(){
;MyProject.c,134 :: 		TRISA = 0x01;
	MOVLW      1
	MOVWF      TRISA+0
;MyProject.c,135 :: 		TRISB = 0X08;
	MOVLW      8
	MOVWF      TRISB+0
;MyProject.c,136 :: 		TRISC = 0X20;
	MOVLW      32
	MOVWF      TRISC+0
;MyProject.c,137 :: 		TRISD = 0XF4;
	MOVLW      244
	MOVWF      TRISD+0
;MyProject.c,139 :: 		PORTA = 0x00;
	CLRF       PORTA+0
;MyProject.c,140 :: 		PORTB = 0x00;
	CLRF       PORTB+0
;MyProject.c,141 :: 		PORTC = 0x00;
	CLRF       PORTC+0
;MyProject.c,142 :: 		PORTD = 0x00;
	CLRF       PORTD+0
;MyProject.c,145 :: 		OPTION_REG = 0b00000010; // Prescaler 1:8
	MOVLW      2
	MOVWF      OPTION_REG+0
;MyProject.c,146 :: 		INTCON = 0xA0;           // Enable TMR0 interrupt & global interrupt
	MOVLW      160
	MOVWF      INTCON+0
;MyProject.c,147 :: 		TMR0 = 231;
	MOVLW      231
	MOVWF      TMR0+0
;MyProject.c,149 :: 		T1CON = 0x10;            // Timer1 setup
	MOVLW      16
	MOVWF      T1CON+0
;MyProject.c,150 :: 		ADC_init();               // ADC init
	CALL       _ADC_init+0
;MyProject.c,151 :: 		PWM_init();               // PWM init
	CALL       _PWM_init+0
;MyProject.c,152 :: 		setServoAngle(0);
	CLRF       FARG_setServoAngle_angle+0
	CLRF       FARG_setServoAngle_angle+1
	CALL       _setServoAngle+0
;MyProject.c,153 :: 		operationStage=1;
	MOVLW      1
	MOVWF      _operationStage+0
	MOVLW      0
	MOVWF      _operationStage+1
;MyProject.c,154 :: 		lightFlag=0;
	CLRF       _lightFlag+0
	CLRF       _lightFlag+1
;MyProject.c,155 :: 		flagStage2=0;
	CLRF       _flagStage2+0
	CLRF       _flagStage2+1
;MyProject.c,156 :: 		flagStage3=0;
	CLRF       _flagStage3+0
	CLRF       _flagStage3+1
;MyProject.c,157 :: 		delay_ms_T0(3000);
	MOVLW      184
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      11
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,158 :: 		PORTB= PORTB |   0X01;
	BSF        PORTB+0, 0
;MyProject.c,159 :: 		while(1){
L_main16:
;MyProject.c,160 :: 		if(operationStage == 1){
	MOVLW      0
	XORWF      _operationStage+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main122
	MOVLW      1
	XORWF      _operationStage+0, 0
L__main122:
	BTFSS      STATUS+0, 2
	GOTO       L_main18
;MyProject.c,162 :: 		lightSensorValue = ADC_read();
	CALL       _ADC_read+0
	MOVF       R0+0, 0
	MOVWF      _lightSensorValue+0
	MOVF       R0+1, 0
	MOVWF      _lightSensorValue+1
;MyProject.c,164 :: 		if(lightSensorValue < 750){
	MOVLW      128
	XORWF      R0+1, 0
	MOVWF      R2+0
	MOVLW      128
	XORLW      2
	SUBWF      R2+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main123
	MOVLW      238
	SUBWF      R0+0, 0
L__main123:
	BTFSC      STATUS+0, 0
	GOTO       L_main19
;MyProject.c,165 :: 		PORTD |= 0b00000010;
	BSF        PORTD+0, 1
;MyProject.c,166 :: 		lightFlag = 1;
	MOVLW      1
	MOVWF      _lightFlag+0
	MOVLW      0
	MOVWF      _lightFlag+1
;MyProject.c,167 :: 		} else {
	GOTO       L_main20
L_main19:
;MyProject.c,168 :: 		PORTD &= 0b11111101;
	MOVLW      253
	ANDWF      PORTD+0, 1
;MyProject.c,169 :: 		if(lightFlag == 1) {
	MOVLW      0
	XORWF      _lightFlag+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main124
	MOVLW      1
	XORWF      _lightFlag+0, 0
L__main124:
	BTFSS      STATUS+0, 2
	GOTO       L_main21
;MyProject.c,170 :: 		operationStage = 2;
	MOVLW      2
	MOVWF      _operationStage+0
	MOVLW      0
	MOVWF      _operationStage+1
;MyProject.c,171 :: 		}
L_main21:
;MyProject.c,172 :: 		}
L_main20:
;MyProject.c,175 :: 		if(!(PORTD & 0b00100000) && !(PORTD & 0b00010000)){
	BTFSC      PORTD+0, 5
	GOTO       L_main24
	BTFSC      PORTD+0, 4
	GOTO       L_main24
L__main98:
;MyProject.c,176 :: 		moveForward(); setLeftMotorSpeed(75); setRightMotorSpeed(75);
	CALL       _moveForward+0
	MOVLW      75
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      75
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,177 :: 		} else if(!(PORTD & 0b00100000) && (PORTD & 0b00010000)){
	GOTO       L_main25
L_main24:
	BTFSC      PORTD+0, 5
	GOTO       L_main28
	BTFSS      PORTD+0, 4
	GOTO       L_main28
L__main97:
;MyProject.c,178 :: 		moveForward(); setLeftMotorSpeed(0); setRightMotorSpeed(75);
	CALL       _moveForward+0
	CLRF       FARG_setLeftMotorSpeed_speed+0
	CLRF       FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      75
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,179 :: 		} else if((PORTD & 0b00100000) && !(PORTD & 0b00010000)){
	GOTO       L_main29
L_main28:
	BTFSS      PORTD+0, 5
	GOTO       L_main32
	BTFSC      PORTD+0, 4
	GOTO       L_main32
L__main96:
;MyProject.c,180 :: 		moveForward(); setLeftMotorSpeed(75); setRightMotorSpeed(0);
	CALL       _moveForward+0
	MOVLW      75
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	CLRF       FARG_setRightMotorSpeed_speed+0
	CLRF       FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,181 :: 		} else if((PORTD & 0b00100000) && (PORTD & 0b00010000)){
	GOTO       L_main33
L_main32:
	BTFSS      PORTD+0, 5
	GOTO       L_main36
	BTFSS      PORTD+0, 4
	GOTO       L_main36
L__main95:
;MyProject.c,182 :: 		stopMotors(); delay_ms_T0(1000);
	CALL       _stopMotors+0
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,183 :: 		moveForward(); setLeftMotorSpeed(75); setRightMotorSpeed(75);
	CALL       _moveForward+0
	MOVLW      75
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      75
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,184 :: 		delay_ms_T0(300); stopMotors(); delay_ms_T0(1000);
	MOVLW      44
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      1
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
	CALL       _stopMotors+0
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,185 :: 		turnLeft(); setLeftMotorSpeed(75); setRightMotorSpeed(75);
	CALL       _turnLeft+0
	MOVLW      75
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      75
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,186 :: 		while(!(PORTD & 0b00010000));
L_main37:
	BTFSC      PORTD+0, 4
	GOTO       L_main38
	GOTO       L_main37
L_main38:
;MyProject.c,187 :: 		stopMotors(); delay_ms_T0(1000);
	CALL       _stopMotors+0
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,188 :: 		}
L_main36:
L_main33:
L_main29:
L_main25:
;MyProject.c,190 :: 		} else {
	GOTO       L_main39
L_main18:
;MyProject.c,192 :: 		if(flagStage2 == 0){
	MOVLW      0
	XORWF      _flagStage2+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main125
	MOVLW      0
	XORWF      _flagStage2+0, 0
L__main125:
	BTFSS      STATUS+0, 2
	GOTO       L_main40
;MyProject.c,193 :: 		stopMotors(); delay_ms_T0(1000);
	CALL       _stopMotors+0
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,194 :: 		moveForward(); setLeftMotorSpeed(70); setRightMotorSpeed(70);
	CALL       _moveForward+0
	MOVLW      70
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      70
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,195 :: 		delay_ms_T0(400); stopMotors(); delay_ms_T0(1000);
	MOVLW      144
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      1
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
	CALL       _stopMotors+0
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,196 :: 		turnRight(); setLeftMotorSpeed(70); setRightMotorSpeed(70);
	CALL       _turnRight+0
	MOVLW      70
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      70
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,197 :: 		delay_ms_T0(300); stopMotors(); delay_ms_T0(1000);
	MOVLW      44
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      1
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
	CALL       _stopMotors+0
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,198 :: 		flagStage2 = 1;
	MOVLW      1
	MOVWF      _flagStage2+0
	MOVLW      0
	MOVWF      _flagStage2+1
;MyProject.c,199 :: 		} else {
	GOTO       L_main41
L_main40:
;MyProject.c,200 :: 		if(flagStage3 == 0){
	MOVLW      0
	XORWF      _flagStage3+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main126
	MOVLW      0
	XORWF      _flagStage3+0, 0
L__main126:
	BTFSS      STATUS+0, 2
	GOTO       L_main42
;MyProject.c,201 :: 		if((PORTD & 0b00100000) || (PORTD & 0b00010000)){
	BTFSC      PORTD+0, 5
	GOTO       L__main94
	BTFSC      PORTD+0, 4
	GOTO       L__main94
	GOTO       L_main45
L__main94:
;MyProject.c,202 :: 		flagStage3 = 1;
	MOVLW      1
	MOVWF      _flagStage3+0
	MOVLW      0
	MOVWF      _flagStage3+1
;MyProject.c,203 :: 		} else { flagStage3 = 0; }
	GOTO       L_main46
L_main45:
	CLRF       _flagStage3+0
	CLRF       _flagStage3+1
L_main46:
;MyProject.c,205 :: 		measuredDistance = readDistanceCM();
	CALL       _readDistanceCM+0
	MOVF       R0+0, 0
	MOVWF      _measuredDistance+0
	MOVF       R0+1, 0
	MOVWF      _measuredDistance+1
;MyProject.c,208 :: 		if((measuredDistance < 25) && (PORTC&0B10000000) && (PORTC&0B01000000)){
	MOVLW      128
	XORWF      R0+1, 0
	MOVWF      R2+0
	MOVLW      128
	SUBWF      R2+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main127
	MOVLW      25
	SUBWF      R0+0, 0
L__main127:
	BTFSC      STATUS+0, 0
	GOTO       L_main49
	BTFSS      PORTC+0, 7
	GOTO       L_main49
	BTFSS      PORTC+0, 6
	GOTO       L_main49
L__main93:
;MyProject.c,209 :: 		stopMotors(); delay_ms_T0(1000);
	CALL       _stopMotors+0
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,210 :: 		turnRight();
	CALL       _turnRight+0
;MyProject.c,211 :: 		setLeftMotorSpeed(70);
	MOVLW      70
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
;MyProject.c,212 :: 		setRightMotorSpeed(70);
	MOVLW      70
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,213 :: 		delay_ms_T0(300);
	MOVLW      44
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      1
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,214 :: 		stopMotors();
	CALL       _stopMotors+0
;MyProject.c,215 :: 		delay_ms_T0(1000);
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,216 :: 		}else if ((measuredDistance<20) && (PORTC&0B10000000) &&  !(PORTC&0B01000000)){
	GOTO       L_main50
L_main49:
	MOVLW      128
	XORWF      _measuredDistance+1, 0
	MOVWF      R0+0
	MOVLW      128
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main128
	MOVLW      20
	SUBWF      _measuredDistance+0, 0
L__main128:
	BTFSC      STATUS+0, 0
	GOTO       L_main53
	BTFSS      PORTC+0, 7
	GOTO       L_main53
	BTFSC      PORTC+0, 6
	GOTO       L_main53
L__main92:
;MyProject.c,217 :: 		stopMotors();
	CALL       _stopMotors+0
;MyProject.c,218 :: 		delay_ms_T0(1000);
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,219 :: 		moveBackward();
	CALL       _moveBackward+0
;MyProject.c,220 :: 		setLeftMotorSpeed(80);
	MOVLW      80
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
;MyProject.c,221 :: 		setRightMotorSpeed(80);
	MOVLW      80
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,222 :: 		delay_ms_T0(250);
	MOVLW      250
	MOVWF      FARG_delay_ms_T0_ms+0
	CLRF       FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,223 :: 		stopMotors();
	CALL       _stopMotors+0
;MyProject.c,224 :: 		delay_ms_T0(1000);
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,225 :: 		turnLeft();
	CALL       _turnLeft+0
;MyProject.c,226 :: 		setLeftMotorSpeed(80);
	MOVLW      80
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
;MyProject.c,227 :: 		setRightMotorSpeed(80);
	MOVLW      80
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,228 :: 		delay_ms_T0(100);
	MOVLW      100
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      0
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,229 :: 		stopMotors();
	CALL       _stopMotors+0
;MyProject.c,230 :: 		delay_ms_T0(1000);
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,232 :: 		}else if ((measuredDistance<20) && !(PORTC&0B10000000) &&  (PORTC&0B01000000) ) {
	GOTO       L_main54
L_main53:
	MOVLW      128
	XORWF      _measuredDistance+1, 0
	MOVWF      R0+0
	MOVLW      128
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main129
	MOVLW      20
	SUBWF      _measuredDistance+0, 0
L__main129:
	BTFSC      STATUS+0, 0
	GOTO       L_main57
	BTFSC      PORTC+0, 7
	GOTO       L_main57
	BTFSS      PORTC+0, 6
	GOTO       L_main57
L__main91:
;MyProject.c,233 :: 		stopMotors();
	CALL       _stopMotors+0
;MyProject.c,234 :: 		delay_ms_T0(500);
	MOVLW      244
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      1
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,235 :: 		moveBackward();
	CALL       _moveBackward+0
;MyProject.c,236 :: 		setLeftMotorSpeed(80);
	MOVLW      80
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
;MyProject.c,237 :: 		setRightMotorSpeed(80);
	MOVLW      80
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,238 :: 		delay_ms_T0(200);
	MOVLW      200
	MOVWF      FARG_delay_ms_T0_ms+0
	CLRF       FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,239 :: 		stopMotors();
	CALL       _stopMotors+0
;MyProject.c,240 :: 		delay_ms_T0(500);
	MOVLW      244
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      1
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,241 :: 		turnRight();
	CALL       _turnRight+0
;MyProject.c,242 :: 		setLeftMotorSpeed(80);
	MOVLW      80
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
;MyProject.c,243 :: 		setRightMotorSpeed(80);
	MOVLW      80
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,244 :: 		delay_ms_T0(100);
	MOVLW      100
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      0
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,245 :: 		stopMotors();
	CALL       _stopMotors+0
;MyProject.c,246 :: 		delay_ms_T0(1000);
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,247 :: 		}else if ((measuredDistance>=20) && (PORTC&0B10000000) &&  !(PORTC&0B01000000)){  //001
	GOTO       L_main58
L_main57:
	MOVLW      128
	XORWF      _measuredDistance+1, 0
	MOVWF      R0+0
	MOVLW      128
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main130
	MOVLW      20
	SUBWF      _measuredDistance+0, 0
L__main130:
	BTFSS      STATUS+0, 0
	GOTO       L_main61
	BTFSS      PORTC+0, 7
	GOTO       L_main61
	BTFSC      PORTC+0, 6
	GOTO       L_main61
L__main90:
;MyProject.c,248 :: 		moveForward(); setLeftMotorSpeed(70); setRightMotorSpeed(80);
	CALL       _moveForward+0
	MOVLW      70
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      80
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,250 :: 		}  else if ((measuredDistance>=20) && !(PORTC&0B10000000) &&  (PORTC&0B01000000)){     //010
	GOTO       L_main62
L_main61:
	MOVLW      128
	XORWF      _measuredDistance+1, 0
	MOVWF      R0+0
	MOVLW      128
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main131
	MOVLW      20
	SUBWF      _measuredDistance+0, 0
L__main131:
	BTFSS      STATUS+0, 0
	GOTO       L_main65
	BTFSC      PORTC+0, 7
	GOTO       L_main65
	BTFSS      PORTC+0, 6
	GOTO       L_main65
L__main89:
;MyProject.c,251 :: 		moveForward(); setLeftMotorSpeed(80); setRightMotorSpeed(70);
	CALL       _moveForward+0
	MOVLW      80
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      70
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,252 :: 		}
	GOTO       L_main66
L_main65:
;MyProject.c,254 :: 		moveForward(); setLeftMotorSpeed(70); setRightMotorSpeed(70);
	CALL       _moveForward+0
	MOVLW      70
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      70
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,257 :: 		}
L_main66:
L_main62:
L_main58:
L_main54:
L_main50:
;MyProject.c,260 :: 		} else {
	GOTO       L_main67
L_main42:
;MyProject.c,262 :: 		if(!(PORTD & 0b00100000) && !(PORTD & 0b00010000)){
	BTFSC      PORTD+0, 5
	GOTO       L_main70
	BTFSC      PORTD+0, 4
	GOTO       L_main70
L__main88:
;MyProject.c,263 :: 		moveForward(); setLeftMotorSpeed(60); setRightMotorSpeed(60);
	CALL       _moveForward+0
	MOVLW      60
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      60
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,264 :: 		} else if(!(PORTD & 0b00100000) && (PORTD & 0b00010000)){
	GOTO       L_main71
L_main70:
	BTFSC      PORTD+0, 5
	GOTO       L_main74
	BTFSS      PORTD+0, 4
	GOTO       L_main74
L__main87:
;MyProject.c,265 :: 		moveForward(); setLeftMotorSpeed(0); setRightMotorSpeed(60);
	CALL       _moveForward+0
	CLRF       FARG_setLeftMotorSpeed_speed+0
	CLRF       FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      60
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,266 :: 		} else if((PORTD & 0b00100000) && !(PORTD & 0b00010000)){
	GOTO       L_main75
L_main74:
	BTFSS      PORTD+0, 5
	GOTO       L_main78
	BTFSC      PORTD+0, 4
	GOTO       L_main78
L__main86:
;MyProject.c,267 :: 		moveForward(); setLeftMotorSpeed(60); setRightMotorSpeed(0);
	CALL       _moveForward+0
	MOVLW      60
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	CLRF       FARG_setRightMotorSpeed_speed+0
	CLRF       FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,268 :: 		} else if((PORTD & 0b00100000) && (PORTD & 0b00010000)){
	GOTO       L_main79
L_main78:
	BTFSS      PORTD+0, 5
	GOTO       L_main82
	BTFSS      PORTD+0, 4
	GOTO       L_main82
L__main85:
;MyProject.c,269 :: 		stopMotors(); delay_ms_T0(1000);
	CALL       _stopMotors+0
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,270 :: 		moveForward(); setLeftMotorSpeed(80); setRightMotorSpeed(80);
	CALL       _moveForward+0
	MOVLW      80
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      80
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,271 :: 		delay_ms_T0(350); stopMotors();
	MOVLW      94
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      1
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
	CALL       _stopMotors+0
;MyProject.c,272 :: 		stopMotors(); delay_ms_T0(1000);
	CALL       _stopMotors+0
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,273 :: 		turnRight(); setLeftMotorSpeed(80); setRightMotorSpeed(80);
	CALL       _turnRight+0
	MOVLW      80
	MOVWF      FARG_setLeftMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setLeftMotorSpeed_speed+1
	CALL       _setLeftMotorSpeed+0
	MOVLW      80
	MOVWF      FARG_setRightMotorSpeed_speed+0
	MOVLW      0
	MOVWF      FARG_setRightMotorSpeed_speed+1
	CALL       _setRightMotorSpeed+0
;MyProject.c,274 :: 		delay_ms_T0(300);
	MOVLW      44
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      1
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,275 :: 		stopMotors();
	CALL       _stopMotors+0
;MyProject.c,276 :: 		delay_ms_T0(1000);
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,278 :: 		setServoAngle(90);
	MOVLW      90
	MOVWF      FARG_setServoAngle_angle+0
	MOVLW      0
	MOVWF      FARG_setServoAngle_angle+1
	CALL       _setServoAngle+0
;MyProject.c,279 :: 		delay_ms_T0(1000);
	MOVLW      232
	MOVWF      FARG_delay_ms_T0_ms+0
	MOVLW      3
	MOVWF      FARG_delay_ms_T0_ms+1
	CALL       _delay_ms_T0+0
;MyProject.c,281 :: 		while(1){
L_main83:
;MyProject.c,282 :: 		stopMotors();
	CALL       _stopMotors+0
;MyProject.c,283 :: 		PORTB = PORTB & 0XFE;
	MOVLW      254
	ANDWF      PORTB+0, 1
;MyProject.c,284 :: 		}
	GOTO       L_main83
;MyProject.c,285 :: 		}
L_main82:
L_main79:
L_main75:
L_main71:
;MyProject.c,286 :: 		}
L_main67:
;MyProject.c,287 :: 		}
L_main41:
;MyProject.c,288 :: 		}
L_main39:
;MyProject.c,289 :: 		}
	GOTO       L_main16
;MyProject.c,290 :: 		}
L_end_main:
	GOTO       $+0
; end of _main
