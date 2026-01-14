
//          GLOBAL VARIABLES

int servoAngle;                 // Desired servo angle (0-180)
int flagStage2 = 0;             // Flag for stage 2 initialization
int flagStage3 = 0;             // Flag for stage 3 check
int timerTick = 0;              // Timer0 tick counter
int servoPulseWidth = 15;       // Servo pulse width (10–20) default center
int delayCounter = 0;           // Software delay counter
int measuredDistance;           // Distance from ultrasonic sensor
int lightSensorValue;           // ADC value from light sensor
int operationStage = 1;         // Current stage of robot
int lightFlag = 0;              // Flag for light detection


//          TIMER0 Interrupt
void interrupt(void){
    if(INTCON & 0x04){  // Check Timer0 overflow
        TMR0 = 231;     // Reload Timer0 for 100 us

        // Handle software delay
        if (delayCounter > 0) { delayCounter--; }

        // Servo PWM generation
        timerTick++;
        if (timerTick < servoPulseWidth){
            PORTD = PORTD | 0x01;    // Set servo HIGH
        } else{
            PORTD = PORTD & 0b11111110; // Set servo LOW
        }

        // Reset tick every 20 ms (200 * 100us)
        if (timerTick >= 200){
            timerTick = 0;
        }

        INTCON &= 0xFB; // Clear Timer0 interrupt flag
    }
}

//       SOFTWARE DELAYS

void delay_ms_T0(int ms){
    delayCounter = ms * 10;
    while(delayCounter > 0);
}

void delay_us_T0(int us){
    delayCounter = us / 100;
    while(delayCounter > 0);
}


//       SERVO CONTROL

void setServoAngle(int angle){
    if(angle > 180) angle = 180;
    servoPulseWidth = 10 + (angle * 10) / 180;
}


//        ADC (LIGHT SENSOR)




//      ULTRASONIC SENSOR

int readDistanceCM(){
    int distance = 0;
    TMR1H = 0; TMR1L = 0;                  // Reset Timer1
    PORTC |= 0b00010000;                    // Trigger HIGH
    delay_us_T0(10);                        // 10 µs pulse
    PORTC &= 0b11101111;                    // Trigger LOW

    while(!(PORTC & 0b00100000));          // Wait for echo start
    T1CON |= 0b00000001;                    // Start Timer
    while(PORTC & 0b00100000);             // Wait for echo end
    T1CON &= 0b11111110;                    // Stop Timer

    distance = (TMR1L | (TMR1H << 8));
    distance = distance / 58.82;           // Convert to cm
    return distance;
}


//       MOTOR PWM
void PWM_init(){
    T2CON = 0x07;      // Timer2 prescaler 1:16
    CCP1CON = 0x0C;    // PWM enable CCP1
    CCP2CON = 0x0C;    // PWM enable CCP2
    PR2 = 250;         // 2ms period
}


void ADC_init(){
    ADCON0 = 0x41; // ADC ON, channel 0, Fosc/16
    ADCON1 = 0xCE; // RA0 analog, others digital, right aligned
}

int ADC_read(){
    ADCON0 |= 0x04;          // Start conversion
    while(ADCON0 & 0x04);    // Wait until done
    return (ADRESH << 8) | ADRESL;
}


void setLeftMotorSpeed(int speed){
CCPR1L = speed; }
void setRightMotorSpeed(int speed){
CCPR2L = speed; }

//       MOTOR DIRECTIONS
void moveForward(){
PORTB = (PORTB & 0b01011111) | 0b01010000;
}
void moveBackward(){
PORTB = (PORTB & 0b10101111) | 0b10100000;
}
void turnRight(){
PORTB = (PORTB & 0b01101111) | 0b01100000;
}
void turnLeft(){
PORTB = (PORTB & 0b10011111) | 0b10010000;
}
void stopMotors(){
PORTB &= 0b00001111;
}


//            MAIN
void main(){
    // PORT Configuration
    TRISA = 0x01;
    TRISB = 0X08;
    TRISC = 0X20;
    TRISD = 0XF4;

    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;

    // Timer0 setup
    OPTION_REG = 0b00000010; // Prescaler 1:8
    INTCON = 0xA0;           // Enable TMR0 interrupt & global interrupt
    TMR0 = 231;

    T1CON = 0x10;            // Timer1 setup
    ADC_init();               // ADC init
    PWM_init();               // PWM init
    setServoAngle(0);
    operationStage=1;
    lightFlag=0;
    flagStage2=0;
    flagStage3=0;
    delay_ms_T0(3000);
    PORTB= PORTB |   0X01;
    while(1){
        if(operationStage == 1){
            // Read light sensor
            lightSensorValue = ADC_read();

            if(lightSensorValue < 750){
                PORTD |= 0b00000010;
                lightFlag = 1;
            } else {
                PORTD &= 0b11111101;
                if(lightFlag == 1) {
                operationStage = 2;
                }
            }

            // Line following logic
            if(!(PORTD & 0b00100000) && !(PORTD & 0b00010000)){
                moveForward(); setLeftMotorSpeed(75); setRightMotorSpeed(75);
            } else if(!(PORTD & 0b00100000) && (PORTD & 0b00010000)){
                moveForward(); setLeftMotorSpeed(0); setRightMotorSpeed(75);
            } else if((PORTD & 0b00100000) && !(PORTD & 0b00010000)){
                moveForward(); setLeftMotorSpeed(75); setRightMotorSpeed(0);
            } else if((PORTD & 0b00100000) && (PORTD & 0b00010000)){
                stopMotors(); delay_ms_T0(1000);
                moveForward(); setLeftMotorSpeed(75); setRightMotorSpeed(75);
                delay_ms_T0(300); stopMotors(); delay_ms_T0(1000);
                turnLeft(); setLeftMotorSpeed(75); setRightMotorSpeed(75);
                while(!(PORTD & 0b00010000));
                stopMotors(); delay_ms_T0(1000);
            }

        } else {
            // Stage 2 obstacle handling logic
            if(flagStage2 == 0){
                stopMotors(); delay_ms_T0(1000);
                moveForward(); setLeftMotorSpeed(70); setRightMotorSpeed(70);
                delay_ms_T0(400); stopMotors(); delay_ms_T0(1000);
                turnRight(); setLeftMotorSpeed(70); setRightMotorSpeed(70);
                delay_ms_T0(300); stopMotors(); delay_ms_T0(1000);
                flagStage2 = 1;
            } else {
                if(flagStage3 == 0){
                    if((PORTD & 0b00100000) || (PORTD & 0b00010000)){
                        flagStage3 = 1;
                    } else { flagStage3 = 0; }

                    measuredDistance = readDistanceCM();

                    // Obstacle avoidance
                    if((measuredDistance < 25) && (PORTC&0B10000000) && (PORTC&0B01000000)){
                        stopMotors(); delay_ms_T0(1000);
                        turnRight();
                        setLeftMotorSpeed(70);
                        setRightMotorSpeed(70);
                        delay_ms_T0(300);
                        stopMotors();
                        delay_ms_T0(1000);
                    }else if ((measuredDistance<20) && (PORTC&0B10000000) &&  !(PORTC&0B01000000)){
                      stopMotors();
                      delay_ms_T0(1000);
                      moveBackward();
                      setLeftMotorSpeed(80);
                      setRightMotorSpeed(80);
                      delay_ms_T0(250);
                      stopMotors();
                      delay_ms_T0(1000);
                      turnLeft();
                      setLeftMotorSpeed(80);
                      setRightMotorSpeed(80);
                      delay_ms_T0(100);
                      stopMotors();
                      delay_ms_T0(1000);

      }else if ((measuredDistance<20) && !(PORTC&0B10000000) &&  (PORTC&0B01000000) ) {
       stopMotors();
                    delay_ms_T0(500);
                    moveBackward();
                      setLeftMotorSpeed(80);
                      setRightMotorSpeed(80);
                      delay_ms_T0(200);
                      stopMotors();
                      delay_ms_T0(500);
                      turnRight();
                        setLeftMotorSpeed(80);
                        setRightMotorSpeed(80);
                        delay_ms_T0(100);
                        stopMotors();
                        delay_ms_T0(1000);
      }else if ((measuredDistance>=20) && (PORTC&0B10000000) &&  !(PORTC&0B01000000)){  //001
      moveForward(); setLeftMotorSpeed(70); setRightMotorSpeed(80);

      }  else if ((measuredDistance>=20) && !(PORTC&0B10000000) &&  (PORTC&0B01000000)){     //010
      moveForward(); setLeftMotorSpeed(80); setRightMotorSpeed(70);
      }
      else {
       moveForward(); setLeftMotorSpeed(70); setRightMotorSpeed(70);


      }


                } else {
                    // Normal line following stage 3
                    if(!(PORTD & 0b00100000) && !(PORTD & 0b00010000)){
                        moveForward(); setLeftMotorSpeed(60); setRightMotorSpeed(60);
                    } else if(!(PORTD & 0b00100000) && (PORTD & 0b00010000)){
                        moveForward(); setLeftMotorSpeed(0); setRightMotorSpeed(60);
                    } else if((PORTD & 0b00100000) && !(PORTD & 0b00010000)){
                        moveForward(); setLeftMotorSpeed(60); setRightMotorSpeed(0);
                    } else if((PORTD & 0b00100000) && (PORTD & 0b00010000)){
                        stopMotors(); delay_ms_T0(1000);
                        moveForward(); setLeftMotorSpeed(80); setRightMotorSpeed(80);
                        delay_ms_T0(350); stopMotors();
                        stopMotors(); delay_ms_T0(1000);
                        turnRight(); setLeftMotorSpeed(80); setRightMotorSpeed(80);
                        delay_ms_T0(300);
                        stopMotors();
                        delay_ms_T0(1000);

                        setServoAngle(90);
                        delay_ms_T0(1000);

                        while(1){
                        stopMotors();
                        PORTB = PORTB & 0XFE;
                        }
                    }
                }
            }
        }
    }
}