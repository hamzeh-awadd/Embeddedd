#line 1 "C:/Users/User/Desktop/New folder (6)/MyProject.c"



int servoAngle;
int flagStage2 = 0;
int flagStage3 = 0;
int timerTick = 0;
int servoPulseWidth = 15;
int delayCounter = 0;
int measuredDistance;
int lightSensorValue;
int operationStage = 1;
int lightFlag = 0;



void interrupt(void){
 if(INTCON & 0x04){
 TMR0 = 231;


 if (delayCounter > 0) { delayCounter--; }


 timerTick++;
 if (timerTick < servoPulseWidth){
 PORTD = PORTD | 0x01;
 } else{
 PORTD = PORTD & 0b11111110;
 }


 if (timerTick >= 200){
 timerTick = 0;
 }

 INTCON &= 0xFB;
 }
}



void delay_ms_T0(int ms){
 delayCounter = ms * 10;
 while(delayCounter > 0);
}

void delay_us_T0(int us){
 delayCounter = us / 100;
 while(delayCounter > 0);
}




void setServoAngle(int angle){
 if(angle > 180) angle = 180;
 servoPulseWidth = 10 + (angle * 10) / 180;
}









int readDistanceCM(){
 int distance = 0;
 TMR1H = 0; TMR1L = 0;
 PORTC |= 0b00010000;
 delay_us_T0(10);
 PORTC &= 0b11101111;

 while(!(PORTC & 0b00100000));
 T1CON |= 0b00000001;
 while(PORTC & 0b00100000);
 T1CON &= 0b11111110;

 distance = (TMR1L | (TMR1H << 8));
 distance = distance / 58.82;
 return distance;
}



void PWM_init(){
 T2CON = 0x07;
 CCP1CON = 0x0C;
 CCP2CON = 0x0C;
 PR2 = 250;
}


void ADC_init(){
 ADCON0 = 0x41;
 ADCON1 = 0xCE;
}

int ADC_read(){
 ADCON0 |= 0x04;
 while(ADCON0 & 0x04);
 return (ADRESH << 8) | ADRESL;
}


void setLeftMotorSpeed(int speed){
CCPR1L = speed; }
void setRightMotorSpeed(int speed){
CCPR2L = speed; }


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



void main(){

 TRISA = 0x01;
 TRISB = 0X08;
 TRISC = 0X20;
 TRISD = 0XF4;

 PORTA = 0x00;
 PORTB = 0x00;
 PORTC = 0x00;
 PORTD = 0x00;


 OPTION_REG = 0b00000010;
 INTCON = 0xA0;
 TMR0 = 231;

 T1CON = 0x10;
 ADC_init();
 PWM_init();
 setServoAngle(0);
 operationStage=1;
 lightFlag=0;
 flagStage2=0;
 flagStage3=0;
 delay_ms_T0(3000);
 PORTB= PORTB | 0X01;
 while(1){
 if(operationStage == 1){

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


 if((measuredDistance < 25) && (PORTC&0B10000000) && (PORTC&0B01000000)){
 stopMotors(); delay_ms_T0(1000);
 turnRight();
 setLeftMotorSpeed(70);
 setRightMotorSpeed(70);
 delay_ms_T0(300);
 stopMotors();
 delay_ms_T0(1000);
 }else if ((measuredDistance<20) && (PORTC&0B10000000) && !(PORTC&0B01000000)){
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

 }else if ((measuredDistance<20) && !(PORTC&0B10000000) && (PORTC&0B01000000) ) {
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
 }else if ((measuredDistance>=20) && (PORTC&0B10000000) && !(PORTC&0B01000000)){
 moveForward(); setLeftMotorSpeed(70); setRightMotorSpeed(80);

 } else if ((measuredDistance>=20) && !(PORTC&0B10000000) && (PORTC&0B01000000)){
 moveForward(); setLeftMotorSpeed(80); setRightMotorSpeed(70);
 }
 else {
 moveForward(); setLeftMotorSpeed(70); setRightMotorSpeed(70);


 }


 } else {

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
