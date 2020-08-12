#include <Arduino.h>
#include <MeMegaPi.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard Encoder_3(SLOT3);
MeMegaPiDCMotor Arm_1(PORT4B);

MeUltrasonicSensor sonic(PORT_6);
Me7SegmentDisplay disp(PORT_8);

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

void isr_process_encoder3(void)
{
  if(digitalRead(Encoder_3.getPortB()) == 0)
  {
    Encoder_3.pulsePosMinus();
  }
  else
  {
    Encoder_3.pulsePosPlus();
  }
}

long lasttime = millis();

void arm_open() {
  Arm_1.run(-255);
  delay(1200);
  Arm_1.stop();
}

void arm_close() {
  Arm_1.run(100);
  delay(3000);
  Arm_1.stop();
}

void setup()
{
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(Encoder_3.getIntNum(), isr_process_encoder3, RISING);
  arm_open();
  Serial.begin(115200);
  disp.init();
  disp.set(BRIGHTNESS_2);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);//PIN12
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);//PIN8
  TCCR2B = _BV(CS21);

  TCCR3A = _BV(WGM30);//PIN9
  TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);
   
  TCCR4A = _BV(WGM40);//PIN5
  TCCR4B = _BV(CS41) | _BV(CS40) | _BV(WGM42);
  
  Encoder_1.setMotionMode(PID_MODE);
  Encoder_2.setMotionMode(PID_MODE);
  Encoder_3.setMotionMode(PID_MODE);
  Encoder_1.setPulse(7);
  Encoder_2.setPulse(7);
  Encoder_3.setPulse(7);
  Encoder_1.setRatio(26.9);
  Encoder_2.setRatio(26.9);
  Encoder_3.setRatio(26.9);
  Encoder_1.setPosPid(1.8,0,1.2);
  Encoder_2.setPosPid(1.8,0,1.2);
  Encoder_3.setPosPid(1.8,0,1.2);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setSpeedPid(0.18,0,0);
  Encoder_3.setSpeedPid(0.18,0,0);
  Encoder_1.setPulsePos(0);
  Encoder_2.setPulsePos(0);
  Encoder_3.setPulsePos(0);
  //Encoder_1.moveTo(1000,150);
  //Encoder_2.moveTo(-1000,150);
  //Encoder_3.moveTo(500,100);
  //Arm_1.run(50);
  delay(1000);
  arm_close();
}

void loop() {
  if (millis() - lasttime > 500) {
    
    //uint8_t ListDisp[4];
    uint16_t distanc = (uint16_t)sonic.distanceCm();
    disp.display(distanc);      

    Serial.print("E3: PulsePos: ");
    Serial.print(Encoder_3.getPulsePos());
    Serial.print(" speed: ");
    Serial.print(Encoder_3.getCurrentSpeed());
    Serial.print(" CurPWM: ");
    Serial.print(Encoder_3.getCurPwm());
    Serial.print(" goTo: ");
    Serial.print(Encoder_3.distanceToGo());
    Serial.print(" CurPos: ");
    Serial.print(Encoder_3.getCurPos());
    // Serial.print(",E2: PulsePos: ");
    // Serial.print(Encoder_2.getPulsePos());
    // Serial.print(" speed: ");
    // Serial.print(Encoder_2.getCurrentSpeed());
    // Serial.print(" CurPWM: ");
    // Serial.print(Encoder_2.getCurPwm());
    // Serial.print(" goTo: ");
    // Serial.print(Encoder_2.distanceToGo());
    // Serial.print(" CurPos: ");
    // Serial.println(Encoder_2.getCurPos());
    Serial.print(" distance: ");
    Serial.print(distanc);
    Serial.println("");
    lasttime = millis();
  }  
  Encoder_1.loop();
  Encoder_2.loop();
  Encoder_3.loop();
  //delay(100);
}