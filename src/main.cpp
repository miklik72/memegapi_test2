#include <Arduino.h>
#include <MeMegaPi.h>

#include <MeMegaPi.h>
#define DEBUG_INFO

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

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

void setup()
{
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Serial.begin(115200);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  
  //Encoder_1.reset(SLOT1);
  //Encoder_1.setTarPWM(100);
  //Encoder_1.move(100,100);
  //Encoder_1.setMotorPwm(100);
  //Encoder_2.move(300,200);
  Serial.print("Slot: ");
  Serial.print(Encoder_1.getSlotNum());
  Serial.print(" Int: ");
  Serial.print(Encoder_1.getIntNum());
  Serial.print(" portA: ");
  Serial.print(Encoder_1.getPortA());
  Serial.print(" portB: ");
  Serial.println(Encoder_1.getPortB());
  Serial.print("PulsePos: ");
  Serial.print(Encoder_1.getPulsePos());
  Serial.print(" speed: ");
  Serial.print(Encoder_1.getCurrentSpeed());
  Serial.print(" CurPWM: ");
  Serial.print(Encoder_1.getCurPwm());
  Serial.print(" CurPos: ");
  Serial.println(Encoder_1.getCurPos());
  //delay(500);
  //Encoder_1.setMotionMode(PID_MODE);
  Encoder_1.setMotorPwm(200);
  Encoder_1.moveTo(1000,100);
}

void loop()
{
  //Encoder_1.setMotorPwm(100);
  //Encoder_2.setMotorPwm(100);
  //Encoder_1.updateCurPos();
  //Encoder_1.updateSpeed();
  //Encoder_2.updateCurPos();
  //Serial.print("Spped 1:");
  //Serial.print(Encoder_1.getCurrentSpeed());
  //Serial.print(" ,Spped 2:");
  //Serial.println(Encoder_2.getCurrentSpeed());
  Serial.print("PulsePos: ");
  Serial.print(Encoder_1.getPulsePos());
  Serial.print(" speed: ");
  Serial.print(Encoder_1.getCurrentSpeed());
  Serial.print(" CurPWM: ");
  Serial.print(Encoder_1.getCurPwm());
  Serial.print(" goTo: ");
  Serial.print(Encoder_1.distanceToGo());
  Serial.print(" CurPos: ");
  Serial.println(Encoder_1.getCurPos());
  Encoder_1.loop();
  //delay(100);
}