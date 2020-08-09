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

long lasttime = millis();

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
  
  Encoder_1.setMotionMode(PID_MODE);
  Encoder_2.setMotionMode(PID_MODE);
  Encoder_1.setPulse(7);
  Encoder_2.setPulse(7);
  Encoder_1.setRatio(26.9);
  Encoder_2.setRatio(26.9);
  Encoder_1.setPosPid(1.8,0,1.2);
  Encoder_2.setPosPid(1.8,0,1.2);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setSpeedPid(0.18,0,0);
  Encoder_1.setPulsePos(0);
  Encoder_2.setPulsePos(0);

  Encoder_1.moveTo(1000,150);
  Encoder_2.moveTo(-1000,150);
}

void loop() {
  if (millis() - lasttime > 500) {
    Serial.print("E1: PulsePos: ");
    Serial.print(Encoder_1.getPulsePos());
    Serial.print(" speed: ");
    Serial.print(Encoder_1.getCurrentSpeed());
    Serial.print(" CurPWM: ");
    Serial.print(Encoder_1.getCurPwm());
    Serial.print(" goTo: ");
    Serial.print(Encoder_1.distanceToGo());
    Serial.print(" CurPos: ");
    Serial.print(Encoder_1.getCurPos());
    Serial.print(",E2: PulsePos: ");
    Serial.print(Encoder_2.getPulsePos());
    Serial.print(" speed: ");
    Serial.print(Encoder_2.getCurrentSpeed());
    Serial.print(" CurPWM: ");
    Serial.print(Encoder_2.getCurPwm());
    Serial.print(" goTo: ");
    Serial.print(Encoder_2.distanceToGo());
    Serial.print(" CurPos: ");
    Serial.println(Encoder_2.getCurPos());
    lasttime = millis();
  }  
  Encoder_1.loop();
  Encoder_2.loop();
  //delay(100);
}