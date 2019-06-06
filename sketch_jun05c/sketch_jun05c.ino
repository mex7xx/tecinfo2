#include <Ticker.h>

Ticker blinker;

#define LED 2  //On board LED


void ICACHE_RAM_ATTR onTimerISR(){
    digitalWrite(LED,!(digitalRead(LED)));  //Toggle LED Pin
    //timer1_write(600000);//12us
}

void fu1() {
      digitalWrite(LED,!(digitalRead(LED)));  //Toggle LED Pin
      Serial.println("Hello");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("");

    pinMode(LED,OUTPUT);

    //Initialize Ticker every 0.5s

    blinker.attach_ms(100,fu1);
    
    //timer1_attachInterrupt(onTimerISR);
    //timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
    //timer1_write(600000); //120000 us
}

void loop()
{
  Serial.print(".");
  
}
