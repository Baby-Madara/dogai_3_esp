#include <system.h>






void setup()
{
    
    taskInitializers();
}

void loop()
{

}













/*
void setup()
{
    Serial.begin(115200);
    servoDriverWire.begin(SDA_2, SCL_2);
    pwm.begin();
    pwm.setOscillatorFrequency(26600000);   // it is something between 23 to 27MHz (needed to be calibrated  --> calibration 26600000Hz)
    pwm.setPWMFreq(50);                     // of the output PWM signal from channels (for each servo)
    pinMode(4, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    // pwm.setPWM(1,0,1000);
    
}

void loop()
{
    angle =0;
    pwm.writeMicroseconds(0, map(angle, 0, 180, 500, 2500));
    delay(1000);

    angle =90;
    pwm.writeMicroseconds(0, map(angle, 0, 180, 500, 2500));
    delay(1000);

    angle =180;
    pwm.writeMicroseconds(0, map(angle, 0, 180, 500, 2500));
    delay(1000);
    double duration=0;
    for(int i =0; i <10; i++){
        duration += pulseIn(4, LOW);
    }
    duration /= 10;
    Serial.println("180 deg = "+String(duration)+" us");
    
    // String("man{is}")
    // detach?
    pwm.writeMicroseconds(0, 0);
    delay(5000);
    
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    // put your main code here, to run repeatedly:
}
*/





