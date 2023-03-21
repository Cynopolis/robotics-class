#include "Motor.h"

Motor::Motor(uint8_t forwardPin, uint8_t backwardPin, uint8_t pwmPin, uint8_t pwmChannel, volatile int* incriment) : 
forwardPin(forwardPin), backwardPin(backwardPin), pwmPin(pwmPin), pwmChannel(pwmChannel), incriment(incriment){}

void Motor::setWheelRadius(float wheelRadius){
    this->wheelRadius = wheelRadius;
    this->stepsToMM = 2*PI*wheelRadius*stepsPerRevolution;
}

float Motor::getWheelRadius(){
    return this->wheelRadius;
}

float Motor::getVelocity(){
    return this->currentVelocity;
}

float Motor::getDistanceSinceLastUpdate(){
    return this->distanceSinceLastUpdate;
}

float Motor::update(){
    // calculate dt
    unsigned long now = millis();
    int time_diff = int(now - lastTime);
    if(time_diff < 1 || *incriment == 0) return 0.0;
    lastTime = now;
    float dt = float(time_diff)*0.001;

    // calculate velocity
    lastEncoderSteps = encoderSteps;
    encoderSteps += *incriment;
    this->distanceSinceLastUpdate = (float(*incriment) * stepsToMM);
    currentVelocity = this->distanceSinceLastUpdate / dt;

    // reset incriment to 0
    *incriment = 0;
    return this->distanceSinceLastUpdate;
}

void Motor::begin(){  
    Serial.print("Initializing Motor: ");
    Serial.print(pwmPin);
    Serial.print(" ");
    Serial.println(pwmChannel);
    ledcAttachPin(pwmPin, pwmChannel);
    ledcSetup(pwmChannel, 24000, 8);
    Serial.println("PWM init done.");
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);
    ledcWrite(pwmChannel, 0);
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
    Serial.print("Motor::begin() pwmChannel: ");
    Serial.print(pwmChannel);
    Serial.print(" pwmPin: ");
    Serial.print(pwmPin);
    Serial.print(" forwardPin: ");
    Serial.print(forwardPin);
    Serial.print(" backwardPin: ");
    Serial.println(backwardPin);
    //ledcSetup(pwmChannel, 30000, 8);
}

void Motor::setVelocity(int velocity){
    // make sure the velocity is in the range of -255 to 255
    if(velocity > 255){
        velocity = 255;
    } else if(velocity < -255){
        velocity = -255;
    }
    this->targetVelocity = velocity;

    if(velocity > 0){
        digitalWrite(forwardPin, HIGH);
        digitalWrite(backwardPin, LOW);
    } else if(velocity < 0){
        digitalWrite(forwardPin, LOW);
        digitalWrite(backwardPin, HIGH);
    } else {
        digitalWrite(forwardPin, LOW);
        digitalWrite(backwardPin, LOW);
    }
    ledcWrite(pwmChannel, abs(velocity));
}
