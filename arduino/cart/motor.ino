#define MR_1 4
#define MR_2 18
#define MR_PWM 19
#define MR_A 23
#define MR_B 32

#define ML_1 14
#define ML_2 27
#define ML_PWM 26
#define ML_A 25
#define ML_B 33

int counts[] = { 0, 0 };

void IRAM_ATTR falling_MR_A() {
    counts[0] += digitalRead(MR_B) == LOW ? 1 : -1;
}

void IRAM_ATTR falling_ML_A() {
    counts[1] += digitalRead(ML_B) == LOW ? 1 : -1;
}

void motor_setup() {
    pinMode(MR_1,   OUTPUT);
    pinMode(MR_2,   OUTPUT);
    pinMode(MR_PWM, OUTPUT);

    pinMode(MR_A, INPUT);
    pinMode(MR_B, INPUT);

    pinMode(ML_1,   OUTPUT);
    pinMode(ML_2,   OUTPUT);
    pinMode(ML_PWM, OUTPUT);

    pinMode(ML_A, INPUT);
    pinMode(ML_B, INPUT);

    attachInterrupt(MR_A, falling_MR_A, FALLING);
    attachInterrupt(ML_A, falling_ML_A, FALLING);

    counts[0] = counts[1] = 0;

    // testing
    Serial.print("Testing DC Motor...");
}

void set_speed(int pin_pwm, float speed){
    analogWrite(pin_pwm, 80 + int((255 - 80) * speed));    
}

void motor_loop() {
    int span = 2000;
    int tick = millis() % (6 * span);

    for(int idx = 0; idx < 2; idx++){

        int pin1, pin2, pin_pwm;

        if(idx == 0){
            pin1 = MR_1;
            pin2 = MR_2;
            pin_pwm = MR_PWM;
        }
        else{
            pin1 = ML_1;
            pin2 = ML_2;
            pin_pwm = ML_PWM;

        }

        if(2 * span <= tick && tick < 3 * span || 5 * span <= tick){

            digitalWrite(pin1, LOW);
            digitalWrite(pin2, LOW);
            set_speed(pin_pwm, 0);
        }
        else{

            if(tick < 2 * span){

                digitalWrite(pin1, LOW);
                digitalWrite(pin2, HIGH); 
            }
            else{
                tick -= 3 * span;

                digitalWrite(pin1, HIGH);
                digitalWrite(pin2, LOW); 
            }

            if(tick < span){

                set_speed(pin_pwm, tick / (float)span);
            }
            else{

                set_speed(pin_pwm, (2 * span - tick) / (float)span);
            }
        }
    }
}
