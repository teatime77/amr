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

int encoder_counts[] = { 0, 0 };
unsigned long prev_time_ms;

void IRAM_ATTR falling_MR_A() {
    encoder_counts[0] += digitalRead(MR_B) == LOW ? 1 : -1;
}

void IRAM_ATTR falling_ML_A() {
    encoder_counts[1] += digitalRead(ML_B) == LOW ? 1 : -1;
}

void get_encoder_counts(int &msec, int &c1, int &c2){
    unsigned long time_ms = millis();

    msec = (int)(time_ms - prev_time_ms);
    c1 = encoder_counts[0];
    c2 = encoder_counts[1];

    prev_time_ms = time_ms;
    encoder_counts[0]   = 0;
    encoder_counts[1]   = 0;
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

    encoder_counts[0] = encoder_counts[1] = 0;

    // testing
    Serial.print("Testing DC Motor...");
}

void motor_loop(short pwm_l, short pwm_r) {
    for(int idx = 0; idx < 2; idx++){

        int pin1, pin2, pin_pwm;
        short pwm;

        if(idx == 0){
            pin1 = MR_1;
            pin2 = MR_2;
            pin_pwm = MR_PWM;
            pwm = pwm_r;
        }
        else{
            pin1 = ML_1;
            pin2 = ML_2;
            pin_pwm = ML_PWM;
            pwm = pwm_l;
        }

        if(pwm == 0){

            digitalWrite(pin1, LOW);
            digitalWrite(pin2, LOW);
            analogWrite(pin_pwm, 0);
        }
        else if(0 < pwm){

            digitalWrite(pin1, LOW);
            digitalWrite(pin2, HIGH); 
            analogWrite(pin_pwm, pwm);
        }
        else{
            
            digitalWrite(pin1, HIGH);
            digitalWrite(pin2, LOW); 
            analogWrite(pin_pwm, -pwm);
        }
    }
}
