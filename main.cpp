/* Sweep
  by BARRAGAN <http://barraganstudio.com>
  This example code is in the public domain.

  modified 8 Nov 2013
  by Scott Fitzgerald
  https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/
#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>
#define SHO_REF 28
#define ELB_REF 36
#define PIT_REF 107
const uint8_t dirPin = 2;
const uint8_t stepPin = 3;
#define MICRO_STEP 1

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

uint8_t SRV_PINs[] = {6, 9, 10, 11};
Servo servos[4];

#define INIT_DEG \
    (uint8_t[]) { 5, 5, 5, 5}

char buff[50];

void go_to_default_position_all();
int deg_for_big_servo(int deg);
void gently_rotate_to(uint8_t srv_idx, int16_t degree);
uint16_t get_current_pos_pulse(uint8_t srv_idx);
int16_t get_current_pos(uint8_t srv_idx);
int big_servo_to_deg(int servoRead);
void go_to_degree_big_servo(int servoIdx, int deg);
long stepper_step(int deg, int microstep);
void go_to_degree_stepper(int deg);
int stepper_degree(long step, int microstep);

void setup()
{
    Serial.begin(9600);

    servos[0].attach(SRV_PINs[0]);
    servos[1].attach(SRV_PINs[1]);
    servos[2].attach(SRV_PINs[2]);
    servos[3].attach(SRV_PINs[3]);

    stepper.setMaxSpeed(200);     // SPEED = Steps / second
    stepper.setAcceleration(150); // ACCELERATION = Steps /(second)^2
    // stepper.disableOutputs(); //disable outputs, so the motor is not getting warm (no current)

    // go_to_default_position_all();
    stepper.setCurrentPosition(0);
    // default
    servos[0].write(10);
    gently_rotate_to(1, PIT_REF);
    go_to_degree_big_servo(2, deg_for_big_servo(abs(ELB_REF)));
    go_to_degree_big_servo(3, deg_for_big_servo(SHO_REF+70));
}

#define LOWEST_PWM 525
#define HIGHEST_PWM 2361

void loop()
{

    if (Serial.available())
    {
        int ch = '0';
        ch = Serial.read();
        stepper.disableOutputs();
        switch (ch)
        {
        case 'a': // grip close
        {
            if (servos[0].read() > 9) // decrease degree
            {
                servos[0].write(servos[0].read() - 1);
                sprintf(buff, "Current degree srv1(GRIP-CLOSE): %d", servos[0].read());
                Serial.println(buff);
            }
        }
        break;
        case 'b': // grip open
        {
            if (servos[0].read() <= 60) // increase degree
            {
                servos[0].write(servos[0].read() + 1);
                sprintf(buff, "Current degree srv1(GRIP-OPEN): %d", servos[0].read());
                Serial.println(buff);
            }
        }
        break;
        case 'c': // wrist up
        {
            if (servos[1].read() < 178) // increase degree
            {
                servos[1].write(servos[1].read() + 1);
                sprintf(buff, "Current degree srv2(WRIST-UP): %d", servos[1].read());
                Serial.println(buff);
            }
        }
        break;
        case 'd': // wrist down
        {
            if (servos[1].read() > 2) // decrease degree
            {
                servos[1].write(servos[1].read() - 1);
                sprintf(buff, "Current degree srv2(WRIST-DOWN): %d", servos[1].read());
                Serial.println(buff);
            }
        }
        break;
        case 'e': // Elbow down
        {
            if (servos[2].read() > 2) // decrease degree
            {
                servos[2].write(servos[2].read() - 1);
                sprintf(buff, "Current degree srv3(ELBOW-UP): %d", big_servo_to_deg(servos[2].read()));
                Serial.println(buff);
            }
        }
        break;
        case 'f': // Elbow up
        {
            if (big_servo_to_deg(servos[2].read()) < 178) // increase degree
            {
                servos[2].write(servos[2].read() + 1);
                sprintf(buff, "Current degree srv3(ELBOW-DOWN): %d", big_servo_to_deg(servos[2].read()));
                Serial.println(buff);
            }
        }
        break;
        case 'g': // Shoulder down
        {

            if (servos[3].read() > 2) 
            {
                servos[3].write(servos[3].read() - 1);
                sprintf(buff, "Current degree srv4(Shoulder-DOWN): %d", big_servo_to_deg(servos[3].read()));
                Serial.println(buff);
            }
            

            // Set motor direction anti-clockwise
            // digitalWrite(dirPin, HIGH);
            // for (int x = 0; x < 10; x++)
            // {
            //     Serial.println("clockwise");
            //     digitalWrite(stepPin, HIGH);
            //     delayMicroseconds(1500);
            //     digitalWrite(stepPin, LOW);
            //     delayMicroseconds(1500);
            // }

            // stepper.move(stepper_step(2, MICRO_STEP));
            // stepper.runToPosition();
            // Serial.print("Shoulder down:");
            // Serial.println(stepper_degree(stepper.currentPosition(), MICRO_STEP));
        }
        break;
        case 'h': // Shoulder up
        {
            // Set motor direction clockwise
            // digitalWrite(dirPin, LOW);
            // for (int x = 0; x < 10; x++)
            // {
            //     Serial.println("anti-clockwise");
            //     digitalWrite(stepPin, HIGH);
            //     delayMicroseconds(1500);
            //     digitalWrite(stepPin, LOW);
            //     delayMicroseconds(1500);
            // }

            // stepper.move(stepper_step(-2, MICRO_STEP));
            // stepper.runToPosition();
            // Serial.print("Shoulder down:");
            // Serial.println(stepper_degree(stepper.currentPosition(), MICRO_STEP));

            if (big_servo_to_deg(servos[3].read()) < 178) // increase degree
            {
                servos[3].write(servos[3].read() + 1);
                sprintf(buff, "Current degree srv3(Shoulder-DOWN): %d", big_servo_to_deg(servos[3].read()));
                Serial.println(buff);
            }
        }
        break;
        case 'i': // Base up
        {
            // Set motor direction clockwise
            // digitalWrite(dirPin, HIGH);
            // for (int x = 0; x < 10; x++)
            // {
            //     Serial.println("clockwise");
            //     digitalWrite(stepPin, HIGH);
            //     delayMicroseconds(1500);
            //     digitalWrite(stepPin, LOW);
            //     delayMicroseconds(1500);
            // }
            stepper.enableOutputs();
            stepper.move(stepper_step(2, MICRO_STEP));
            stepper.runToPosition();
            Serial.print("Base clocwise:");
            Serial.println(stepper_degree(stepper.currentPosition(), MICRO_STEP));
        }
        break;
        case 'j': // Base down
        {
            // Set motor direction counterclockwise
            // digitalWrite(dirPin, LOW);
            // for (int x = 0; x < 10; x++)
            // {
            //     Serial.println("anti-clockwise");
            //     digitalWrite(stepPin, HIGH);
            //     delayMicroseconds(1500);
            //     digitalWrite(stepPin, LOW);
            //     delayMicroseconds(1500);
            // }
            stepper.enableOutputs();
            stepper.move(stepper_step(-2, MICRO_STEP));
            stepper.runToPosition();
            Serial.print("Base anti-clockwise:");
            Serial.println(stepper_degree(stepper.currentPosition(), MICRO_STEP));
        }
        break;
        case 'q':
        {
            // go_to_default_position_all();
            Serial.println(big_servo_to_deg(180));
            Serial.println(deg_for_big_servo(270));
        }
        break;
        case 'x':
        {
            int deg = Serial.parseInt();
            Serial.print("Elbow(Servo): Got degree:");
            Serial.println(deg);
            if (deg >= 0 && deg <= 135)
            {
                go_to_degree_big_servo(2, deg);
            } 
            else 
            {
                Serial.println("Degree in danger zone");
            }
        }
        break;
        case 'y':
        {
            int deg = Serial.parseInt();
            Serial.print("Shoulder(Servo): Got degree:");
            Serial.println(deg);
            if (deg >= 0 && deg <= 170)
            {
                go_to_degree_big_servo(3, deg);
            } 
            else 
            {
                Serial.println("Degree in danger zone");
            }
        }
        break;
        case 'z':
        {
            int deg = Serial.parseInt();
            Serial.print("pitch(servo): Got degree:");
            Serial.println(deg);
            if (deg >= 0 && deg <= 170)
            {
                gently_rotate_to(1, deg);
            } 
            else 
            {
                Serial.println("Degree in danger zone");
            }
        }
        break;
        case 't':
        {
            int deg = Serial.parseInt();
            Serial.print("grip(servo): Got degree:");
            Serial.println(deg);
            if (deg >= 9 && deg <= 30)
            {
                gently_rotate_to(0, deg);
            } 
            else 
            {
                Serial.println("Degree in danger zone");
            }
        }
        break;
        default:
            stepper.disableOutputs();
            break;
        }
    }
}

void go_to_default_position_all()
{
    gently_rotate_to(0, INIT_DEG[0]); // for grip
    gently_rotate_to(1, INIT_DEG[1]); // for wrist
    servos[2].write(INIT_DEG[2]);     // for elbow
    servos[3].write(INIT_DEG[3]);     // for shoulder
}

int deg_for_big_servo(int deg)
{
    // Serial.print("deg:");
    // Serial.println(deg);
    return (int)map((long)deg, 0, 270, 0, 180);
}

int big_servo_to_deg(int servoRead)
{
    // Serial.print("ServoRead:");
    // Serial.println(servoRead);
    return (int)map((long)servoRead, 0, 180, 0, 270);
}

void gently_rotate_to(uint8_t srv_idx, int16_t degree)
{
    int16_t c_pos = get_current_pos(srv_idx);
    int16_t pos;
    if (degree < c_pos)
    {
        for (pos = c_pos; pos >= degree; pos -= 1)
        {
            servos[srv_idx].write(pos);
            Serial.print("degree: ");
            Serial.println(pos);
            delay(15);
        }
    }
    else if (degree > c_pos)
    {
        for (pos = c_pos; pos <= degree; pos += 1)
        {
            servos[srv_idx].write(pos);
            Serial.print("degree: ");
            Serial.println(pos);
            delay(15);
        }
    }
    Serial.print("Final angle:");
    Serial.println(degree);
}

uint16_t get_current_pos_pulse(int16_t srv_idx)
{
    pinMode(SRV_PINs[srv_idx], OUTPUT);
    digitalWrite(SRV_PINs[srv_idx], HIGH);
    delayMicroseconds(50); // send a 50 us pulse to get the current position
    digitalWrite(SRV_PINs[srv_idx], LOW);
    pinMode(SRV_PINs[srv_idx], INPUT);
    uint16_t pos_pulse = pulseIn(SRV_PINs[srv_idx], HIGH); // e.g. 500-2500
    return pos_pulse;
}

void go_to_degree_big_servo(int servoIdx, int deg)
{
    if (big_servo_to_deg(servos[servoIdx].read()) == deg)
        return;

    if (big_servo_to_deg(servos[servoIdx].read()) < deg)
    {
        while (big_servo_to_deg(servos[servoIdx].read()) <= deg)
        {
            servos[servoIdx].write(servos[servoIdx].read() + 1);
            delay(100);
        }
    }
    else
    {
        while (big_servo_to_deg(servos[servoIdx].read()) >= deg)
        {
            servos[servoIdx].write(servos[servoIdx].read() - 1);
            delay(100);
        }
    }
}

void go_to_degree_stepper(int deg) {
    stepper.moveTo(stepper_step(deg, MICRO_STEP));
    stepper.runToPosition();
}

int16_t get_current_pos(uint8_t srv_idx)
{
    uint16_t pos_pulse = get_current_pos_pulse((int16_t)srv_idx);
    uint8_t pos_degree = map(pos_pulse, LOWEST_PWM, HIGHEST_PWM, 0, 180);
    return pos_degree;
}


long stepper_step(int deg, int microstep)
{
    return (long)(deg * microstep / 1.8);
}

int stepper_degree(long step, int microstep) 
{
    return (int) (step * 1.8 / microstep);
}