#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h> //https://stackoverflow.com/a/69649954/8928251
#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <string.h>

uint8_t SRV_PINs[] = {6, 9, 10, 11};
Servo servos[4];
#define INIT_DEG \
  (uint8_t[]) { 10, 10, 10, 10 }
#define LOWEST_PWM 525
#define HIGHEST_PWM 2361
#define GRIP_CLOSE_ANGLE 9
#define GRIP_OPEN_ANGLE 30

#define SHO_REF 28 // 28
#define ELB_REF 80 // 80
#define PIT_REF 25 

const uint8_t dirPin = 2;
const uint8_t stepPin = 3;
#define MICRO_STEP 1
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

char buff[50];
bool runEnabled = false;
bool newCommand = false;
bool grip_close;
int16_t base_angle, shoulder_angle, elbow_angle, pitch_angle;
void go_to_default_position_all();
int deg_for_big_servo(int deg);
void gently_rotate_to(uint8_t srv_idx, int16_t degree);
uint16_t get_current_pos_pulse(uint8_t srv_idx);
int16_t get_current_pos(uint8_t srv_idx);
int big_servo_to_deg(int servoRead);
void go_to_degree_big_servo(uint8_t servoIdx, int16_t deg);
long stepper_step(int deg, uint8_t microstep);
void go_to_degree_stepper(int deg);
int stepper_degree(long step, int microstep);

void move_elbow_onedeg();
void move_shoulder_onedeg();
void pitch_to(int16_t deg);
void grip();
void messageCb(const std_msgs::Int16MultiArray &msg);

bool shoulderDone = true, elbowDone = true, baseDone = true;

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16MultiArray> sub("arduino_input", messageCb);
std_msgs::Int16 int_msg;
ros::Publisher output("arduino_output", &int_msg);


void messageCb(const std_msgs::Int16MultiArray &msg)
{
  // Text format: "{base angle},{shoulder angle from horizontal},{elbow angle from vertical},{pitch angle from vertical}"
  if (msg.data_length != 5)
  {
    nh.loginfo("got unsupported msg");
    return;
  }

  baseDone = base_angle == msg.data[0];
  shoulderDone = shoulder_angle == SHO_REF + msg.data[1];
  elbowDone = elbow_angle == ELB_REF + msg.data[2];

  base_angle = msg.data[0];
  shoulder_angle = SHO_REF + msg.data[1];
  elbow_angle = ELB_REF + msg.data[2];
  pitch_angle = PIT_REF + msg.data[3];
  grip_close = msg.data[4] > 0; // 1=close, 0=open
  if (newCommand)
  {
    nh.loginfo("Got angles before execusion done");
  }
  sprintf(buff, "Angles:bs:%d,sh:%d,el:%d,pi:%d,gr:%d", base_angle, shoulder_angle, elbow_angle, pitch_angle, grip_close);
  nh.loginfo(buff);
  nh.loginfo(buff); // Jeno dekhte miss na jay
  
  newCommand = true;
  runEnabled = true;
  
}

void setup()
{
  Serial.begin(57600);
  servos[0].attach(SRV_PINs[0]);
  servos[1].attach(SRV_PINs[1]);
  servos[2].attach(SRV_PINs[2]);
  servos[3].attach(SRV_PINs[3]);

  stepper.setMaxSpeed(200);     // SPEED = Steps / second
  stepper.setAcceleration(150); // ACCELERATION = Steps /(second)^2

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(output);
  nh.spinOnce();
  nh.loginfo("In setup....");
  delay(100);

  servos[0].write(10);
  servos[1].write(PIT_REF);
  servos[2].write(deg_for_big_servo(abs(ELB_REF)));
  servos[3].write(deg_for_big_servo(SHO_REF));
}

void loop()
{
  // wait until you are actually connected
  while (!nh.connected())
  {
    nh.spinOnce();
    delay(500);
  }

  if (newCommand && runEnabled)
  {
    sprintf(buff, "Inside if..%d, %d, %d", baseDone, shoulderDone, elbowDone);
    nh.loginfo(buff);
    if (!baseDone)
    {
      stepper.runToPosition();
      delayMicroseconds(10);
      baseDone = !stepper.run();
      sprintf(buff, "Base Done:bs:%d,el:%d", shoulderDone, elbowDone);
      nh.loginfo(buff);
    }
    if (!shoulderDone)
    {
      move_shoulder_onedeg();
    }
    if (!elbowDone)
    {
      move_elbow_onedeg();
    }
    if (shoulderDone && elbowDone) // because base is oneshot
    {
      sprintf(buff, "B,S,E Done:bs:%d,sh:%d,el:%d", baseDone, shoulderDone, elbowDone);
      nh.logdebug(buff);
      pitch_to(pitch_angle);
      grip();
      int_msg.data = 1;
      output.publish(&int_msg);
      nh.loginfo("Sent msg");
      newCommand = false;
    }
    else
    {
      int_msg.data = 0;
      output.publish(&int_msg);
      sprintf(buff,"arduino doing job...");
      nh.loginfo(buff);
    }
  }
  else 
  {
    sprintf(buff, "newCommand==false:arduino doing nothing...");
    nh.loginfo(buff);
  }

  nh.spinOnce();
  delayMicroseconds(10);
}

void stopMotor() // function activated by the pressed microswitch
{
  // Stop motor, disable outputs; here we should also reset the numbers if there are any
  runEnabled = false; // disable running

  stepper.stop();                // stop motor
  stepper.setCurrentPosition(0); // reset position
  // stepper.disableOutputs(); //disable power
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

void go_to_degree_big_servo(uint8_t servoIdx, int16_t deg)
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

int16_t get_current_pos(uint8_t srv_idx)
{
  uint16_t pos_pulse = get_current_pos_pulse((int16_t)srv_idx);
  uint8_t pos_degree = map(pos_pulse, LOWEST_PWM, HIGHEST_PWM, 0, 180);
  return pos_degree;
}

long stepper_step(int deg, uint8_t microstep)
{
  return (long)(deg * microstep / 1.8);
}

int stepper_degree(long step, int microstep)
{
  return (int)(step * 1.8 / microstep);
}

void move_elbow_onedeg()
{
  int16_t current_deg = big_servo_to_deg(servos[2].read());
  if (current_deg < elbow_angle)
  {
    go_to_degree_big_servo(2, current_deg + 1);
  }
  else if (current_deg > elbow_angle)
  {
    go_to_degree_big_servo(2, current_deg - 1);
  }
  else
  {
    elbowDone = true;
  }
}

void move_shoulder_onedeg()
{
  int16_t current_deg = big_servo_to_deg(servos[3].read());
  if (current_deg < shoulder_angle)
  {
    go_to_degree_big_servo(3, current_deg + 1);
  }
  else if (current_deg > shoulder_angle)
  {
    go_to_degree_big_servo(3, current_deg - 1);
  }
  else
  {
    shoulderDone = true;
  }
}

void pitch_to(int16_t deg)
{
  // todo angle conversion
  gently_rotate_to(1, deg);
}

void grip()
{
  if (grip_close) // close the gripper
  {
    gently_rotate_to(0, GRIP_CLOSE_ANGLE);
  }
  else
  { // open the gripper
    gently_rotate_to(0, GRIP_OPEN_ANGLE);
  }
}