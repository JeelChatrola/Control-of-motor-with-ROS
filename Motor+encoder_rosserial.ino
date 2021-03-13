#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

std_msgs::Int64 encoder; 


ros::Publisher encoderValue("encoderValue", &encoder);



// arduino hardware pins
#define MOTOR_1 0
#define BRAKE 0
#define CW    1
#define CCW   2


#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8

#define PWM_MOTOR_1 5

const int encoderPinA = 19;
const int encoderPinB = 20;
volatile int64_t currentPosition = 0;


void pwm_input( const std_msgs::Int16& pwm_value){
  int pwm =0;
  pwm = pwm_value.data;
  
  if ( pwm > 0 )
  {
  motorGo(MOTOR_1,CCW,pwm);
  }
  else
  {
  motorGo(MOTOR_1,CW,abs(pwm));
  }
 
}
ros::Subscriber<std_msgs::Int16> pwm("PWM_Values", &pwm_input);

void setup()
{
  nh.initNode();
  nh.advertise(encoderValue);
  nh.subscribe(pwm);
  
  pinMode (encoderPinA, INPUT_PULLUP);
  pinMode (encoderPinB, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (encoderPinA), readEncoderA, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPinB), readEncoderB, CHANGE);

  TCCR1B = TCCR1B & 0b11111000 | 1;
}



void motorGo(int16_t motor, int16_t direct, uint16_t pwm)         //Function that controls the variables: motor(0 ou 1), direction (cw ou ccw) e pwm (entra 0 e 255);
{
  if(motor == MOTOR_1)
  {
    if(direct == CW)
    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == CCW)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_1, pwm); 
  }
  
}
void readEncoderA()
{
  if (digitalRead(encoderPinA) != digitalRead(encoderPinB))
  {
    currentPosition++;
  }
  else
  {
    currentPosition--;
  }
}

void readEncoderB()
{
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB))
  {
    currentPosition++;
  }
  else
  {
    currentPosition--;
  }
}


void loop()
{
    
  nh.loginfo("Encoder Value");
  
  encoder.data = currentPosition;
  encoderValue.publish( &encoder );
  
  nh.spinOnce();
  delay(100);
  
}
