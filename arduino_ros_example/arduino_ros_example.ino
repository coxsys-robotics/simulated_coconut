
#include <ros.h>
#include <std_msgs/Bool.h>
#include <rosserial_arduino/Adc.h>

const int button_pin = A1;
const int led_D1 = 13;
const int led_D2 = 12;
const int led_D3 = 11;
const int led_D4 = 10;
const int timer_value = 62410;  // 5 Hz

int t0 = 0;
int count = 0;
int state_1 = 0;
int state_2 = 0;
int gui_button = 0;
int adc_reading = 0;
//int test_state = 0;


ros::NodeHandle nh;                                     // create node
rosserial_arduino::Adc adc_msg;

std_msgs::Bool pushed_msg;                              // Bool ROS Message
ros::Publisher pub_button("pushed", &pushed_msg);       // Publisher for "pushed"
ros::Publisher pub_adc("adc", &adc_msg);


void messageCb( const std_msgs::Bool& led_msg){
  gui_button = led_msg.data;
}
ros::Subscriber<std_msgs::Bool> sub("/LED", &messageCb );

void setup_timer()
{
  noInterrupts();
  // for UNO WIFI Rev 2
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CCMP = 25000;
  TCB0.INTCTRL = TCB_CAPT_bm;
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;
  /*
  // for Mga 2560
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = timer_value;
  TCCR1B|=(1<<CS10)|(1<<CS12);
  TIMSK1|=(1<<TOIE1);
  */
  interrupts();
}

void setup_LED()
{
  pinMode(led_D1, OUTPUT);
  pinMode(led_D2, OUTPUT);
  pinMode(led_D3, OUTPUT);
  pinMode(led_D4, OUTPUT);
  digitalWrite(led_D1, HIGH);
  digitalWrite(led_D2, HIGH);
  digitalWrite(led_D3, HIGH);
  digitalWrite(led_D4, HIGH);
}

void setup_node()
{
  nh.initNode();
  nh.advertise(pub_button);
  nh.advertise(pub_adc);
  nh.subscribe(sub);
}
/*
 * State-machine 1 : Debouncing a button (A1)
 * The state will only publish true  when the first rising-edge 
 * and look for the next falling edge in the next 50 milliseconds.
 * When the falling edge occurs, it will publish false and wait for
 * another 50 milliseconds to detect the next rising-edge.
 * 
 */
void state_machine_01(bool reading)
{
  switch(state_1){
  case 0:
    if (reading){
      pushed_msg.data = reading;
      pub_button.publish(&pushed_msg);
      state_1 = 1;
      t0 = millis();
      }
    break;
  case 1:
    if (count<=50){
      count = millis()-t0;
      }
    else{
      count = 0;
      state_1 = 2;
      }
    break;
  case 2:
    if (!reading){
      pushed_msg.data = reading;
      pub_button.publish(&pushed_msg);
      state_1 = 3;
      t0 = millis();
      }
    break;
  case 3:
    if (count<=50){
      count = millis()-t0;
      }
    else {
      count = 0;
      state_1 = 0;
      }
    break;
    }
}

void setup()
{
  setup_node();
  setup_LED();
  pinMode(button_pin, INPUT);
  setup_timer();
}

/*
 * State-machine 2 : Running LEDs
 * If gui button's state is true, LEDs will turn on and off 
 * in sequence (D1,D2,D3,D4) with a delay of 100 ms=illiseconds.
 */
void state_machine_02()
{
  switch(state_2){
  case 0:
    if (gui_button){
      digitalWrite(led_D1, LOW);
      delay(100);
      digitalWrite(led_D1, HIGH);
      state_2 = 1;
      }
      break;
  case 1:
    if (gui_button){
      digitalWrite(led_D2, LOW);
      delay(100);
      digitalWrite(led_D2, HIGH);
      state_2 = 2;
      }
      break;
  case 2:
    if (gui_button){
      digitalWrite(led_D3, LOW);
      delay(100);
      digitalWrite(led_D3, HIGH);
      state_2 = 3;
      }
      break;
  case 3:
    if (gui_button){
      digitalWrite(led_D4, LOW);
      delay(100);
      digitalWrite(led_D4, HIGH);
      state_2 = 0;
      }
      break;
    } 
}
/*
 * This program consists of 2 concurrent state machines.
 * 
 */
void loop()
{
  bool reading = ! digitalRead(button_pin);
  
  state_machine_01(reading);
  state_machine_02();
  nh.spinOnce();
}

// for Mega 2560
/*
ISR(TIMER1_OVF_vect){
  TCNT1 = timer_value;
  adc_reading = analogRead(A0);
  adc_msg.adc0 = adc_reading;
  pub_adc.publish(&adc_msg);
}
*/
// for UNO WIFI Rev 2
ISR(TCB0_INT_vect)
{
  adc_reading = analogRead(A0);
  adc_msg.adc0 = adc_reading;
  pub_adc.publish(&adc_msg);
  TCB0.INTFLAGS = TCB_CAPT_bm;
  //digitalWrite(led_D1,test_state);
  //test_state = 1-test_state;
}
