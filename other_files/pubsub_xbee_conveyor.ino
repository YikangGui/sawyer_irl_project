/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;

char hello[13] = "hello world!";

void messageCb( const std_msgs::Empty& toggle_msg);

ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );


ros::Publisher chatter("chatter", &str_msg);

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  str_msg.data = hello;
  chatter.publish( &str_msg );
}

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(100);
}
