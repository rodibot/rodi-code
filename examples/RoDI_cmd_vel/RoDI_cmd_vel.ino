/*
 * rosserial RoDI Example
 * Subscribes to cmd_vel and publishes /ultrasound
 */

#include <Servo.h>
#include <HCSR04.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;

sensor_msgs::Range range_msg;

Servo lServo;
Servo rServo;
HCSR04 dist(A2, A0); // Trigger, Echo

void messageCb( const geometry_msgs::Twist& cmd_msg) {
  if ( cmd_msg.angular.z == 0 && cmd_msg.linear.x == 0 ) {
    move_RoDI(0, 0);
  } else {
    if ( cmd_msg.angular.z < -5 ) {  // atras
      move_RoDI(-100, -100);
    } else if ( cmd_msg.angular.z > 5 ) { // adelante
      move_RoDI(100, 100);
    } else if ( cmd_msg.linear.x < -2.5 ) { // derecha
      move_RoDI(100, -100);
    } else if ( cmd_msg.linear.x > 2.5 ) {   // izquierda
      move_RoDI(-100, 100);
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", messageCb );
ros::Publisher pub_range( "/ultrasound", &range_msg);

char frameid[] = "/ultrasound";

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(pub_range);
  nh.subscribe(sub_vel);

  dist.begin();
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;
}

long range_time;

void loop()
{
  if ( millis() >= range_time ) {
    int r = 0;
    range_msg.range = dist.readCm() / 100;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 50;
  }
  nh.spinOnce();

}

void move_RoDI(int lSpeed, int rSpeed) {
  if (lSpeed == 0) {
    lServo.detach();
  } else {
    lServo.attach(5);
    lServo.write(map(lSpeed, 100, -100, 0, 180));
  }

  if (rSpeed == 0) {
    rServo.detach();
  } else {
    rServo.attach(6);
    rServo.write(map(rSpeed, -100, 100, 0, 180));
  }

}
