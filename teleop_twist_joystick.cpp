//Using the betop joystick to drive the turtle of turtlesim
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> //includes the twist msg so that we can publish twist commands to the turtle
#include <sensor_msgs/Joy.h>//includes the joystick msg so that we can listen to the joy topic


//create the TeleopTurtle class and define the joyCallback function that will take a joy msg.
class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;
				
  int Axis0_angular, Axis1_linear, Axis2_angular, Axis3_linear, Axis4_angular, Axis5_linear;
  //axis control the turtle go forward,go back,turn left and turn right
  
  int Buttons0, Buttons1, Buttons2, Buttons3, Buttons4, Buttons5, Buttons6, Buttons7, Buttons8, Buttons9, Buttons10, Buttons11;
  //the array subscript of each button
  double Axis0_angular_scale, Axis1_linear_scale, Axis2_angular_scale, Axis3_linear_scale, Axis4_angular_scale, Axis5_linear_scale;
  
  bool Buttons0_status, Buttons1_status, Buttons2_status, Buttons3_status, Buttons4_status, Buttons5_status, Buttons6_status, Buttons7_status, Buttons8_status, Buttons9_status, Buttons10_status, Buttons11_status;
  
  ros::Publisher vel_pub_;  
  ros::Subscriber joy_sub_; 

};

TeleopTurtle::TeleopTurtle() :
Axis0_angular(0), Axis1_linear(1), Axis2_angular(2), Axis3_linear(3), Axis4_angular(4), Axis5_linear(5), Buttons0(0), Buttons1(1), Buttons2(2), Buttons3(3),
Buttons4(4), Buttons5(5), Buttons6(6),Buttons7(7), Buttons8(8), Buttons9(9), Buttons10(10), Buttons11(11)
//initialize the subscript of each axis and button 

{
	
		nh_.param("axis0_angular",Axis0_angular,Axis0_angular);
		nh_.param("axis1_linear", Axis1_linear, Axis1_linear);
		nh_.param("axis2_angular", Axis2_angular, Axis2_angular);
		nh_.param("axis3_linear", Axis3_linear, Axis3_linear);
		nh_.param("axis4_angular", Axis4_angular, Axis4_angular);
		nh_.param("axis5_linear", Axis5_linear, Axis5_linear);
	    
	    nh_.param("axis0_angular_scale",Axis0_angular_scale,Axis0_angular_scale);
		nh_.param("axis1_linear_scale", Axis1_linear_scale, Axis1_linear_scale);
		nh_.param("axis2_angular_scale", Axis2_angular_scale, Axis2_angular_scale);
		nh_.param("axis3_linear_scale", Axis3_linear_scale, Axis3_linear_scale);
		nh_.param("axis4_angular_scale", Axis4_angular_scale, Axis4_angular_scale);
		nh_.param("axis5_linear_scale", Axis5_linear_scale, Axis5_linear_scale);
		
		vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    	//create a publisher that will advertise on the command_velocity topic of the turtle
	
		joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTurtle::joyCallback, this);
        //subscribe to the joystick topic for the input to drive the turtle.
		//if our node is slow in processing incoming messages on the joystick topic,up to 1 messages
		//will be buffered before any are lost
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;
	twist.angular.z = Axis0_angular_scale*joy->axes[Axis0_angular];
	twist.linear.x = Axis1_linear_scale*joy->axes[Axis1_linear];
    //take the data from the joystick and manipulate it by scaling
	//it and using independent axes to control the linear and angular
	//velocities of the turtle.
	
	vel_pub_.publish(twist);//publish the twist message
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_turtle");
	TeleopTurtle teleop_turtle;
	
    ros::spin();
}
