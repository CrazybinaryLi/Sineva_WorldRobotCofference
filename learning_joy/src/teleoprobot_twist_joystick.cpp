//Using the betop joystick to drive the Robot
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> //includes the twist msg so that we can publish twist commands to the Robot
#include <sensor_msgs/Joy.h>//includes the joystick msg so that we can listen to the joy topic


/*
Reading from the joystick and Publishing to Robot!
-------------------------------------------------------------------------------------------------
Under P&D-A mode
Key distribution:
The front of the joystick:
          (Axis1 up)                                
               |                     (BACK)  -     - (START)             (ButtonY)   Y
(Axis0 left) -   - (Axis0 right)     (Pairing ) -                       (ButtonX) X     B (ButtonB) 
               |                                                           (ButtonA) A
         (Axis1 down)	    
		    (Axis5 up)                                      (Axis3 up)
		        |                                                |
(Axis4 left) ！   ！(Axis4 right)                  (Axis2 left)！  ！(Axis2 right) 
                |	                                             |
		  (Axis5 down)                                     (Axis3 down) 

The side of the joystick:
         (ButtonLB)                                                             (ButtonRB)
             -                                                                     -
             |                                                                     |
         (ButtonLT)                                                             (ButtonRT)
--------------------------------------------------------------------------------------------------
*/


//Create the TeleopRobot class and define the joyCallback function that will take a joy msg.
class TeleopRobot
{
public:
  TeleopRobot();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;
  
  int Axis0_turn, Axis1_linear, Axis2_turn, Axis3_linear, Axis4_turn, Axis5_linear;
  //axis control the direction
  
  int ButtonY,ButtonB, ButtonA, ButtonX, ButtonLB, ButtonRB, ButtonLT, ButtonRT, BACK, START, AxisandButton1, AxisandButton2;
  //the array subscript of each button in the buttons[]
  double Axis0_turn_scale, Axis1_linear_scale, Axis2_turn_scale, Axis3_linear_scale, Axis4_turn_scale, Axis5_linear_scale;
  
  double ButtonsY_scale, ButtonsB_scale, ButtonsA_scale, ButtonsX_scale, ButtonsLB_scale, ButtonsRB_scale, ButtonsLT_scale, ButtonsRT_scale, BACK_scale, START_scale, AxisandButton1,AxisandButton2;
  
  ros::Publisher vel_pub_base,vel_pub_head;
  ros::Subscriber joy_sub_; 

};

TeleopRobot::TeleopRobot() :
Axis0_turn(0), Axis1_linear(1), 
Axis2_turn(2), Axis3_linear(3), 
Axis4_turn(4), Axis5_linear(5), 
ButtonY(0), ButtonB(1), ButtonA(2), ButtonX(3),
ButtonLB(4), ButtonRB(5), ButtonLT(6),ButtonRT(7), 
BACK(8),START(9),AxisandButton1(10), AxisandButton2(11)
//initialize the subscript of each axis and button in the axes[] and buttons[] 

{
	
		nh_.param("axis_base_angular",Axis0_turn,Axis0_turn);
		nh_.param("axis_base_linear", Axis1_linear, Axis1_linear);
		nh_.param("axis_head_tilt", Axis2_turn, Axis2_turn);
		nh_.param("axis_head_pan", Axis3_linear, Axis3_linear);
		nh_.param("axis4_turn", Axis4_turn, Axis4_turn);
		nh_.param("axis5_linear", Axis5_linear, Axis5_linear);
	    
	    nh_.param("vel_scale_base_angular",Axis0_turn_scale,Axis0_turn_scale);
		nh_.param("vel_scale_base_linear", Axis1_linear_scale, Axis1_linear_scale);
		nh_.param("vel_scale_head_tilt", Axis2_turn_scale, Axis2_turn_scale);
		nh_.param("vel_scale_head_pan", Axis3_linear_scale, Axis3_linear_scale);
		nh_.param("axis4_turn_scale", Axis4_turn_scale, Axis4_turn_scale);
		nh_.param("axis5_linear_scale", Axis5_linear_scale, Axis5_linear_scale);
		
		vel_pub_base = nh_.advertise<geometry_msgs::Twist>("base/cmd_vel", 1);
    	//create a publisher that will advertise on the command_velocity topic of the Robot's base

		vel_pub_head = nh_.advertise<geometry_msgs::Twist>("head/cmd_vel", 1);
		//create a publisher that will advertise on the command_velocity topic of the Robot's head
	
		joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTurtle::joyCallback, this);
        //subscribe to the joystick topic for the input to drive the Robot.
		//if our node is slow in processing incoming messages on the joystick topic,up to 1 messages
		//will be buffered before any are lost
}

void TeleopRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist_base,twist_head;
	twist_base.angular.z = Axis0_turn_scale*joy->axes[Axis0_turn];
	twist_base.linear.x  = Axis1_linear_scale*joy->axes[Axis1_linear];
    //take the data from the joystick and manipulate it by scaling
	//it and using independent axes to control the linear and angular
	//velocities of the Robot's base.

	twist_head.angular.y = Axis2_turn_scale*joy->axes[Axis2_turn];
	twist_head.angular.z = Axis3_linear_scale*joy->axes[Axis3_linear];
	
	vel_pub_base.publish(twist_base);
	vel_pub_head.publish(twist_head);
	//publish the twist message
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_robot");
	TeleopRobot teleop_robot;
	
    ros::spin();
}