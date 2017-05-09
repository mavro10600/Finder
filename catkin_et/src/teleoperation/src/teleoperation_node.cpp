#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <math.h>  
//Definicion de los botones y ejes del control de Xbox
//lalalalalala
#define Abutton joy->buttons[0]
#define Bbutton joy->buttons[1]
#define Xbutton joy->buttons[2]
#define Ybutton joy->buttons[3]

std_msgs::Int16 base_out,
				shoulder_out,
				elbow_out,
				roll_out,
				pitch_out,
				yaw_out,
				gripper_out;
				flipper1_out;
				flipper2_out;
				flipper3_out;
				flipper4_out;
				
int scale = 100;


/*
Esta funcion es la principal, al usar el joystick, lo que se deberia de hacer ahora es incluir los topicos de la base y switchear entre dos modos de uso
quizas hay que replantear la progrmamcion orientada a objetos para que sea más pequeña esta función, ya que sigue creciendo. aunque primero solo hayq ue definir 
las instrucciones así, de maera directa, y luego repensar la programacion orientada a objetos
*/

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	if(base_out.data > 2000)
		base_out.data = 2000;
	else{
		if(base_out.data < 1000)
			base_out.data = 1000;
		else
//			base_out.data += scale * round(joy->axes[0]); 
			if(joy->buttons[5] && joy->axes[0]>0.2) 
			base_out.data=round(1700+scale*joy->axes[0]);
			if(joy->buttons[5] && joy->axes[0]<-0.2) 
			base_out.data=round(1300+scale*joy->axes[0]);
			if(joy->buttons[4] && joy->axes[0]<=0.2 && joy->axes[0]>=-0.2) 
			base_out.data=1500;
			//else
			//base_out.data=1500;
	} 	
	
		if(roll_out.data > 127)
		roll_out.data = 127;
	else{
		if(roll_out.data <= 0)
			roll_out.data = 0;
		else
			if(!joy->buttons[5] && joy->axes[3]>0.2) 
			roll_out.data=round(64+18*joy->axes[3]);
			if(!joy->buttons[5] && joy->axes[3]<-0.2) 
			roll_out.data=round(64+18*joy->axes[3]);
			if(!joy->buttons[5] && joy->axes[3]<=0.2 && joy->axes[3]>=-0.2) 
			roll_out.data=64;
		}

		if(pitch_out.data > 127)
		pitch_out.data = 127;
	else{
		if(pitch_out.data <= 0)
			pitch_out.data = 0;
		else
			if(joy->buttons[5] && joy->axes[3]>0.2) 
			pitch_out.data=round(64+25*joy->axes[3]);
			if(joy->buttons[5] && joy->axes[3]<-0.2) 
			pitch_out.data=round(64+25*joy->axes[3]);
			if(joy->buttons[5] && joy->axes[3]<=0.2 && joy->axes[3]>=-0.2) 
			pitch_out.data=64;
		}			
			
	if(shoulder_out.data > 127)
		shoulder_out.data = 127;
	else{
		if(shoulder_out.data <= 0)
			shoulder_out.data = 0;
		else
			if(joy->buttons[4] && joy->axes[1]>0.1) 
			shoulder_out.data=round(64+50*joy->axes[1]);
			if(joy->buttons[4] && joy->axes[1]<-0.1) 
			shoulder_out.data=round(64+50*joy->axes[1]);
			if(joy->buttons[4] && joy->axes[1]<=0.1 && joy->axes[1]>=-0.1) 
			shoulder_out.data=64;
		}

	if(elbow_out.data > 127)
		elbow_out.data = 127;
	else{
		if(elbow_out.data <= 0)
			elbow_out.data = 0;
		else
			if(joy->buttons[4] && joy->axes[4]>0.2) 
			elbow_out.data=round(64+50*joy->axes[4]);
			if(joy->buttons[4] && joy->axes[4]<-0.2) 
			elbow_out.data=round(64+50*joy->axes[4]);
			if(joy->buttons[4] && joy->axes[4]<=0.1 && joy->axes[4]>=-0.1) 
			elbow_out.data=64;
		}
	//agregado recien
	if(Xbutton == 1)
		gripper_out.data = 3;
	else
		if(Ybutton == 1)
			gripper_out.data = -3;
		else	
			gripper_out.data = 0;

	if(Abutton == 1)
		yaw_out.data = 3;
	else
		if(Bbutton == 1)
			yaw_out.data = -3;
		else	
			yaw_out.data = 0;
}

//Función principal
int main(int argc, char **argv){
	std::cout << "Iniciallizing teleoperation FinDER node"<< std::endl;
	ros::init(argc,argv,"teleoperation_finder");
	ros::NodeHandle n;
	ros::Subscriber subJoy 		= n.subscribe<sensor_msgs::Joy>("joy",10,joyCallback);

	ros::Publisher 	base_pub 	= n.advertise<std_msgs::Int16>("base_out",10),
					shoulder_pub= n.advertise<std_msgs::Int16>("shoulder_out",10),
					elbow_pub 	= n.advertise<std_msgs::Int16>("elbow_out",10),
					roll_pub 	= n.advertise<std_msgs::Int16>("roll_out",10),
					pitch_pub 	= n.advertise<std_msgs::Int16>("pitch_out",10),
					yaw_pub		= n.advertise<std_msgs::Int16>("yaw_out",10),
					gripper_pub	= n.advertise<std_msgs::Int16>("gripper_out",10);
					flipper1_pub	= n.advertise<std_msgs::Int16>("flipper1_out",10);
					flipper2_pub	= n.advertise<std_msgs::Int16>("flipper2_out",10);
					flipper3_pub	= n.advertise<std_msgs::Int16>("flipper3_out",10);
					flipper4_pub	= n.advertise<std_msgs::Int16>("flipper4_out",10);
	ros::Rate loop_rate(10);
	std::cout << "starting publishing joy data"<<std::endl;
	base_out.data = 1500;
	roll_out.data=64;
	shoulder_out.data = 64;
	elbow_out.data=64;
	pitch_out.data=64;
	flipper1_out.data=64;
	flipper2_out.data=64;
	flipper3_out.data=64;
	flipper4_out.data=64;
	while(ros::ok()){
		//Publishing desired angles 
		base_pub.publish(base_out);
		shoulder_pub.publish(shoulder_out);
		elbow_pub.publish(elbow_out);
		roll_pub.publish(roll_out);
		pitch_pub.publish(pitch_out);
		yaw_pub.publish(yaw_out);
		gripper_pub.publish(gripper_out);
		flipper1_pub.publish(flipper1_out);
		flipper2_pub.publish(flipper2_out);
		flipper3_pub.publish(flipper3_out);
		flipper4_pub.publish(flipper4_out);

		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}
