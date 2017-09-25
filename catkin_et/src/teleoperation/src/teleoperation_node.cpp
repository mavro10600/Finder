#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <math.h>  
#include <iostream>
#include <vector>
using namespace std;
//Definicion de los botones y ejes del control de Xbox
//lalalalalala
#define Abutton joy->buttons[0]
#define Bbutton joy->buttons[1]
#define Xbutton joy->buttons[2]
#define Ybutton joy->buttons[3]
#define RAVstick joy->axes[4]
#define RAHstick joy->axes[3]
#define LAVstick joy->axes[1]
#define LAHstick joy->axes[0]
#define LBbutton joy->buttons[4]
#define RBbutton joy->buttons[5] 
#define LTbutton joy->axes[2]  //va de 1 a -1
#define RTbutton joy->axes[5]  //va de 1 a -1
#define DpadV joy->axes[7]
#define DpadH joy->axes[6]


std::vector<int> flipper_order;
//initialize flipper vect
std::vector<int> sentido;

		
	

std_msgs::Int16 base_out,
				shoulder_out,
				elbow_out,
				roll_out,
				pitch_out,
				yaw_out,
				gripper_out,
				flipper1_out,
				flipper2_out,
				flipper3_out,
				flipper4_out;
std_msgs::Float32 base_pub,
					left_out,
					right_out;
				
int scale = 100;
int angular_rate=0;
int linear_rate=0;

/*
Esta funcion es la principal, al usar el joystick, lo que se deberia de hacer ahora es incluir los topicos de la base y switchear entre dos modos de uso
quizas hay que replantear la progrmamcion orientada a objetos para que sea más pequeña esta función, ya que sigue creciendo. aunque primero solo hayq ue definir 
las instrucciones así, de maera directa, y luego repensar la programacion orientada a objetos
*/

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
float sens=0.1;
float tsens=0.2;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Rutinas que mueven los flippers
/*
	if(flipper1_out.data > 127)
		flipper1_out.data = 127;
	else{
		if(flipper1_out.data <= 0)
			flipper1_out.data = 0;
		else
			{
			if(LBbutton && RBbutton && RAVstick>tsens) 
			flipper1_out.data=round(64+50*RAVstick);
			if(LBbutton && RBbutton && RAVstick<-tsens) 
			flipper1_out.data=round(64+50*RAVstick);
			if( (LBbutton &&  RBbutton && RAVstick>=-tsens && RAVstick<=tsens) ||(!LBbutton && !RBbutton && RAVstick>tsens) || (!LBbutton && !RBbutton && RAVstick<-tsens) || ((!LBbutton || !RBbutton) && RAVstick>tsens) || ((!LBbutton || !RBbutton) && RAVstick<-tsens) ) 
			flipper1_out.data=64;
			}
		}			
		
	if(flipper2_out.data > 127)
		flipper2_out.data = 127;
	else{
		if(flipper2_out.data <= 0)
			flipper2_out.data = 0;
		else
			{
			if(LBbutton && RBbutton && RAHstick>tsens) 
			flipper2_out.data=round(64+50*RAHstick);
			if(LBbutton && RBbutton && RAHstick<-tsens) 
			flipper2_out.data=round(64+50*RAHstick);
			if((LBbutton &&  RBbutton && RAHstick>=-tsens && RAHstick<=tsens  ) ||(!LBbutton && !RBbutton && RAHstick>tsens) || (!LBbutton && !RBbutton && RAHstick<-tsens) || ((!LBbutton || !RBbutton) && RAHstick>tsens) || ((!LBbutton || !RBbutton) && RAHstick<-tsens)) 
			flipper2_out.data=64;
			}
		}			
		
			if(flipper3_out.data > 127)
		flipper3_out.data = 127;
	else{
		if(flipper3_out.data <= 0)
			flipper3_out.data = 0;
		else
			{
			if(LBbutton && RBbutton && LAVstick>tsens) 
			flipper3_out.data=round(64+50*LAVstick);
			if(LBbutton && RBbutton && LAVstick<-tsens) 
			flipper3_out.data=round(64+50*LAVstick);
			if((LBbutton &&  RBbutton && LAVstick<=tsens && LAVstick>=-tsens)   ||(!LBbutton && !RBbutton && LAVstick>tsens) || (!LBbutton && !RBbutton && LAVstick<-tsens) || ((!LBbutton || !RBbutton) && LAVstick>tsens) || ((!LBbutton || !RBbutton) && LAVstick<-tsens)) 
			flipper3_out.data=64;
			}
		}			
		
			if(flipper4_out.data > 127)
		flipper4_out.data = 127;
	else{
		if(flipper4_out.data <= 0)
			flipper4_out.data = 0;
		else
{
			if(LBbutton && RBbutton && LAHstick>tsens) 
			flipper4_out.data=round(64+50*LAHstick);
			if(LBbutton && RBbutton && LAHstick<-tsens) 
			flipper4_out.data=round(64+50*LAHstick);
			if((LBbutton &&  RBbutton && LAHstick>=-tsens && LAHstick<=tsens  ) ||(!LBbutton && !RBbutton && LAHstick>tsens) || (!LBbutton && !RBbutton && LAHstick<-tsens) || ((!LBbutton || !RBbutton) && LAHstick>tsens) || ((!LBbutton || !RBbutton) && LAHstick<-tsens)) 
			flipper4_out.data=64;
			}
		}			

*/



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Rutinas que mueven los flippers
	
	if(flipper1_out.data > 127)
		flipper1_out.data = 127;
	else{
		if(flipper1_out.data <= 0)
			flipper1_out.data = 0;
		else
			{
			
			if(LBbutton && RBbutton && joy->axes[int(flipper_order[0])]>tsens)
			{ 
			if(sentido[0]==0){ 
				flipper1_out.data=round(64+50*joy->axes[int(flipper_order[0])]);}
			if(sentido[0]==1){ 
				flipper1_out.data=round(64+50*(-joy->axes[int(flipper_order[0])]));}

			}
			
			if(LBbutton && RBbutton && joy->axes[int(flipper_order[0])]<-tsens)
			{
				if(sentido[0]==0){ 
				flipper1_out.data=round(64+50*joy->axes[int(flipper_order[0])]);}
				if(sentido[0]==1){ 
				flipper1_out.data=round(64+50*(-joy->axes[int(flipper_order[0])]));}
			}
			
			if( (LBbutton &&  RBbutton && joy->axes[int(flipper_order[0])]<=tsens && joy->axes[int(flipper_order[0])]>=-tsens) ||(!LBbutton && !RBbutton && joy->axes[int(flipper_order[0])]>tsens) || (!LBbutton && !RBbutton && joy->axes[int(flipper_order[0])]<-tsens) || ((!LBbutton || !RBbutton) && joy->axes[int(flipper_order[0])]>tsens) || ((!LBbutton || !RBbutton) && joy->axes[int(flipper_order[0])]<-tsens) ) 
			flipper1_out.data=64;
			}
		}			
		
		
		
	if(flipper2_out.data > 127)
		flipper2_out.data = 127;
	else{
		if(flipper2_out.data <= 0)
			flipper2_out.data = 0;
		else
			{
			if(LBbutton && RBbutton && joy->axes[int(flipper_order[1])]>tsens)
			{ 
				if(sentido[1]==0){ 
				flipper2_out.data=round(64+50*joy->axes[int(flipper_order[1])]);}
				if(sentido[1]==1){ 
				flipper2_out.data=round(64+50*(-joy->axes[int(flipper_order[1])]));}

			}
			if(LBbutton && RBbutton && joy->axes[int(flipper_order[1])]<-tsens)
			{ 
				if(sentido[1]==0){ 
				flipper2_out.data=round(64+50*joy->axes[int(flipper_order[1])]);}
				if(sentido[1]==1){ 
				flipper2_out.data=round(64+50*(-joy->axes[int(flipper_order[1])]));}
			
			}
						
			if((LBbutton &&  RBbutton && joy->axes[int(flipper_order[1])]<=tsens && joy->axes[int(flipper_order[1])]>=-tsens  ) ||(!LBbutton && !RBbutton && joy->axes[int(flipper_order[1])]>tsens) || (!LBbutton && !RBbutton && joy->axes[int(flipper_order[1])]<-tsens) || ((!LBbutton || !RBbutton) && joy->axes[int(flipper_order[1])]>tsens) || ((!LBbutton || !RBbutton) && joy->axes[int(flipper_order[1])]<-tsens)) 
			flipper2_out.data=64;
			}
		}			
		
			if(flipper3_out.data > 127)
		flipper3_out.data = 127;
	else{
		if(flipper3_out.data <= 0)
			flipper3_out.data = 0;
		else
			{
			if(LBbutton && RBbutton && joy->axes[flipper_order[2]]>tsens) 
			{
							if(sentido[2]==0){ 
				flipper3_out.data=round(64+50*joy->axes[int(flipper_order[2])]);}
				if(sentido[2]==1){ 
				flipper3_out.data=round(64+50*(-joy->axes[int(flipper_order[2])]));}

			}
			if(LBbutton && RBbutton && joy->axes[flipper_order[2]]<-tsens)
			{ 
							if(sentido[2]==0){ 
				flipper3_out.data=round(64+50*joy->axes[int(flipper_order[2])]);}
				if(sentido[2]==1){ 
				flipper3_out.data=round(64+50*(-joy->axes[int(flipper_order[2])]));}

			}
			if((LBbutton &&  RBbutton && joy->axes[flipper_order[2]]<=tsens && joy->axes[int(flipper_order[2])]>=-tsens)   ||(!LBbutton && !RBbutton && joy->axes[int(flipper_order[2])]>tsens) || (!LBbutton && !RBbutton && joy->axes[int(flipper_order[2])]<-tsens) || ((!LBbutton || !RBbutton) && joy->axes[int(flipper_order[2])]>tsens) || ((!LBbutton || !RBbutton) && joy->axes[int(flipper_order[2])]<-tsens)) 
			flipper3_out.data=64;
			}
		}			
		
			if(flipper4_out.data > 127)
		flipper4_out.data = 127;
	else{
		if(flipper4_out.data <= 0)
			flipper4_out.data = 0;
		else
{
			if(LBbutton && RBbutton && joy->axes[int(flipper_order[3])]>tsens) 
			flipper4_out.data=round(64+50*joy->axes[int(flipper_order[3])]);
			if(LBbutton && RBbutton && joy->axes[int(flipper_order[3])]<-tsens) 
			flipper4_out.data=round(64+50*joy->axes[int(flipper_order[3])]);
			if((LBbutton &&  RBbutton && joy->axes[int(flipper_order[3])]<=tsens && joy->axes[int(flipper_order[3])]>=-tsens  ) ||(!LBbutton && !RBbutton && joy->axes[int(flipper_order[3])]>tsens) || (!LBbutton && !RBbutton && joy->axes[int(flipper_order[3])]<-tsens) || ((!LBbutton || !RBbutton) && joy->axes[int(flipper_order[3])]>tsens) || ((!LBbutton || !RBbutton) && joy->axes[int(flipper_order[3])]<-tsens)) 
			flipper4_out.data=64;
			}
		}			
		
//////////////////////////////////////////////////////////////////////////////////////////////////////////

//Rutinas extras que mueven los bracitos combinados


/* 
	if(flipper1_out.data > 100 || flipper2_out.data>100)
		{if (flipper1_out.data>100) flipper1_out.data = 100;
		if(flipper2_out.data>100) flipper2_out.data=100;}
	else{
		if(flipper1_out.data < -100  || flipper1_out.data < -100)
			{if (flipper1_out.data<-100) flipper1_out.data = -100;
			if (flipper2_out.data<-100) flipper2_out.data=-100;}
		else
//			base_out.data += scale * round(joy->axes[0]); 
			if(!joy->buttons[4] && !joy->buttons[5] && joy->axes[3]>0.2) 
			{
			flipper1_out.data=round(64+50*joy->axes[3]);
			flipper2_out.data=round(64+50*joy->axes[3]);
			}
			if(!joy->buttons[4] && !joy->buttons[5] && joy->axes[3]<-0.2) 
			{
			flipper1_out.data=round(64+50*joy->axes[3]);
			flipper2_out.data=round(64+50*joy->axes[3]);
			}
			if(!joy->buttons[4]  && !joy->buttons[5] && joy->axes[3]<=0.2 && joy->axes[3]>=-0.2) 
			{
			flipper1_out.data=0;
			flipper2_out.data=0;
			}
			//else
			//base_out.data=1500;
	}
	
	if(flipper3_out.data > 100 || flipper4_out.data>100)
		{if (flipper3_out.data>100) flipper3_out.data = 100;
		if(flipper4_out.data>100) flipper4_out.data=100;}
	else{
		if(flipper3_out.data < -100  || flipper4_out.data < -100)
			{if (flipper3_out.data<-100) flipper3_out.data = -100;
			if (flipper4_out.data<-100) flipper4_out.data=-100;}
		else
//			base_out.data += scale * round(joy->axes[0]); 
			if(!joy->buttons[4] && !joy->buttons[5] && joy->axes[4]>0.2) 
			{
			flipper3_out.data=round(64+50*joy->axes[4]);
			flipper4_out.data=round(64+50*joy->axes[4]);
			}
			if(!joy->buttons[4] && !joy->buttons[5] && joy->axes[4]<-0.2) 
			{
			flipper3_out.data=round(64+50*joy->axes[4]);
			flipper4_out.data=round(64+50*joy->axes[4]);
			}
			if(!joy->buttons[4]  && !joy->buttons[5] && joy->axes[4]<=0.2 && joy->axes[4]>=-0.2) 
			{
			flipper3_out.data=0;
			flipper4_out.data=0;
			}
			//else
			//base_out.data=1500;
	}	
	 		
*/
	
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Rutina que mueve las ruedas de tracción	

	
	if(left_out.data > 100 || right_out.data>100)
		{if (left_out.data>100) left_out.data = 100;
		if(right_out.data>100) right_out.data=100;}
	else{
		if(left_out.data < -100  || right_out.data < -100)
			{if (left_out.data<-100) left_out.data = -100;
			if (right_out.data<-100)right_out.data=-100;}
		else
			{
//			base_out.data += scale * round(joy->axes[0]); 
			if((!LBbutton && !RBbutton && LAHstick>tsens) || (!LBbutton && !RBbutton && LAVstick>tsens) || (!LBbutton && !RBbutton && DpadV>0.9) || (!LBbutton && !RBbutton && DpadH>0.9)) 
			{
			if(LAHstick>tsens || LAVstick> tsens)
			{
			angular_rate=LAHstick*100;
			linear_rate=LAVstick*100;
			}
			if(DpadV >0.9 || DpadH > 0.9)
			{
			angular_rate=0.5*100;
			linear_rate=0.5*100;
			}
			right_out.data=linear_rate+angular_rate;
			left_out.data=linear_rate-angular_rate;
			}
			if((!LBbutton && !RBbutton && LAHstick<-tsens )|| ( !LBbutton && !RBbutton && LAVstick<-tsens ) || ( !LBbutton && !RBbutton && DpadH<-0.9) ||  ( !LBbutton && !RBbutton && DpadV<-0.9)) 
			{
			if(LAHstick< -tsens || LAVstick< -tsens)
			{
			angular_rate=LAHstick*100;
			linear_rate=LAVstick*100;
			}
			if(DpadV < -0.9 || DpadH < -0.9)
			{
			angular_rate=0.5*100;
			linear_rate=0.5*100;
			}
			
			right_out.data=linear_rate+angular_rate;
			left_out.data=linear_rate-angular_rate;
			}
			
			if(!LBbutton  && !RBbutton && LAHstick<=0.2 && LAHstick>=-0.2 && LAVstick<=0.2 && LAVstick>=-0.2) 
			{left_out.data=0;
			right_out.data=0;}
			//else
			//base_out.data=1500;
			}
	} 
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Rutinas que mueven el brazo	
	
	if(base_out.data > 2000)
		base_out.data = 2000;
	else{
		if(base_out.data < 1000)
			base_out.data = 1000;
		else
		{
//			base_out.data += scale * round(joy->axes[0]); 
			if(LBbutton && !RBbutton  && LAHstick>0.3) 
			base_out.data=round(1700+scale*LAHstick);
			if(LBbutton && !RBbutton && LAHstick<-0.3) 
			base_out.data=round(1300+scale*LAHstick);
			if(LBbutton && !RBbutton && LAHstick<= 0.3 && LAHstick >= -0.3) 
			base_out.data=1500;
			//else
			//base_out.data=1500;
		}
	} 	
	


	if(shoulder_out.data > 127)
		shoulder_out.data = 127;
	else{
		if(shoulder_out.data <= 0)
			shoulder_out.data = 0;
		else
			{
			if(LBbutton && !RBbutton && LAVstick>tsens) 
			shoulder_out.data=round(64+50*LAVstick);
			if(LBbutton && !RBbutton && LAVstick<-tsens) 
			shoulder_out.data=round(64+50*LAVstick);
			if(LBbutton && !RBbutton && LAVstick<=tsens && LAVstick>=-tsens) 
			shoulder_out.data=64;
			}
		}

	if(elbow_out.data > 100)
		elbow_out.data = 100;
	else{
		if(elbow_out.data <= 30)
			elbow_out.data = 30;
		else
			{
			if(LBbutton && !RBbutton && RAVstick>tsens) 
			elbow_out.data=round(64+30*RAVstick);
			if(LBbutton && !RBbutton && RAVstick<-tsens) 
			elbow_out.data=round(64+30*RAVstick);
			if(LBbutton && !RBbutton && RAVstick<=tsens && RAVstick>=-tsens) 
			elbow_out.data=64;
			}
		}
	
	/*
		if(roll_out.data > 100)
		roll_out.data = 100;
	else{
		if(roll_out.data <= 30)
			roll_out.data = 30;
		else
			{
			if(LBbutton && !RBbutton && RAHstick>tsens) 
			roll_out.data=round(64+10*RAHstick);
			if(LBbutton && !RBbutton && RAHstick<-tsens) 
			roll_out.data=round(64+10*RAHstick);
			if(LBbutton && !RBbutton && RAHstick<=tsens && RAHstick>=-tsens) 
			roll_out.data=64;
			}
		}
*/
		if(pitch_out.data > 100)
		pitch_out.data = 100;
	else{
		if(pitch_out.data <= 30)
			pitch_out.data = 30;
		else
			{
			if(!LBbutton && RBbutton && RAVstick>tsens) 
			pitch_out.data=round(64+30*RAVstick);
			if(!LBbutton && RBbutton && RAVstick<-tsens) 
			pitch_out.data=round(64+3*RAVstick);
			if(!LBbutton && RBbutton && RAVstick<=tsens && RAVstick>=-tsens) 
			pitch_out.data=64;
			}
		}			
	/*		
		if(yaw_out.data > 100)
		yaw_out.data = 100;
	else{
		if(yaw_out.data <= 30)
			yaw_out.data = 30;
		else
			{
			if(!LBbutton && RBbutton && RAHstick>tsens) 
			yaw_out.data=round(64+10*RAHstick);
			if(!LBbutton && RBbutton && RAHstick<-tsens) 
			yaw_out.data=round(64+10*RAHstick);
			if(!LBbutton && RBbutton && RAHstick<=tsens && RAHstick>=-tsens) 
			yaw_out.data=64;
			}
		}
*/
		//yaw_out.data=64;
	//agregado recien
	if(Xbutton == 1)
		gripper_out.data = 5;
	else
		if(Ybutton == 1)
			gripper_out.data = -5;
		else	
			gripper_out.data = 0;

}

void mapeo(std::vector<int>  order,std::vector<int> &flipper_order)
{
//std::vector<int>::iterator i;
	for (int i=0;i<4;i++)
	{
		if (order[i]==1){flipper_order[i]=4;}
		if (order[i]==2)
		{//flipper_order.insert(i,3);
		flipper_order[i]=3;
		cout<<"order:"<<i;}
		if (order[i]==3){
		flipper_order[i]=1;
		//flipper_order.insert(i,1);
		cout<<"order"<<i;}
		if (order[i]==4){
		flipper_order[i]=0;
		//flipper_order.insert(i,0);
		cout<<"order="<<i<<endl;}
	}
	cout<<flipper_order[0]<<endl;
	cout<<flipper_order[1]<<endl;
	cout<<flipper_order[2]<<endl;	
	cout<<flipper_order[3]<<endl;
}

//Función principal
int main(int argc, char **argv){


	std::cout << "Iniciallizing teleoperation FinDER node"<< std::endl;
	ros::init(argc,argv,"teleoperation_finder");
	ros::NodeHandle n;
	std::vector<int> order;


  flipper_order.push_back(4);
  flipper_order.push_back(3);
  flipper_order.push_back(1);
  flipper_order.push_back(0); 
 cout<<"flipper_order"<<endl;
    cout<<flipper_order[0]<<endl;
	cout<<flipper_order[1]<<endl;
	cout<<flipper_order[2]<<endl;	
	cout<<flipper_order[3]<<endl;

if(ros::param::has("sentido"))
{
	n.getParam("sentido",sentido);
cout<<"sentido"<<endl;
	cout<<sentido[0]<<endl;
	cout<<sentido[1]<<endl;
	cout<<sentido[2]<<endl;	
	cout<<sentido[3]<<endl;
}
else
{
	sentido.push_back(0);
	sentido.push_back(0);
	sentido.push_back(0);
	sentido.push_back(0);
	cout <<"No param found, using default values for sentido"<<endl;
}	


  

if(ros::param::has("order"))
{
	n.getParam("order",order);
cout<<"order"<<endl;
	cout<<order[0]<<endl;
	cout<<order[1]<<endl;
	cout<<order[2]<<endl;	
	cout<<order[3]<<endl;
}
else
{
	order.push_back(1);
	order.push_back(2);
	order.push_back(3);
	order.push_back(4);
	cout <<"No param found, using default values"<<endl;
}	
	
mapeo(order,flipper_order);
/*cout<<"flipper_order"<<endl;
    cout<<flipper_order[0]<<endl;
	cout<<flipper_order[1]<<endl;
	cout<<flipper_order[2]<<endl;	
	cout<<flipper_order[3]<<endl;
	*/
	ros::Subscriber subJoy 		= n.subscribe<sensor_msgs::Joy>("joy",10,joyCallback);

	ros::Publisher 	base_pub 	= n.advertise<std_msgs::Int16>("base_out",10),
					shoulder_pub= n.advertise<std_msgs::Int16>("shoulder_out",10),
					elbow_pub 	= n.advertise<std_msgs::Int16>("elbow_out",10),
					roll_pub 	= n.advertise<std_msgs::Int16>("roll_out",10),
					pitch_pub 	= n.advertise<std_msgs::Int16>("pitch_out",10),
					yaw_pub		= n.advertise<std_msgs::Int16>("yaw_out",10),
					gripper_pub	= n.advertise<std_msgs::Int16>("gripper_out",10),
					flipper1_pub	= n.advertise<std_msgs::Int16>("flipper1_out",10),
					flipper2_pub	= n.advertise<std_msgs::Int16>("flipper2_out",10),
					flipper3_pub	= n.advertise<std_msgs::Int16>("flipper3_out",10),
					flipper4_pub	= n.advertise<std_msgs::Int16>("flipper4_out",10),
					left_pub=n.advertise<std_msgs::Float32>("left_des",10),
					right_pub=n.advertise<std_msgs::Float32>("right_des",10);
	ros::Rate loop_rate(10);
	std::cout << "starting publishing joy data"<<std::endl;
	base_out.data = 1500;
	roll_out.data=64;
	shoulder_out.data = 64;
	elbow_out.data=64;
	pitch_out.data=64;
	yaw_out.data=64;
	flipper1_out.data=64;
	flipper2_out.data=64;
	flipper3_out.data=64;
	flipper4_out.data=64;
	left_out.data=0;
	right_out.data=0;
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
		left_pub.publish(left_out);
		right_pub.publish(right_out);
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}
