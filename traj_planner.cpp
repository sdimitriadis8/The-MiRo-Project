// This is a path planner of MiRo Prototype,
// using either Square/Cubic polynomials of time
// or Lie brackets.
// Designed to run strictly in Robot Operating System (ROS)!!

#include <ros/ros.h> //This header includes the standard ROS classes
#include <iostream> //This header includes standard I/O objects
#include <sstream> //This header includes string-based streams
#include <fstream> //This header includes file-based streams
#include <cmath> // This header includes C++ mathematical library
//				 // like the Standard C Library header @c math.h
#include <cstring>  // This is the C++ version of the Standard C
//					// Library header @c string.h
//
// Other headers including ROS libraries		
#include <tf/transform_broadcaster.h> // A transform broadcaster from the tf library
#include <geometry_msgs/Pose2D.h> // Message used for transponding poses of robot on the xy-plane
#include <std_msgs/String.h> // Message used for transponding strings
#include <std_msgs/Int32.h> // Message used for transponding 32-bit integers
#include <miro3/AngVel.h> // Custom-defined message used for transmitting angular velocities
#include <miro3/AskConf.h> // Custom-defined service giving configurations when requested
#include <miro3/AskVel.h> // Custom-defined service giving angular velocities when requested

using namespace std;

// Global variables:

#define PI 3.14159265

//Defining some strings/colors/preferences
const std::string red("\033[0;31m"); // Simple red color text
const std::string redb("\033[1;31m"); // Bold red color text

const std::string blue("\033[0;34m"); // Simple blue color text
const std::string blueb("\033[1;34m"); // Bold blue color text

const std::string green("\033[0;32m"); // Simple green color text
const std::string greenb("\033[1;32m"); // Bold green color text

const std::string white("\033[0;37m"); // Simple white color text

const std::string un("\033[4;37m"); // Underlined white text
const std::string unoff("\033[27;37m"); // Stop underlining text
const std::string boff("\033[21;37m"); // Stop bolding text

// Initial & Current configuration variables
double CMX;
double CMY;
double CMpsi;

// Arrays for storing the desired configurations
double X;
double Y;
double TH;

// Goal/Final configuration
double GoalX = 0.0;
double GoalY = 0.0;
double GoalTH = 0.0;

double D_X;
double D_Y;
double D_TH;

double dt=pow(10,-4);

double timef;

int concheck=0;

int choice=0;
// char pauses='N';

double CMX_DD;
double CMY_DD;
double CMTH_DD;

double n=0;

bool init_conf(miro3::AskConf::Request &req, miro3::AskConf::Response &res)
{
	if (req.r==1)
	{
		cout<<blue<<"The initial configuration is..."<<endl;
    		// Building up the initial configuration...
		// ... consisted of: (x,y,theta)
        	res.pose.x = CMX;
        	res.pose.y = CMY;
        	res.pose.theta = CMpsi;

		// Configuration is sent as it was get by the response
		// for completeness and verification
		cout<<white<<"("<<res.pose.x<<", "<<res.pose.y<<", "<<res.pose.theta<<")"<<endl;
			
		req.r=0;
	}
	return true;
}

bool final_conf(miro3::AskConf::Request &req, miro3::AskConf::Response &res)
{
	if (req.r==1)
	{
		cout<<blue<<"The goal configuration is..."<<endl;
    		// Building up the initial configuration...
		// ... consisted of: (x,y,theta)
        	res.pose.x = GoalX;
        	res.pose.y = GoalY;
        	res.pose.theta = GoalTH;

		// Configuration is sent as it was get by the response
		// for completeness and verification
		cout<<white<<"("<<res.pose.x<<", "<<res.pose.y<<", "<<res.pose.theta<<")"<<endl;
			
		req.r=0;
	}
	return true;
}

// Path planning function, getting the initial and
// the goal configuration of the robot and returning
// a sequence of configurations to reach it, using 
// Lie brackets
void lie_brackets()
{

	cout<<green<<"Done!!"<<white<<endl;
}

// Path planning function, getting the initial and
// the goal configuration of the robot and returning
// a sequence of configurations to reach it, using 
// 2nd order polynomials
void traj_plan2()
{
	D_X = GoalX-CMX; // Difference on x-axis
	D_Y = GoalY-CMY; // Difference on y-axis
	D_TH = GoalTH-CMpsi; // Difference on z-axis (orientation)
	
	// Square polynomials of time, constructing a sequence
	// of poses (x,y,theta)
	X = CMX+D_X*pow((n/(double)timef),2);
	Y = CMY+D_Y*pow((n/(double)timef),2);
	TH = CMpsi+D_TH*pow((n/(double)timef),2);

	cout<<green<<"Done!!"<<white<<endl;
}

// Path planning function, getting the initial and
// the goal configuration of the robot and returning
// a sequence of configurations to reach it, using 
// 3rd order polynomials
void traj_plan3()
{
	D_X = GoalX-CMX; // Difference on x-axis
	D_Y = GoalY-CMY; // Difference on y-axis
	D_TH = GoalTH-CMpsi; // Difference on z-axis (orientation)
	
	// Uncomment the if-else section in case you are
	// interested in particular end-point velocity and
	// DO NOT insert yes ("Y"/"y") when prompted
	// if(pauses=='y' || pauses=='Y');
	//	{
			// Cubic polynomials of time, constructing a sequence
			// of poses (x,y,theta)
			X = CMX+3*D_X*pow((n/(double)timef),2)-2*D_X*pow((n/(double)timef),3);
			Y = CMY+3*D_Y*pow((n/(double)timef),2)-2*D_Y*pow((n/(double)timef),3);
			TH = CMpsi+3*D_TH*pow((n/(double)timef),2)-2*D_TH*pow((n/(double)timef),3);

	//	}
	//else
	//{
	//	cout<<"Insert the desired end-point velocity vector"<<endl;
	//	cin>>CMX_DD>>CMY_DD>>CMTH_DD;
	//
	//	for(int i=0; i<(dt+1); i++)
	//	{
	//		double j=i;
	//		X[i] = CMX+CMX_D*j+3*D_X*pow((j/dt),2)-(CMX_DD-CMX_DC)*pow(j,2)/dt-2*D_X*pow((j/dt),3);
	//		Y[i]=CMY+3*D_Y*pow((j/dt),2)-2*D_Y*pow((j/dt),3);
	//		TH[i]=CMpsi+3*D_TH*pow((j/dt),2)-2*D_TH*pow((j/dt),3);
	//	}
	//}
	cout<<green<<"Done!!"<<white<<endl;
}

int main(int argc, char **argv)
{
	fstream fsX("PlanX.dat",fstream::out);
	fstream fsY("PlanY.dat",fstream::out);
	fstream fsTH("PlanTH.dat",fstream::out);
	fstream fsP("PlanXY.dat",fstream::out);

	//Initialize the ROS system.
    	ros::init(argc, argv, "miro3_traj");

    	//Establish this program as a ROS node.
    	ros::NodeHandle nt;

	// Output messages for the user to input initial/goal values
	cout<<endl;
	cout<<blueb<<"Welcome to MiRo Path Planner"<<endl;
	cout<<"----------------------------------"<<endl;
	cout<<white<<"Insert the initial configuration (in decimals)"<<endl;
	cout<<"Type of input: double "<<endl;
	cout<<"Measures: coordinates in meters, angles in radiants"<<endl;
	cin>>CMX>>CMY>>CMpsi;
	cout<<"Insert the final configuration, likewise"<<endl;
	cin>>GoalX>>GoalY>>GoalTH;
	
	while(timef<=0)
	{
		// Output message for the user to choose an option
		// from the ones provided below
		cout<<"Select one of the following procedures: "<<endl;
		cout<<"1) Lie Brackets "<<endl;
		cout<<"2) 2nd Order (Square) Polynomials "<<endl;
		cout<<"3) 3rd Order (Cubic) Polynomials "<<endl;
		cin>>choice;
		// Catching false data type inputs
		while (!cin || (choice!=1 && choice!=2 && choice!=3) )
		{
			std::cin.clear(); // Clearing bad input flag
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discarding input
			std::cout<<redb<<"There is not an available selection labeled "<<choice<<" !!"<<white<<endl;
			std::cout<<"Please enter a valid option"<<endl;
			cin>>choice;
		}

		// Uncomment the following lines if particular end-point velocity is desired 
		//cout<<"Would you like to allow pauses at intermediate points? (Y/N)"<<endl;
		//cin>>pauses;

		cout<<"Enter preferred time used for planning"<<endl;
		cin>>timef;
	}

	// Service servers responding the initial and the final configuration  
	// to the controller, respectively
	ros::ServiceServer serv_init = nt.advertiseService("/q_init", init_conf);

	ros::ServiceServer serv_qf = nt.advertiseService("/q_fin", final_conf);
	
	ros::Publisher q_pub = nt.advertise<geometry_msgs::Pose2D>("/q_des", 1000);
	
	geometry_msgs::Pose2D des_pos;

	ros::Rate lr(1000); // Loop rate in Hz

	while(ros::ok())
	{
		if(choice==1)
		{
			cout<<blue<<"Getting next configuration using lie brackets. Please wait... "<<endl;
			cout<<white<<endl;
			lie_brackets();
		}
		else if(choice==2)
		{
			cout<<blue<<"Getting next configuration using square polynomial of time. Please wait... "<<endl;
			cout<<white<<endl;
			traj_plan2();
		}
		else if(choice==3)
		{
			cout<<blue<<"Getting next configuration using cubic polynomial of time. Please wait... "<<endl;
			cout<<white<<endl;
			traj_plan3();
		}
		else
		{
			cout<<blue<<"No more configurations left to sent!"<<endl;
			cout<<"Pending platform to reach the goal..."<<endl;
			cout<<white<<endl;
		}

		if(n<timef)
		{
			des_pos.x = X;
			des_pos.y = Y;
			des_pos.theta = TH;
			
			fsX<<n<<"\t"<<X<<endl;
			fsY<<n<<"\t"<<Y<<endl;
			fsTH<<n<<"\t"<<TH<<endl;
			fsP<<X<<"\t"<<Y<<endl;

			// Perform the iteration in the main loop of ROS
			cout<<"Transmitting desired configuration..."<<endl;
			cout<<des_pos<<endl;
			cout<<"Time: "<<n<<endl;
			n=n+dt;
		}
		else
		{
			choice=0;
		}

		q_pub.publish(des_pos);

		ros::spinOnce();

		// We sleep for the required time to get the set/desired frequency
		lr.sleep();
        	
	}

	fsX.close();
	fsY.close();
	fsTH.close();
	fsP.close();

	return 0;
}




