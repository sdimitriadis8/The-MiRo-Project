// This is an actuator controller for MiRo Prototype,
// designed to run strictly in Robot Operating System (ROS)

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

double tol = pow(10,-5); // Error tolerance

double d = 0.025; // Platform radius

double wd;
double we;
double wdD;
double weD;

double vd;
double ve;
double vdD;
double veD;


double v1;
double v2;

double x_df;
double y_df;
double th_df;
double phi_d;

double a1;
double a2;
double a_d;
double a_dt;
double eta_d;
double r1;
double rev_d;
double r2;
double rt;
double b1;
double b_d;
double b_dt;
double r2t;

double xg_df;
double yg_df;
double thg_df;

double vdes;
double od;
double a;
double b;

double q1;
double q2;
double q3;
double g;
double g1;
double g2;

double p1;
double p2;
double p3;
double p4;
double p5;
double p6;

double f;
double f1;
double f2;

double t=0;
double step = pow(10,-4);

double kv;
double kt;

double ky;
double kp;

int check = 0;
int reached = 0;
int rev_ang = 0;

geometry_msgs::Pose2D q_des;  // Desired configuration of platform
geometry_msgs::Pose2D q_fin; // Goal configuration of platform
geometry_msgs::Pose2D q_cur;  // Current configuration of platform

miro3::AngVel w;

fstream fswd("wd.dat",fstream::out);
fstream fswe("we.dat",fstream::out);

// Function used to wrap angles 
// from [-PI,PI) to [0,2PI)
double wrapTo2PI(double x)
{
	x = fmod(x,2*PI);
	if (x < 0)
	{
		x += 2*PI;
	}
	return x;
}

// The function that controls the angular
// velocities of the platform based on 
// the current configuration and a 
// given desired one.
void Contr(const geometry_msgs::Pose2D &q_cur)
{
	//Current position of platform
	cout<<"My current configuration is: "<<endl;
	cout<<q_cur<<endl;
	cout<<endl;
	cout<<"The desired configuration is: "<<endl;
	cout<<q_des<<endl;

	x_df = q_des.x - q_cur.x;
	y_df = q_des.y - q_cur.y;
	th_df = q_des.theta - q_cur.theta;

	// Finding the angle between the starting point
	// and the desired destination via the arctangent
	// of the differences of the respective coordinates
	phi_d = atan2(y_df,x_df);

	// Finding the representation of angle phi in the 
	// range of [0,2PI] (in radiants)
	a1 = wrapTo2PI(phi_d);

	// Finding the representation of platform's angle  
	// in the range of [0,2PI] (in radiants)
	a2 = wrapTo2PI(q_cur.theta);

	// Finding the angle between the initial angle of
	// center of mass (/platform) and phi
	// WE CARE ABOUT THE SIGN OF THIS ANGLE!!
	a_d = a1 - a2;

	// Making sure we get the smallest difference
	// i.e. not exceeding 180 degrees, because then 
	// the shortest way is the other way around
	if(abs(a_d)>PI)
	{
		a_dt = 2*PI - abs(a_d);
	}
	else
	{
		a_dt = abs(a_d);
	}

	// Finding the symmetric point of the desired one
	// on the unit circle and its angle with the x-axis.
	// We denote this angle as eta
	eta_d = PI + a1;
	// Finding the representation of angle eta 
	// in the range of [0,2PI] (in radiants)
	r1 = wrapTo2PI(eta_d);

	// Finding the angle between the initial angle of
	// center of mass (/platform) and eta. We will
	// refer to the specific angle as reversed angle
	// from now on.
	// WE CARE ABOUT THE SIGN OF THIS ANGLE TOO!!
	rev_d = a2 - r1;

	// Making sure we get the smallest difference
	// between them
	if(abs(rev_d)>PI)
	{
		rt=2*PI-abs(rev_d);
	}
	else
	{
		rt=abs(rev_d);
	}

	// Indicates if the platform goes forwards or backwards
	// '0' is the default forward motion and '1' if the 
	// reverse gear is set
	if(rt<a_dt)
	{
		rev_ang = 1;
	}	
	else
	{
		rev_ang = 0;
	}

	// Finding the representation of the desired platform's   
	// angle, in the range of [0,2PI] (in radiants)
	b1 = wrapTo2PI(q_des.theta);

	// Finding the angle between the desired angle of
	// the platform (as b1) and phi (as a1).
	// WE CARE ABOUT THE SIGN OF THIS ANGLE!!
	b_d = a1-b1;
	// Making sure we get the smallest difference
	// between them
	if(abs(b_d)>PI)
	{
		b_dt=2*PI-abs(b_d);
	}
	else
	{
		b_dt=abs(b_d);
	}
	    
	// Finding the angle between the desired angle of
	// platform (as b1) and eta (as r1).
	// WE CARE ABOUT THE SIGN OF THIS ANGLE TOO!!
	r2 = b1 - r1;
	// Making sure we get the smallest angle
	if(abs(r2)>PI)
	{
		r2t=2*PI-abs(r2);
	}
	else
	{
		r2t=abs(r2);
	}
	cout<<"Angle phi_d = "<<phi_d<<endl;

	// Control Law
	if(sqrt(pow(x_df,2) + pow(y_df,2) )<pow(10,-5))
	{	
		if(abs(th_df)<pow(10,-5))
		{  
			wd=0;
			we=0;
		}
		else if(th_df<0)
		{    
			if(abs(th_df)>PI)
			{
			        wd=-1080;
			        we=-1080;
			}
		    	else
		        {
				wd=1080;
		        	we=1080;
		        }
		}
		else if(th_df>0)
		{
			if(abs(th_df)>PI)
			{
				wd=1080;
		        	we=1080;
		        }
		    	else
		        {
				wd=-1080;
			        we=-1080;
			}
		}
	}

	// While platform's orientation is not within
	// the range [phi_d-0.03,phi_d+0.03], nor is
	// it's back, rotate until one of them is by
	// using the following law for preferring 
	// which one is the best choice
	else if(a_dt>0.03 && rt>0.03)
	{
		// Check which way to prefer:
		// 1) Rotate until platform faces target,
		// reach it and then rotate again to get 
		// the desired angle, or
		// 2) Rotate the other way until platform's 
		// back faces target and go backwards. Then  
		// rotate again to get the desired angle
		if(rt+r2t<a_dt+b_dt)
		{
			if((rev_d>0 && rev_d<PI) || (rev_d<0 && abs(rev_d)>PI))
			{
				wd = 1080;
				we = 1080;
			}
			else
			{
				wd = -1080;
				we = -1080;
			}
			rev_ang=1;
		}
		else
		{
			if((a_d>0 && a_d<PI) || (a_d<0 && abs(a_d)>PI))
			{
				wd = -1080;
				we = -1080;
			}
			else
			{
				wd = 1080;
				we = 1080;
			}
			rev_ang=0;
		}
	}
	else
	{
		if(q_des.theta>PI/2 && q_des.theta<3*PI/2)
		{
			vdD = -ky*y_df;
			veD = ky*y_df;

			vd = vd + vdD*step;
			ve = ve + veD*step;

			vd=abs(vd);
			ve=abs(ve);

			p1 = 8.764e+10;
			p2 = -2.676e+08;
			p3 = 3.902e+05;
			p4 = 850.9;

			wd = p1*pow(vd,3) + p2*pow(vd,2) + p3*vd + p4;
			we = p1*pow(ve,3) + p2*pow(ve,2) + p3*ve + p4;
		}
		else
		{
			vdD = ky*y_df;
			veD = -ky*y_df;

			vd = vd + vdD*step;
			ve = ve + veD*step;

			vd=abs(vd);
			ve=abs(ve);

			p1 = 8.764e+10;
			p2 = -2.676e+08;
			p3 = 3.902e+05;
			p4 = 850.9;

			wd = p1*pow(vd,3) + p2*pow(vd,2) + p3*vd + p4;
			we = p1*pow(ve,3) + p2*pow(ve,2) + p3*ve + p4;
		}
	
		if(rev_ang==1)
		{
			wd=-wd;
		}
		else
		{
			we=-we;
		}

		//wd = ky*y_df - kp*phi_d;
		//we = - ky*y_df - kp*phi_d;

		//wd = wd+wdD*step;
		//we = we+weD*step;
	}

	// 
	cout<<"wd= "<<wd<<endl;
	cout<<"we= "<<we<<endl;

	// wd-value bounding
	if(abs(wd)<900 && wd>0)
	{			
		wd=900;
	}
	else if(abs(wd)<900 && wd<0)
	{			
		wd=-900;
	}
	else if(abs(wd)>1080 && wd>0)
	{
		wd=1080;
	}
	else if(abs(wd)>1080 && wd<0)
	{			
		wd=-1080;
	}
	else
	{
		wd=wd;
	}

	// we-value bounding
	if(abs(we)<900 && we>0)
	{			
		we=900;
	}
	else if(abs(we)<900 && we<0)
	{			
		we=-900;
	}
	else if(abs(we)>1080 && we>0)
	{
		we=1080;
	}
	else if(abs(we)>1080 && we<0)
	{			
		we=-1080;
	}
	else
	{
		we=we;
	}

}

void Des_pose_Cb(const geometry_msgs::Pose2D &des_pose)
{
	cout<<blue<<"The desired configuration is:"<<white<<endl;
	q_des=des_pose;
}

int main(int argc, char **argv)
{
    	//Initialize the ROS system.
    	ros::init(argc, argv, "miro3_con");

    	//Establish this program as a ROS node.
    	ros::NodeHandle nc;

   	 //Send some output as a log message.
    	cout<<blueb<<"Welcome to MiRo Controller"<<endl;
	cout<<"--------------------------"<<white<<endl;

    	// Subscriber at the desired configuration(q')
    	ros::Subscriber des_conf = nc.subscribe("/q_des",10, Des_pose_Cb);	

   	// Subscriber at the current configuration(q)
	ros::Subscriber cur_conf = nc.subscribe("/q_cur",10, Contr);

   	//ros::Subscriber cur_conf = nc.subscribe("/q_cur",10, My_control);

	// Angular velocities Publisher
    	ros::Publisher w_pub = nc.advertise<miro3::AngVel>("/w_des", 10000);
		
	// A service server responding angular velocities to the simulator
	// ros::ServiceServer server = nc.advertiseService("/w_des", Check_Vel);

	// A service client that requests the trajectory planner for configuration
	ros::ServiceClient client_q = nc.serviceClient<miro3::AskConf>("/q_des");
	miro3::AskConf q_desServ;
	
	ros::ServiceClient client_qf = nc.serviceClient<miro3::AskConf>("/q_fin");
	miro3::AskConf q_finServ;

	// Output messages for the user to input the gain parameters
	//cout<<"Enter the linear velocity gain parameter (real number)"<<endl;
	//cin>>kv;
	//cout<<"Enter the angular velocity gain parameter (real number)"<<endl;
	//cin>>kt;

	// Messages to the user to input the gain parameters
	cout<<"Enter the gain parameter ky (real number)"<<endl;
	cin>>ky;
	//cout<<"Enter the gain parameter kp (real number)"<<endl;
	//cin>>kp;

	ros::Rate lr(1000); // Loop rate in Hz

	q_desServ.request.r = 1;
		
	if (client_q.call(q_desServ))
	{
		cout<<blue<<"Getting next configuration..."<<endl;
		cout<<greenb<<"Done!!"<<endl;
		cout<<white<<q_desServ.response.pose<<endl;
		q_des = q_desServ.response.pose;
		cout<<"q_des:"<<q_des<<endl;
	}
	else
	{
		cout<<red<<"Failed to get configuration!"<<white<<endl;
	} 
	
	q_finServ.request.r = 1;
		
	if (client_qf.call(q_finServ))
	{
		cout<<blue<<"Getting goal configuration..."<<endl;
		cout<<greenb<<"Done!!"<<endl;
		cout<<white<<q_finServ.response.pose<<endl;
		q_fin = q_finServ.response.pose;
		cout<<"q_fin:"<<q_fin<<endl;
	}
	else
	{
		cout<<red<<"Failed to get goal configuration!"<<white<<endl;
	} 
	

	while(ros::ok() && reached==0)
	{	
		if(check==1)
		{
			q_desServ.request.r = 1;
			if (client_q.call(q_desServ))
			{
				cout<<blue<<"Getting next configuration..."<<endl;
				cout<<greenb<<"Done!!"<<endl;
				cout<<white<<q_desServ.response.pose<<endl;
				q_des = q_desServ.response.pose;
				cout<<"q_des:"<<q_des<<endl;
			}
			else
			{
				cout<<red<<"Failed to get configuration!"<<white<<endl;
			} 
			check=0;
		}

		t = t+step;
		w.wd=round(wd);
		w.we=round(we);
	
		cout<<"Publishing : "<<"wd = "<<w.wd<<", we = "<<w.we<<endl;
		w_pub.publish(w);

		ros::spinOnce();

		// We sleep for the required time to get the set/desired frequency
		lr.sleep();
	}
	
	fswd.close();
	fswe.close();
	return 0;
}
