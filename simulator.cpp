// This is the simulator of MiRo Prototype
// Deformable/Solid Body Model
// It is designed to run strictly in Robot Operating System (ROS)

#include <ros/ros.h> //This header includes the standard ROS classes
#include <iostream> //This header includes standard I/O objects
#include <sstream> //This header includes string-based streams
#include <fstream> //This header includes file-based streams
#include <cmath> // This header includes C++ mathematical library
//				 // like the Standard C Library header @c math.h
#include <cstring>  // This is the C++ version of the Standard C
//					// Library header @c string.h
#include <tf/transform_broadcaster.h>// This header includes the tf 
// package library for transform broadcasting
#include <tf2_ros/static_transform_broadcaster.h>
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

// Defining sign function
double sign(double x)
{
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  if (x = 0.0) return 0.0;
}

// Initializing the Robot parameters:
double m = 0.00021; // Eccentric mass
double r = 0.00177; // Eccentricity of the rotating mass
double M = 0.12; // Platform mass
double g = 9.81; // Gravitational acceleration
double mi = 0.4; // Coefficient of kinetic friction
double Iz = 7.5*pow(10,-5); // Polar moment of i
double d = 0.025; // Platform radius
double h = 0.003; // Motor axis height
double ho = 0.0045; // Eccentric load height

double md=m;
double me=m;

// Resistance of each string
double k1 = pow(10,9);
double k2 = pow(10,9);
double k3 = pow(10,9);

double D1 = 0.012; // Distance of motor D/E from (0,0)
double Dl = 0.05; // Distance between contact points
double DH = sqrt(3)*Dl/2; 

//double radius=0.0; 
//double cx = radius*cos(PI/4); 
//double cy = radius*sin(PI/4);
double cx = 0.0; 
double cy = 0.0;
//0.04; //
// Masses calculation
double M1 = (2*DH*Dl*(M+md+me)-3*Dl*M*cx-6*DH*(M*cy-(md-me)*D1))/(6*DH*Dl);
double M2 = (double)1/3*(md+me+M+3*M*cx/DH);
double M3 = (2*DH*Dl*(M+md+me)-3*Dl*M*cx+6*DH*(M*cy-(md-me)*D1))/(6*DH*Dl);

// Variables indicating the position of
// the contact points A,B,C 
double qAx = DH/3;
double qAy = Dl/2;
double qBx = -2*DH/3;
double qBy = 0.0;
double qCx = DH/3;
double qCy = -Dl/2;

double wmega = 1000;
double wmegaInit = 0;
double maxwmega = 1084;
double minwmega = -1084;

// Integration step (dt)
double step = pow(10,-4);
double angle = 0;

 // Time parameter
double t = 0.0;
double vtol = pow(10,-5);
double vtols = vtol;

double thetad = 0.0; // Angle of the D motor
double wd = 0.0; // Angular velocity of the D motor

double thetae = 0.0; // Angle of the E motor
double we = 0.0; // Angular velocity of the E motor

// Factors Ad and Ae for motors D and E respectively
double Ad = 0.0;
double Ae = 0.0;

// Actuation forces generated during asynchronous rotation
// of motors D and E, on x'x and z'z axes, respectively
double FAx = 0.0;
double FAy = 0.0;
double FAz = 0.0;
double FBx = 0.0;
double FBy = 0.0;
double FBz = 0.0;
double FCx = 0.0;
double FCy = 0.0;
double FCz = 0.0;

double Ax = 0.0;
double Ay = 0.0;
double Az = 0.0;
double Bx = 0.0;
double By = 0.0;
double Bz = 0.0;
double Cx = 0.0;
double Cy = 0.0;
double Cz = 0.0;

// 
double fax = 0.0;
double fay = 0.0;
double fbx = 0.0;
double fby = 0.0;
double fcx = 0.0;
double fcy = 0.0;

// Friction forces on points A,B and C
double nAx = 0.0;
double nAy = 0.0;
double nBx = 0.0;
double nBy = 0.0;
double nCx = 0.0;
double nCy = 0.0;

// Velocities of points A,B and C on x,y and z axes respectively
double vax = 0.0;
double vay = 0.0;
double vbx = 0.0;
double vby = 0.0;
double vcx = 0.0;
double vcy = 0.0;

double X1_DD = 0.0;
double X2_DD = 0.0;
double X3_DD = 0.0;
double Y1_DD = 0.0;
double Y2_DD = 0.0;
double Y3_DD = 0.0;

double X1_D = 0.0;
double X2_D = 0.0;
double X3_D = 0.0;
double Y1_D = 0.0;
double Y2_D = 0.0;
double Y3_D = 0.0;

double X1 = 0.0;
double Y1 = 0.0;
double X2 = 0.0;
double Y2 = 0.0;
double X3 = 0.0;
double Y3 = 0.0;

// Acceleration and velocity of platform on each axis
double ax = 0.0;
double ay = 0.0;
double Vx = 0.0;
double Vy = 0.0;

double CMX = 0.0;
double CMY = 0.0;

// Angle of rotation of the platform's center of mass
double CMpsi = 0.0;
double CMpsiDeg = 0.0;

// Angular velocity of the platform's center of mass
double CMpsiD = 0.0;

// Angular acceleration of the platform's center of mass
double CMpsiDD = 0.0;

double vaxRatio = 0.0;
double vayRatio = 0.0;
double vbxRatio = 0.0;
double vbyRatio = 0.0;
double vcxRatio = 0.0;
double vcyRatio = 0.0;


double qadot1 = 0.0;
double qadot2 = 0.0;
double qbdot1 = 0.0;
double qbdot2 = 0.0;
double qcdot1 = 0.0;
double qcdot2 = 0.0;

int state = 0;

// A callback function of angular velocities
void vel_Cb(const miro3::AngVel &w)
{
	cout<<M1<<","<<M2<<","<<M3<<","<<DH<<endl;

	wd = w.wd; //Desired angular velocity of D-actuator
	we = w.we; //Desired angular velocity of E-actuator

	cout<<"Angular velocities applied on actuators: "<<wd<<" , "<<we<<" "<<endl;
	cout<<endl;
	
	t = t+step;
	thetad = t*wd;
	thetae = t*we;

	Ad = m*r*pow(wd,2);
	Ae = m*r*pow(we,2);

	cout<<Ad<<endl;
	cout<<Ae<<endl;

	cout<<"qAx= "<<qAx<<", qAy= "<<qAy<<endl;
	cout<<"qBx= "<<qBx<<", qBy= "<<qBy<<endl;
	cout<<"qCx= "<<qCx<<", qCy= "<<qCy<<endl;

	FAx = Ad*sin(thetad);
	FCx = -Ae*sin(thetae);
	FAz = -m*g-Ad*cos(thetad);
	FCz = -m*g-Ae*cos(thetae);
	cout<<"FAx= "<<FAx<<", FAz= "<<FAz<<endl;
	cout<<"FCx= "<<FCx<<", FCz= "<<FCz<<endl;

	cout<<"Phase= "<<state<<endl;

	// Static Phase / Deformable-Body Model
   	 if(state == 0)
        {
		vax = X1_D;
		vay = Y1_D;
		cout<<"vax= "<<vax<<", vay= "<<vay<<endl;

		vbx = X2_D;
		vby = Y2_D;
		cout<<"vbx= "<<vbx<<", vby= "<<vby<<endl;

		vcx = X3_D;
		vcy = Y3_D;
		cout<<"vcx= "<<vcx<<", vcy= "<<vcy<<endl;

		// Suppress small velocities (for avoiding chattering)
		if(sqrt(pow(vax,2)+pow(vay,2))<vtol)
		{
			vax=0;
			vay=0;
		}
		if(sqrt(pow(vbx,2)+pow(vby,2))<vtol)
		{
			vbx=0;
			vby=0;
		}
		if(sqrt(pow(vcx,2)+pow(vcy,2))<vtol)
		{
			vcx=0;
			vcy=0;
		}
		
		// Vertical reactions on each point
		Az = (-2*DH*Dl*(FAz+FCz)+g*M*(-3*Dl*cx+2*DH*(Dl-3*cy))+6*DH*(-FAz+FCz)*D1+0*3*Dl*(FAx+FCx)*ho)/(6*DH*Dl);
		
		Bz = (-(DH*(FAz+FCz))+g*M*(DH+3*cx)-0*3*(FAx+FCx)*ho)/(3*DH);

		Cz = (-2*DH*Dl*(FAz+FCz)+g*M*(-3*Dl*cx+2*DH*(Dl+3*cy))+6*DH*(FAz-FCz)*D1+0*3*Dl*(FAx+FCx)*ho)/(6*DH*Dl);

		cout<<"Az= "<<Az<<", Bz= "<<Bz<<", Cz= "<<Cz<<endl;

		// Friction conditions for each mass:
		// Mass 1
		nAx = FAx - (sqrt(3)*k1*X1)/2 + (sqrt(3)*k1*X2)/2;
		nAy = FAy + (-k1/2 - k3)*Y1 + (k1*Y2)/2 + k3*Y3;
		cout<<"nAx= "<<nAx<<"nAy= "<<nAy<<endl;

		if(sqrt(pow(vax,2)+pow(vay,2))==0)
		{
			if(sqrt(pow(nAx,2)+pow(nAy,2)) < mi*abs(Az))
		       {
				fax = -nAx;
			        fay = -nAy;
			}
		    	else
		        {
				fax = -mi*abs(Az)*(nAx/sqrt(pow(nAx,2)+pow(nAy,2)));
			        fay = -mi*abs(Az)*(nAy/sqrt(pow(nAx,2)+pow(nAy,2)));
			}
		}
		else
		{
			fax = -mi*abs(Az)*(vax/sqrt(pow(vax,2)+pow(vay,2)));
			fay = -mi*abs(Az)*(vay/sqrt(pow(vax,2)+pow(vay,2)));
		}        
		cout<<"fax= "<<fax<<", fay= "<<fay<<endl;

		// Mass 2
		nBx = FBx + (sqrt(3)*k1*X1)/2 + (-(sqrt(3)*k1)/2 - (sqrt(3)*k2)/2)*X2 + (sqrt(3)*k2*X3)/2;
		nBy = FBy + (k1*Y1)/2 + ((-k1 - k2)*Y2)/2 + (k2*Y3)/2;
		cout<<"nBx= "<<nBx<<", nBy= "<<nBy<<endl;

		if(sqrt(pow(vbx,2)+pow(vby,2))==0)
		{
			if(sqrt(pow(nBx,2)+pow(nBy,2)) < mi*abs(Bz))
		       {
				fbx = -nBx;
			        fby = -nBy;
			}
		    	else
		        {
				fbx = -mi*abs(Bz)*(nBx/sqrt(pow(nBx,2)+pow(nBy,2)));
			        fby = -mi*abs(Bz)*(nBy/sqrt(pow(nBx,2)+pow(nBy,2)));
			}
		}
		else
		{
			fbx = -mi*abs(Bz)*(vbx/sqrt(pow(vbx,2)+pow(vby,2)));
			fby = -mi*abs(Bz)*(vby/sqrt(pow(vbx,2)+pow(vby,2)));
		}    
		cout<<"fbx= "<<fbx<<", fby= "<<fby<<endl;

		// Mass 3
		nCx = FCx + (sqrt(3)*k2*X2)/2 - (sqrt(3)*k2*X3)/2;
		nCy = FCy + k3*Y1 + (k2*Y2)/2 + (-k2/2 - k3)*Y3;
		cout<<"nCx= "<<nCx<<", nCy= "<<nCy<<endl;

		if(sqrt(pow(vcx,2)+pow(vcy,2))==0)
		{
			if(sqrt(pow(nCx,2)+pow(nCy,2)) < mi*abs(Cz))
		       {
				fcx = -nCx;
			        fcy = -nCy;
			}
		    	else
		        {
				fcx = -mi*abs(Cz)*(nCx/sqrt(pow(nCx,2)+pow(nCy,2)));
			        fcy = -mi*abs(Cz)*(nCy/sqrt(pow(nCx,2)+pow(nCy,2)));
			}
		}
		else
		{
			fcx = -mi*abs(Cz)*(vcx/sqrt(pow(vcx,2)+pow(vcy,2)));
			fcy = -mi*abs(Cz)*(vcy/sqrt(pow(vcx,2)+pow(vcy,2)));
		}    
		cout<<"fcx= "<<fcx<<", fcy= "<<fcy<<endl;

		// Accelerations of the three masses on xy-plane
		X1_DD = (fax + FAx - (sqrt(3)*k1*X1)/2 + (sqrt(3)*k1*X2)/2)/M1;
		X2_DD = (fbx + FBx + (sqrt(3)*k1*X1)/2 + (-(sqrt(3)*k1)/2 - (sqrt(3)*k2)/2)*X2 + (sqrt(3)*k2*X3)/2)/M2;
		X3_DD = (fcx + FCx + (sqrt(3)*k2*X2)/2 - (sqrt(3)*k2*X3)/2)/M3;
		Y1_DD = (fay + FAy + (-k1/2-k3)*Y1 + (k1*Y2)/2 + k3*Y3)/M1;
		Y2_DD = (fby + FBy + (k1*Y1)/2 + ((-k1 - k2)*Y2)/2 + (k2*Y3)/2)/M2;
		Y3_DD = (fcy + FCy + k3*Y1 + (k2*Y2)/2 + (-k2/2-k3)*Y3)/M3;

		cout<<"X1_DD= "<<X1_DD<<", Y1_DD= "<<Y1_DD<<endl;
		cout<<"X2_DD= "<<X2_DD<<", Y2_DD= "<<Y2_DD<<endl;
		cout<<"X3_DD= "<<X3_DD<<", Y3_DD= "<<Y3_DD<<endl;

		X1_D = X1_D+X1_DD*step;
		X2_D = X2_D+X2_DD*step;
		X3_D = X3_D+X3_DD*step;
		Y1_D = Y1_D+Y1_DD*step;
		Y2_D = Y2_D+Y2_DD*step;
		Y3_D = Y3_D+Y3_DD*step;

		cout<<"X1_D= "<<X1_D<<", Y1_D= "<<Y1_D<<endl;
		cout<<"X2_D= "<<X2_D<<", Y2_D= "<<Y2_D<<endl;
		cout<<"X3_D= "<<X3_D<<", Y3_D= "<<Y3_D<<endl;

		X1 = X1+X1_D*step;
		X2 = X2+X2_D*step;
		X3 = X3+X3_D*step;
		Y1 = Y1+Y1_D*step;
		Y2 = Y2+Y2_D*step;
		Y3 = Y3+Y3_D*step;
		
		cout<<"X1= "<<X1<<", Y1= "<<Y1<<endl;
		cout<<"X2= "<<X2<<", Y2= "<<Y2<<endl;
		cout<<"X3= "<<X3<<", Y3= "<<Y3<<endl;

		// Friction forces
		Ax = fax;
		Bx = fbx;
		Cx = fcx;
		Ay = fay;
		By = fby;
		Cy = fcy;
		cout<<"Ax= "<<Ax<<", Ay= "<<Ay<<endl;
		cout<<"Bx= "<<Bx<<", By= "<<By<<endl;
		cout<<"Cx= "<<Cx<<", Cy= "<<Cy<<endl;

	}
	else
	{
		// Velocities are expressed wrt the body-fixed frame of reference
		vax = cos(CMpsi+angle)*qadot1 + sin(CMpsi+angle)*qadot2;
		vay = -sin(CMpsi+angle)*qadot1 + cos(CMpsi+angle)*qadot2;

		vbx = cos(CMpsi+angle)*qbdot1 + sin(CMpsi+angle)*qbdot2;
		vby = -sin(CMpsi+angle)*qbdot1 + cos(CMpsi+angle)*qbdot2;

		vcx = cos(CMpsi+angle)*qcdot1 + sin(CMpsi+angle)*qcdot2;
		vcy = -sin(CMpsi+angle)*qcdot1 + cos(CMpsi+angle)*qcdot2;
		
		cout<<"vax= "<<vax<<", vay= "<<vay<<endl;

		cout<<"vbx= "<<vbx<<", vby= "<<vby<<endl;

		cout<<"vcx= "<<vcx<<", vcy= "<<vcy<<endl;

		// Suppress the small velocities (for avoiding chattering)
		if (sqrt(pow(vax,2)+pow(vay,2))<vtol)
		{
			vaxRatio = 0;
			vayRatio = 0;
		}
		else
		{
		 	vaxRatio = -(vax/sqrt(pow(vax,2)+pow(vay,2)));
			vayRatio = -(vay/sqrt(pow(vax,2)+pow(vay,2)));
		}
		cout<<"vaxRatio= "<<vaxRatio<<", vayRatio= "<<vayRatio<<endl;

		if (sqrt(pow(vbx,2)+pow(vby,2))<vtol)
		{
			vbxRatio = 0;
			vbyRatio = 0;
		}
		else
		{
			vbxRatio = -(vbx/sqrt(pow(vbx,2)+pow(vby,2)));
			vbyRatio = -(vby/sqrt(pow(vbx,2)+pow(vby,2)));
		}
		cout<<"vbxRatio= "<<vbxRatio<<", vbyRatio= "<<vbyRatio<<endl;

		if (sqrt(pow(vcx,2)+pow(vcy,2))<vtol)
		{
			vcxRatio = 0;
		 	vcyRatio = 0;
		}		
		else
		{
		 	vcxRatio = -(vcx/sqrt(pow(vcx,2)+pow(vcy,2)));
		 	vcyRatio = -(vcy/sqrt(pow(vcx,2)+pow(vcy,2)));
		}
		cout<<"vcxRatio= "<<vcxRatio<<", vcyRatio= "<<vcyRatio<<endl;

		// Reaction forces during general motion
		Az = (-2*DH*Dl*(FAz + FCz) + g*M*(-3*Dl*cx + 2*DH*(Dl-3*cy)) + 6*DH*(-FAz + FCz)*D1 + 0*3*Dl*(FAx + FCx)*ho)/(6*DH*Dl);

		Bz = (-(DH*(FAz + FCz)) + g*M*(DH + 3*cx) - 0*3*(FAx + FCx)*ho)/(3*DH);

		Cz = (-2*DH*Dl*(FAz + FCz) + g*M*(-3*Dl*cx + 2*DH*(Dl + 3*cy)) + 6*DH*(FAz - FCz)*D1 + 0*3*Dl*(FAx + FCx)*ho)/(6*DH*Dl);
		
		cout<<"Az= "<<Az<<", Bz= "<<Bz<<", Cz= "<<Cz<<endl;

		if(Az<0)
		{
			Az = 0;
			cout<<"Az reaction is zero. The platform is tipping"<<endl;
			cout<<"Please reduce actuation speed."<<endl;
		}

		if(Bz<0)
		{
			Bz = 0;
			cout<<"Bz reaction is zero. The platform is tipping"<<endl;
			cout<<"Please reduce actuation speed."<<endl;
		}

		if(Cz<0)
		{		
			Cz = 0;
			cout<<"Cz reaction is zero. The platform is tipping"<<endl;
			cout<<"Please reduce actuation speed."<<endl;
		}

		//  General plane motion of the platform
		Ax = mi*Az*vaxRatio;
		Ay = mi*Az*vayRatio;

		Bx = mi*Bz*vbxRatio;
		By = mi*Bz*vbyRatio;

		Cx = mi*Cz*vcxRatio;
		Cy = mi*Cz*vcyRatio;
		
		cout<<"Ax= "<<Ax<<"Ay= "<<Ay<<endl;
		cout<<"Bx= "<<Bx<<"By= "<<By<<endl;
		cout<<"Cx= "<<Cx<<"Cy= "<<Cy<<endl;

	}

	// Equations of motion
	ax = ((Ax + Bx + Cx + FAx + FCx)*cos(angle+CMpsi) - (Ay + By + Cy + FAy + FBy + FCy)*sin(angle+CMpsi))/M;
	
	ay = ((Ay + By + Cy + FAy + FBy + FCy)*cos(angle+CMpsi) + (Ax + Bx + Cx + FAx + FCx)*sin(angle+CMpsi))/M;

	CMpsiDD = (2*Ay*DH - 4*By*DH + 2*Cy*DH - 3*Ax*Dl + 3*Cx*Dl - 6*FAx*D1 + 6*FCx*D1 + 6*FAy*0 - 12*FBy*0 + 6*FCy*0)/(6*Iz);
	
	cout<<"ax= "<<ax<<"ay= "<<ay<<endl;
	
	CMpsiD = CMpsiD + CMpsiDD*step;
	CMpsi = CMpsi + CMpsiD*step;
	cout<<"CMpsiDD= "<<CMpsiDD<<"CMpsiD= "<<CMpsiD<<endl;

	// Linear velocities of platform on x and y axes respectively
	Vx = Vx+ax*step;
	Vy = Vy+ay*step;
	cout<<"Vx= "<<Vx<<"Vy= "<<Vy<<endl;

	qadot1 = Vx+(-qAy*cos(CMpsi+angle) - qAx*sin(CMpsi+angle))*CMpsiD;
	qadot2 = Vy+(qAx*cos(CMpsi+angle) - qAy*sin(CMpsi+angle))*CMpsiD;
	qbdot1 = Vx+(-qBy*cos(CMpsi+angle) - qBx*sin(CMpsi+angle))*CMpsiD;
	qbdot2 = Vy+(qBx*cos(CMpsi+angle) - qBy*sin(CMpsi+angle))*CMpsiD;
	qcdot1 = Vx+(-qCy*cos(CMpsi+angle) - qCx*sin(CMpsi+angle))*CMpsiD;
	qcdot2 = Vy+(qCx*cos(CMpsi+angle) - qCy*sin(CMpsi+angle))*CMpsiD;

	CMX = CMX + Vx*step;
	CMY = CMY + Vy*step;

	CMpsiDeg = CMpsi*(180.0/PI);
	
	cout<<"CMX= "<<CMX<<" _ _ CMY= "<<CMY<<" _ _ psi= "<<CMpsi<<endl;
	cout<<endl;
	cout<<"BEFORE IF"<<endl;
	cout<<"vax= "<<vax<<", vay= "<<vay<<endl;
	cout<<"vbx= "<<vbx<<", vby= "<<vby<<endl;
	cout<<"vcx= "<<vcx<<", vcy= "<<vcy<<endl;
	cout<<endl;
	cout<<"qadot1= "<<qadot1<<", qadot2= "<<qadot2<<endl;
	cout<<"qbdot1= "<<qbdot1<<", qbdot2= "<<qbdot2<<endl;
	cout<<"qcdot1= "<<qcdot1<<", qcdot2= "<<qcdot2<<endl;
	cout<<endl;

	 if(  ((sqrt(pow(qadot1,2)+pow(qadot2,2)) < vtols) && (sqrt(pow(qbdot1,2)+pow(qbdot2,2)) < vtols) && (sqrt(pow(qcdot1,2)+pow(qcdot2,2)) < vtols)) || 
	((sqrt(pow(qadot1,2)+pow(qadot2,2)) < vtols) && (sqrt(pow(qbdot1,2)+pow(qbdot2,2)) < vtols) && (sqrt(pow(qcdot1,2)+pow(qcdot2,2)) >= vtols))||
        ((sqrt(pow(qadot1,2)+pow(qadot2,2)) < vtols) && (sqrt(pow(qbdot1,2)+pow(qbdot2,2)) >= vtols) && (sqrt(pow(qcdot1,2)+pow(qcdot2,2)) < vtols))||
        ((sqrt(pow(qadot1,2)+pow(qadot2,2)) >= vtols) && (sqrt(pow(qbdot1,2)+pow(qbdot2,2)) < vtols) && (sqrt(pow(qcdot1,2)+pow(qcdot2,2)) < vtols)))
        {
		state = 0;
	}        
	else
	{
		if(state==0)
            	{
			    // Reset velocities of points A,B and C  
			    X1_D = 0;
			    Y1_D = 0;
			    X2_D = 0;
			    Y2_D = 0;
			    X3_D = 0;
			    Y3_D = 0;
			    
			    // Reset positions of points A,B and C 
			    X1 = 0;
			    Y1 = 0;
			    X2 = 0;
			    Y2 = 0;
			    X3 = 0;
			    Y3 = 0;
		}
		state = 1;
	}

}

int main(int argc, char **argv)
{			
	fstream fsX("Xpos.dat",fstream::out);
	fstream fsY("Ypos.dat",fstream::out);
	fstream fsTH("psi.dat",fstream::out);
	fstream fsP("XY.dat",fstream::out);
	fstream fsXD("Vx.dat",fstream::out);
	fstream fsCMpsiD("psiD.dat",fstream::out);

    	//Initialize the ROS system.
    	ros::init(argc, argv, "miro3_sim");

    	//Establish this program as a ROS node.
    	ros::NodeHandle ns;

    	//Send some output as a log message.
    	cout<<blueb<<"Welcome to MiRo Simulator"<<endl;
	cout<<"-------------------------"<<white<<endl;
    	
	// A service client request the trajectory planner for configuration
	ros::ServiceClient client_qinit = ns.serviceClient<miro3::AskConf>("/q_init");
	miro3::AskConf q_init;
	q_init.request.r = 0;

	// Configuration(q) Publisher
   	ros::Publisher q_pub = ns.advertise<geometry_msgs::Pose2D>("/q_cur", 1000);

    	// Desired angular velocities(w) Subscriber
    	ros::Subscriber w_sub = ns.subscribe("/w_des", 1000, vel_Cb);
	
	int flag=0;

	geometry_msgs::Pose2D cur_pos;

	while(q_init.request.r != 1 && flag!=1)
	{
		cout<<blueb<<"Pending request of initial configuration..."<<endl;
		cin>>q_init.request.r;
		
		if (client_qinit.call(q_init))
		{
			cout<<blue<<"Getting initial configuration..."<<endl;
			cout<<greenb<<"Done!!"<<endl<<endl;
			cout<<white<<"Initial configuration is:"<<endl<<q_init.response.pose<<endl;
			cur_pos = q_init.response.pose;
			cout<<"If you want to initiate the simulation, please type '1' "<<endl; 
			cin>>flag;
			CMX=cur_pos.x;
			CMY=cur_pos.y;
			CMpsi=cur_pos.theta;
		}
		else
		{
			cout<<red<<"Failed to get initial configuration!"<<white<<endl;
			flag=0;
		}

	}
	ros::Rate lr(1000); // Loop rate in Hz

	while(ros::ok() && flag==1)
	{
		cur_pos.x = CMX;
		cur_pos.y = CMY;
		cur_pos.theta = CMpsi;
		
		fsX<<t<<"\t"<<CMX<<endl;
		fsY<<t<<"\t"<<CMY<<endl;
		fsTH<<t<<"\t"<<CMpsi<<endl;
		fsP<<CMX<<"\t"<<CMY<<endl;
		fsXD<<thetad<<"\t"<<Vx<<endl;
		fsCMpsiD<<t<<"\t"<<CMpsiD<<endl;

		// Printing the new configuration for visualization
		cout<<cur_pos<<endl;

		// Publishing the new configuration
		q_pub.publish(cur_pos);

		// Perform the iteration in the main loop
		// of ROS, allowing the user to perform
		// actions between iterations
		ros::spinOnce();
		// (in contrast with spin() function)

		// We sleep for the required time to get the set/desired frequency
		lr.sleep();
	}
    	fsX.close();
	fsY.close();
	fsTH.close();
	fsP.close();
	fsXD.close();
	fsCMpsiD.close();

}
