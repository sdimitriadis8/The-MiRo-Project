%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%
%&%                                                   %&%
%&%             Αλγόριθμος Προσομοίωσης               %&%
%&%                                                   %&%
%&%         Μοντέλου Παραμορφώσιμου Σώματος           %&%
%&%                                                   %&%
%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%
%--------------------------------------------------------
clear all;
clc

% Initialization of the robot parameters(motors,global constants,etc.)
m = 0.00021; % Eccentric mass in kg
r = 0.00177; % Eccentricity of the rotating mass in m
M = 0.12; % Platform mass in kg
g = 9.81; % Gravitational acceleration in m/s^2
mi = 0.4; % Coefficient of kinetic friction
Iz = 7.5*10^(-5); % Polar moment of inertia
d = 0.025; % Platform radius
h = 0.003; % Motor axis height
ho = 0.0045; % Eccentric load height
R = 10; % Electrical Resistance in Ohm (or 10.7)
Kt = 1.91*10^(-4); % Torque constant in mA (or 3.64*10^(-4))
V = 0.8798; % Volts (Or 3.7v for 1 cell Lipo)
c = 1.72*10^(-5); % Coulomb friction (or 1.34*10^(-5))
bb = 2*10^(-9); % Viscous friction in Ns/m (or 2.94*10^(-9))
tmech = 175; % Mechanical time constant in ms
J = 7.6*10^(-10); % Eccentric's load moment of inertia (or 2.67*10^(-9))
z = [0;0;1]; % The unit vector of z-axis
threshold = 10^(-4); % Threshold for motion parameter measurements
w = 1080; % Angular velocity of the actuation motors in synchronous model
vtols = 1e-5;
vtol = vtols;

md=m;
me=m;

k1 = 10^9;
k2 = 10^9;
k3 = 10^9;

D1 = 0.012; % Distance of motor D/E from (0,0)
Dl = 0.05; % Distance between contact points
DH = sqrt(3)*Dl/2; % Difference of contact points on x axis (or 0.0433)

radius=0; %0.0028;
cx = radius*cos(pi/4); %0.002;
cy = radius*sin(pi/4); %0.002;

% Masses calculation
M1 = (2*DH*Dl*(M+md+me)-3*Dl*M*cx-6*DH*(M*cy-(md-me)*D1))/(6*DH*Dl);
M2 = 1/3*(md+me+M+3*M*cx/DH);
M3 = (2*DH*Dl*(M+md+me)-3*Dl*M*cx+6*DH*(M*cy-(md-me)*D1))/(6*DH*Dl);

% Initializing the step of integration dt
step = 10^(-4);
% Calculation of dt
t = 0:step:1;
% Calculation of angle theta (w=theta/t ~> rad/s)
th = w*t;

% Platform's angle of rotation
CMpsi = pi;
% And its' 1st & 2nd order derivatives repectively
CMpsi_D = 0; % Platform's rotational speed
CMpsi_DD = 0; % Platform's rotational acceleration

% Position of platform on x and y axes
Px = 0;
Py = 0;

% The desired configuration coordinates
Xdes = 0.00028;
Ydes = -0.0000017;
Thdes = 0.00;

% Coordinate differences between the current  
% configuration and the desired one
DeltaX = Xdes-Px;
DeltaY = Ydes-Py;
DeltaTh = Thdes-CMpsi;

% Finding the angle between the starting point
% and the desired destination via the arctangent
% of the differences of the respective coordinates
phi_d = atan2(DeltaY,DeltaX);

% Finding the representation of angle phi in the 
% range of [0,2pi] (in radiants)
a1 = wrapTo2Pi(phi_d);

% Finding the representation of platform's angle  
% in the range of [0,2pi] (in radiants)
a2 = wrapTo2Pi(CMpsi);

% Finding the angle between the initial angle of
% center of mass (/platform) and phi
% WE CARE ABOUT THE SIGN OF THIS ANGLE!!
a_d = a1 - a2;

% Making sure we get the smallest difference
% i.e. not exceeding 180 degrees, because then 
% the shortest way is the other way around
if(abs(a_d)>pi)
    a_dt = 2*pi - abs(a_d);
else
    a_dt = abs(a_d);
end

% Finding the symmetric point of the desired one
% on the unit circle and its angle with the x-axis.
% We denote this angle as eta
eta_d = mod((pi + a1),2*pi);
% Finding the representation of angle eta 
% in the range of [0,2pi] (in radiants)
r1 = wrapTo2Pi(eta_d);

% Finding the angle between the initial angle of
% center of mass (/platform) and eta. We will
% refer to the specific angle as reversed angle
% from now on.
% WE CARE ABOUT THE SIGN OF THIS ANGLE TOO!!
rev_d = a2 - r1;

% Making sure we get the smallest difference
% between them
if(abs(rev_d)>pi)
    rt=2*pi-abs(rev_d);
else
    rt=abs(rev_d);
end

% Finding the representation of the desired platform's   
% angle, in the range of [0,2pi] (in radiants)
b1 = wrapTo2Pi(Thdes);

% Finding the angle between the desired angle of
% the platform (as b1) and phi (as a1).
% WE CARE ABOUT THE SIGN OF THIS ANGLE!!
b_d = a1-b1;
% Making sure we get the smallest difference
% between them
if(abs(b_d)>pi)
    b_dt=2*pi-abs(b_d);
else
    b_dt=abs(b_d);
end
    
% Finding the angle between the desired angle of
% platform (as b1) and eta (as r1).
% WE CARE ABOUT THE SIGN OF THIS ANGLE TOO!!
r2 = b1 - r1;
% Making sure we get the smallest angle
if(abs(r2)>pi)
    r2t=2*pi-abs(r2);
else
    r2t=abs(r2);
end
    
% Gain parameter
ky = 1000;

% Angular velocities of motors D and E
wd = 0;
thetad = 0;
we = 0;
thetae = 0;

% Linear accelerations and velocities of motors D and E, respectively
vdD = 0;
vd = 0;
veD = 0;
ve = 0;

% Holds the state of motion the platform is in
% '0' stands for static, but '1' stands for kinetic
state = 0;

% Indicates if the platform goes forwards or backwards
% '0' is the default forward motion and '1' if the 
% reverse gear is set
if(rt<a_dt)
    reverse = 1;
else
    reverse = 0;
end

% Matrices used for calculating friction forces on points A,B and C
fax = 0;
fay = 0;
fbx = 0;
fby = 0;
fcx = 0;
fcy = 0;

% Matrices used for holding friction forces on points A,B and C
nAx = 0;
nAy = 0;
nBx = 0;
nBy = 0;
nCx = 0;
nCy = 0;

% Accelerations of masses M1,M2 and M3 on x and y axes respectively
X1_DD = 0;
X2_DD = 0;
X3_DD = 0;
Y1_DD = 0;
Y2_DD = 0;
Y3_DD = 0;

% Velocities of masses M1,M2 and M3 on x and y axes respectively
X1_D = 0;
X2_D = 0;
X3_D = 0;
Y1_D = 0;
Y2_D = 0;
Y3_D = 0;

X1 = 0;
X2 = 0;
X3 = 0;
Y1 = 0;
Y2 = 0;
Y3 = 0;

% Friction forces of points A,B and C on x,y and z axes respectively
Ax = 0;
Bx = 0;
Cx = 0;
Ay = 0;
By = 0;
Cy = 0;
Az = 0;
Bz = 0;
Cz = 0;

% Velocities of points A,B and C on x,y and z axes respectively
vax = 0;
vay = 0;
vbx = 0;
vby = 0;
vcx = 0;
vcy = 0;

% Linear accelerations of platform on x and y axes
ax = 0;
ay = 0;
% Linear velocities of platform on x and y axes
Vx = 0;
Vy = 0;

% Contact points coordinates
qAx = DH/3;
qAy = Dl/2;

qBx = -2*DH/3;
qBy = 0;

qCx = DH/3;
qCy = -Dl/2;


%%
% Actuation forces that act on points A,B and C, 
% across each axis, with respect to the body-frame
FAx = 0;
FAz = 0;

FCx = 0;
FCz = 0;

CMvel = zeros(4,numel(t));
CMpos = zeros(3,numel(t));
CMrot = zeros(3,numel(t));

qadot = [0 0];
qbdot = [0 0];
qcdot = [0 0];

omegad = zeros(1,numel(t));
omegae = zeros(1,numel(t));

rot = zeros(1,numel(t));

phi = zeros(1,numel(t));
rev = zeros(1,numel(t));
a = zeros(1,numel(t));
b = zeros(1,numel(t));

for i=2:numel(t)
    phi_d = atan2(DeltaY,DeltaX);

    a1 = wrapTo2Pi(phi_d);
    
    a2 = wrapTo2Pi(CMpsi);
    
    a_d = a1 - a2;
    
    if(abs(a_d)>pi)
        a_dt = 2*pi - abs(a_d);
    else
        a_dt = abs(a_d);
    end
    
    eta_d = mod((pi + a1),2*pi);

    r1 = wrapTo2Pi(eta_d);

    rev_d = a2 - r1;

    if(abs(rev_d)>pi)
        rt=2*pi-abs(rev_d);
    else
        rt=abs(rev_d);
    end

    b1 = wrapTo2Pi(Thdes);

    b_d = a1-b1;

    if(abs(b_d)>pi)
        b_dt=2*pi-abs(b_d);
    else
        b_dt=abs(b_d);
    end

    r2 = b1 - r1;

    if(abs(r2)>pi)
        r2t=2*pi-abs(r2);
    else
        r2t=abs(r2);
    end

    if(abs(wd)<900 && wd>0)
        wd=900;
    elseif(abs(wd)<900 && wd<0)
        wd=-900;
    elseif(abs(wd)>1080 && wd>0)
        wd=1080;
    elseif(abs(wd)>1080 && wd<0)
        wd=-1080;
    end
    
    if(abs(we)<900 && we>0)
        we=900;
    elseif(abs(we)<900 && we<0)
        we=-900;
    elseif(abs(we)>1080 && we>0)
        we=1080;
    elseif(abs(we)>1080 && we<0)
        we=-1080;
    end

    thetad = thetad + wd*step;
    thetae = thetae + we*step;

    FAx = m*r*wd^2*sin(thetad);
    FAz = -m*g-m*r*wd^2*cos(thetad);

    FCx = -m*r*we^2*sin(thetae);
    FCz = -m*g-m*r*we^2*cos(thetae);

    if (norm(qadot)<vtol && norm(qbdot)<vtol && norm(qcdot)<vtol) || ...
        (norm(qadot)<vtol && norm(qbdot)<vtol && norm(qcdot)>=vtol) || ...
        (norm(qadot)<vtol && norm(qbdot)>=vtol && norm(qcdot)<vtol) || ...
        (norm(qadot)>=vtol && norm(qbdot)<vtol && norm(qcdot)<vtol)
        state = 0;
    else
        state = 1;
    end
    
    
    %% Static Phase / Deformable-Body Model
    if(state == 0)

        vax = X1_D;
        vay = Y1_D;

        vbx = X2_D;
        vby = Y2_D;

        vcx = X3_D;
        vcy = Y3_D;

        % Suppress small velocities (for avoiding chattering)
        if(sqrt(vax^2+vay^2)<vtol)
            vax=0;
            vay=0;
        end

        if(sqrt(vbx^2+vby^2)<vtol)
            vbx=0;
            vby=0;
        end

        if(sqrt(vcx^2+vcy^2)<vtol)
            vcx=0;
            vcy=0;
        end
        
        % Vertical reactions on each point
        Az = (-2*DH*Dl*(FAz+FCz)+g*M*(-3*Dl*cx+2*DH*(Dl-3*cy))+6*DH*(-FAz+FCz)*D1+0*3*Dl*(FAx+FCx)*ho)/(6*DH*Dl);

        Bz = (-(DH*(FAz+FCz))+g*M*(DH+3*cx)-0*3*(FAx+FCx)*ho)/(3*DH);

        Cz = (-2*DH*Dl*(FAz+FCz)+g*M*(-3*Dl*cx+2*DH*(Dl+3*cy))+6*DH*(FAz-FCz)*D1+0*3*Dl*(FAx+FCx)*ho)/(6*DH*Dl);

        % Friction conditions for each mass:
        
        % Mass 1
        nAx = FAx - (sqrt(3)*k1*X1)/2 + (sqrt(3)*k1*X2)/2;
        nAy = (-k1/2 - k3)*Y1 + (k1*Y2)/2 + k3*Y3;

        if(sqrt(vax^2+vay^2)==0)
            if(sqrt(nAx^2+nAy^2) < mi*abs(Az))
                fax = -nAx;
                fay = -nAy;
            else
                fax = -mi*abs(Az)*(nAx/sqrt(nAx^2+nAy^2));
                fay = -mi*abs(Az)*(nAy/sqrt(nAx^2+nAy^2));
            end
        else
            fax = -mi*abs(Az)*(vax/sqrt(vax^2+vay^2));
            fay = -mi*abs(Az)*(vay/sqrt(vax^2+vay^2));
        end

        % Mass 2
        nBx = (sqrt(3)*k1*X1)/2 + (-(sqrt(3)*k1)/2 - (sqrt(3)*k2)/2)*X2 + (sqrt(3)*k2*X3)/2;
        nBy = (k1*Y1)/2 + ((-k1 - k2)*Y2)/2 + (k2*Y3)/2;

        if(sqrt(vbx^2+vby^2)==0)
            if(sqrt(nBx^2+nBy^2)<mi*abs(Bz))
                fbx = -nBx;
                fby = -nBy;
            else
                fbx = -mi*abs(Bz)*(nBx/sqrt(nBx^2+nBy^2));
                fby = -mi*abs(Bz)*(nBy/sqrt(nBx^2+nBy^2));
            end
        else
            fbx = -mi*abs(Bz)*(vbx/sqrt(vbx^2+vby^2));
            fby = -mi*abs(Bz)*(vby/sqrt(vbx^2+vby^2));
        end

        % Mass 3
        nCx = FCx + (sqrt(3)*k2*X2)/2 - (sqrt(3)*k2*X3)/2;
        nCy = k3*Y1 + (k2*Y2)/2 + (-k2/2 - k3)*Y3;

        if(sqrt(vcx^2+vcy^2)==0)
            if(sqrt(nCx^2+nCy^2) < mi*abs(Cz))
                fcx = -nCx;
                fcy = -nCy;
            else
                fcx = -mi*abs(Cz)*(nCx/sqrt(nCx^2+nCy^2));
                fcy = -mi*abs(Cz)*(nCy/sqrt(nCx^2+nCy^2));
            end
        else
            fcx = -mi*abs(Cz)*(vcx/sqrt(vcx^2+vcy^2));
            fcy = -mi*abs(Cz)*(vcy/sqrt(vcx^2+vcy^2));
        end

        % Accelerations of the three masses on xy-plane
        X1_DD = (fax + FAx - (sqrt(3)*k1*X1)/2 + (sqrt(3)*k1*X2)/2)/M1;
        X2_DD = (fbx + (sqrt(3)*k1*X1)/2 + (-(sqrt(3)*k1)/2 - (sqrt(3)*k2)/2)*X2 + (sqrt(3)*k2*X3)/2)/M2;
        X3_DD = (fcx + FCx + (sqrt(3)*k2*X2)/2 - (sqrt(3)*k2*X3)/2)/M3;
        Y1_DD = (fay + (-k1/2-k3)*Y1 + (k1*Y2)/2 + k3*Y3)/M1;
        Y2_DD = (fby + (k1*Y1)/2 + ((-k1 - k2)*Y2)/2 + (k2*Y3)/2)/M2;
        Y3_DD = (fcy + k3*Y1 + (k2*Y2)/2 + (-k2/2-k3)*Y3)/M3;

        X1_D = X1_D+X1_DD*step;
        X2_D = X2_D+X2_DD*step;
        X3_D = X3_D+X3_DD*step;
        Y1_D = Y1_D+Y1_DD*step;
        Y2_D = Y2_D+Y2_DD*step;
        Y3_D = Y3_D+Y3_DD*step;

        X1 = X1+X1_D*step;
        X2 = X2+X2_D*step;
        X3 = X3+X3_D*step;
        Y1 = Y1+Y1_D*step;
        Y2 = Y2+Y2_D*step;
        Y3 = Y3+Y3_D*step;
         
        % Friction forces
        Ax = fax;
        Bx = fbx;
        Cx = fcx;
        Ay = fay;
        By = fby;
        Cy = fcy;

        % Equations of motion
        ax = ((Ax + Bx + Cx + FAx + FCx)*cos(CMpsi) - (Ay + By + Cy)*sin(CMpsi))/M;

        ay = ((Ay + By + Cy)*cos(CMpsi) + (Ax + Bx + Cx + FAx + FCx)*sin(CMpsi))/M;

        CMpsi_DD = (2*Ay*DH - 4*By*DH + 2*Cy*DH - 3*Ax*Dl + 3*Cx*Dl - 6*FAx*D1 + 6*FCx*D1)/(6*Iz);

    % Kinetic Phase/Solid body
    elseif(state == 1)
        
        % Velocities are expressed wrt the body-fixed frame of reference
        vax = cos(CMpsi)*qadot(1) + sin(CMpsi)*qadot(2);
        vay = -sin(CMpsi)*qadot(1) + cos(CMpsi)*qadot(2);

        vbx = cos(CMpsi)*qbdot(1) + sin(CMpsi)*qbdot(2);
        vby = -sin(CMpsi)*qbdot(1) + cos(CMpsi)*qbdot(2);

        vcx = cos(CMpsi)*qcdot(1) + sin(CMpsi)*qcdot(2);
        vcy = -sin(CMpsi)*qcdot(1) + cos(CMpsi)*qcdot(2);
        
        % Suppress the small velocities (for avoiding chattering)
        % cos(theta) and sin(theta) where theta is the velocity angle wrt the
        % x-axis of the inertial frame of reference

        if (sqrt(vax^2+vay^2) < vtol)
            vaxRatio = 0;
            vayRatio = 0;
        else
            vaxRatio = -(vax/sqrt(vax^2+vay^2));
            vayRatio = -(vay/sqrt(vax^2+vay^2));
        end

        if (sqrt(vbx^2+vby^2) < vtol)
            vbxRatio = 0;
            vbyRatio = 0;
        else
            vbxRatio = -(vbx/sqrt(vbx^2+vby^2));
            vbyRatio = -(vby/sqrt(vbx^2+vby^2));
        end

        if (sqrt(vcx^2+vcy^2) < vtol)
            vcxRatio = 0;
            vcyRatio = 0;
        else
            vcxRatio = -(vcx/sqrt(vcx^2+vcy^2));
            vcyRatio = -(vcy/sqrt(vcx^2+vcy^2));
        end
        
        % Reaction forces during general motion
        Az = (-2*DH*Dl*(FAz + FCz) + g*M*(-3*Dl*cx + 2*DH*(Dl-3*cy)) + ...
       6*DH*(-FAz + FCz)*D1 + 0*3*Dl*(FAx + FCx)*ho)/(6*DH*Dl);

        Bz = (-(DH*(FAz + FCz)) + g*M*(DH + 3*cx) - 0*3*(FAx + FCx)*ho)/(3*DH);

        Cz = (-2*DH*Dl*(FAz + FCz) + g*M*(-3*Dl*cx + 2*DH*(Dl + 3*cy)) + ...
       6*DH*(FAz - FCz)*D1 + 0*3*Dl*(FAx + FCx)*ho)/(6*DH*Dl);

        if(Az<0)
           Az = 0;
           disp('Az reaction is zero. The platform is tipping')
           disp('Please reduce actuation speed.')
        end

        if(Bz<0)
           Bz = 0;
           disp('Bz reaction is zero. The platform is tipping')
           disp('Please reduce actuation speed.')
        end

        if(Cz<0)
           Cz = 0;
           disp('Cz reaction is zero. The platform is tipping')
           disp('Please reduce actuation speed.')
        end

        %  General plane motion of the platform
        %  disp('general plane motion')
        
        Ax = mi*Az*vaxRatio;
        Ay = mi*Az*vayRatio;

        Bx = mi*Bz*vbxRatio;
        By = mi*Bz*vbyRatio;

        Cx = mi*Cz*vcxRatio;
        Cy = mi*Cz*vcyRatio;
        
        % Equations of motion
        ax = ((Ax + Bx + Cx + FAx + FCx)*cos(CMpsi) - (Ay + By + Cy)*sin(CMpsi))/M;

        ay = ((Ay + By + Cy)*cos(CMpsi) + (Ax + Bx + Cx + FAx + FCx)*sin(CMpsi))/M;

        CMpsi_DD = (2*Ay*DH - 4*By*DH + 2*Cy*DH - 3*Ax*Dl + 3*Cx*Dl - 6*FAx*D1 + 6*FCx*D1)/(6*Iz);

        % Reset velocities of points A,B and C  
        X1_D = 0;
        Y1_D = 0;
        X2_D = 0;
        Y2_D = 0;
        X3_D = 0;
        Y3_D = 0;

        % Reset positions of points A,B and C 
        X1 = 0;
        Y1 = 0;
        X2 = 0;
        Y2 = 0;
        X3 = 0;
        Y3 = 0;
    end

    % Linear velocities of platform on x and y axes respectively
    Vx = Vx+ax*step;
    Vy = Vy+ay*step;

    Px = Px+Vx*step;
    Py = Py+Vy*step;
    
    % Angular velocity of platform
    CMpsi_D = CMpsi_D+CMpsi_DD*step;

    % Angle of platform
    CMpsi = CMpsi+CMpsi_D*step;
        
    % Relation of linear velocities of platform with the velocities of
    % points A,B and C
    qadot = [1, 0, -(qAy*cos(CMpsi)) - qAx*sin(CMpsi); 0, 1, qAx*cos(CMpsi) - qAy*sin(CMpsi)]*[Vx, Vy, CMpsi_D]';

    qbdot = [1, 0, -(qBy*cos(CMpsi)) - qBx*sin(CMpsi); 0, 1, qBx*cos(CMpsi) - qBy*sin(CMpsi)]*[Vx, Vy, CMpsi_D]'; 

    qcdot = [1, 0, -(qCy*cos(CMpsi)) - qCx*sin(CMpsi); 0, 1, qCx*cos(CMpsi) - qCy*sin(CMpsi)]*[Vx, Vy, CMpsi_D]';

    % Velocities of platform on x and y axes
    CMvel(1,i) = t(i);
    CMvel(2,i) = Vx;
    CMvel(3,i) = Vy;
    CMvel(4,i) = CMpsi_D; % Angular velocity of platform

    CMpos(1,i) = t(i);
    CMpos(2,i) = Px;
    CMpos(3,i) = Py;

    CMrot(1,i) = t(i);
    CMrot(2,i) = CMpsi_D;
    CMrot(3,i) = CMpsi;
    
    % Finding the angle between the current (as a2) 
    % and the desired angle of the platform (as b1).
    % WE CARE ABOUT THE SIGN OF THIS ANGLE TOO!!
    DeltaTh = b1 - a2;
    
    phi(i) = phi_d;
    rev(i) = eta_d;
    a(i) = a_d;
    b(i) = b_d;
    
    if(sqrt(DeltaX^2 + DeltaY^2)<10^(-5))
        if(abs(DeltaTh)<10^(-5))
            wd=0;
            we=0;
            thetad=0;
            thetae=0;
            CMpsi_D=0;
            CMpsi_DD=0;
            Vx=0;
            Vy=0;
            rot(i)=10;
        elseif(DeltaTh<0)
            if(abs(DeltaTh)>pi)
                wd=-1080;
                we=-1080;
                rot(i)=-11;
            else
                wd=1080;
                we=1080;
                rot(i)=11;
            end
            thetad=0;
            thetae=0;
            CMpsi_D=0;
            CMpsi_DD=0;
            Vx=0;
            Vy=0;
        elseif(DeltaTh>0)
            if(abs(DeltaTh)>pi)
                wd=1080;
                we=1080;
                rot(i)=11;
            else
                wd=-1080;
                we=-1080;
                rot(i)=-11;
            end
            thetad=0;
            thetae=0;
            CMpsi_D=0;
            CMpsi_DD=0;
            Vx=0;
            Vy=0;
        end
        
    % While platform's orientation is not within
    % the range [phi_d-0.03,phi_d+0.03], nor is
    % it's back, rotate until one of them is by
    % using the following law for preferring 
    % which one is the best choice
    elseif(a_dt>0.03 && rt>0.03)
        % Check which way to prefer:
        % 1) Rotate until platform faces target,
        % reach it and then rotate again to get 
        % the desired angle, or
        % 2) Rotate the other way until platform's 
        % back faces target and go backwards. Then  
        % rotate again to get the desired angle
        if(rt+r2t<a_dt+b_dt)
            if((rev_d>0 && rev_d<pi) || (rev_d<0 && abs(rev_d)>pi))
                wd = 1080;
                we = 1080;
                rot(i)=1;
            else
                wd = -1080;
                we = -1080;
                rot(i)=-1;
            end
            reverse=1;
        else
            if((a_d>0 && a_d<pi) || (a_d<0 && abs(a_d)>pi))
                wd = -1080;
                we = -1080;
                rot(i)=-1;
            else
                wd = 1080;
                we = 1080;
                rot(i)=1;
            end
            reverse=0;
        end
    else
        
            vdD = ky*DeltaY;
            veD = -ky*DeltaY;

            vd = vd + vdD*step;
            ve = ve + veD*step;

            vd=abs(vd);
            ve=abs(ve);

            p1 = 8.764e+10;
            p2 = -2.676e+08;
            p3 = 3.902e+05;
            p4 = 850.9;

            wd = p1*vd^3 + p2*vd^2 + p3*vd + p4;
            we = p1*ve^3 + p2*ve^2 + p3*ve + p4;

        if(reverse==1)
            wd=-wd;
        else
            we=-we;
        end
        
        wd=900;
        we=-1080;
        
        rot(i)=0;
    end
    
    
    
    omegad(i)=wd;
    omegae(i)=we;
    
end

%Important Plots
figure
subplot(3,1,1)
plot(t,CMvel(2,:))
title('Velocity of platform on X axis')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
subplot(3,1,2)
plot(t,CMvel(3,:))
title('Velocity of platform on Y axis')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
subplot(3,1,3)
plot(t,CMrot(2,:))
title('Rotational speed of CM')
xlabel('Time (s)')
ylabel('RSpeed (rad/s)')

figure
subplot(3,1,1)
plot(t,CMpos(2,:))
title('Position of platform on X axis')
xlabel('Time (s)')
ylabel('Position (m)')
subplot(3,1,2)
plot(t,CMpos(3,:))
title('Position of platform on Y axis')
xlabel('Time (s)')
ylabel('Position (m)')
subplot(3,1,3)
plot(t,CMrot(3,:))
title('Angle of CM, psi')
xlabel('Time (s)')
ylabel('Angle psi(rad)')

figure
plot(CMpos(2,:),CMpos(3,:))
axis equal
title('Position of platform on XY plane')
xlabel('X')
ylabel('Y')

figure
subplot(2,1,1)
plot(t,b1)
xlabel('Time(s)')
ylabel('Angle b(rad)')
subplot(2,1,2)
plot(t,a)
xlabel('Time(s)')
ylabel('Angle a(rad)')

figure
subplot(2,1,1)
plot(t,phi)
xlabel('Time(s)')
ylabel('Phi_d')
subplot(2,1,2)
plot(t,rev)
xlabel('Time(s)')
ylabel('Reversed Angle')

figure
subplot(2,1,1)
plot(t,omegad(1,:))
xlabel('Time(s)')
ylabel('Omega D')
subplot(2,1,2)
plot(t,omegae(1,:))
xlabel('Time(s)')
ylabel('Omega E')

figure
plot(t,rot)
title('Rotation State')
xlabel('Time (s)')
ylabel('State')

vars=[phi_d,a1,a2,a_d,b_d,a_dt,b_dt,a_dt+b_dt,eta_d,r1,rev_d,rt,r2,r2t,rt+r2t,b1,DeltaTh,wd,we];
