function [p_d,v_d,a_d,psi_d,waypoints]=label_representative_trajectory(t,R_turn_V,R_turn_H)
%This function returns the acceleration, velocity and position related to
%the representative trajectory chosen to further evaluate the present
%controller implementation. It also provides the heading angle. Details on paper. 

%Modified to receive R_turn as an input to facilitate trajectory choice for
%task 2. 


%Define Trajectory Parameters:
%AB
a_AB=0.5;   %[m/s^2]
dt_AB=4;     %[s]


%BC - DE - FG (all turns)
v_turn=2;   %[m/s]
Omega_turn_V=v_turn/R_turn_V; %[rad/s]
Omega_turn_H=v_turn/R_turn_H; %[rad/s]
dt_turn_V=pi/2/Omega_turn_V; %[s]
dt_turn_H=pi/2/Omega_turn_H; %[s]


% Accelerations (CD and EF)
dt_accel=5/3; %[s]
a_accel=1.2;  %[m/s^2]

% Dashes
dt_dash_CD=10/4; %[s]
dt_dash_EF=5/4;  %[s]
v_dash=4;        %[m/s]

% Decelerations
dt_decel=5/3; %[s]
a_decel=-1.2; %[m/s^2]

% GH
dt_GH=4;      %[s]
a_GH=0.5;    %[m/s^2]

%Define timestamps: 
t_A=0;
t_B=t_A+dt_AB;
t_C=t_B+dt_turn_V;
t_CD1=t_C+dt_accel;
t_CD2=t_CD1+dt_dash_CD;
t_D=t_CD2+dt_decel;
t_E=t_D+dt_turn_H;
t_EF1=t_E+dt_accel;
t_EF2=t_EF1+dt_dash_EF;
t_F=t_EF2+dt_decel;
t_G=t_F+dt_turn_V;
t_H=t_G+dt_GH;


%Define trajectory waypoints
p_A=[0;0;1];
p_B=p_A+[0;0;1/2*a_AB*dt_AB^2];
p_C=p_B+R_turn_V*[1;0;1];
p_CD1=p_C+(1/2*a_accel*dt_accel^2+v_turn*dt_accel)*[1;0;0];
p_CD2=p_CD1+(v_dash*dt_dash_CD)*[1;0;0];
p_D=p_CD2+(1/2*a_decel*dt_decel^2+v_dash*dt_decel)*[1;0;0];
p_E=p_D+R_turn_H*[1;1;0];
p_EF1=p_E+(1/2*a_accel*dt_accel^2+v_turn*dt_accel)*[0;1;0];
p_EF2=p_EF1+v_dash*dt_dash_EF*[0;1;0];
p_F=p_EF2+(1/2*a_decel*dt_decel^2+v_dash*dt_decel)*[0;1;0];
p_G=p_F+R_turn_V*[0;1;-1];
p_H=p_G+(1/2*a_GH*dt_GH^2-v_turn*dt_GH)*[0;0;1];

waypoints=[p_A,p_B,p_C,p_D,p_E,p_F,p_G,p_H];

if t<t_B %Between A and B
    p_d=p_A+[0;0;1/2*a_AB*t^2];
    v_d=a_AB*t*[0;0;1];
    a_d=a_AB*[0;0;1];
    psi_d=0;
elseif t<t_C %Between B and C
    dt=t-t_B;
    p_d=p_B+[R_turn_V*(1-cos(Omega_turn_V*dt));0;R_turn_V*sin(Omega_turn_V*dt)];
    v_d=v_turn*[sin(Omega_turn_V*dt);0;cos(Omega_turn_V*dt)];
    a_d=v_turn*Omega_turn_V*[cos(Omega_turn_V*dt);0;-sin(Omega_turn_V*dt)];
    psi_d=0;
elseif t<t_CD1 %Acceleration after C
    dt=t-t_C;
    p_d=p_C+(1/2*a_accel*dt^2+v_turn*dt)*[1;0;0];
    v_d=(v_turn+a_accel*dt)*[1;0;0];
    a_d=a_accel*[1;0;0];
    psi_d=0;
elseif t<t_CD2 % CD dash
    dt=t-t_CD1;
    p_d=p_CD1+v_dash*dt*[1;0;0];
    v_d=v_dash*[1;0;0];
    a_d=[0;0;0];
    psi_d=0;
elseif t<t_D % CD deceleration
    dt=t-t_CD2;
    p_d=p_CD2+(1/2*a_decel*dt^2+v_dash*dt)*[1;0;0];
    v_d=(v_dash+a_decel*dt)*[1;0;0];
    a_d=a_decel*[1;0;0];
    psi_d=0;
elseif t<t_E % DE turn
    dt=t-t_D;
    p_d=p_D+[R_turn_H*sin(Omega_turn_H*dt);R_turn_H*(1-cos(Omega_turn_H*dt));0];
    v_d=v_turn*[cos(Omega_turn_H*dt);sin(Omega_turn_H*dt);0];
    a_d=v_turn*Omega_turn_H*[-sin(Omega_turn_H*dt);cos(Omega_turn_H*dt);0];
    psi_d=Omega_turn_H*dt;
elseif t<t_EF1 %EF acceleration
    dt=t-t_E;
    p_d=p_E+(1/2*a_accel*dt^2+v_turn*dt)*[0;1;0];
    v_d=(v_turn+a_accel*dt)*[0;1;0];
    a_d=a_accel*[0;1;0];
    psi_d=pi/2;
elseif t<t_EF2 %EF dash
    dt=t-t_EF1;
    p_d=p_EF1+v_dash*dt*[0;1;0];
    v_d=v_dash*[0;1;0];
    a_d=[0;0;0];
    psi_d=pi/2;
elseif t<t_F %EF deceleration
    dt=t-t_EF2;
    p_d=p_EF2+(1/2*a_decel*dt^2+v_dash*dt)*[0;1;0];
    v_d=(v_dash+a_decel*dt)*[0;1;0];
    a_d=a_decel*[0;1;0];
    psi_d=pi/2;
elseif t<t_G %FG turn
    dt=t-t_F;
    p_d=p_F+[0;R_turn_V*sin(Omega_turn_V*dt);-R_turn_V*(1-cos(Omega_turn_V*dt))];
    v_d=v_turn*[0;cos(Omega_turn_V*dt);-sin(Omega_turn_V*dt)];
    a_d=v_turn*Omega_turn_V*[0;-sin(Omega_turn_V*dt);-cos(Omega_turn_V*dt)];
    psi_d=pi/2;
elseif t<t_H %GH descent
    dt=t-t_G;
    p_d=p_G+[0;0;1/2*a_GH*dt^2-v_turn*dt]; %note that the initial velocity is [0;0;-v_turn] since one comes out of the circle going downwards
    v_d=(-v_turn+a_GH*dt)*[0;0;1];
    a_d=a_GH*[0;0;1];
    psi_d=pi/2;
else %at t>t_H, remain at point H 
    p_d=p_H;
    v_d=[0;0;0];
    a_d=[0;0;0];
    psi_d=pi/2;
end
end