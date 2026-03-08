%This script checks the implementation of the representative trajectory
%chosen to further analyze the current controller implementation.

%Define Trajectory Waypoints: I'll use them to check the trajectory: make
%sense for R_turn=0.5 m both for the vertical and the horizontal turns. 
p_A=[0;0;1];
p_B=[0;0;5];
p_C=[0.5;0;5.5];
p_D=[20.5;0;5.5];
p_E=[21;0.5;5.5];
p_F=[21;15.5;5.5];
p_G=[21;16;5];
p_H=[21;16;1];

%Plot trajectory:
tv=linspace(0,30,1000);
trajectory=nan(3,length(tv));
R_turn=0.5; %[m]
for i=1:length(tv)
    [trajectory(:,i),~,~,~]=representative_trajectory(tv(i),R_turn,R_turn);
end

figure
clf
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:))
hold on
plot3(p_A(1),p_A(2),p_A(3),'r*')
plot3(p_B(1),p_B(2),p_B(3),'r*')
plot3(p_C(1),p_C(2),p_C(3),'r*')
plot3(p_D(1),p_D(2),p_D(3),'r*')
plot3(p_E(1),p_E(2),p_E(3),'r*')
plot3(p_F(1),p_F(2),p_F(3),'r*')
plot3(p_G(1),p_G(2),p_G(3),'r*')
plot3(p_H(1),p_H(2),p_H(3),'r*')