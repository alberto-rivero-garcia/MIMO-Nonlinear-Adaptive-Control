%This script plots the representative trajectory for the project
%presentation: 
%Plot trajectory:
R_turn_H=5;
R_turn_V=1.5;
tv=linspace(0,25,1000);
ref_trajectory=nan(3,length(tv));

for i=1:length(tv)
    [ref_trajectory(:,i),~,~,~,waypoints]=label_representative_trajectory(tv(i),R_turn_V,R_turn_H);
end

figure
clf
plot3(ref_trajectory(1,:),ref_trajectory(2,:),ref_trajectory(3,:))
hold on
for i=1:8
    scatter3(waypoints(1,i),waypoints(2,i),waypoints(3,i),'r.','SizeData',100)
end
text(waypoints(1,1)+0.2,waypoints(2,1),waypoints(3,1),'A')
text(waypoints(1,2)+0.2,waypoints(2,2),waypoints(3,2),'B')
text(waypoints(1,3),waypoints(2,3),waypoints(3,3)+0.2,'C')
text(waypoints(1,4),waypoints(2,4),waypoints(3,4)+0.2,'D')
text(waypoints(1,5),waypoints(2,5),waypoints(3,5)+0.2,'E')
text(waypoints(1,6),waypoints(2,6),waypoints(3,6)+0.2,'F')
text(waypoints(1,7)+0.2,waypoints(2,7),waypoints(3,7),'G')
text(waypoints(1,8)+0.2,waypoints(2,8),waypoints(3,8),'H')