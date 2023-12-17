function T=cal_running_time(pass,max_velocity,acc)
x=45;y=45;Theta=0;n=0;
MAZE_SECTION=90;
[turning_data]=input_turning_1000;
center_velocity=turning_data.TurnCentervelocity;
for i=1:length(pass)
if pass(i)==-1
elseif pass(i)>0 && pass(i)<500
    [x,y,Theta,n]=ST_orbit(pass(i)*MAZE_SECTION/2,center_velocity,center_velocity,max_velocity,acc,x,y,Theta,n);
    if pass(i)*90>(max_velocity^2-center_velocity^2)/acc
    %daikei
        T(i)=(max_velocity-center_velocity)/acc*2+(pass(i)*MAZE_SECTION/2-(max_velocity^2-center_velocity^2)/acc)/max_velocity;

    else
        T(i)=((center_velocity^2+pass(i)*MAZE_SECTION/2*acc)^(1/2)-center_velocity)/acc*2;
    end
elseif pass(i)>=500
    [x,y,Theta,n]=ST_orbit((pass(i)-500)*MAZE_SECTION/2*2^(1/2),center_velocity,center_velocity,max_velocity,acc,x,y,Theta,n);
    if (pass(i)-500)*MAZE_SECTION/2*2^(1/2)>(max_velocity^2-center_velocity^2)/acc
    %daikei
        T(i)=(max_velocity-center_velocity)/acc*2+((pass(i)-500)*MAZE_SECTION/2*2^(1/2)-(max_velocity^2-center_velocity^2)/acc)/max_velocity;
    else
        T(i)=((center_velocity^2+(pass(i)-500)*MAZE_SECTION/2*2^(1/2)*acc)^(1/2)-center_velocity)/acc*2;
    end
elseif pass(i)==0
else
    [T(i),x,y,Theta,n]=time_turning(pass(i),turning_data,x,y,Theta,n);
end

end

figure()
hold on
plot([0 180*16],[0 0],'-k','LineWidth',1.5)
plot([0 180*16],[180*16 180*16],'-k','LineWidth',1.5)
plot([0 0],[0 180*16],'-k','LineWidth',1.5)
plot([180*16 180*16],[0 180*16],'-k','LineWidth',1.5)

for n=1:31
    plot([0 180*16],[90*n 90*n],':k','LineWidth',0.3)
    plot([90*n 90*n],[0 180*16],':k','LineWidth',0.3)
end
plot(x,y)

xlim([-100 180*16+100])
ylim([-100 180*16+100])
pbaspect([1 1 1])

end



function [x,y,Theta,n]=ST_orbit(displacement,start_velocity,end_velocity,max_velocity,acc,x,y,Theta,n)
dt=0.001;
now_displacement=0;
now_velocity=start_velocity;

while now_velocity<max_velocity && (displacement >(2*now_velocity^2-start_velocity^2-end_velocity^2)/2/acc)
    n=n+1;
    now_displacement=now_displacement+dt * now_velocity;
    x(n+1) = x(n)+dt * now_velocity * sin(Theta*pi/180);
    y(n+1) = y(n)+dt * now_velocity * cos(Theta*pi/180);
    Theta=Theta+dt * 0;
    now_velocity=now_velocity+acc*dt;
end

while (displacement-now_displacement >(now_velocity^2-end_velocity^2)/2/acc)
    n=n+1;
    now_displacement=now_displacement+dt * now_velocity;
    x(n+1) = x(n)+dt * now_velocity * sin(Theta*pi/180);
    y(n+1) = y(n)+dt * now_velocity * cos(Theta*pi/180);
    Theta=Theta+dt * 0;
    now_velocity=now_velocity+0*dt;
end

while now_velocity > end_velocity
    n=n+1;
    now_displacement=now_displacement+dt * now_velocity;
    x(n+1) = x(n)+dt * now_velocity * sin(Theta*pi/180);
    y(n+1) = y(n)+dt * now_velocity * cos(Theta*pi/180);
    Theta=Theta+dt * 0;
    now_velocity=now_velocity-acc*dt;
end



end




function [T,x,y,Theta,n]=time_turning(mode,turning_data,x,y,Theta,n)

switch mode
    case{-2}
     [T,x,y,Theta,n]=cal_turning_time(-90,turning_data.slalom_R,x,y,Theta,n);
    case{-3}
     [T,x,y,Theta,n]=cal_turning_time(90,turning_data.slalom_L,x,y,Theta,n);
    case{-4}
    [T,x,y,Theta,n]=cal_turning_time(-90,turning_data.turn90_R,x,y,Theta,n);
    case{-5}
     [T,x,y,Theta,n]=cal_turning_time(90,turning_data.turn90_L,x,y,Theta,n);
    case{-6}
           [T,x,y,Theta,n]=cal_turning_time(-180,turning_data.turn180_R,x,y,Theta,n); 
    case{-7}
         [T,x,y,Theta,n]=cal_turning_time(180,turning_data.turn180_L,x,y,Theta,n); 
    case{-8}
        [T,x,y,Theta,n]=cal_turning_time(-45,turning_data.turn45in_R,x,y,Theta,n);
    case{-9}
         [T,x,y,Theta,n]=cal_turning_time(45,turning_data.turn45in_L,x,y,Theta,n);
    case{-10}
        [T,x,y,Theta,n]=cal_turning_time(-135,turning_data.turn135in_R,x,y,Theta,n);
    case{-11}
         [T,x,y,Theta,n]=cal_turning_time(135,turning_data.turn135in_L,x,y,Theta,n);
    case{-12}
        [T,x,y,Theta,n]=cal_turning_time(-45,turning_data.turn45out_R,x,y,Theta,n);
    case{-13}
         [T,x,y,Theta,n]=cal_turning_time(45,turning_data.turn45out_L,x,y,Theta,n);
    case{-14}
        [T,x,y,Theta,n]=cal_turning_time(-135,turning_data.turn135out_R,x,y,Theta,n);
    case{-15}
         [T,x,y,Theta,n]=cal_turning_time(135,turning_data.turn135out_L,x,y,Theta,n);
    case{-16}
         [T,x,y,Theta,n]=cal_turning_time(-90,turning_data.V90_R,x,y,Theta,n);
    case{-17}
        [T,x,y,Theta,n]=cal_turning_time(90,turning_data.V90_L,x,y,Theta,n);
end



end

