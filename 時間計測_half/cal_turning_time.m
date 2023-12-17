function [Time,x,y,Theta,n]=cal_turning_time(theta,turning_data,x,y,Theta,n)
D=2;
%theta=90;
center_velocity=turning_data.g_speed;
omega_max=turning_data.t_speed;


CC=1;
T=0.1;

fun1 = @(A)mollifier_seki(A,T,theta);

A1=-T^2/4;
MOLLIFIER = @(t) (T./2)^(-D/2).*exp(-(T./2).^D./((T./2).^D-t.^D).*CC);
dt=0.001;
% THETA=0;
% for tt=[-T/2+dt:dt:T/2-dt]
%     THETA=THETA+dt*MOLLIFIER(tt);
% end
THETA=integral(MOLLIFIER,-T/2,T/2);


%%kokokara--
T=2*abs(theta)/THETA*exp(-1)/omega_max;
MOLLIFIER = @(t) -(T./2)^(-D/2).*theta./THETA.*exp(-(T./2).^D./((T./2).^D-t.^D).*CC);
MOLLIFIER_DIFF = @(t) -(T./2)^(-D/2).*theta./THETA.*(-T.^2./2.*t)./((T.^2./4-t.^2).^2).*CC.*exp(-(T./2).^D./((T./2).^D-t.^D).*CC);



% Theta=0;
% 
% x=0;
% y=0;
n0=n;
omega=0;
for i=[0:dt:turning_data.f_ofset/turning_data.g_speed]
    n=n+1;
    x(n+1) = x(n)+dt * center_velocity * sin(Theta*pi/180);
    y(n+1) = y(n)+dt * center_velocity * cos(Theta*pi/180);
    omega(n+1)=0;
    A_omega2(n+1)=0;
    Theta=Theta+dt * 0;
end
for tt=[-T/2+dt:dt:T/2-dt]
    n=n+1;
    x(n+1) = x(n)+dt * center_velocity * sin(Theta*pi/180);
    y(n+1) = y(n)+dt * center_velocity * cos(Theta*pi/180);
    omega(n+1)=MOLLIFIER(tt);
    A_omega2(n+1)=MOLLIFIER_DIFF(tt);
    Theta=Theta+dt * MOLLIFIER(tt);

end
for i=[0:dt:turning_data.e_ofset/turning_data.g_speed]
    n=n+1;
    x(n+1) = x(n)+dt * center_velocity * sin(Theta*pi/180);
    y(n+1) = y(n)+dt * center_velocity * cos(Theta*pi/180);
    omega(n+1)=0;
    A_omega2(n+1)=0;
    Theta=Theta+dt * 0;
end

Time=(n-n0)*dt;

end


function [X]=mollifier_seki(A,T,theta)

MOLLIFIER = @(t) exp(A/(T^2/4-t^2));
dt=0.001;

X=0;
n=0;
for tt=[-T/2+dt:dt:T/2-dt]
    n=n+1;
    x(n)=MOLLIFIER(tt);
    X=X+dt * MOLLIFIER(tt);

end
X=X-1;

end



%% solver
function [x,fval,exitflag,output,jacobian] = fslove_maker(x0,MaxIterations_Data,fun)
%% これは、最適化ツールで自動的に生成された MATLAB ファイルです。

%% 既定のオプションで開始
options = optimoptions('fsolve');
%% オプションの設定の変更
options = optimoptions(options,'Display', 'iter-detailed');
options = optimoptions(options,'MaxIterations', MaxIterations_Data);
options = optimoptions(options,'Algorithm', 'levenberg-marquardt');
[x,fval,exitflag,output,jacobian] = ...
fsolve(fun,x0,options);



end