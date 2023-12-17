

%discount_v[V_NUM_MAX]={180,118,100,91,90,45};
%discount_d[D_NUM_MAX]={127,91,79,71,65,64,32};
Gain=10000/5;

Turn_velocity=1000;
max_velocity=3000;
acc=10000;

One_aria_v=90;
One_aria_d=45*sqrt(2);

Cost_v(2)=One_aria_v/Turn_velocity;

for i=1:32

    if One_aria_v*i<(max_velocity^2-Turn_velocity^2)/acc
        peak_velocity=(Turn_velocity^2+acc*One_aria_v*i)^(1/2);
        Cost_v(i+1)=2*(peak_velocity-Turn_velocity)/acc;
    else
        const_dis=One_aria_v*i-(max_velocity^2-Turn_velocity^2)/acc;
        Cost_v(i+1)=2*(max_velocity-Turn_velocity)/acc+const_dis/max_velocity;
    end

end


Cost_v_diff=diff(Cost_v)*Gain;
Cost_v_diff(1)=One_aria_v/Turn_velocity*Gain;
CostG_v= uint16(cumsum(Cost_v_diff));

Cost_d(2)=One_aria_d/Turn_velocity;

for i=1:64

    if One_aria_d*i<(max_velocity^2-Turn_velocity^2)/acc
        peak_velocity=(Turn_velocity^2+acc*One_aria_d*i)^(1/2);
        Cost_d(i+1)=2*(peak_velocity-Turn_velocity)/acc;
    else
        const_dis=One_aria_d*i-(max_velocity^2-Turn_velocity^2)/acc;
        Cost_d(i+1)=2*(max_velocity-Turn_velocity)/acc+const_dis/max_velocity;
    end

end

Cost_d_diff=diff(Cost_d)*Gain;
Cost_d_diff(1)=One_aria_d/Turn_velocity*Gain;
CostG_d= uint16(cumsum(Cost_d_diff));
