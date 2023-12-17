max_v=3000;
acc=10000;
%　pass
A_pass=double(Pass');
TT8=cal_running_time(A_pass,max_v,acc);
A_time=sum(TT8)