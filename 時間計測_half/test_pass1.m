max_v=3000;
acc=10000;
%　pass
A_pass=[2 -8 -17 -16 51 -16 -17 -16 51 -16 -17 -16 51 -14 2 -7 2];
TT8=cal_running_time(A_pass,max_v,acc);
A_time=sum(TT8)

%　pass
A_pass=[2 -8 -17 -16 51 -16 -17 -16 53 -12 2 -4 2 -4 2 -7 2];
TT8=cal_running_time(A_pass,max_v,acc);
B_time=sum(TT8)

