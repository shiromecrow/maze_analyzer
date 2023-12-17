max_v=5000;
acc=24000;
my_pass=[-10 -13 8 -9 -16 -13 6 -7 4 -6 4 -7 16 -4 2 -10 -13 8 -11 51 -13];

% TT6=cal_running_time(pass1.VarName4,max_v,acc);
% my_time=sum(TT6)
% 
% TT=cal_running_time(pass3.VarName4,max_v,acc);
% my_pass_new=sum(TT)

taiwan_pass=[-10 -13 8 -9 -16 -13 10 -5 24 -5 -5 8 -9 -12 -8 53 -16 54 -13];
TT2=cal_running_time( taiwan_pass,max_v,acc);
taiwan_pass2=[-10 -13 8 -9 -16 -13 6 -9 53 -13 20 -5 -5 8 -9 -12 -8 53 -16 54 -13];
taiwan_pass1=sum(TT2)
TT3=cal_running_time( taiwan_pass2,max_v,acc);
taiwan_pass2=sum(TT3)
taiwan_pass3=[-10 -13 8 -9 -16 -13 6 -9 53 -13 18 -11 51 -13  2 -9 -12 -8 53 -16 54 -13];
TT4=cal_running_time( taiwan_pass3,max_v,acc);
taiwan_pass3=sum(TT4)

nakasima_pass=[12 -8 -13 6 -9 -12 2 -4 10 -8 -13 -9 -12 2 -8 51 -12 8 -9 -12 -8 53 -16 54 -13];
TT5=cal_running_time(nakasima_pass,max_v,acc);
nakasima_time=sum(TT5)

%　東日本地区大会のpass
utunomiya_pass=[-8 -17 -16 -17 -12 -4 4 -7 4 -4 2 -4 12 -7 12 -4 4 -4 22 -6 -5 -8 52 -13 4 -4];
TT8=cal_running_time(utunomiya_pass,max_v,acc);
utunomiya_time=sum(TT8)

