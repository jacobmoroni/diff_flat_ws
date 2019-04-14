clc
clear
time_start = 500;
time_stop = 600;
load data/pid_data2.mat
pid_pn = pn(time_start:time_stop);
pid_pe = pe(time_start:time_stop);
pid_pd = pd(time_start:time_stop);
pid_pn_c = pn_c(time_start:time_stop);
pid_pe_c = pe_c(time_start:time_stop);
pid_pd_c = pd_c(time_start:time_stop);
pid_u = u(time_start:time_stop);
pid_v = v(time_start:time_stop);
pid_w = w(time_start:time_stop);
pid_u_c = u_c(time_start:time_stop);
pid_v_c = v_c(time_start:time_stop);
pid_w_c = w_c(time_start:time_stop);
pid_yaw_c = yaw_c(time_start:time_stop);
pid_pitch_c = pitch_c(time_start:time_stop);
pid_roll_c = roll_c(time_start:time_stop);
pid_yaw = yaw(time_start:time_stop);
pid_pitch = pitch(time_start:time_stop);
pid_roll = roll(time_start:time_stop);
pid_p = p(time_start:time_stop);
pid_q = q(time_start:time_stop);
pid_r = r(time_start:time_stop);
pid_p_c = p_c(time_start:time_stop);
pid_q_c = q_c(time_start:time_stop);
pid_r_c = r_c(time_start:time_stop);
load data/diff_flat_data2.mat
pn = pn(time_start:time_stop);
pe = pe(time_start:time_stop);
pd = pd(time_start:time_stop);
pn_c = pn_c(time_start:time_stop);
pe_c = pe_c(time_start:time_stop);
pd_c = pd_c(time_start:time_stop);
u = u(time_start:time_stop);
v = v(time_start:time_stop);
w = w(time_start:time_stop);
u_c = u_c(time_start:time_stop);
v_c = v_c(time_start:time_stop);
w_c = w_c(time_start:time_stop);
yaw_c = yaw_c(time_start:time_stop);
pitch_c = pitch_c(time_start:time_stop);
roll_c = roll_c(time_start:time_stop);
yaw = yaw(time_start:time_stop);
pitch = pitch(time_start:time_stop);
roll = roll(time_start:time_stop);
p = p(time_start:time_stop);
q = q(time_start:time_stop);
r = r(time_start:time_stop);
p_c = p_c(time_start:time_stop);
q_c = q_c(time_start:time_stop);
r_c = r_c(time_start:time_stop);

xmatrix = [pn_c;pn;pid_pn]';
ymatrix = [pe_c;pe;pid_pe]';
zmatrix = [pd_c;pd;pid_pd-1]';
createfigure(xmatrix,ymatrix,zmatrix)

for i = 1:length(pitch_c)
    if pitch_c(i) ==0
        pitch_c(i) = (pitch_c(i-1)+pitch_c(i+1))/2;
    end
end

t = time(time_start:time_stop);
pn_data = [pn_c;pn;pid_pn]';
pe_data = [pe_c;pe;pid_pe]';
pd_data = [pd_c;pd;pid_pd-1]';
u_data = [u_c;u;pid_u]';
v_data = [v_c;v;pid_v]';
w_data = [w_c;w;pid_w]';
roll_data = [roll_c;roll;pid_roll]';
pitch_data = [pitch_c;pitch;pid_pitch]';
r_data = [r_c;r;pid_r]';
createfigure1(t,pn_data,pe_data,pd_data,u_data,v_data,w_data,roll_data,pitch_data,r_data)

en = pn_c-pn;
ee = pe_c-pe;
ed = pd_c-pd;

figure(3);clf
hold on
plot (t,en,'r','LineWidth',2)
plot (t,ee,'b','LineWidth',2)
plot (t,ed,'k--','LineWidth',2)
legend3 = legend('Pn','Pe','Pd');
set(legend3,...
    'fontsize',12);
xlabel('time (s)','fontsize',12)
ylabel ('error (m)','fontsize',12)
title('Position Error Over Time','fontsize',16)

% figure(2);clf
% subplot(4,3,1)
% hold on
% plot(t,pn_c,'r','LineWidth',2)
% plot(t,pn,'b','LineWidth',2)
% plot(t,pid_pn,'k--','LineWidth',2)
% legend('Command','With Differential Flatness','PID Only')
% ylabel('Pn')
% 
% subplot(3,3,2)
% hold on
% plot(t,pe_c,'r','LineWidth',2)
% plot(t,pe,'b','LineWidth',2)
% plot(t,pid_pe,'k--','LineWidth',2)
% title('UAV States Over Time')
% ylabel('Pe')
% 
% subplot(3,3,3)
% hold on
% plot(t,pd_c,'r','LineWidth',2)
% plot(t,pd,'b','LineWidth',2)
% plot(t,pid_pd-1,'k--','LineWidth',2)
% ylabel('Pd')
% 
% subplot(3,3,4)
% hold on
% plot(t,u_c,'r','LineWidth',2)
% plot(t,u,'b','LineWidth',2)
% plot(t,pid_u,'k--','LineWidth',2)
% ylabel('u')
% 
% subplot(3,3,5)
% hold on
% plot(t,v_c,'r','LineWidth',2)
% plot(t,v,'b','LineWidth',2)
% plot(t,pid_v,'k--','LineWidth',2)
% ylabel('v')
% 
% subplot(3,3,6)
% hold on
% plot(t,w_c,'r','LineWidth',2)
% plot(t,w,'b','LineWidth',2)
% plot(t,pid_w,'k--','LineWidth',2)
% ylabel('w')
% 
% subplot(3,3,7)
% hold on
% plot(t,roll_c,'r','LineWidth',2)
% plot(t,roll,'b','LineWidth',2)
% plot(t,pid_roll,'k--','LineWidth',2)
% ylabel('roll')
% xlabel('t')
% 
% subplot(3,3,8)
% hold on
% plot(t,pitch_c,'r','LineWidth',2)
% plot(t,pitch,'b','LineWidth',2)
% plot(t,pid_pitch,'k--','LineWidth',2)
% ylabel('pitch')
% xlabel('t')
% 
% subplot(3,3,9)
% hold on
% plot(t,r_c,'r','LineWidth',2)
% plot(t,r,'b','LineWidth',2)
% plot(t,pid_r,'k--','LineWidth',2)
% ylabel('r')
% xlabel('t')