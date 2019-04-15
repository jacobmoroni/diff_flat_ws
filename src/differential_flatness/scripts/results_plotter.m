clc
clear

%% Load Data
time_start = 1500;
time_stop = 1800;

load data/lqr_no_rollin.mat
lqr_nr_pn = pn(time_start:time_stop);
lqr_nr_pe = pe(time_start:time_stop);
lqr_nr_pd = pd(time_start:time_stop);
lqr_nr_pn_c = pn_c(time_start:time_stop);
lqr_nr_pe_c = pe_c(time_start:time_stop);
lqr_nr_pd_c = pd_c(time_start:time_stop);
lqr_nr_u = u(time_start:time_stop);
lqr_nr_v = v(time_start:time_stop);
lqr_nr_w = w(time_start:time_stop);
lqr_nr_u_c = u_c(time_start:time_stop);
lqr_nr_v_c = v_c(time_start:time_stop);
lqr_nr_w_c = w_c(time_start:time_stop);
lqr_nr_yaw_c = yaw_c(time_start:time_stop);
lqr_nr_pitch_c = pitch_c(time_start:time_stop);
lqr_nr_roll_c = roll_c(time_start:time_stop);
lqr_nr_yaw = yaw(time_start:time_stop);
lqr_nr_pitch = pitch(time_start:time_stop);
lqr_nr_roll = roll(time_start:time_stop);
lqr_nr_p = p(time_start:time_stop);
lqr_nr_q = q(time_start:time_stop);
lqr_nr_r = r(time_start:time_stop);
lqr_nr_p_c = p_c(time_start:time_stop);
lqr_nr_q_c = q_c(time_start:time_stop);
lqr_nr_r_c = r_c(time_start:time_stop);

load data/ff_only.mat
ff_pn = pn(time_start:time_stop);
ff_pe = pe(time_start:time_stop);
ff_pd = pd(time_start:time_stop);
ff_pn_c = pn_c(time_start:time_stop);
ff_pe_c = pe_c(time_start:time_stop);
ff_pd_c = pd_c(time_start:time_stop);
ff_u = u(time_start:time_stop);
ff_v = v(time_start:time_stop);
ff_w = w(time_start:time_stop);
ff_u_c = u_c(time_start:time_stop);
ff_v_c = v_c(time_start:time_stop);
ff_w_c = w_c(time_start:time_stop);
ff_yaw_c = yaw_c(time_start:time_stop);
ff_pitch_c = pitch_c(time_start:time_stop);
ff_roll_c = roll_c(time_start:time_stop);
ff_yaw = yaw(time_start:time_stop);
ff_pitch = pitch(time_start:time_stop);
ff_roll = roll(time_start:time_stop);
ff_p = p(time_start:time_stop);
ff_q = q(time_start:time_stop);
ff_r = r(time_start:time_stop);
ff_p_c = p_c(time_start:time_stop);
ff_q_c = q_c(time_start:time_stop);
ff_r_c = r_c(time_start:time_stop);

load data/pid_ff.mat
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

load data/pid_only.mat
pid_only_pn = pn(time_start:time_stop);
pid_only_pe = pe(time_start:time_stop);
pid_only_pd = pd(time_start:time_stop);
pid_only_pn_c = pn_c(time_start:time_stop);
pid_only_pe_c = pe_c(time_start:time_stop);
pid_only_pd_c = pd_c(time_start:time_stop);
pid_only_u = u(time_start:time_stop);
pid_only_v = v(time_start:time_stop);
pid_only_w = w(time_start:time_stop);
pid_only_u_c = u_c(time_start:time_stop);
pid_only_v_c = v_c(time_start:time_stop);
pid_only_w_c = w_c(time_start:time_stop);
pid_only_yaw_c = yaw_c(time_start:time_stop);
pid_only_pitch_c = pitch_c(time_start:time_stop);
pid_only_roll_c = roll_c(time_start:time_stop);
pid_only_yaw = yaw(time_start:time_stop);
pid_only_pitch = pitch(time_start:time_stop);
pid_only_roll = roll(time_start:time_stop);
pid_only_p = p(time_start:time_stop);
pid_only_q = q(time_start:time_stop);
pid_only_r = r(time_start:time_stop);
pid_only_p_c = p_c(time_start:time_stop);
pid_only_q_c = q_c(time_start:time_stop);
pid_only_r_c = r_c(time_start:time_stop);

load data/lqr_ff_data.mat
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


%% Generate Trajectory figure
xmatrix = [pn_c;pid_pn;pn;pid_only_pn]';
ymatrix = [pe_c;pid_pe;pe;pid_only_pe]';
zmatrix = [pd_c;pid_pd;pd;pid_only_pd]';
createfigure(xmatrix,ymatrix,zmatrix)


%% Generate States Figure
t = time(time_start:time_stop);
pn_data = [pn;pid_pn;pid_only_pn]';
pe_data = [pe;pid_pe;pid_only_pe]';
pd_data = [pd;pid_pd;pid_only_pd]';
u_data = [u;pid_u;pid_only_u]';
v_data = [v;pid_v;pid_only_v]';
w_data = [w;pid_w;pid_only_w]';
roll_data = [roll;pid_roll;pid_only_roll]';
pitch_data = [pitch;pid_pitch;pid_only_pitch]';
r_data = [r;pid_r;pid_only_r]';
createfigure1(t,pn_data,pe_data,pd_data,u_data,v_data,w_data,roll_data,pitch_data,r_data)
%% States comparing control

figure(3);clf
subplot(3,1,1)
hold on
plot (t,pn,'r','LineWidth',2)
plot (t,pid_pn,'b','LineWidth',2)
plot (t,pid_only_pn,'k','LineWidth',2)
plot (t,ff_pn,'--m','LineWidth',2)

ylabel ('P_n (m)','fontsize',12)
title('Position Over Time','fontsize',16)
legend3 = legend('LQR w/FF','PID w/FF', 'PID only', 'Feed Forward only');
set(legend3,...
    'fontsize',12);

subplot(3,1,2)
hold on
plot (t,pe,'r','LineWidth',2)
plot (t,pid_pe,'b','LineWidth',2)
plot (t,pid_only_pe,'k','LineWidth',2)
plot (t,ff_pe,'--m','LineWidth',2)
ylabel ('P_e (m)','fontsize',12)

subplot(3,1,3)
hold on
plot (t,pd,'r','LineWidth',2)
plot (t,pid_pd,'b','LineWidth',2)
plot (t,pid_only_pd,'k','LineWidth',2)
plot (t,ff_pd,'--m','LineWidth',2)


xlabel('time (s)','fontsize',12)
ylabel ('P_d (m)','fontsize',12)

%% Error Between LQR and PID
en_lqr = pn_c-pn;
ee_lqr = pe_c-pe;
ed_lqr = pd_c-pd;

en_pid = pid_pn_c-pid_pn;
ee_pid = pid_pe_c-pid_pe;
ed_pid = pid_pd_c-pid_pd;

figure(4);clf
subplot(3,1,1)
hold on
plot (t,en_lqr,'r','LineWidth',2)
plot (t,en_pid,'b','LineWidth',2)
ylabel ('error north (m)','fontsize',12)
title('Position Error Over Time','fontsize',16)
legend3 = legend('LQR','PID');
set(legend3,...
    'fontsize',12);

subplot(3,1,2)
hold on
plot (t,ee_lqr,'r','LineWidth',2)
plot (t,ee_pid,'b','LineWidth',2)
ylabel ('error east (m)','fontsize',12)

subplot(3,1,3)
hold on
plot (t,ed_lqr,'r','LineWidth',2)
plot (t,ed_pid,'b','LineWidth',2)


xlabel('time (s)','fontsize',12)
ylabel ('error down (m)','fontsize',12)

%% Error Between Roll in and no roll in
% en_lqr = pn_c-pn;
% ee_lqr = pe_c-pe;
% ed_lqr = pd_c-pd;
% 
% en_lqr_nr = lqr_nr_pn_c-lqr_nr_pn;
% ee_lqr_nr = lqr_nr_pe_c-lqr_nr_pe;
% ed_lqr_nr = lqr_nr_pd_c-lqr_nr_pd;
% 
% figure(3);clf
% subplot(3,1,1)
% hold on
% plot (t,en_lqr,'r','LineWidth',2)
% plot (t,en_lqr_nr,'b','LineWidth',2)
% ylabel ('error north (m)','fontsize',12)
% title('Position Error Over Time','fontsize',16)
% legend3 = legend('With Roll In','No Roll In');
% set(legend3,...
%     'fontsize',12);
% 
% subplot(3,1,2)
% hold on
% plot (t,ee_lqr,'r','LineWidth',2)
% plot (t,ee_lqr_nr,'b','LineWidth',2)
% ylabel ('error east (m)','fontsize',12)
% 
% subplot(3,1,3)
% hold on
% plot (t,ed_lqr,'r','LineWidth',2)
% plot (t,ed_lqr_nr,'b','LineWidth',2)
% 
% 
% xlabel('time (s)','fontsize',12)
% ylabel ('error down (m)','fontsize',12)


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