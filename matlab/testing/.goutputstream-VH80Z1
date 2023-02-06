
clear all;
close all;

%% %%%%%%% SuperDroid SD2 %%%%%%% %%
%% SD2: select rosbag file
bag = rosbag("rosbag\SD2_20220901_trial3_small.bag") ;  

%% SD2: retrieve data topic '/joy_raw'
SS_load_topic_joy_raw


%% SD2: retrieve data topic '/joy'
SS_load_topic_joy


%% SD2: plot linear velocity data
f = figure;

%%% shrink figure along vertical axis
%choose
factor_vertShrink = .8;

pos_temp = get(f,'Position') ;
set(f, 'Position',[pos_temp(1) pos_temp(2) pos_temp(3) factor_vertShrink*pos_temp(4)]); %shrink figure along the vertical axis 

%%% continue plotting
plot(time_joy_raw-time_joy_raw(1),data_v_joy_raw,'b-', 'linewidth',2, 'displayname','v^{des}'); hold on; grid on;
plot(time_joy-time_joy_raw(1),data_v_joy,'g--', 'linewidth',2, 'displayname','v^{r}'); 

xlabel('Time [sec]');
ylabel('Normalized values [-]');

axis tight
%{
%ver1: xlim
%xlim([min(time_joy_raw(1),time_joy(1)) max(time_joy_raw(end),time_joy(end))] - time_joy_raw(1) );
%}

%{-
%ver2: xlim
xlim_vals = [-.5 17.5]; 
xlim(xlim_vals);
%}

ylim(1.05*ylim);

legend('show', 'location','southwest');


%% SD2: plot angular velocity data
f = figure;

pos_temp = get(f,'Position') ;
set(f, 'Position',[pos_temp(1) pos_temp(2) pos_temp(3) factor_vertShrink*pos_temp(4)]); %shrink figure along the vertical axis 

%%% shrink figure along vertical axis
pos_temp = get(f,'Position') ;
set(f, 'Position',[pos_temp(1) pos_temp(2) pos_temp(3) factor_vertShrink*pos_temp(4)]); %shrink figure along the vertical axis 

%%% continue plotting
plot(time_joy_raw-time_joy_raw(1),data_omega_joy_raw,'b-', 'linewidth',2, 'displayname','\omega^{des}'); hold on; grid on;
plot(time_joy-time_joy_raw(1),data_omega_joy,'g--', 'linewidth',2, 'displayname','\omega^{r}'); 

xlabel('Time [sec]');
ylabel('Normalized values [-]');

axis tight
%{
%ver1: xlim
%xlim([min(time_joy_raw(1),time_joy(1)) max(time_joy_raw(end),time_joy(end))] - time_joy_raw(1) );
%}

%{-
%ver2: xlim
xlim(xlim_vals);
%}

ylim(1.05*ylim);

legend('show', 'location','southwest');


%% SD2: plot odometry data
bag_odom = select(bag,"Topic","/wheel_odom");
angleU0Lros = deg2rad(-64+90); %[rad]

SS_load_topic_odom

axis(1.1*axis); 
ylim([-.2 .2]);


%% %%%%%%% AgileX Scout Mini %%%%%%% %%
clear all;

%% AxM: select rosbag file
bag = rosbag("rosbag\AxM_2022-11-09-13-55-53.bag") ;  

%% AxM: retrieve data topic '/joy_raw'
SS_load_topic_joy_raw


%% AxM: retrieve data topic '/cmd_vel'
bag_cmd_vel = select(bag,"Topic","/cmd_vel") ;

time_cmd_vel = bag_cmd_vel.MessageList.Time ;

msgs_cmd_vel = readMessages(bag_cmd_vel,"DataFormat","struct") ;

% ini & malloc
data_v_cmd_vel     = nan(size(msgs_joy_raw,1),1);
data_omega_cmd_vel = data_v_cmd_vel;

for id__msgs_cmd_vel = 1:size(msgs_cmd_vel,1) 
    data_v_cmd_vel(id__msgs_cmd_vel)     = msgs_cmd_vel{id__msgs_cmd_vel,1}.Linear.X; %[m/s]
    data_omega_cmd_vel(id__msgs_cmd_vel) = msgs_cmd_vel{id__msgs_cmd_vel,1}.Angular.Z;
end %for id__msgs_cmd_vel =

%{
figure;
plot(time_cmd_vel-time_cmd_vel(1),data_v_cmd_vel,'b-', 'linewidth',2, 'displayname','v^{r}'); hold on; grid on;
plot(time_cmd_vel-time_cmd_vel(1),data_omega_cmd_vel,'g--', 'linewidth',2, 'displayname','\omega^{r}'); 

xlabel('Time [sec]');
ylabel('Normalized values [-]');
legend('show');
%}

%% AxM: post-process: normalize data_v_cmd_vel and data_omega_cmd_vel
data_v_cmd_vel = data_v_cmd_vel/1.5; %[-]

% identify scaling factor on omega_cmd_vel that ensures normalization 
t_instant_temp = 3.51; %choose
y_axis_val1 = interp1(time_joy_raw-time_joy_raw(1),data_omega_joy_raw, t_instant_temp, 'linear') ; 
y_axis_val2 = interp1(time_cmd_vel-time_joy_raw(1),data_omega_cmd_vel, t_instant_temp, 'linear') ; 
scale_factor_omega = y_axis_val2/y_axis_val1;

%conclude
data_omega_cmd_vel = data_omega_cmd_vel/scale_factor_omega; 


%% AxM: plot linear velocity data
f = figure;

%%% shrink figure along vertical axis
%choose
factor_vertShrink = .8;

pos_temp = get(f,'Position') ;
set(f, 'Position',[pos_temp(1) pos_temp(2) pos_temp(3) factor_vertShrink*pos_temp(4)]); %shrink figure along the vertical axis 

%%% continue plotting
plot(time_joy_raw-time_joy_raw(1),data_v_joy_raw,'b-', 'linewidth',2, 'displayname','v^{des}'); hold on; grid on;
plot(time_cmd_vel-time_joy_raw(1),data_v_cmd_vel,'g--', 'linewidth',2, 'displayname','v^{r}'); 

xlabel('Time [sec]');
ylabel('Normalized values [-]');

axis tight
%{
%ver1: xlim
%xlim([min(time_joy_raw(1),time_joy(1)) max(time_joy_raw(end),time_joy(end))] - time_joy_raw(1) );
%}

%{-
%ver2: xlim
xlim_vals = [-.5 9.5]; 
xlim(xlim_vals);
%}

ylim(1.05*ylim);

legend('show', 'location','northeast');


%% AxM: plot angular velocity data
f = figure;

pos_temp = get(f,'Position') ;
set(f, 'Position',[pos_temp(1) pos_temp(2) pos_temp(3) factor_vertShrink*pos_temp(4)]); %shrink figure along the vertical axis 

%%% shrink figure along vertical axis
pos_temp = get(f,'Position') ;
set(f, 'Position',[pos_temp(1) pos_temp(2) pos_temp(3) factor_vertShrink*pos_temp(4)]); %shrink figure along the vertical axis 

%%% continue plotting
plot(time_joy_raw-time_joy_raw(1),data_omega_joy_raw,'b-', 'linewidth',2, 'displayname','\omega^{des}'); hold on; grid on;
plot(time_cmd_vel-time_joy_raw(1),data_omega_cmd_vel,'g--', 'linewidth',2, 'displayname','\omega^{r}'); 

xlabel('Time [sec]');
ylabel('Normalized values [-]');

axis tight
%{
%ver1: xlim
%xlim([min(time_joy_raw(1),time_joy(1)) max(time_joy_raw(end),time_joy(end))] - time_joy_raw(1) );
%}

%{-
%ver2: xlim
xlim(xlim_vals);
%}

ylim(1.05*ylim);

legend('show', 'location','northeast');


%% AxM: plot odometry data
bag_odom = select(bag,"Topic","/odom");
angleU0Lros = deg2rad(137.5); %[rad]

SS_load_topic_odom

axis(1.1*axis); 
ylim([-.2 .2]);




