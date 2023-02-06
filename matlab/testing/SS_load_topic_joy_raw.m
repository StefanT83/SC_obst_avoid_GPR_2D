%% retrieve data topic '/joy_raw'

bag_joy_raw = select(bag,"Topic","/joy_raw");

time_joy_raw = bag_joy_raw.MessageList.Time ;

msgs_joy_raw = readMessages(bag_joy_raw,"DataFormat","struct") ;

% select accordingly the indices inside .Axes: see shared_control_realsense_node.cpp > void joypadSubscCallback(.)   
idx_matl_v_joy = 5;     %[-] \in [-1,1] the 5th element inside array axes represents advancing bwd/fwd user intention (max fwd = +1; max bwd = -1)
idx_matl_omega_joy = 4; %[-] \in [-1,1]  the 4th element inside array axes represents turning right/left user intention (max left = +1; max right=-1)

% ini & malloc
data_v_joy_raw     = nan(size(msgs_joy_raw,1),1);
data_omega_joy_raw = data_v_joy_raw;

for id__msgs_joy_raw = 1:size(msgs_joy_raw,1) 
    data_v_joy_raw(id__msgs_joy_raw)     = msgs_joy_raw{id__msgs_joy_raw,1}.Axes(idx_matl_v_joy) ;
    data_omega_joy_raw(id__msgs_joy_raw) = msgs_joy_raw{id__msgs_joy_raw,1}.Axes(idx_matl_omega_joy) ;
end

%{
figure;
plot(time_joy_raw-time_joy_raw(1),data_v_joy_raw,'b-', 'linewidth',2, 'displayname','v^{des}'); hold on; grid on;
plot(time_joy_raw-time_joy_raw(1),data_omega_joy_raw,'g--', 'linewidth',2, 'displayname','\omega^{des}'); 

xlabel('Time [sec]');
ylabel('Normalized values [-]');
legend('show');
%}