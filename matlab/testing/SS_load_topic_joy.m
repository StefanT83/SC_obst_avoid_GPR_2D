%% retrieve data topic '/joy'

bag_joy = select(bag,"Topic","/joy");

time_joy = bag_joy.MessageList.Time ;

msgs_joy = readMessages(bag_joy,"DataFormat","struct") ;

% ini & malloc
data_v_joy     = nan(size(msgs_joy,1),1);
data_omega_joy = data_v_joy;

for id__msgs_joy = 1:size(msgs_joy,1) 
    data_v_joy(id__msgs_joy)     = msgs_joy{id__msgs_joy,1}.Axes(idx_matl_v_joy) ;
    data_omega_joy(id__msgs_joy) = msgs_joy{id__msgs_joy,1}.Axes(idx_matl_omega_joy) ;
end

%{
figure;
plot(time_joy-time_joy(1),data_v_joy,'b-', 'linewidth',2, 'displayname','v^{r}'); hold on; grid on;
plot(time_joy-time_joy(1),data_omega_joy,'g--', 'linewidth',2, 'displayname','\omega^{r}'); 

xlabel('Time [sec]');
ylabel('Normalized values [-]');
legend('show');
%}