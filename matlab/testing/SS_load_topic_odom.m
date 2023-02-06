
time_odom = bag_odom.MessageList.Time ;

msgs_odom = readMessages(bag_odom,"DataFormat","struct") ;

% ini & malloc
data_odom_xUrosLrobot = nan(size(msgs_odom,1),1);
data_odom_yUrosLrobot = data_odom_xUrosLrobot;

for id__msgs_odom = 1:size(msgs_odom,1) 
    data_odom_xUrosLrobot(id__msgs_odom) = msgs_odom{id__msgs_odom,1}.Pose.Pose.Position.X ;
    data_odom_yUrosLrobot(id__msgs_odom) = msgs_odom{id__msgs_odom,1}.Pose.Pose.Position.Y ;
end %for

%offset initial position to (0,0)
data_odom_xUrosLrobot = data_odom_xUrosLrobot - data_odom_xUrosLrobot(1);
data_odom_yUrosLrobot = data_odom_yUrosLrobot - data_odom_yUrosLrobot(1);

pUrosLrobot = [mrv(data_odom_xUrosLrobot); mrv(data_odom_yUrosLrobot)] ;

Rz = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)] ; %rotation around z-azis [B2005SpHuVi, p.50]

%ini & malloc
pU0Lrobot = nan(size(pUrosLrobot));

for id_pUrosLrobot = 1:size(pUrosLrobot,2)
    pU0Lrobot(:,id_pUrosLrobot) = Rz(angleU0Lros)*pUrosLrobot(:,id_pUrosLrobot); 
end %for


%{-
f = figure;
plot( pU0Lrobot(1,:),pU0Lrobot(2,:), 'linewidth',2 ); grid on;

xlabel('x_0 [m]');
ylabel('y_0 [m]');

axis equal;
%}