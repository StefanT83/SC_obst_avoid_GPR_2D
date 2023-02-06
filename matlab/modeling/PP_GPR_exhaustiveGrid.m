%% GPR modeling: exhaustive grid approach
% Prerequirement: make sure to load the GPML library freely available on http://www.gaussianprocess.org/gpml/code/matlab/doc/ otherwise this code will not work: addpath(genpath('GPML_library'))

clear all
close all

%% load Experimental data
% import data = (v [m/sec], xobst [m], tstop [sec]) 
data = [ ...
    %VID_20220202_132631.mp4
0.05,.60, 11.999;          %00:38.231 to 00:50.230
0.05,.50,  9.832;          %00:40.398 to 00:50.230
0.05,.40,  8.132;          %00:42.098 to 00:50.230
0.05,.30,  6.099;          %00:44.131 to 00:50.230
0.05,.20,  4.299;          %00:45.931 to 00:50.230
...
0.05,.60, 11.499;          %01:17.794 to 01:29.293
0.05,.50,  9.733;          %01:19.560 to 01:29.293
0.05,.40,  7.833;          %01:21.460 to 01:29.293
0.05,.30,  5.900;          %01:23.393 to 01:29.293
0.05,.20,  3.867;          %01:25.426 to 01:29.293
...
0.05,.60, 11.732;          %01:48.258 to 01:59.990
0.05,.50,  9.899;          %01:50.091 to 01:59.990
0.05,.40,  8.033;          %01:51.957 to 01:59.990
0.05,.30,  6.133;          %01:53.857 to 01:59.990
0.05,.20,  4.033;          %01:55.957 to 01:59.990
...
0.05,.60, 12.099;          %02:16.488 to 02:28.587
0.05,.50, 10.299;          %02:18.288 to 02:28.587
0.05,.40,  8.399;          %02:20.188 to 02:28.587
0.05,.30,  6.499;          %02:22.088 to 02:28.587
0.05,.20,  4.533;          %02:24.054 to 02:28.587
...
0.05,.60, 11.866;          %03:22.848 to 03:34.714
0.05,.50, 10.033;          %03:24.681 to "
0.05,.40,  8.199;          %03:26.515 to "
0.05,.30,  6.266;          %03:28.448 to "
0.05,.20,  4.433;          %03:30.281 to "
...
0.05,.60, 11.798;          %03:54.179 to 04:05.977
0.05,.50,  9.799;          %03:56.178 to "
0.05,.40,  8.066;          %03:57.911 to "
0.05,.30,  6.099;          %03:59.878 to "
0.05,.20,  4.199;          %04:01.778 to "
...
%VID_20220202_133109.mp4
0.10,.60, 7.566;          %00:20.264 to 00:27.830
0.10,.50, 6.499;          %00:21.331 to "
0.10,.40, 5.499;          %00:22.331 to "
0.10,.30, 4.466;          %00:23.364 to "
0.10,.20, 3.333;          %00:24.497 to "
...
0.10,.60, 7.505;          %00:52.495 to 1:00.000
0.10,.50, 6.439;          %00:53.561 to "
0.10,.40, 5.472;          %00:54.528 to "
0.10,.30, 4.439;          %00:55.561 to "
0.10,.20, 3.406;          %00:56.594 to "
...
0.10,.60, 7.300;          %01:40.723 to 01:48.023
0.10,.50, 6.333;          %01:41.690 to "
0.10,.40, 5.300;          %01:42.723 to "
0.10,.30, 4.300;          %01:43.723 to "
0.10,.20, 3.233;          %01:44.790 to "
...
0.10,.60, 7.133;          %02:53.150 to 03:00.283
0.10,.50, 6.100;          %02:54.183 to "
0.10,.40, 5.133;          %02:55.150 to "
0.10,.30, 4.100;          %02:56.183 to "
0.10,.20, 3.100;          %02:57.183 to "
...
0.10,.60, 7.433;          %03:33.446 to 03:40.879
0.10,.50, 6.366;          %03:34.513 to "
0.10,.40, 5.366;          %03:35.513 to "
0.10,.30, 4.333;          %03:36.546 to "
0.10,.20, 3.333;          %03:37.546 to "
...
%VID_20220202_133603.mp4
0.15,.60, 5.732;          %01:06.460 to 01:12.192
0.15,.50, 5.033;          %01:07.159 to "
0.15,.40, 4.366;          %01:07.826 to "
0.15,.30, 3.733;          %01:08.459 to "
0.15,.20, 3.066;          %01:09.126 to "
...
0.15,.60, 5.600;          %01:39.889 to 01:45.489
0.15,.50, 4.833;          %01:40.656 to "
0.15,.40, 4.166;          %01:41.323 to "
0.15,.30, 3.533;          %01:41.956 to "
0.15,.20, 2.900;          %01:42.589 to "
...
0.15,.60, 5.466;          %02:07.320 to 02:12.786
0.15,.50, 4.799;          %02:07.987 to "
0.15,.40, 4.166;          %02:08.620 to "
0.15,.30, 3.566;          %02:09.220 to "
0.15,.20, 2.900;          %02:09.886 to "
...
0.15,.60, 5.7;            %02:24.618 to 02:30.318
0.15,.50, 4.9;            %02:25.418 to "
0.15,.40, 4.2;            %02:26.118 to "
0.15,.30, 3.567;          %02:26.751 to "
0.15,.20, 2.967;          %02:27.351 to "
...
0.15,.60, 5.466;          %02:51.182 to 02:56.648
0.15,.50, 4.766;          %02:51.882 to "
0.15,.40, 4.066;          %02:52.582 to "
0.15,.30, 3.433;          %02:53.215 to "
0.15,.20, 2.766;          %02:53.882 to "
...
%VID_20220202_133930
0.20,.60, 4.766;          %00:35.735 to 00:40.501
0.20,.50, 4.233;          %00:36.268 to "
0.20,.40, 3.799;          %00:36.702 to "
0.20,.30, 3.300;          %00:37.201 to "
0.20,.20, 2.833;          %00:37.668 to "
...
0.20,.60, 4.733;          %00:52.267 to 00:57.000
0.20,.50, 4.133;          %00:52.867 to "
0.20,.40, 3.700;          %00:53.300 to "
0.20,.30, 3.200;          %00:53.800 to "
0.20,.20, 2.733;          %00:54.267 to "
...
0.20,.60, 4.600;          %01:05.832 to 01:10.432
0.20,.50, 4.067;          %01:06.365 to "
0.20,.40, 3.600;          %01:06.832 to "
0.20,.30, 3.133;          %01:07.299 to "
0.20,.20, 2.600;          %01:07.832 to "
...
0.20,.60, 4.500;          %01:45.995 to 01:50.495
0.20,.50, 3.933;          %01:46.562 to "
0.20,.40, 3.467;          %01:47.028 to "
0.20,.30, 3.0;          %01:47.495 to "
0.20,.20, 2.5;          %01:47.995 to "
...
%VID_20220202_134151
0.25,.60, 4.299;          %01:09.893 to 01:14.192
0.25,.50, 3.833;          %01:10.359 to "
0.25,.40, 3.433;          %01:10.759 to "
0.25,.30, 3.033;          %01:11.159 to "
...
0.25,.60, 3.966;          %01:24.958 to 01:28.924
0.25,.50, 3.566;          %01:25.358 to "
0.25,.40, 3.166;         %01:25.758 to "
0.25,.30, 2.766;          %01:26.158 to "
...
0.25,.60, 4.133;          %01:38.023 to 01:42.156
0.25,.50, 3.700;          %01:38.456 to "
0.25,.40, 3.300;          %01:38.856 to "
0.25,.30, 2.866;          %01:39.290 to "
...
0.25,.60, 4.099;          %02:16.986 to 02:21.085
0.25,.50, 3.666;          %02:17.419 to "
0.25,.40, 3.266;          %02:17.819 to "
0.25,.30, 2.866;          %02:18.219 to "
...
0.25,.60, 4.000;          %02:32.584 to 02:36.584
0.25,.50, 3.633;          %02:32.951 to "
0.25,.40, 3.2;          %02:33.384 to "
0.25,.30, 2.8;          %02:33.784 to "
...
%VID_20220202_134527.mp4
0.30,.60, 3.967;          %00:27.953 to 0:31.920
0.30,.50, 3.567;          %00:28.353 to "
0.30,.40, 3.200;          %00:28.720 to "
0.30,.30, 2.867;          %00:29.053 to "
...
0.30,.60, 3.966;          %01:36.214 to 01:40.180
0.30,.50, 3.600;          %01:36.580 to "
0.30,.40, 3.266;          %01:36.914 to "
0.30,.30, 2.900;          %01:37.280 to "
...
0.30,.60, 3.900;          %01:53.145 to 01:57.045
0.30,.50, 3.500;          %01:53.545 to "
0.30,.40, 3.133;          %01:53.912 to "
0.30,.30, 2.766;          %01:54.279 to "
...
%VID_20220202_134758.mp4
0.35,.60, 3.9;          %00:07.232 to 00:11.132
0.35,.50, 3.567;          %00:07.565 to "
0.35,.40, 3.2;          %00:07.932 to "
0.35,.30, 2.9;          %00:08.232 to "
...
0.35,.60, 3.899;          %00:31.130 to 00:35.029
0.35,.50, 3.533;          %00:31.496 to "
0.35,.40, 3.233;          %00:31.796 to "
0.35,.30, 2.899;          %00:32.130 to "
...
0.35,.60, 4.167;          %01:08.559 to 01:12.726
0.35,.50, 3.833;          %01:08.893 to "
0.35,.40, 3.533;          %01:09.193 to "
0.35,.30, 3.2;          %01:09.526 to "
...
0.35,.60, 4.;          %01:43.989 to 01:47.989
0.35,.50, 3.667;          %01:44.322 to "
0.35,.40, 3.367;          %01:44.622 to "
0.35,.30, 3.067;          %01:44.922 to "
...
0.35,.60, 4.1;          %02:32.251 to 02:36.351
0.35,.50, 3.767;          %02:32.584 to "
0.35,.40, 3.433;          %02:32.918 to "
0.35,.30, 3.133;          %02:33.218 to "
...
%VID_20220202_135500
0.40,.60, 3.799;          %00:28.064 to 00:31.863
0.40,.50, 3.533;          %00:28.330 to "
0.40,.40, 3.266;          %00:28.597 to "
0.40,.30, 2.966;          %00:28.897 to "
...
0.40,.60, 3.833;          %00:44.495 to 00:48.328
0.40,.50, 3.533;          %00:44.795 to "
0.40,.40, 3.233;          %00:45.095 to "
0.40,.30, 2.933;          %00:45.395 to "
...
0.40,.60, 3.866;          %01:59.155 to 02:03.021
0.40,.50, 3.566;          %01:59.455 to "
0.40,.40, 3.266;          %01:59.755 to "
0.40,.30, 2.999;          %02:00.022 to "
];


%% plot surface: raw experimental data
figure;

for id_v = 1:size(data,1)
    plot3(data(id_v,2),data(id_v,1),data(id_v,3),'kx', 'LineWidth',2 , 'DisplayName','raw measurement data');
    hold on;
end

box on; grid on; 
xlabel('x^{obst} [m]');
ylabel('v [m/sec]');
zlabel('\Delta{}t_{stop} [sec]');

xlim([0 max(data(:,2))]);
ylim([0 max(data(:,1))]);
zlim([0 max(data(:,3))]);

title('Raw experimental data');
%view(2);
legend('experimental data', 'Location','best');


%% interface gp.m and minimize.m: exp data;  data = (v [m/sec], xobst [m], deltatstop [sec]) 
%exclude exp data corresp to v = 0.05; reason for exclusion: for visualisation purpose, it makes it hard to distinguish betw the different crosses corresp to the different data points   

idv = find(data(:,1)>.05+eps,1); %find 1st occurrence where v \neq 0.05  
data = data(idv:end,:); %overwrite

x  = data(:,1:2); %= (v [m/sec], xobst [m]);
y  = data(:,3); %= deltatstop [sec];

%def
v_range     = linspace(0.00,max(data(:,1)),5e2);       %0.04
xobst_range = linspace(0.00,max(data(:,2))+.05,5e2+1); %0.08 

%ini:malloc
xs = nan(length(v_range)*length(xobst_range),2) ; %= (v [m/sec], xobst [m]);

%define xs: compact
xs = [kron(mcv(v_range),mcv(ones(length(xobst_range),1)))    repmat(mcv(xobst_range),length(v_range),1) ]; %= (v [m/sec], xobst [m]);

%% main algorithm, largely inspired from GPML > demoRegression.m
meanfunc = [];                    % empty: don't use a mean function

covfunc = @covSEard;               %squared exponential covariance function with automatic relevance determination (ARD)
hyp_ini = struct('mean', [], 'cov', [0 0  0], 'lik', -1);

likfunc = @likGauss;              % Gaussian likelihood

%%%%% step1: compute/identify hyperparameters by optimizing the (log) marginal likelihood
hyp4 = minimize(hyp_ini, @gp, -100, @infGaussLik, meanfunc, covfunc, likfunc, x, y)

%%%%% step2: make predictions
[ymu ys2 fmu fs2] = gp(hyp4, @infGaussLik, meanfunc, covfunc, likfunc, x, y, xs); %note by ctdr: ymu and fmu are the same, which makes sense: the mean function is the same for y(x*)=def=f(x*)+nu i.e. y*=f*+nu; 


%% prepare surf: build the surface matrix  deltatstop_mat = deltatstop_mat(id_v_range,id_xobst_range)

%ini:malloc
deltatstop_mean_mat = nan(length(v_range),length(xobst_range)) ;
deltatstop_variance_mat = deltatstop_mean_mat;

%define deltatstop_mat: compact
sz = size(deltatstop_mean_mat) ;
deltatstop_mean_mat     = transpose(reshape(ymu,sz(end:-1:1))) ;
deltatstop_variance_mat = transpose(reshape(ys2,sz(end:-1:1))) ;


%% plot surface: mean function + intersection with horiz plane located at deltatstop_desired   
figure
%colormap('summer');

s1 = surf(xobst_range,v_range,deltatstop_mean_mat); hold on;
xlabel('x^{obst} [m]');
ylabel('v [m/sec]');

shading interp; %shading(gca,'interp')
set(s1,'edgecolor','none','meshstyle','both','linewidth',.15); %set(s1,'LineStyle','none');
%set(s1,'FaceAlpha',0.7); 

view(2);
axis tight;
colorbar

title('Mean function E[y*:=y(x*)]');

% plot the location of exp data points
plot3(x(:,2),x(:,1),max(deltatstop_mean_mat(:))*ones(size(x,1),1),'xk','markersize', 14, 'LineWidth',2, 'DisplayName','experimental data');


%% plot surface: variance
figure
%colormap('summer');

s1 = surf(xobst_range,v_range,deltatstop_variance_mat); hold on;
xlabel('x^{obst} [m]');
ylabel('v [m/sec]');

shading interp; %shading(gca,'interp')
set(s1,'edgecolor','none','meshstyle','both','linewidth',.15); %set(s1,'LineStyle','none');
%set(s1,'FaceAlpha',0.7); 

view(2);
axis tight;
colorbar

title('Var[y*:=y(x*)]');

%%% plot the location of exp data points
plot3(x(:,2),x(:,1),max(deltatstop_mean_mat(:))*ones(size(x,1),1),'xk','markersize', 14, 'LineWidth',2, 'DisplayName','experimental data');


%% plot surface: zoom on (obst_range,v_range) region figure; isocurves of the variance surface 
figure;
colormap('cool');

% define boundaries of the (obst_range,v_range) region
idx_xobst_range_start = find(xobst_range>=.298,1) ;
idx_xobst_range_end   = find(xobst_range>=min(.601,max(xobst_range(:))),1) ;

idx_v_range_start = find(v_range>=.175,1) ;
idx_v_range_end   = find(v_range>=.325,1) ;

[M,c] = contour3(xobst_range(idx_xobst_range_start:idx_xobst_range_end),v_range(idx_v_range_start:idx_v_range_end),deltatstop_variance_mat(idx_v_range_start:idx_v_range_end,idx_xobst_range_start:idx_xobst_range_end), 30); %'ShowText','on' 
hold on; 
c.LineWidth = 2;

xlabel('x^{obst} [m]');
ylabel('v [m/sec]');
colorbar

title('zoom Var[y*:=y(x*)]');

%%% plot the location of exp data points
plot3(x(:,2),x(:,1),max(deltatstop_mean_mat(:))*ones(size(x,1),1),'xk','markersize', 18, 'LineWidth',3, 'DisplayName','experimental data');

xlim([xobst_range(idx_xobst_range_start) xobst_range(idx_xobst_range_end)]);
ylim([v_range(idx_v_range_start) v_range(idx_v_range_end)]);

view(2);


%% plot surface: zoom on (obst_range,v_range) region figure; isocurves of the mean function surface 
figure;
colormap('cool');

[M,c] = contour3(xobst_range(idx_xobst_range_start:idx_xobst_range_end),v_range(idx_v_range_start:idx_v_range_end),deltatstop_mean_mat(idx_v_range_start:idx_v_range_end,idx_xobst_range_start:idx_xobst_range_end), 30); %'ShowText','on' 
hold on; 
c.LineWidth = 2;

xlabel('x^{obst} [m]');
ylabel('v [m/sec]');
colorbar

title('zoom Mean function E[y*:=y(x*)]');

%%% plot the location of exp data points
plot3(x(:,2),x(:,1),max(deltatstop_mean_mat(:))*ones(size(x,1),1),'xk','markersize', 18, 'LineWidth',3, 'DisplayName','experimental data');

xlim([xobst_range(idx_xobst_range_start) xobst_range(idx_xobst_range_end)]);
ylim([v_range(idx_v_range_start) v_range(idx_v_range_end)]);

view(2);


%% plot surfaces of the confidence interval: mean \pm factor*sigma

%choose
factor = 3 ;

%%%conseq: CI is used for plotting purpose
%ini
CI = nan; %by default; confidence interval, e.g. 95%, 99.7% etc.
switch factor
    case 2
        CI = 95;
    case 3
        CI = 99.7;
end
%otherwise keep default value for CI

mean_plus_factor_mul_stdDev = deltatstop_mean_mat + factor*sqrt(deltatstop_variance_mat); %along dim1 (y-axis) is v_range; along dim2 (x-axis) is xobst_range 
mean_minus_factor_mul_stdDev = deltatstop_mean_mat - factor*sqrt(deltatstop_variance_mat); %along dim1 (y-axis) is v_range; along dim2 (x-axis) is xobst_range 

figure;

surf_wGrid(4, 4, xobst_range,v_range,mean_plus_factor_mul_stdDev, 'LineStyle','none', 'FaceColor',[1 0 0],'FaceAlpha',0.5); hold on;
surf_wGrid(4, 4, xobst_range,v_range,mean_minus_factor_mul_stdDev, 'LineStyle','none', 'FaceColor',[0 1 0],'FaceAlpha',0.5);


%%% plot the projection of one point on the (v,xobst)-axes plane
%choose/select a point on the (v,xobst)-axes plane 
idx__xobst_range = find(xobst_range>=.234,1) ; %.434; .234
idx__v_range     = find(v_range>=.1,1) ; %.4; .1

%plot corresp point sitting on the surface mean_plus_factor_mul_stdDev 
plot3(xobst_range(idx__xobst_range),v_range(idx__v_range),mean_plus_factor_mul_stdDev(idx__v_range,idx__xobst_range),'ko', 'linewidth',2);

%plot corresp point sitting on the surface mean_minus_factor_mul_stdDev 
plot3(xobst_range(idx__xobst_range),v_range(idx__v_range),mean_minus_factor_mul_stdDev(idx__v_range,idx__xobst_range),'ko', 'linewidth',2);

%plot projection line 
plot3(xobst_range(idx__xobst_range)*ones(2,1), v_range(idx__v_range)*ones(2,1), [mean_plus_factor_mul_stdDev(idx__v_range,idx__xobst_range); 0],'k-.', 'linewidth',2);

axis([0.2000    0.6000    0.1000    0.4000    2.1000    7.7000]); %similar to fig exp data 
box on;
xlabel('x^{obst} [m]');
ylabel('v [m/sec]');
zlabel('\Delta{}t_{stop} [sec]');


%% plot 2D confidence interval and mean function used by shared control
figure;

%choose 
deltatstop_desired = 4 ; %[sec]

%%% plot confidence interval: mean + factor*sigma
pair_v_xobst__s_up = compute__pair_v_xobst(v_range,xobst_range, deltatstop_mean_mat + factor*sqrt(deltatstop_variance_mat),  deltatstop_desired);
%plot( pair_v_xobst__s_up(:,2), pair_v_xobst__s_up(:,1)); hold on;

%%% plot confidence interval: mean + factor*sigma
pair_v_xobst__s_down = compute__pair_v_xobst(v_range,xobst_range, deltatstop_mean_mat - factor*sqrt(deltatstop_variance_mat),  deltatstop_desired);
%plot( pair_v_xobst__s_down(:,2), pair_v_xobst__s_down(:,1)); hold on;

fill([pair_v_xobst__s_up(1,2); mcv(pair_v_xobst__s_up(:,2)); pair_v_xobst__s_down(end,2); flip(mcv(pair_v_xobst__s_down(:,2)),1)], ....
     [pair_v_xobst__s_down(1,1); mcv(pair_v_xobst__s_up(:,1)); pair_v_xobst__s_up(end,1); flip(mcv(pair_v_xobst__s_down(:,1)),1)], ...
     [7 7 7]/8, 'DisplayName', [num2str(CI,"%.1f"),'% confidence interval']); %'EdgeColor', [7 7 7]/8,
hold on; 

%%% plot experim data
plot3(x(:,2),x(:,1),max(deltatstop_mean_mat(:))*ones(size(x,1),1),'xk','markersize', 14, 'LineWidth',3, 'DisplayName','experimental data');

%%% plot mean
pair_v_xobst__mu = compute__pair_v_xobst(v_range,xobst_range, deltatstop_mean_mat,  deltatstop_desired);
plot(pair_v_xobst__mu(:,2), pair_v_xobst__mu(:,1), 'k-','markersize', 14, 'LineWidth',2, 'DisplayName','mean function'); hold on;

legend('show', 'Location','best');
grid on;
xlabel('x^{obst} [m]');
ylabel('v [m/sec]');
xlim([0 max(xobst_range)]);


