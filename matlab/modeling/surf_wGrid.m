function [] = surf_wGrid(N_dim2, N_dim1, dim2_range, dim1_range, surface, varargin)
%surface = surface(dim1,dim2)

surf(dim2_range, dim1_range, surface, varargin{1:end}); hold on;


%% plot curves on surface along one axis
grid__dim1_range = linspace(dim1_range(1),dim1_range(end),N_dim1);

for id___grid__dim1_range=1:length(grid__dim1_range)
    plot3_dim1 = grid__dim1_range(id___grid__dim1_range)*ones(length(dim2_range),1);
    plot3_dim2 = mcv(dim2_range);
    plot3_zaxis = interp2(dim2_range, dim1_range, surface,  plot3_dim2,plot3_dim1);

    plot3(plot3_dim2, plot3_dim1, plot3_zaxis,'k--', 'LineWidth',1);    
end %for id___grid__dim1_range=


%% plot curves on surface along the other axis
grid__dim2_range = linspace(dim2_range(1),dim2_range(end),N_dim2);

for id___grid__dim2_range=1:length(grid__dim2_range)
    plot3_dim1 = mcv(dim1_range);   
    plot3_dim2 = grid__dim2_range(id___grid__dim2_range)*ones(length(dim1_range),1); 
    plot3_zaxis = interp2(dim2_range, dim1_range, surface,  plot3_dim2,plot3_dim1);

    plot3(plot3_dim2, plot3_dim1, plot3_zaxis,'k--', 'LineWidth',1);    
end %for id___grid__dim1_range=

