function costmap = CreateDilatedCostmap(Nfe)
global planning_scale_
xmin = planning_scale_.xmin;
ymin = planning_scale_.ymin;
global three_dim_astar_
resolution_x = three_dim_astar_.resolution_x;
resolution_y = three_dim_astar_.resolution_y;
NumGridsX = three_dim_astar_.num_nodes_x;
NumGridsY = three_dim_astar_.num_nodes_y;
global vehicle_geometrics_
disc_radius = vehicle_geometrics_.radius;
costmap = cell(1,Nfe);
basic_dilation_pattern = strel('disk', ceil(disc_radius / resolution_x));

global obstacle_frame_x_ obstacle_frame_y_
Nobs = size(obstacle_frame_x_, 2);
for index = 1 : Nfe
    map_element = zeros(NumGridsX, NumGridsY);
    for ii = 1 : NumGridsX
        for jj = 1 : NumGridsY
            cur_x = xmin + (ii - 1) * resolution_x;
            cur_y = ymin + (jj - 1) * resolution_y;
            for kk = 1 : Nobs
                Vobs.x = [obstacle_frame_x_(index,kk,1), obstacle_frame_x_(index,kk,2), obstacle_frame_x_(index,kk,3), obstacle_frame_x_(index,kk,4), obstacle_frame_x_(index,kk,1)];
                Vobs.y = [obstacle_frame_y_(index,kk,1), obstacle_frame_y_(index,kk,2), obstacle_frame_y_(index,kk,3), obstacle_frame_y_(index,kk,4), obstacle_frame_y_(index,kk,1)];
               % Vobs.A = CalculatePolygonArea(Vobs);
                if (inpolygon(cur_x, cur_y, Vobs.x, Vobs.y) == 1)
                %if (IsPointCollidingWithPolygon([cur_x, cur_y], Vobs) == 1)
                    map_element(ii, jj) = 1;
                    break;
                end
            end
        end
    end
    map_element = imdilate(map_element, basic_dilation_pattern);
    costmap{index} = map_element;
end
end