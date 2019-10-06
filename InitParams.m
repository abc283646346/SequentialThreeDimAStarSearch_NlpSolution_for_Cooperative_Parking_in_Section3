global planning_scale_
planning_scale_.xmin = -20;
planning_scale_.xmax = 20;
planning_scale_.ymin = -20;
planning_scale_.ymax = 20;
planning_scale_.xhorizon = 40;
planning_scale_.yhorizon = 40;

global vehicle_geometrics_
vehicle_geometrics_.wheelbase = 2.8;
vehicle_geometrics_.front_hang = 0.96;
vehicle_geometrics_.rear_hang = 0.929;
vehicle_geometrics_.width = 1.942;
vehicle_geometrics_.length = vehicle_geometrics_.wheelbase + vehicle_geometrics_.front_hang + vehicle_geometrics_.rear_hang;
vehicle_geometrics_.radius1 = hypot(0.25 * vehicle_geometrics_.length, 0.5 * vehicle_geometrics_.width);
vehicle_geometrics_.radius2 = hypot(0.5 * vehicle_geometrics_.length, 0.5 * vehicle_geometrics_.width);
vehicle_geometrics_.radius = vehicle_geometrics_.radius2;
vehicle_geometrics_.r2x = 0.25 * vehicle_geometrics_.length - vehicle_geometrics_.rear_hang;
vehicle_geometrics_.f2x = 0.75 * vehicle_geometrics_.length - vehicle_geometrics_.rear_hang;

global vehicle_physics_
vehicle_physics_.v_max = 2.5;
vehicle_physics_.a_max = 0.5;
vehicle_physics_.phy_max = 0.7;
vehicle_physics_.w_max = 0.5;
vehicle_physics_.kappa_max = tan(vehicle_physics_.phy_max) / vehicle_geometrics_.wheelbase;
vehicle_physics_.turning_radius_min = vehicle_geometrics_.wheelbase / tan(vehicle_physics_.phy_max);

global three_dim_astar_
three_dim_astar_.resolution_x = 1;
three_dim_astar_.resolution_y = 1;
three_dim_astar_.num_nodes_x = ceil(planning_scale_.xhorizon / three_dim_astar_.resolution_x) + 1;
three_dim_astar_.num_nodes_y = ceil(planning_scale_.yhorizon / three_dim_astar_.resolution_x) + 1;
three_dim_astar_.multiplier_H = 3.0;
three_dim_astar_.max_iter = 1000;