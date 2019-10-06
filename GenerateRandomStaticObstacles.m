function my_cell = GenerateRandomStaticObstacles(Nobs)
global planning_scale_
my_cell = cell(1, Nobs);
counter = 0;
global fixed_num_vertexes
fixed_num_vertexes = 4;
while (counter < Nobs)
    geometric_center = [7.5+rand,2.45+rand];
    vertex_cell = GenerateConvexVertexes(geometric_center, fixed_num_vertexes);
    if (IsObstacleValid(vertex_cell))
        counter = counter + 1;
        my_cell{counter} = vertex_cell;
    end
end
end

function vertex_cell = GenerateConvexVertexes(geometric_center, num_vertexes)
x = geometric_center(1) + randn(1, num_vertexes) .* 1;
y = geometric_center(2) + randn(1, num_vertexes) .* 1;
[K, A] = convhull(x, y);
while (A > 3)
    x = geometric_center(1) + randn(1, num_vertexes) .* 1;
    y = geometric_center(2) + randn(1, num_vertexes) .* 1;
    [K, A] = convhull(x, y);
end
vertex_cell.x = x(K);
vertex_cell.y = y(K);
vertex_cell.A = A;
end

function is_valid = IsObstacleValid(V)
global fixed_num_vertexes
if (length(V.x) ~= fixed_num_vertexes + 1)
    is_valid = 0;
    return;
end

is_valid = 1;
global BV_
for index = 1 : length(BV_.x0)
    x0 = BV_.x0(index);
    y0 = BV_.y0(index);
    theta0 = BV_.theta0(index);
    xtf = BV_.xtf(index);
    ytf = BV_.ytf(index);
    thetatf = BV_.thetatf(index);
    if (IsVehiclePoseCollidingVertexes(x0,y0,theta0,V)||IsVehiclePoseCollidingVertexes(xtf,ytf,thetatf,V))
        is_valid = 0;
        return;
    end
end
end

function is_collided = IsVehiclePoseCollidingVertexes(x0, y0, theta0, V)
is_collided = 0;
global vehicle_geometrics_
cos_theta = cos(theta0);
sin_theta = sin(theta0);
lon_lb = -vehicle_geometrics_.rear_hang;
lon_ub = vehicle_geometrics_.front_hang + vehicle_geometrics_.wheelbase;
lat_lb = -vehicle_geometrics_.width * 0.5;
lat_ub = vehicle_geometrics_.width * 0.5;

for lon = [lon_lb : 0.1 : lon_ub, lon_ub]
    for lat = [lat_lb : 0.1 : lat_ub, lat_ub]
        x = x0 + lon * cos_theta + lat * sin_theta;
        y = y0 + lon * sin_theta - lat * cos_theta;
        is_collided = inpolygon(x,y,V.x,V.y);
        if (is_collided)
            break;
        end
    end
    if (is_collided)
        break;
    end
end
end