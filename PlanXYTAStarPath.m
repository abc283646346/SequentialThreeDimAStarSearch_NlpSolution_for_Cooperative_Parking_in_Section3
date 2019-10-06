function [x, y, theta] = PlanXYTAStarPath(iv)
global Nfe
global three_dim_astar_
global BV_
global planning_scale_
global costmap_

grid_space_ = cell(three_dim_astar_.num_nodes_x, three_dim_astar_.num_nodes_y, Nfe);
init_index = [ceil((BV_.x0(iv) - planning_scale_.xmin) / three_dim_astar_.resolution_x) + 1, ceil((BV_.y0(iv) - planning_scale_.ymin) / three_dim_astar_.resolution_y) + 1, 1];
global end_index
end_index = [ceil((BV_.xtf(iv) - planning_scale_.xmin) / three_dim_astar_.resolution_x) + 1, ceil((BV_.ytf(iv) - planning_scale_.ymin) / three_dim_astar_.resolution_y) + 1, Nfe];

init_node = zeros(1,11);
init_node(1:3) = init_index;
init_node(6) = CalculateH(init_node);
init_node(5) = 0;
init_node(4) = init_node(5) + init_node(6);
init_node(7) = 1;
init_node(9:11) = -999;
openlist_ = init_node;
grid_space_{init_node(1), init_node(2), init_node(3)} = init_node;
expansion_pattern = [1 1 1;
    1 0 1;
    1 -1 1;
    0 1 1;
    0 0 1;
    0 -1 1;
    -1 1 1;
    -1 0 1;
    -1 -1 1];
expansion_length = [1.414, 1, 1.414, 1, 0.5, 1, 1.414, 1, 1.414];
iter = 0;
complete_flag = 0;
best_ever_distance = Inf;

while ((~isempty(openlist_)) && (iter <= three_dim_astar_.max_iter) && (complete_flag == 0))
    iter = iter + 1;
    local_index = find(openlist_(:,4) == min(openlist_(:,4))); local_index = local_index(end);
    % Name it as cur_node and prepare for extension
    cur_node = openlist_(local_index, :);
    % Remove cur_node from openlist and add it in closed list
    openlist_(local_index, :) = [];
    grid_space_{cur_node(1), cur_node(2), cur_node(3)}(7) = 0;
    grid_space_{cur_node(1), cur_node(2), cur_node(3)}(8) = 1;
    
    for ii = 1 : size(expansion_pattern,1)
        child_node_trial = cur_node(1:3) + expansion_pattern(ii, :);
        if ((child_node_trial(1) > three_dim_astar_.num_nodes_x)||(child_node_trial(1) < 1)||(child_node_trial(2) > three_dim_astar_.num_nodes_y)||(child_node_trial(2) < 1)||(child_node_trial(3) > Nfe))
            continue;
        end
        child_node = zeros(1,11);
        child_node(1:3) = child_node_trial;
        child_node(9:11) = cur_node(1:3);
        if ((~isempty(grid_space_{child_node(1), child_node(2), child_node(3)}))...
                &&(grid_space_{child_node(1), child_node(2), child_node(3)}(8) == 1))
            continue;
        end
        if ((isempty(grid_space_{child_node(1), child_node(2), child_node(3)}))||((~isempty(grid_space_{child_node(1), child_node(2), child_node(3)}))&&(grid_space_{child_node(1), child_node(2), child_node(3)}(7) == 0)))
            % If the child node has never been explored
            if (costmap_{child_node(3)}(child_node(1), child_node(2)) == 0)
                if ((child_node(1) == end_index(1))&&(child_node(2) == end_index(2))&&(child_node(3) == end_index(3)))
                    complete_flag = 1;
                    best_ever_index = child_node;
                    grid_space_{child_node(1), child_node(2), child_node(3)} = child_node;
                    break;
                end
                child_node(5) = cur_node(5) + expansion_length(ii);
                child_node(6) = CalculateH(child_node);
                child_node(4) = child_node(5) + child_node(6);
                child_node(7) = 1;
                child_node(8) = 0;
                openlist_ = [openlist_; child_node];
            else
                child_node(7) = 0;
                child_node(8) = 1;
            end
            grid_space_{child_node(1), child_node(2), child_node(3)} = child_node;
            child_distance_to_goal = abs(end_index(1) - child_node(1)) + abs(end_index(2) - child_node(2));
            if (child_distance_to_goal < best_ever_distance)
                best_ever_distance = child_distance_to_goal;
                best_ever_index = child_node;
            end
        else
            % If the child node has already been in the openlist
            node_child_g_attempt = cur_node(5) + expansion_length(ii);
            if (grid_space_{child_node(1), child_node(2), child_node(3)}(5) > node_child_g_attempt)
                local_child_index = find(openlist_(:,4) == grid_space_{child_node(1), child_node(2), child_node(3)}(4));
                child_node(5) = node_child_g_attempt;
                child_node(6) = CalculateH(child_node);
                child_node(4) = child_node(6) + child_node(5);
                child_node(7) = 1;
                child_node(8) = 0;
                grid_space_{child_node(1), child_node(2), child_node(3)} = child_node;
                openlist_(local_child_index, :) = [];
                openlist_ = [openlist_; child_node];
            end
        end
    end
end

% Derive A* search result
cur_best_parent_index = grid_space_{best_ever_index(1), best_ever_index(2), best_ever_index(3)}(9:11);
final_path_x = [grid_space_{best_ever_index(1), best_ever_index(2), best_ever_index(3)}(1)];
final_path_y = [grid_space_{best_ever_index(1), best_ever_index(2), best_ever_index(3)}(2)];
final_path_time = [grid_space_{best_ever_index(1), best_ever_index(2), best_ever_index(3)}(3)];
while (cur_best_parent_index(1) ~= -999)
    cur_node = grid_space_{cur_best_parent_index(1), cur_best_parent_index(2), cur_best_parent_index(3)};
    cur_best_parent_index = cur_node(9:11);
    final_path_x = [cur_node(1), final_path_x];
    final_path_y = [cur_node(2), final_path_y];
    final_path_time = [cur_node(3), final_path_time];
end

x = (final_path_x - 1) * three_dim_astar_.resolution_x + planning_scale_.xmin;
y = (final_path_y - 1) * three_dim_astar_.resolution_y + planning_scale_.ymin;
theta = zeros(1, length(x));
for ii = 2 : length(x)
    dx = x(ii) - x(ii-1);
    dy = y(ii) - y(ii-1);
    theta(ii) = atan2(dy, dx);
end
end
