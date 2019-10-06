clear all
close all
clc

% Basic parameter setting
InitParams;
% Specify starting and goal configs
global Nv
Nv = 3;
global BV_
BV_.x0 = -15 + 30 * rand(1,Nv);
BV_.y0 = -15 + 30 * rand(1,Nv);
BV_.theta0 = 2 * pi * rand(1,Nv);
BV_.v0 = zeros(1,Nv);
BV_.a0 = zeros(1,Nv);
BV_.phy0 = zeros(1,Nv);
BV_.w0 = zeros(1,Nv);
BV_.xtf = -15 + 30 * rand(1,Nv);
BV_.ytf = -15 + 30 * rand(1,Nv);
BV_.thetatf = 2 * pi * rand(1,Nv);
BV_.vtf = zeros(1,Nv);
BV_.atf = zeros(1,Nv);
BV_.phytf = zeros(1,Nv);
BV_.wtf = zeros(1,Nv);

global num_static_obs num_dynamic_obs
num_static_obs = 1;
num_dynamic_obs = 1;
global Nfe
Nfe = 101;
global norm_tf
norm_tf = 40;
global obstacle_vertexes_
obstacle_vertexes_ = GenerateRandomStaticObstacles(num_static_obs);
global moving_obstacle_vertexes_
moving_obstacle_vertexes_ = GenerateRandomStaticObstacles(num_dynamic_obs);
global obstacle_frame_x_ obstacle_frame_y_
[obstacle_frame_x_, obstacle_frame_y_] = GenerateWholeObstacles(Nfe, obstacle_vertexes_, moving_obstacle_vertexes_);
global original_costmap_
original_costmap_ = CreateDilatedCostmap(Nfe);
WriteFilesForNLP(Nfe, num_static_obs, num_dynamic_obs);

% Step 1: Plan path for each vehicle to estimate the path length for ranking
path_length = zeros(1,Nv);
for ii = 1 : Nv
    [path_length(ii), ~] = PlanXYTAStarPathForSingleVehiclePreliminarily(ii);
end
% Step 2: Prepare Nrank ranking attempts according to Step 1 and random rankings.
global Nrank;
Nrank = 10;
ranklist = zeros(Nrank, Nv);
[~, ranklist(1,:)] = sort(path_length);
[~, ranklist(2,:)] = sort(path_length,'descend');
for ii = 3 : Nrank
    ranklist(ii,:) = randperm(Nv);
end
% Step 3: Evaluate each ranking attempt.
cost = zeros(1,Nrank);
for ranking_attempt = 1 : Nrank
    global costmap_
    costmap_ = original_costmap_;
    cost(ranking_attempt) = 0;
    for index = 1 : Nv
        iv = ranklist(ranking_attempt,index);
        cost(ranking_attempt) = cost(ranking_attempt) + PlanXYTAStarPathForRankingqualityEvaluation(iv);
    end
end
[~,ranking_attempts] = sort(cost);
selected_rank = ranklist(ranking_attempts(1),:);
% Step 4: Generate coarse path with specified ranking order for warm-starting
global costmap_
costmap_ = original_costmap_;
cost(ranking_attempt) = 0;
global coarse_x coarse_y
coarse_x = zeros(Nv,Nfe);
coarse_y = zeros(Nv,Nfe);
coarse_theta = zeros(Nv,Nfe);
for index = 1 : Nv
    iv = selected_rank(index);
    [x, y, theta] = PlanXYTAStarPath(iv);
    [coarse_x(iv,:), coarse_y(iv,:), coarse_theta(iv,:), ~] = ResampleConfig(x, y, theta, theta, Nfe);
end
WriteInitialGuessForNLP(coarse_x, coarse_y, coarse_theta);
% Step 5: NLP solution
!ampl r1.run
asd
% Generate dynamic solutions
% dsa