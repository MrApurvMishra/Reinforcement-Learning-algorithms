% create_gridworld: Script for creating a grid world model
%
% --
% Control for Robotics
% AER1517 Spring 2022
% Assignment 4
%
% --
% University of Toronto Institute for Aerospace Studies
% Dynamic Systems Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistant: 
% SiQi Zhou
% siqi.zhou@robotics.utias.utoronto.ca
% Lukas Brunke
% lukas.brunke@robotics.utias.utoronto.ca
% Adam Hall
% adam.hall@robotics.utias.utoronto.ca
%
% --
% Revision history
% [20.03.07, SZ]    first version

clear all;
close all;
clc;

%% General
% Add path
addpath(genpath(pwd));

% Number of bins
num_x_partitions = 4; % num of rows [x is row index]
num_y_partitions = 12; % num of columns [y is column index]

% Start and goal state
s_start = [1;1];
s_goal = [1;12];

% Define obstacle states
s_obstables = [1,1,1,1,1,1,1,1,1,1;...
    2,3,4,5,6,7,8,9,10,11];

% Discounting rate
gamma = 0.99;

% Save struct
world.user_defn.num_row = num_x_partitions;
world.user_defn.num_col = num_y_partitions;
world.user_defn.s_start = s_start;
world.user_defn.s_goal = s_goal;
world.user_defn.s_obstables = s_obstables;

%% Create grid world
% Create x-y partitions
x_boxes = 1:1:num_x_partitions;
y_boxes = 1:1:num_y_partitions;

% Define possible states
[X,Y] = meshgrid(x_boxes, y_boxes);
STATES = [reshape(X,1,[]); reshape(Y,1,[])];
num_states = size(STATES,2);

% Define possible actions
num_actions = 4; % top, down, left, right
ACTIONS = 1:1:num_actions;

% Indeces for start and goal state
s_start_index = state_index_lookup(STATES, s_start);
s_goal_index = state_index_lookup(STATES, s_goal);

% Indeces for obstacle
s_obstacle_index = state_index_lookup(STATES, s_obstables);
    
% Save struct
world.user_defn.x_boxes = x_boxes;
world.user_defn.y_boxes = y_boxes;
world.user_defn.X = X;
world.user_defn.Y = Y;
world.user_defn.s_start_index = s_start_index;
world.user_defn.s_goal_index = s_goal_index;
world.user_defn.s_obstacle_index = s_obstacle_index;
world.mdp.STATES = STATES;
world.mdp.ACTIONS = ACTIONS;
world.mdp.s_start_index = s_start_index;
world.mdp.s_goal_index = s_goal_index;

%% Create MDP
% Initalize T{a}(s,sp) and  R{a}(s,sp)
for action_index = 1:1:num_actions
    T{action_index} = zeros(num_states, num_states);
    R{action_index} = zeros(num_states, num_states);
end

% Loop through all possible states
for x = 1:1:num_x_partitions
    for y = 1:1:num_y_partitions
        cur_state_index = state_index_lookup(STATES, [x;y]);
        for action_index = 1:1:num_actions 
            % a: 1 up, 2 down, 3 left, 4 right
            [next_state_index, ~, reward] = ...
                one_step_gw_model(world, cur_state_index, action_index, 1);

            % state transition
            T{action_index}(cur_state_index, next_state_index) = 1;
            R{action_index}(cur_state_index, next_state_index) = reward;
        end
    end
end

% Save struct
world.mdp.T = T;
world.mdp.R = R;
world.mdp.gamma = gamma;

%% Save grid world
save('./gridworld_model/grid_world', 'world');