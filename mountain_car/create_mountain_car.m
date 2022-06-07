% create_mountain_car: Main script for creating mountain car MDPs
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
%
% --
% Useful References:
% [4] P. Abbeel, "Lecture 3: Solving Continuous MDPs with Discretization," 
%     2019. Link: https://people.eecs.berkeley.edu/~pabbeel/cs287-fa19/slides/Lec3-discretization-of-continuous-state-space-MDPs.pdf

clear all;
close all;
clc;

%% General
% Add path
addpath(genpath(pwd));

% Bounds on states and action
pos_bounds = [-1.2, 0.5]; % state 1: position
vel_bounds = [-0.07, 0.07]; % state 2: velocity
acc_bounds = [-1, 1]; % action: acceleration

% Numbers of discrete values for states and action
num_pos = 20; % state 1: position
num_vel = 20; % state 2: velocity
num_acc = 5; % input: acceleration

% Partition
pos_values = linspace(pos_bounds(1), pos_bounds(2), num_pos);
vel_values = linspace(vel_bounds(1), vel_bounds(2), num_vel);
acc_values = linspace(acc_bounds(1), acc_bounds(2), num_acc);
 
% Define discrete states
[POS, VEL] = meshgrid(pos_values, vel_values);
POS_reshape = reshape(POS, 1, []); % row vector
VEL_reshape = reshape(VEL, 1, []); % row vector
STATES = [POS_reshape; VEL_reshape];
num_states = size(STATES, 2);

% Define discrete actions
ACTIONS = acc_values;
num_actions = size(ACTIONS, 2);

% Start and goal state
s_start = [-pi/6; 0];
s_goal = [ones(1,num_vel)*0.5; vel_values];

% Discounting rate
gamma = 0.95;

%% Model information
% Bounds on states and actions
world.param.pos_bounds = pos_bounds;
world.param.vel_bounds = vel_bounds;
world.param.acc_bounds = acc_bounds;

% Start and goal states and maximum time steps
world.param.s_start = s_start;
world.param.s_goal = s_goal;
world.param.max_steps = 500;

% Partitions for state and input
world.param.num_pos = num_pos;
world.param.num_vel = num_vel;
world.param.num_acc = num_acc;

% Discrete state and actions
world.mdp.STATES = STATES;
world.mdp.ACTIONS = ACTIONS;
world.mdp.POS = POS;
world.mdp.VEL = VEL;

% One-step model
world.one_step_model = @one_step_mc_model;

%% Initialization
% [TODO] Change 'mdp_approach' correspondingly to 'nearest_neighbour' for
% part (a) and 'linear_interp' for part (b)
mdp_approach = 'linear_interp'; % {nearest_neighbour, linear_interp}

% Initalize T{a}(s,sp) and R{a}(s,sp)
for action_index = 1:1:num_actions
    T{action_index} = zeros(num_states, num_states);
    R{action_index} = zeros(num_states, num_states);
end

%% Problem 4.2: (a)-(b)  Create stochastic MDPs for the mountain car problem
% Instruction: Implement the Nearest Neighbour and Linear Interpolation
%              approaches for creating discrete stochastic MDPs
% Reference: see [4] for linear_interp implementation

% ========================= [TODO] Discretization =========================
% Complete implementations in 'build_stochastic_mdp_nn' and 
% 'build_stochastic_mdp_li'
switch mdp_approach
    case 'nearest_neighbour'
        % for each state-action pair run generate multiple samples
        num_samples = 50;

        % build stochastic MDP based on nearest neighbour
        [T, R] = build_stochastic_mdp_nn(world, T, R, num_samples);
        
    case 'linear_interp'
        % build stochastic MDP based on linear interpolation
        [T, R] = build_stochastic_mdp_li(world, T, R);
end
% =========================================================================

% Save transition and reward models to struct
world.mdp.T = T;
world.mdp.R = R;
world.mdp.gamma = gamma;

%% Save grid world
switch mdp_approach
    case 'nearest_neighbour'
        save('./mountain_car_model/mountain_car_nn', 'world');
        
    case 'linear_interp'
        save('./mountain_car_model/mountain_car_li', 'world');
end