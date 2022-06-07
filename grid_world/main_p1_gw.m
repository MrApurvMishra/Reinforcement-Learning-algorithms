% main_p1_gw: Main script for Problem 4.1 grid world
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
% [1] J. Buchli J., F. Farshidian, A. Winkler, T. Sandy, and M. Giftthaler,
%     "Optimal and Learning Control for Autonomous Robots," 2017.
%     Link: https://arxiv.org/pdf/1708.09342.pdf
%
% [2] P. Abbeel, "Lecture 2: Markov Decision Processes and Exact Solution
%     Methods," 2019. 
%     Link: https://people.eecs.berkeley.edu/~pabbeel/cs287-fa19/slides/Lec2-mdps-exact-methods.pdf
%
% [3] D.P. De Farias and B. Van Roy, "The linear programming approach to
%     approximate dynamic programming," 2003. 
%     Link: http://www.mit.edu/~pucci/discountedLP.pdf

clear all;
close all;
clc;

%% General
% Load world
load('./gridworld_model/grid_world');

% Add path
addpath(genpath(pwd));

% Result and plot directory
save_dir = './results/';
mkdir(save_dir);

%% Problem 4.1: (a) Generalized Policy Iteration (GPI)
% Instruction: Implement the GPI algorithm to solve the grid world problem
% Reference: Section 2.8 of [1]

% Parameters of the GPI algorithm
precision_pi = 0.1;
precision_pe = 0.01;
max_ite_pi = 100;
max_ite_pe = 100;

% ======================= [TODO] GPI Implementation - PI =======================
% Complete implementation in 'generalized_policy_iteration'
[v_gpi, policy_gpi] = generalized_policy_iteration_PI(world, precision_pi, ...
    precision_pe, max_ite_pi, max_ite_pe);
% =========================================================================

% Visualization
plt_title = 'Generalized Policy Iteration - PI';
plt_path = true;
plt_gpi = visualize_gw_solution(world, v_gpi, policy_gpi, ...
    plt_title, plt_path);

% Save results and figure to report
save(strcat(save_dir, 'gpi_results_PI.mat'), 'v_gpi', 'policy_gpi');
saveas(plt_gpi, strcat(save_dir, 'gpi_plot_PI.png'), 'png');

% ======================= [TODO] GPI Implementation - VI =======================
% Complete implementation in 'generalized_policy_iteration'
[v_gpi, policy_gpi] = generalized_policy_iteration_VI(world, precision_pi, ...
    precision_pe, max_ite_pi, max_ite_pe);
% =========================================================================

% Visualization
plt_title = 'Generalized Policy Iteration - VI';
plt_path = true;
plt_gpi = visualize_gw_solution(world, v_gpi, policy_gpi, ...
    plt_title, plt_path);

% Save results and figure to report
save(strcat(save_dir, 'gpi_results_VI.mat'), 'v_gpi', 'policy_gpi');
saveas(plt_gpi, strcat(save_dir, 'gpi_plot_VI.png'), 'png');

%% Problem 4.1 (b): Linear Programming (LP)
% % Extra credit: Solve the grid world problem with the LP approach
% % References: [2, 3]
% % Possible tools for solving LPs: 
% %   1) Matlab's linprog function: 
% %        https://www.mathworks.com/help/optim/ug/linprog.html
% %   2) cvx library (requires installation): http://cvxr.com/cvx/
% 
% % =================== [TODO] LP Approach Implementation ===================
% % Complete implementation in 'linear_programming'
% [v_lp, policy_lp] = linear_programming(world);
% =========================================================================
% 
% % Visualization
% plt_title = 'Linear Program';
% plt_path = true;
% plt_lp = visualize_gw_solution(world, v_lp, policy_lp, ...
%     plt_title, plt_path);
% 
% % Save results and figure to report
% save(strcat(save_dir, 'lp_results.mat'), 'v_lp', 'policy_lp');
% saveas(plt_lp, strcat(save_dir, 'lp_plot.png'), 'png');

%% Problem 4.1 (c): Monte Carlo
% Instruction: Implement the on-policy Monte Carlo control algorithm (with
% epsilon-soft policies) to solve the grid world algorithm
% Reference: Section 2.9 of [1]

% Algorithm parameters
epsilon = 0.2; % epsilon-soft policy parameter (see Eqn. (2.40) in [1])
k_epsilon = 1; % epsilon decay factor
omega = 0.1; % learning rate for updating Q
training_iterations = 500; % maximum number of training episodes
episode_length = 500; % length of each training episode

% =================== [TODO] Monte-Carlo Implementation ===================
% Complete implementation in 'monte_carlo'
[q_mc, policy_mc] = monte_carlo(world, epsilon, k_epsilon, omega, ...
    training_iterations, episode_length);
% =========================================================================

% Visualization
plt_title = 'Monte Carlo';
plt_path = true;
plt_mc = visualize_gw_solution(world, q_mc, policy_mc, ...
    plt_title, plt_path);

% Save results and figure to report
save(strcat(save_dir, 'mc_results.mat'), 'q_mc', 'policy_mc');
saveas(plt_mc, strcat(save_dir, 'mc_plot.png'), 'png');

%% Problem 4.1 (d): Q Learning
% Instruction: Implement the off-policy Q-learning algorithm (with
% epsilon-soft policies) to solve the grid world problem
% Reference: Section 2.10 of [1]

% Algorithm parameters
epsilon = 0.3; % epsilon-soft policy parameter (see Eqn. (2.40) in [1])
k_epsilon = 0.995; % epsilon decay factor
omega = 0.1; % learning rate for updating Q
training_iterations = 500; % maximum number of training episodes
episode_length = 500; % length of each training episode
noise_alpha = 0.8; % measurement noise for simulation

% =================== [TODO] Q-Learning Implementation ====================
% Complete implementation in 'q_learning'
[q_ql, policy_ql] = q_learning(world, epsilon, k_epsilon, omega, ...
    training_iterations, episode_length, noise_alpha);
% =========================================================================

% Visualization
plt_title = 'Q Learning';
plt_path = true;
plt_ql = visualize_gw_solution(world, q_ql, policy_ql, ...
    plt_title, plt_path);

% Save results and figure to report
save(strcat(save_dir, 'ql_results.mat'), 'q_ql', 'policy_ql');
saveas(plt_ql, strcat(save_dir, 'ql_plot.png'), 'png');
