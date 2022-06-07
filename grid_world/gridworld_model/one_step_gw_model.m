% one_step_gw_model: Function for simulating one step in the grid world
% problem
%
% Inputs:
%       world:              A structure containing the MDP model of the 
%                           grid world
%       cur_state_index:    Index of the current state
%       action_index:       Index of action
%       noise_alpha:        A parameter that controls the noisiness of 
%                           observation (observation is noise-free when
%                           noise_alpha is set to 1 and is more corrupted
%                           when it is set to values closer to 0)
%
% Outputs:
%       next_state_index:   Index of the next state
%       next_state_noisy_index: Index of the next state with noise added
%       reward:             Reward after transitioning to the next state
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

function [next_state_index, next_state_noisy_index, reward] = ...
    one_step_gw_model(world, cur_state_index, action_index, noise_alpha)

    % Extract information
    STATES = world.mdp.STATES;
    s_start = world.user_defn.s_start;
    s_goal = world.user_defn.s_goal;
    s_obstables = world.user_defn.s_obstables;
    num_x_partitions = world.user_defn.num_row;
    num_y_partitions = world.user_defn.num_col;

    % Indeces for start and goal state
    s_start_index = state_index_lookup(STATES, s_start);
    s_goal_index = state_index_lookup(STATES, s_goal);

    % Indeces for obstacle
    s_obstacle_index = state_index_lookup(STATES, s_obstables);

    % Current state
    temp = STATES(:, cur_state_index);
    x = temp(1);
    y = temp(2);
    
    is_cur_obstacle = sum(cur_state_index == s_obstacle_index);
    if is_cur_obstacle
        % These states are never visited
        next_state = [x;y];
        next_state_index = state_index_lookup(STATES, next_state);
        next_state_noisy_index = next_state_index;
        reward = 0;
    else
        % Find next state in the grid world
        if noise_alpha ~= 1
            CDF = cumsum([noise_alpha, (1-noise_alpha)/2, (1-noise_alpha)/2]);
            bin_x_idx = find(rand(1,1) < CDF, 1, 'first');
            bin_y_idx = find(rand(1,1) < CDF, 1, 'first');
        else
            bin_x_idx = 1;
            bin_y_idx = 1;
        end
        if action_index == 1 % up
            % With noise
            noise_x = (bin_x_idx == 1)*1+(bin_x_idx == 2)*0+(bin_x_idx == 3)*(-1);
            noise_y = (bin_y_idx == 1)*0+(bin_y_idx == 2)*1+(bin_y_idx == 3)*(-1);
            
            % Noise-free
            delta_x = 1;
            delta_y = 0;
        elseif action_index == 2 % down
            % With noise
            noise_x = (bin_x_idx == 1)*(-1)+(bin_x_idx == 2)*0+(bin_x_idx == 3)*(1);
            noise_y = (bin_y_idx == 1)*0+(bin_y_idx == 2)*1+(bin_y_idx == 3)*(-1);
            
            % Noise-free
            delta_x = -1;
            delta_y = 0;
        elseif action_index == 3 % left
            % With noise
            noise_x = (bin_x_idx == 1)*(0)+(bin_x_idx == 2)*(-1)+(bin_x_idx == 3)*(1);
            noise_y = (bin_y_idx == 1)*(-1)+(bin_y_idx == 2)*1+(bin_y_idx == 3)*(0);
            
            % Noise-free
            delta_x = 0;
            delta_y = -1;
        elseif action_index == 4 % right
            % With noise
            noise_x = (bin_x_idx == 1)*(0)+(bin_x_idx == 2)*(-1)+(bin_x_idx == 3)*(1);
            noise_y = (bin_y_idx == 1)*(1)+(bin_y_idx == 2)*(-1)+(bin_y_idx == 3)*(0);
            
            % Noise-free
            delta_x = 0;
            delta_y = 1;
        end
        
        % Update state
        next_state = [x+delta_x;y+delta_y];
        next_state_noisy = [x+noise_x;y+noise_y];
        
        % Bound states
        next_state(1) = max(next_state(1), 1);
        next_state(1) = min(next_state(1), num_x_partitions);
        next_state(2) = max(next_state(2), 1);
        next_state(2) = min(next_state(2), num_y_partitions);

        % Bound noisy state
        next_state_noisy(1) = max(next_state_noisy(1), 1);
        next_state_noisy(1) = min(next_state_noisy(1), num_x_partitions);
        next_state_noisy(2) = max(next_state_noisy(2), 1);
        next_state_noisy(2) = min(next_state_noisy(2), num_y_partitions);
        
        % Index of next state
        next_state_index = state_index_lookup(STATES, next_state);
        next_state_noisy_index = state_index_lookup(STATES, next_state_noisy);

        % Is next state an obstacle? based on noise-free observation
        is_obstacle = sum(next_state_index == s_obstacle_index);

        % Is next state the goal state? based on noise-free observation
        is_goal = next_state_index == s_goal_index;

        % Reward
        if is_obstacle
            reward = -100;
            next_state_index = s_start_index;
            next_state_noisy_index = s_start_index;
        elseif is_goal
            reward = 0;
        else
            reward = -1;
        end
    end
end