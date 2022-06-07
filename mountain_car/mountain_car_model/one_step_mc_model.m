% one_step_mc_model: Function for simulating one step in the mountain car
%                    problem
%
% Inputs:
%       world:              A structure containing the MDP model of the 
%                           grid world
%       cur_state:          Current state (continuous)
%       action:             Action to be applied (continuous)
%
% Outputs:
%       next_state:         Next state (continuous)
%       reward:             Reward after transitioning to the next state
%       is_goal_state:      Flag indicating if reached the top of the hill
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

function [next_state,reward,is_goal_state] = one_step_mc_model(world, cur_state, action)
    % Extract bounds
    pos_bounds = world.param.pos_bounds;
    vel_bounds = world.param.vel_bounds;
    acc_bounds = world.param.acc_bounds;
    
    % Extract current state
    p0 = cur_state(1);
    v0 = cur_state(2);

    % Threshold position
    p0(p0 < pos_bounds(1)) = pos_bounds(1);
	p0(p0 > pos_bounds(2)) = pos_bounds(2);

    % Threshold velocity
    v0(v0 < vel_bounds(1)) = vel_bounds(1);
	v0(v0 > vel_bounds(2)) = vel_bounds(2);

    % Threshold action
    action(action < acc_bounds(1)) = acc_bounds(1);
	action(action > acc_bounds(2)) = acc_bounds(2);

    % Initialize goal reached flag
	is_goal_state = false;

    if p0 == 0.5
        % If car is already at goal state, remain there indefinitely
        p1 = 0.5;
        v1 = 0;
        reward = 0;        
    elseif p0 == -1.2
        % If car is at the other end, remain there indefinitely
        p1 = -1.2;
        v1 = 0;
        reward = -1;
    else
        % Otherwise ...
        % Propagate velocity forward
        v1 = v0 + 0.001*action - 0.0025*cos(3*p0);
        v1(v1 < vel_bounds(1)) = vel_bounds(1);
        v1(v1 > vel_bounds(2)) = vel_bounds(2);

        % Propagate position forward
        p1 = p0 + v1;
        p1(p1 < pos_bounds(1)) = pos_bounds(1);
        p1(p1 > pos_bounds(2)) = pos_bounds(2);
        
        % Update reward
        if p1 == 0.5
            % Check if goal state reached
            v1 = 0;
            reward = 10;
            is_goal_state = true;
        elseif p1 == -1.2
            % Check if the car is at the other end 
            v1 = 0;
            reward = -1;
        else
            % All other cases
            reward = -1;
        end
    end
    
    % Pack next state to a vector
    next_state = [p1; v1];
end

