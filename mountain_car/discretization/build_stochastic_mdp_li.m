% build_stochastic_mdp_li: Function implementing the Linear Interpolation
%                          approach for creating a stochastic MDP
%
% Inputs:
%       world:                  A structure containing basic parameters for
%                               the mountain car problem
%       T:                      Transition model with elements initialized
%                               to zero
%       R:                      Expected reward model with elements
%                               initialized to zero
%
% Outputs:
%       T:                      Transition model with elements T{a}(s,s')
%                               being the probability of transition to 
%                               state s' from state s taking action a
%       R:                      Expected reward model with elements 
%                               R{a}(s,s') being the expected reward on 
%                               transition from s to s' under action a
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

function [T, R] = build_stochastic_mdp_li(world, T, R)
	% Extract states and actions
    STATES = world.mdp.STATES;
    ACTIONS = world.mdp.ACTIONS;
    
    % Number of discrete states and actions
    num_states = size(STATES, 2);
    num_actions = size(ACTIONS, 2);
    
    % State space dimension
    dim_state = size(STATES, 1);

	% Unique values
    for i = 1:1:dim_state
        unique_states{i} = unique(STATES(i,:));
    end
        
    % Loop through all possible states
    for state_index = 1:1:num_states
        cur_state = STATES(:, state_index);
        fprintf('building model... state %d\n', state_index);

        % Apply each possible action
        for action_index = 1:1:num_actions
            action = ACTIONS(:, action_index);

            % Propagate forward
            [next_state, reward, ~] = world.one_step_model(world, ...
                cur_state, action);

            % Find four vertices enclosing next state index
            for i = 1:1:dim_state
                % find closest discretized values along state dimension i
                node_index_temp = knnsearch(unique_states{i}', ...
                                            next_state(i), 'K', 2);
                node_value_temp = unique_states{i}(node_index_temp);
                
                % for each state dimension i, store the min-max bounds
                box_min = min(node_value_temp);
                box_max = max(node_value_temp);
                node_value(i,1:2) = [box_min, box_max];
                
                % normalize next state values
                next_state_normalized(i,1) = ...
                    (next_state(i,1) - box_min) / (box_max - box_min);
            end
            
            % node values (for two-dim state space)
            node(1:2,1) = [node_value(1,1); node_value(2,1)]; % lower-left
            node(1:2,2) = [node_value(1,2); node_value(2,1)]; % lower-right
            node(1:2,3) = [node_value(1,2); node_value(2,2)]; % upper-right
            node(1:2,4) = [node_value(1,1); node_value(2,2)]; % upper-left
            
            % [TODO] Assign probability to adjacent nodes (bilinear)
            x = next_state_normalized(1);
            y = next_state_normalized(2);
            prob(1) = (1-x)*(1-y);  % min min
            prob(2) = x*(1-y);      % max min
            prob(3) = x*y;          % max max
            prob(4) = (1-x)*y;      % min max
            
            % Update probability and reward for each node
            for i = 1:1:4
                node_index = nearest_state_index_lookup(STATES, node(:,i));
                
                % Update transition and reward models
                T{action_index}(state_index, node_index) = prob(i);
                R{action_index}(state_index, node_index) = reward;
            end
            
        end
    end
end