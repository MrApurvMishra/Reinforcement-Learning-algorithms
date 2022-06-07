% build_stochastic_mdp_nn: Function implementing the Nearest Neighbour
%                          approach for creating a stochastic MDP
%
% Inputs:
%       world:                  A structure containing basic parameters for
%                               the mountain car problem
%       T:                      Transition model with elements initialized
%                               to zero
%       R:                      Expected reward model with elements
%                               initialized to zero
%       num_samples:            Number of samples to use for creating the
%                               stochastic model
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

function [T, R] = build_stochastic_mdp_nn(world, T, R, num_samples)
    % Extract states and actions
    STATES = world.mdp.STATES;
    ACTIONS = world.mdp.ACTIONS;

    % Dimensions
    num_states = size(STATES, 2);
    num_actions = size(ACTIONS, 2);
    
    % random seed
    rng(42);

    % Loop through all possible states
    for state_index = 1:1:num_states
        cur_state = STATES(:, state_index);
        fprintf('building model... state %d\n', state_index);

        % Apply each possible action
        for action_index = 1:1:num_actions
            action = ACTIONS(:, action_index);

            % [TODO] Build a stochastic MDP based on Nearest Neighbour
            % Note: The function 'nearest_state_index_lookup' can be used
            % to find the nearest node to a countinuous state
            for samples = 1:1:num_samples
                [next_state,reward,~] = ...
                        world.one_step_model(world, cur_state, action);
                next_state(1) = next_state(1) + normrnd(0.0,0.001);
                next_state(2) = next_state(2) + normrnd(0.0,0.004);
                next_state_index = ...
                        nearest_state_index_lookup(STATES, next_state);

                % Update transition and reward models
                T{action_index}(state_index, next_state_index) = ...
                    T{action_index}(state_index, next_state_index) + ...
                    1/num_samples;
                R{action_index}(state_index, next_state_index) = reward;
            end
        end
    end
end

