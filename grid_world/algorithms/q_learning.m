% q_learning: Function solving the given MDP using the off-policy
%             Q-Learning method
%
% Inputs:
%       world:                  A structure defining the MDP to be solved
%       epsilon:                A parameter defining the 'sofeness' of the 
%                               epsilon-soft policy
%       k_epsilon:              The decay factor of epsilon per iteration
%       omega:                  Learning rate for updating Q
%       training_iterations:    Maximum number of training episodes
%       episode_length:         Maximum number of steps in each training
%                               episodes
%       noise_alpha:            A parameter that controls the noisiness of 
%                               observation (observation is noise-free when
%                               noise_alpha is set to 1 and is more 
%                               corrupted when it is set to values closer 
%                               to 0)
%
% Outputs:
%       Q:                      An array containing the action value for
%                               each state-action pair 
%       policy_index:           An array summarizing the index of the
%                               optimal action index at each state
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
% This script is adapted from the course on Optimal & Learning Control for
% Autonomous Robots at the Swiss Federal Institute of Technology in Zurich
% (ETH Zurich). Course Instructor: Jonas Buchli. Course Webpage:
% http://www.adrlab.org/doku.php/adrl:education:lecture:fs2015
%
% --
% Revision history
% [20.03.07, SZ]    first version

function [Q, policy_index] = q_learning(world, epsilon, k_epsilon, omega, training_iterations, episode_length, noise_alpha)
    %% Initialization
    % MDP
    mdp = world.mdp;
    gamma = mdp.gamma;

    % States
    STATES = mdp.STATES;
    ACTIONS = mdp.ACTIONS;

    % Dimensionts
    num_states = size(STATES, 2);
    num_actions = size(ACTIONS, 2);

    % Create object for incremental plotting of reward after each episode
    windowSize = 10; %Sets the width of the sliding window fitler used in plotting
    plotter = RewardPlotter(windowSize);

    % Initialize Q
    Q = zeros(num_states, num_actions);
    
    % [TODO] Initialize epsilon-soft policy
    actions_index = randi(num_actions, [num_states, 1]);
    policy = epsilon/num_actions * ones(num_states, num_actions);
    for s = 1:1:num_states
        selected_action = actions_index(s);
        policy(s, selected_action) = 1 - epsilon*(1-1/num_actions);
    end
    
    %% Q-Learning Algorthim (Section 2.9 of [1])
    for train_loop = 1:1:training_iterations
        %% [TODO] Generate a training episode

        total = 0;
        episode_index = 0;
        dataset = [];
        cur_state_index = 1;
        while episode_index < episode_length % episode termination criteria
            episode_index = episode_index + 1;
            
            % Sample current epsilon-soft policy
            u_sampled = rand(1);
            cur_policy = policy(cur_state_index,:);
            for a = 2:1:num_actions
                cur_policy(a) = sum(cur_policy(a-1:a));
            end
            action = 1;
            for a = 1:1:num_actions-1
                if u_sampled > cur_policy(a) && u_sampled <= cur_policy(a+1)
                    action = a + 1;
                end
            end
            
            % Interaction with environment
            % Note: 'next_state_noisy_index' below simulates state
            %       observarions corrupted with noise. Use this for
            %       Q-learning correspondingly for the last part of 
            %       Problem 2.2 (d)
            [next_state_index, next_state_noisy_index, reward] = ...
             one_step_gw_model(world, cur_state_index, action, noise_alpha);
            total = total + reward;

            % Log data for the episode
            dataset = [dataset; cur_state_index, action, reward];

            % Update Q(s,a)
            Q(cur_state_index,action) = Q(cur_state_index,action) + ...
                                        omega * (reward + gamma * Q...
                                        (next_state_noisy_index, ...
                                        actions_index(next_state_noisy_index))...
                                        - Q(cur_state_index,action));
            % terminate episode
            if reward == 0
                break;
            end
            
            % updating
            cur_state_index = next_state_noisy_index;
        end

        %% [TODO] Update policy(s,a)
        for x = 1:1:num_states
            cur_state_Q = Q(x,:);
            actions_index(x) = find(cur_state_Q == max(cur_state_Q),1);
        end
        
        policy = epsilon/num_actions * ones(num_states, num_actions);
        for s = 1:1:num_states
            selected_action = actions_index(s);
            policy(s, selected_action) = 1 - epsilon*(1-1/num_actions);
        end
        
        %% [TODO] Update the reward plot
        EpisodeTotalReturn = total; % Sum of the reward obtained during the episode
        plotter = UpdatePlot(plotter, EpisodeTotalReturn);
        drawnow;
        pause(0.05);

        %% Decrease the exploration
        %Set k_epsilon = 1 to maintain constant exploration
        epsilon = epsilon * k_epsilon; 

    end

    % Return deterministic policy for plotting
    [~, policy_index] = max(policy, [], 2);
end
