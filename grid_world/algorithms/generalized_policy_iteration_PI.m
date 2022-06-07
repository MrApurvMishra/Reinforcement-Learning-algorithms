% generalized_policy_iteration: Function solving the given MDP using the
%                               Generalized Policy Iteration algorithm
%
% Inputs:
%       world:                  A structure defining the MDP to be solved
%       precision_pi:           Maximum value function change before
%                               terminating Policy Improvement step
%       max_ite_pi:             Maximum number of iterations for Policy
%                               Improvement loop
%       precision_pe:           Maximum value function change before
%                               terminating Policy Evaluation step
%       max_ite_pe:             Maximum number of iterations for Policy
%                               Evaluation loop
%
% Outputs:
%       V:                      An array containing the value at each state
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

function [V, policy_index] = generalized_policy_iteration_PI(world, precision_pi, precision_pe, max_ite_pi, max_ite_pe)
    %% Initialization
    % MDP
    mdp = world.mdp;
    T = mdp.T;
    R = mdp.R;
    gamma = mdp.gamma;

    % Dimensions
    num_actions = length(T);
    num_states = size(T{1}, 1);

    % Intialize value function
    V = zeros(num_states, 1);

    % Initialize policy
    % Note: Policy here encodes the action to be executed at state s. We
    %       use deterministic policy here (e.g., [0,1,0,0] means take 
    %       action indexed 2)
    random_act_index = randi(num_actions, [num_states, 1]);
    policy = zeros(num_states, num_actions);
    for s = 1:1:num_states
        selected_action = random_act_index(s);
        policy(s, selected_action) = 1;
    end
    
    % repeat until max iterations reached for policy iterations
    iter = 0;
    act_indices = random_act_index;
    while iter < max_ite_pi
        %% POLICY ITERATION
        %[TODO] Policy Evaluation (PE)
        
        % repeat until max iterations reached
        i = 0;
        while i < max_ite_pe
            
            % initialize absolute difference between value functions
            delta_pe = 0;
            
            % calculate value function
            for x1 = 1:1:num_states
                v = V(x1);
                u = act_indices(x1);
                sum = 0;
                for x2 = 1:1:num_states
                    sum = sum + T{u}(x1,x2) * (R{u}(x1,x2) + gamma*V(x2));
                end
                
                % update value function
                V(x1) = sum;
                delta_pe = max(delta_pe, abs(v-V(x1)));
            end
            
            % break the loop if consecutive value difference too small
            if delta_pe < precision_pe
                break;
            end
            
            % increment iteration variable
            i = i + 1;            
        end
        
        %% [TODO] Policy Improvment (PI)
        
        % flag for stable policy check
        stable = 1;
        
        % calculate new policy
        for x1 = 1:1:num_states
            max_value = -inf;
            b = act_indices(x1);
            for u = 1:1:num_actions
                sum = 0;
                for x2 = 1:1:num_states
                    sum = sum + T{u}(x1,x2) * (R{u}(x1,x2) + gamma*V(x2));
                end
                
                % update policy by applying argmax logic
                if sum > max_value
                    max_value = sum;
                    act_indices(x1) = u;
                end
            end
            
            % check for convergence
            if b ~= act_indices(x1)
                stable = 0;
            end
        end
        
        % update policy
        policy = zeros(num_states, num_actions);
        for s = 1:1:num_states
            selected_action = act_indices(s);
            policy(s, selected_action) = 1;
        end
        
        % check algorithm convergence
        if stable == 1
            break;
        end
        
        % increment iteration variable
        iter = iter + 1;
    end
    
    % Return deterministic policy for plotting
    [~, policy_index] = max(policy, [], 2);
end