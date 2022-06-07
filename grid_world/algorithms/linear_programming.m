% linear_programming: Function solving the given MDP using the Linear
%                     Programming approach
%
% Inputs:
%       world:                  A structure defining the MDP to be solved
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
% --
% Revision history
% [20.03.07, SZ]    first version

function [V, policy] = linear_programming(world)
    %% Initialization
    % MDP
    mdp = world.mdp;
    T = mdp.T;
    R = mdp.R;
    gamma = mdp.gamma;
    
    % Dimensions
    num_actions = length(T);
    num_states = size(T{1}, 1);

    %% [TODO] Compute optimal value function (see [2] for reference)
    % V = ...;
    
    %% [TODO] Compute an optimal policy
    % policy = ...;
end
