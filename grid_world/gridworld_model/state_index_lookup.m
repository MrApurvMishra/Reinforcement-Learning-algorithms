% state_index_lookup: Function for finding index of discrete state
%
% Inputs:
%       STATES:  An array defining the discrete states
%       vector:  An array containing a set of state vectors for which the
%                indeces are to be found
%
% Outputs:
%       index:   An array of state indeces corresponding to that given by
%                the input argument 'vector'
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

function [index] = state_index_lookup(STATES, vector)
    % Dimensions
    state_dim = size(STATES, 1);
    num_vectors = size(vector, 2);
    
    % Find indeces
    index = [];
    for v = 1:1:num_vectors
        vector_temp = vector(:,v);
        temp = true;
        for l = 1:1:state_dim
            temp = temp & STATES(l,:) == vector_temp(l);
        end
        index = [index, find(temp)];
    end
end