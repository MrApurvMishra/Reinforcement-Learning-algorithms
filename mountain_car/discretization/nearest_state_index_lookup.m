% nearest_state_index_lookup: Functiong for finding nearest neighbour 
%                             state index
%
% Inputs:
%       STATES:  An array defining the discrete states
%       vector:  An array containing a set of state vectors for which the
%                indices are to be found
%
% Outputs:
%       index:   An array of state indeces corresponding to elements in
%                STATES that are cloest to the input argument 'vector'
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

function [sp] = nearest_state_index_lookup(STATES, vector)
    % Nearest neighbour index corresponding to vector
    sp = knnsearch(STATES', vector');
    sp = sp';
end

