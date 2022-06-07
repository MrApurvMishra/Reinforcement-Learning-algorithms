% get_car_height: Function defining the curve that the mountain car drives
%                 on
%
% Input:
%       x:  car x position
%
% Output:
%       y:  car y position
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

function [y] = get_car_height(x)
    % compute y position
    y = sin(3 .* x) .* 0.45 + ones(size(x)) .* 0.55;
end