% visualize_mc_solution: Function plotting result for the mountain car
%                        problem
%
% Inputs:
%       world:              A structure containing the MDP model of the 
%                           grid world
%       state_stack:        State trajectory
%       input_stack:       Action trajectory
%                           state
%       plot_visualize:     Flag indicating if showing animation
%       save_dir:           Directory for saving plots
%
% Output:
%       hdl:                Plot handle
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

function [hdl] = visualize_mc_solution_mpc(world, state_stack, input_stack, plot_visualize, plot_title, save_dir)
    % Setup figure
    linecolor = [1, 1, 1].*0.5;
    fontcolor = [1, 1, 1].*0.5;
    fontsize = 12;

    % Plot state and action trajectory
    timesteps_plot = 1:1:size(state_stack,2);
    
    hdl(1) = figure;
	clf;
    subplot(3,1,1);
    plot(timesteps_plot, state_stack(1,:));
    axis([0, timesteps_plot(end), world.param.pos_bounds]);
    ylabel('Car Position');
    title(plot_title);
    
	subplot(3,1,2);
    plot(timesteps_plot, state_stack(2,:));
    axis([0, timesteps_plot(end), world.param.vel_bounds]);
    ylabel('Car Velocity');
    
	subplot(3,1,3);
    plot(timesteps_plot(1:end-1), input_stack(1,:));
    axis([0, timesteps_plot(end), world.param.acc_bounds]);
    xlabel('Discrete Time Index');
    ylabel('Acceleration Cmd');

	saveas(hdl(1), strcat(save_dir, ...
        sprintf('Trajectories - %s', plot_title)), 'png');

    % Visualize solution if plot_visualize is true
    if plot_visualize
        % Mountain curve
        xvals = linspace(world.param.pos_bounds(1), world.param.pos_bounds(2));
        yvals = get_car_height(xvals);
        height_stack = get_car_height(state_stack(1,:));
     
        % Visualize solution
        for k = 1:1:length(height_stack)
            hdl(2) = figure(2);
            clf;
            box on;
            plot(xvals, yvals, 'color', linecolor, 'linewidth', 1.5);
            hold on;            
            plot(state_stack(1,k), height_stack(k), 'ro', 'linewidth', 2);
            text(0.05, 0.95, sprintf('k = %d', k-1), 'units', ...
                    'normalized', 'horizontalalignment', 'left', ...
                    'color', fontcolor);
            if k <= length(input_stack)
                text(0.05, 0.05, sprintf('acc = %.2f', input_stack(k)), 'units', ...
                        'normalized', 'horizontalalignment', 'left', ...
                        'color', fontcolor);
            end
            axis([world.param.pos_bounds, min(yvals), max(yvals) + 0.1]);
            xlabel('x');
            ylabel('y');
            
            if k == 1
                % Wait to start
                text(0.5, 0.5, 'Press any key to start.', 'units', ...
                    'normalized', 'horizontalalignment', 'center', ...
                    'color', fontcolor, 'fontsize', fontsize);
                waitforbuttonpress;
            end
            pause(0.01);
        end
    end
end