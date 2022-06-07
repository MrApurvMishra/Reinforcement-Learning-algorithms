% visualize_mc_solution: Function plotting result for the mountain car
%                        problem
%
% Inputs:
%       world:              A structure containing the MDP model of the 
%                           grid world
%       value:              State or action value function
%       policy:             A vector containing optimized action at each 
%                           state
%       plot_value:         Flag indicating if showing value function plot
%       plot_flowfield:     Flag indicating if showing flowfield plot
%       plot_visualize:     Flag indicating if showing animation
%       plot_title:         A string defining title of plot
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

function [hdl] = visualize_mc_solution(world, value, policy, plot_value, plot_flowfield, plot_visualize, plot_title, save_dir)
    % plot saving directory
	if ~exist(save_dir, 'dir')
       mkdir(save_dir);
    end
    
    % simulation timesteps
    max_steps = world.param.max_steps;
    timesteps = 1:1:max_steps;
    
    % extract discrete states and actions used in training
    STATES = world.mdp.STATES;
    ACTIONS = world.mdp.ACTIONS;
    
    % intializing simulation
    cur_state = world.param.s_start;
    state_stack = cur_state;
    input_stack = [];
    height_stack = get_car_height(cur_state(1));
    timesteps_plot = 0;
    
    % simulate forward based on given policy
    for k = timesteps
        % current state index
        cur_state_index = nearest_state_index_lookup(STATES, cur_state);
        action_index = policy(cur_state_index);
        action = ACTIONS(action_index);
        
        % propagate forward
        [cur_state, ~, goal_reached] = ...
            one_step_mc_model(world, cur_state, action);
        
        % save stack
        timesteps_plot = [timesteps_plot, k];
        state_stack = [state_stack, cur_state];
        input_stack = [input_stack, action];
        height_stack = [height_stack, get_car_height(cur_state(1))];
        
        if goal_reached
            fprintf('Goal reached in %d steps!\n', k);
            break
        end
    end
    input_stack = [input_stack, 0];
    
    % plot state and action trajectory
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
    plot(timesteps_plot, input_stack(1,:));
    axis([0, timesteps_plot(end), world.param.acc_bounds]);
    xlabel('Discrete Time Index');
    ylabel('Acceleration Cmd');

	saveas(hdl(1), strcat(save_dir, ...
        sprintf('Trajectories - %s', plot_title)), 'png');

	% plot final value function if plot_value is true
    if plot_value
        % basics
        num_states = size(world.mdp.STATES, 2);
        num_actions = length(world.mdp.ACTIONS);
        
        % check if the value given is state value V or action value Q
        % if it is the latter, convert to V
        value_size = size(value);
        if sum(value_size) == num_states + num_actions
            % given action value function
            state_value = max(value, [], 2);
        elseif sum(value_size) == num_states + 1
            % given state value function
            state_value = value;
        end
    
        hdl(2) = figure;
        clf;
        surf(world.mdp.POS, world.mdp.VEL, ...
            reshape(state_value, world.param.num_vel, world.param.num_pos));
        view(0, 90);
        axis([world.param.pos_bounds, world.param.vel_bounds]);
        xlabel('Car Position');
        ylabel('Car Velocity');
        title(sprintf('State Value: %s', plot_title));

        saveas(hdl(2), strcat(save_dir, ...
            sprintf('State Values - %s', plot_title)), 'png');
    end
    
    % visualize flowfield
    if plot_flowfield
        grey = [1, 1, 1].*0.3;
        green = [126, 170, 85]./255;
        blue = [77, 115, 190]./255;
        hdl(3) = figure;
        clf;
        for state_index = 1:1:num_states
            cur_state = STATES(:, state_index);

            p0 = cur_state(1); 
            v0 = cur_state(2);
            action_index = policy(state_index);
            action = world.mdp.ACTIONS(action_index);

            v1 = v0 + 0.001*action - 0.0025*cos(3*p0);
            p1 = p0 + v1;
            u_values(1, state_index) = 0.001*action - 0.0025*cos(3*p0);
            v_values(1, state_index) = v1;
        end
        U = reshape(u_values, world.param.num_vel, world.param.num_pos);
        V = reshape(v_values, world.param.num_vel, world.param.num_pos);
        quiver(world.mdp.POS, world.mdp.VEL, V, U, 'color', grey, 'autoscalefactor', 0.5); hold on;
        plot(world.param.s_start(1), world.param.s_start(2), 'o', 'color', blue, 'linewidth', 2);
        plot(world.param.s_goal(1,:), world.param.s_goal(2,:), 'color', green, 'linewidth', 5);
        plot(state_stack(1,:), state_stack(2,:), '-', 'color', [blue, 0.5], 'linewidth', 1);
        
        axis([world.param.pos_bounds, world.param.vel_bounds]);
        xlabel('Car Position');
        ylabel('Car Velocity');
        title(sprintf('Flowfield: %s', plot_title));
        
        saveas(hdl(3), strcat(save_dir, ...
            sprintf('Flowfield - %s', plot_title)), 'png');
    end
    
    % visualize solution if plot_visualize is true
    if plot_visualize
        % mountain curve
        xvals = linspace(world.param.pos_bounds(1), world.param.pos_bounds(2));
        yvals = get_car_height(xvals);
        
        % setup figure
        hdl(4) = figure;
        linecolor = [1, 1, 1].*0.5;
        fontcolor = [1, 1, 1].*0.5;
        fontsize = 12;
     
        % visualize solution
        for k = 1:1:length(height_stack)
            hdl(3);
            clf;
            box on;
            plot(xvals, yvals, 'color', linecolor, 'linewidth', 1.5);
            hold on;            
            plot(state_stack(1,k), height_stack(k), 'ro', 'linewidth', 2);
            text(0.05, 0.95, sprintf('k = %d', k-1), 'units', ...
                    'normalized', 'horizontalalignment', 'left', ...
                    'color', fontcolor);
            text(0.05, 0.05, sprintf('acc = %.2f', input_stack(:,k)), 'units', ...
                    'normalized', 'horizontalalignment', 'left', ...
                    'color', fontcolor);
            axis([world.param.pos_bounds, min(yvals), max(yvals) + 0.1]);
            xlabel('x');
            ylabel('y');
            title(sprintf('Visualization: %s', plot_title));
            
            if k == 1
                % wait to start
                text(0.5, 0.5, 'Press any key to start.', 'units', ...
                    'normalized', 'horizontalalignment', 'center', ...
                    'color', fontcolor, 'fontsize', fontsize);
                waitforbuttonpress;
            end
            pause(0.01);
        end
    end
end