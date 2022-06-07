% visualize_gw_solution: Function plotting result for the grid world
%                        problem
%
% Inputs:
%       world:              A structure containing the MDP model of the 
%                           grid world
%       value:              State or action value function
%       policy:             A vector containing optimized action at each state
%       plot_title:         A string defining title of plot
%       plot_path:          Flag indicating whether plot the simulated path
%
% Outputs:
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

function [hdl] = visualize_gw_solution(world, value, policy, plot_title, plot_path)
    %% General Information
    % Grid world dimension
    num_row = world.user_defn.num_row;
    num_col = world.user_defn.num_col;

    % Dimensions
    num_states = size(world.mdp.STATES, 2);
    num_actions = length(world.mdp.ACTIONS);

    % Check if the value given is state value V or action value Q
    % if it is the latter, convert to V
    value_size = size(value);
	if sum(value_size) == num_states + num_actions
        % given action value function
        state_value = max(value, [], 2);
    elseif sum(value_size) == num_states + 1
        % given state value function
        state_value = value;
    end

    %% Plot Grid World
    % Set up figure
    hdl = figure;
    clf;
    hold on;
    xlims = [1-0.5, num_col+0.5];
    ylims = [1-0.5, num_row+0.5];
    
    colormap_rgb = colormap(gcf);
    cvalues = linspace(min(state_value), max(state_value), size(colormap_rgb,1));
    % Plot grid world
    for x = world.user_defn.x_boxes
        for y = world.user_defn.y_boxes
            cur_state = [x;y];
            cur_state_index = state_index_lookup(world.mdp.STATES, cur_state);
            
            is_obstacle = sum(cur_state_index == world.user_defn.s_obstacle_index);
            if is_obstacle
                % Plot black
                C(x,y,1:3) = [0,0,0];
            else
                % Plot heatmap
                color_index = knnsearch(cvalues', state_value(cur_state_index));
                C(x,y,1:3) = colormap_rgb(color_index, :);
           end
        end
    end
    axis([xlims, ylims]);
    axis equal

    imagesc([1,num_col], [1,num_row], C);

    %% Plot Path
    max_ite = 500;
    not_reached = 1;
    cur_state_index = world.mdp.s_start_index;
    ite = 0;
    while not_reached && ite <= max_ite
        ite = ite + 1;
        % Propagate forward
        action = policy(cur_state_index);
        [next_state_index, ~, ~] = one_step_gw_model(world, cur_state_index, action, 1);

        % Plot path
        cur_state = world.mdp.STATES(:, cur_state_index);
        next_state = world.mdp.STATES(:, next_state_index);
        temp = [cur_state,next_state];
        if plot_path
            plot(temp(2,:), temp(1,:), 'g:', 'linewidth', 2);
        end

        % Update state
        cur_state_index = next_state_index;
        if cur_state_index == world.mdp.s_goal_index
            not_reached = 0;
        end
    end

    % Print out if goal state reached and print the number of steps if yes
    if not_reached == 1
        fprintf('Goal state not reached!\n');
    else
        fprintf('Goal state reached in %d steps!\n', ite);
    end

    %% Overlay Policy Texts
    % Label goal state
    text(world.user_defn.s_goal(2), world.user_defn.s_goal(1), 'G', 'horizontalAlignment', 'center');
    
    % Edge and font colors
    edgecolor = [1,1,1,0.2];
    fontcolor = [1,1,1]*0.1;

    % Grid resolution
    x_grid = mean(diff(world.user_defn.x_boxes));
    y_grid = mean(diff(world.user_defn.y_boxes));
    
    % Loop through and plot edges
    plotlims = get(gca, 'position');
    plotarea_x = [plotlims(1), sum(plotlims([1,3]))];
    plotarea_y = [plotlims(2), sum(plotlims([2,4]))];
    gcaxlims = get(gca, 'XLim');
	gcaylims = get(gca, 'YLim');
    for x = world.user_defn.x_boxes
        for y = world.user_defn.y_boxes
            % Get current state
            cur_state = [x;y];
            cur_state_index = state_index_lookup(world.mdp.STATES, cur_state);

            % Is obstacle state?
            is_obstacle = sum(cur_state_index == world.user_defn.s_obstacle_index);

            % Is goal state?
            is_goal_state = sum(cur_state_index == world.user_defn.s_goal_index);

            % Print policy if this is a nominal state
            if ~is_obstacle && ~is_goal_state
                if policy(cur_state_index) == 1
                    policy_text = '\uparrow';
                elseif policy(cur_state_index) == 2
                    policy_text = '\downarrow';
                elseif policy(cur_state_index) == 3
                    policy_text = '\leftarrow';
                elseif policy(cur_state_index) == 4
                    policy_text = '\rightarrow';
                end
                text(y, x, policy_text, 'horizontalAlignment', 'center', 'color', fontcolor);
            end

            % Plot grid edge
            plot(ones(1,2).*y-0.5, ylims, '-', 'color', edgecolor);
            plot(ones(1,2).*y+0.5, ylims, '-', 'color', edgecolor);
        end

        % Plot grid edge
        plot(xlims, ones(1,2).*x-0.5, '-', 'color', edgecolor);
        plot(xlims, ones(1,2).*x+0.5, '-', 'color', edgecolor);
    end

    % Add plot title
	title(plot_title);
end