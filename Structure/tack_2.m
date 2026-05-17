%% 3D Line Follower - V16 (Buttons + Robot-like Appearance)
%  Features:
%   - Exact track (2×80cm straights, 2×90° turns, 60cm link, 180° U-turn)
%   - 5 IR sensors with analog distances + noise
%   - Restart and Reverse buttons
%   - Robot model: blue body, red front nose, black wheels
%   - Stable PID with rate‑limited angular velocity
clear; clc; close all;

%% 1. Track Generation (returns track and initial pose for given direction)
function [track_x, track_y, track_z, init_pose, N, curvature, path_dist] = generate_track(reverse)
    res = 0.01;
    R90 = 0.20; R180 = 0.30;
    straight_len = 0.80; mid_straight = 0.60;
    
    track_x = []; track_y = [];
    
    % Segment 1: straight +X
    x1 = 0:res:straight_len; y1 = zeros(size(x1));
    track_x = [track_x, x1]; track_y = [track_y, y1];
    
    % Segment 2: 90° left turn (CCW)
    cx2 = straight_len; cy2 = R90;
    th2 = -pi/2:res/R90:0;
    x2 = cx2 + R90*cos(th2); y2 = cy2 + R90*sin(th2);
    track_x = [track_x, x2(2:end)]; track_y = [track_y, y2(2:end)];
    
    % Segment 3: straight +Y
    start3_x = track_x(end); start3_y = track_y(end);
    y3 = start3_y+res:res:start3_y+mid_straight;
    x3 = start3_x*ones(size(y3));
    track_x = [track_x, x3]; track_y = [track_y, y3];
    
    % Segment 4: 90° left turn (CCW)
    cx4 = start3_x - R90; cy4 = start3_y + mid_straight;
    th4 = 0:res/R90:pi/2;
    x4 = cx4 + R90*cos(th4); y4 = cy4 + R90*sin(th4);
    track_x = [track_x, x4(2:end)]; track_y = [track_y, y4(2:end)];
    
    % Segment 5: straight -X
    start5_x = track_x(end); start5_y = track_y(end);
    x5 = start5_x-res:-res:start5_x-straight_len;
    y5 = start5_y*ones(size(x5));
    track_x = [track_x, x5]; track_y = [track_y, y5];
    
    % Segment 6: 180° U-turn (clockwise)
    cx6 = track_x(end)-R180; cy6 = track_y(end)+R180;
    th6 = -pi:-res/R180:-pi/2;
    x6 = cx6 + R180*cos(th6); y6 = cy6 + R180*sin(th6);
    track_x = [track_x, x6(2:end)]; track_y = [track_y, y6(2:end)];
    
    % Closing straight to Y=0
    final_x = track_x(end); final_y = track_y(end);
    if abs(final_y-0) > 0.01
        y_close = final_y-res:-res:0;
        x_close = final_x*ones(size(y_close));
        track_x = [track_x, x_close]; track_y = [track_y, y_close];
    end
    track_z = zeros(size(track_x));
    N = length(track_x);
    
    % Precompute curvature and path distance
    curvature = zeros(1,N);
    win = 10;
    for i = win+1:N-win
        a1 = atan2(track_y(i)-track_y(i-win), track_x(i)-track_x(i-win));
        a2 = atan2(track_y(i+win)-track_y(i), track_x(i+win)-track_x(i));
        curvature(i) = abs(atan2(sin(a2-a1), cos(a2-a1))) / (win*res + 1e-9);
    end
    curvature = min(curvature, 6);
    path_dist = [0, cumsum(sqrt(diff(track_x).^2 + diff(track_y).^2))];
    
    % Initial pose (depending on direction)
    if ~reverse
        init_pose = [track_x(1); track_y(1); 0];
    else
        % Reverse: start at the end, heading opposite of final heading
        % Final heading is direction of last segment
        if N >= 2
            dx_final = track_x(N) - track_x(N-1);
            dy_final = track_y(N) - track_y(N-1);
            final_heading = atan2(dy_final, dx_final);
        else
            final_heading = 0;
        end
        init_pose = [track_x(N); track_y(N); final_heading + pi];
        % Reverse track arrays for the simulation to follow backwards
        track_x = flip(track_x);
        track_y = flip(track_y);
        track_z = zeros(size(track_x));
        curvature = flip(curvature);
        path_dist = cumsum([0, sqrt(diff(track_x).^2 + diff(track_y).^2)]);
    end
end

%% 2. Main simulation function (called by buttons)
function run_simulation(reverse_flag)
    % Generate track and initial pose
    [track_x, track_y, track_z, init_pose, N, curvature, path_dist] = generate_track(reverse_flag);
    
    %% Robot parameters
    bodyL = 0.20; bodyW = 0.12; bodyH = 0.04;
    pose = init_pose;
    
    dt = 0.05; render_every = 4; max_steps = 5000;
    
    % Sensors (tuned for full coverage)
    sensor_forward = 0.08;
    sensor_offsets = [-0.06, -0.03, 0, 0.03, 0.06];
    sensor_thresh = 0.045;
    sensor_noise_std = 0.002;
    
    % Speed and inertia
    v_cruise = 0.10; v_mid = 0.08; v_corner = 0.06; v_search = 0.04;
    v_current = v_cruise;
    max_accel = 0.8;
    
    % PID (stable)
    Kp = 0.4; Kd = 0.1; Ki = 0.0003;
    alpha = 0.30;
    smoothError = 0; prevSmooth = 0; integral = 0;
    max_w_follow = 1.2; max_ang_accel = 5.0; prev_w = 0;
    lookahead_dist = 0.12;
    
    %% Create figures
    fig1 = figure('Name','Black-Rose:Line Follower','Color','w','Position',[50,100,720,620]);
    plot3(track_x, track_y, track_z, 'k', 'LineWidth', 14); hold on;
    grid on; axis equal; view(30,45);
    
    % Robot chassis (main body)
    chassis = patch('FaceColor',[0.1 0.4 0.8],'EdgeColor','k','LineWidth',1.2);
    % Front nose (red triangle)
    nose = patch('FaceColor',[0.9 0.2 0.2],'EdgeColor','k','LineWidth',1);
    % Wheels (two black cylinders approximated by ellipses)
    left_wheel = patch('FaceColor',[0.1 0.1 0.1],'EdgeColor','k');
    right_wheel = patch('FaceColor',[0.1 0.1 0.1],'EdgeColor','k');
    
    % Sensor beams and points
    hBeams = gobjects(1,5);
    for s = 1:5
        hBeams(s) = plot3([0,0],[0,0],[0,0], 'b-', 'LineWidth', 1.5);
    end
    hSensors = scatter3(zeros(1,5),zeros(1,5),zeros(1,5),90,'filled');
    title('Black Rose:Line-Follower','FontSize',12);
    
    fig2 = figure('Name','Telemetry & Control','Color','w','Position',[790,100,780,680]);
    % Telemetry subplots
    ax1=subplot(3,2,1); hS=plot(0,0,'b','LineWidth',1.5); title('Speed (m/s)'); grid on; ylim([0 0.15]); xlabel('t (s)');
    ax2=subplot(3,2,2); hH=plot(0,0,'r','LineWidth',1.5); title('Heading (rad)'); grid on; ylim([-0.6 0.6]); xlabel('t (s)');
    ax3=subplot(3,2,3); hE=plot(0,0,'m','LineWidth',1.5); title('Filtered Error'); grid on; ylim([-0.4 0.4]); xlabel('t (s)');
    ax4=subplot(3,2,4); hD=plot(0,0,'g','LineWidth',1.5); title('Deviation (m)'); grid on; ylim([0 0.05]); xlabel('t (s)');
    % Sensor strip chart
    ax5=subplot(3,2,5:6); hold on;
    hBars = gobjects(1,5);
    for s = 1:5
        hBars(s) = bar(s, 0, 'FaceColor', [0.5 0.5 0.5], 'BarWidth', 0.6);
    end
    title('Sensor Readings (distance to line, m)'); xlabel('Sensor #'); ylabel('Distance (m)'); ylim([0 0.08]);
    set(gca, 'XTick', 1:5, 'XTickLabel', {'L2','L1','C','R1','R2'});
    grid on;
    
    % Buttons
    uicontrol('Style', 'pushbutton', 'String', 'Restart', ...
              'Position', [20 20 100 30], ...
              'Callback', @(~,~) restart_callback());
    uicontrol('Style', 'pushbutton', 'String', 'Reverse Direction', ...
              'Position', [140 20 150 30], ...
              'Callback', @(~,~) reverse_callback());
    
    % Callback functions
    function restart_callback()
        close(fig1); close(fig2);
        run_simulation(reverse_flag);
    end
    function reverse_callback()
        close(fig1); close(fig2);
        run_simulation(~reverse_flag);
    end
    
    %% Simulation loop
    t_hist = []; s_hist = []; h_hist = []; e_hist = []; d_hist = [];
    
    for t_idx = 1:max_steps
        % Nearest point on track
        dq = (track_x-pose(1)).^2 + (track_y-pose(2)).^2;
        [min_dev, m_idx] = min(sqrt(dq));
        la_end = min(m_idx+35, N);
        ahead_curv = max(curvature(m_idx:la_end));
        
        %% Sensor simulation (inline distance)
        sensor_states = false(1,5);
        sensor_distances = zeros(1,5);
        sg_x = zeros(1,5); sg_y = zeros(1,5);
        for s = 1:5
            sg_x(s) = pose(1) + sensor_forward*cos(pose(3)) - sensor_offsets(s)*sin(pose(3));
            sg_y(s) = pose(2) + sensor_forward*sin(pose(3)) + sensor_offsets(s)*cos(pose(3));
            px = sg_x(s); py = sg_y(s);
            min_d = inf;
            for k = 1:N-1
                x1 = track_x(k); y1 = track_y(k);
                x2 = track_x(k+1); y2 = track_y(k+1);
                dx = x2-x1; dy = y2-y1;
                if dx==0 && dy==0
                    d_seg = sqrt((px-x1)^2 + (py-y1)^2);
                else
                    t_param = ((px-x1)*dx + (py-y1)*dy) / (dx*dx + dy*dy);
                    t_param = max(0, min(1, t_param));
                    proj_x = x1 + t_param*dx;
                    proj_y = y1 + t_param*dy;
                    d_seg = sqrt((px-proj_x)^2 + (py-proj_y)^2);
                end
                if d_seg < min_d, min_d = d_seg; end
            end
            d_noisy = max(0, min_d + sensor_noise_std*randn());
            sensor_distances(s) = d_noisy;
            if d_noisy <= sensor_thresh, sensor_states(s) = true; end
        end
        active = find(sensor_states);
        
        %% Speed adaptation
        if isempty(active)
            v_target = v_search;
        elseif ahead_curv > 1.5 || any(sensor_states([1,5]))
            v_target = v_corner;
        elseif ahead_curv > 0.6
            v_target = v_mid;
        else
            v_target = v_cruise;
        end
        dv = max(min(v_target - v_current, max_accel*dt), -max_accel*dt);
        v_current = v_current + dv;
        
        %% Error calculation (confidence-weighted)
        if ~isempty(active)
            weights = [-2.5, -1.0, 0, 1.0, 2.5];
            confidence = max(0, 1 - sensor_distances / sensor_thresh);
            if sum(confidence) > 0, confidence = confidence / sum(confidence);
            else, confidence = zeros(1,5); end
            raw_error = sum(weights .* confidence);
            if abs(raw_error) < 0.02, raw_error = 0; end
        else
            target_dist = path_dist(m_idx) + lookahead_dist;
            if target_dist > path_dist(end), target_dist = path_dist(end); end
            [~, pp_idx] = min(abs(path_dist - target_dist));
            goal_x = track_x(pp_idx); goal_y = track_y(pp_idx);
            target_ang = atan2(goal_y-pose(2), goal_x-pose(1));
            raw_error = atan2(sin(target_ang-pose(3)), cos(target_ang-pose(3)));
            raw_error = max(min(raw_error, 1.0), -1.0);
        end
        
        smoothError = alpha*raw_error + (1-alpha)*smoothError;
        derivative = (smoothError - prevSmooth)/dt;
        derivative = max(min(derivative, 1.0), -1.0);
        integral = integral + smoothError*dt;
        integral = max(min(integral, 0.5), -0.5);
        
        w_raw = Kp*smoothError + Ki*integral + Kd*derivative;
        dyn_max_w = max_w_follow * max(0.5, 1 - min_dev*3);
        w_desired = max(min(w_raw, dyn_max_w), -dyn_max_w);
        w = prev_w + max(min(w_desired-prev_w, max_ang_accel*dt), -max_ang_accel*dt);
        prev_w = w; prevSmooth = smoothError;
        
        %% Physics
        pose(1) = pose(1) + v_current*cos(pose(3))*dt;
        pose(2) = pose(2) + v_current*sin(pose(3))*dt;
        pose(3) = pose(3) + w*dt;
        pose(3) = atan2(sin(pose(3)), cos(pose(3)));
        
        %% Logging
        t_hist(end+1) = t_idx*dt;
        s_hist(end+1) = v_current;
        h_hist(end+1) = pose(3);
        e_hist(end+1) = smoothError;
        d_hist(end+1) = min_dev;
        
        %% Rendering (every 4 steps)
        if mod(t_idx, render_every) == 0 && ishandle(fig1) && ishandle(fig2)
            % Update chassis and nose (robot shape)
            Rmat = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
            % Chassis vertices
            v_chassis = [-bodyL/2 -bodyW/2 0.01; bodyL/2 -bodyW/2 0.01;
                          bodyL/2  bodyW/2 0.01; -bodyL/2  bodyW/2 0.01;
                         -bodyL/2 -bodyW/2 bodyH; bodyL/2 -bodyW/2 bodyH;
                          bodyL/2  bodyW/2 bodyH; -bodyL/2  bodyW/2 bodyH];
            v_rot = (Rmat * v_chassis(:,1:2)')' + pose(1:2)';
            set(chassis, 'Vertices', [v_rot, v_chassis(:,3)], ...
                'Faces', [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8]);
            % Nose (front half of chassis, red)
            nose_pts = [bodyL/4 -bodyW/3 0.02; bodyL/2 -bodyW/4 0.02;
                        bodyL/2  bodyW/4 0.02; bodyL/4  bodyW/3 0.02];
            nose_rot = (Rmat * nose_pts(:,1:2)')' + pose(1:2)';
            set(nose, 'Vertices', [nose_rot, nose_pts(:,3)], ...
                'Faces', [1 2 3 4]);
            % Wheels (simple rectangles on sides)
            wheel_w = 0.04; wheel_h = 0.02;
            wheel_left_pts = [-bodyL/3 -bodyW/2-wheel_w 0.01; bodyL/3 -bodyW/2-wheel_w 0.01;
                              bodyL/3 -bodyW/2 0.01; -bodyL/3 -bodyW/2 0.01];
            wheel_right_pts = [-bodyL/3 bodyW/2+wheel_w 0.01; bodyL/3 bodyW/2+wheel_w 0.01;
                               bodyL/3 bodyW/2 0.01; -bodyL/3 bodyW/2 0.01];
            wl_rot = (Rmat * wheel_left_pts(:,1:2)')' + pose(1:2)';
            wr_rot = (Rmat * wheel_right_pts(:,1:2)')' + pose(1:2)';
            set(left_wheel, 'Vertices', [wl_rot, wheel_left_pts(:,3)], ...
                'Faces', [1 2 3 4]);
            set(right_wheel, 'Vertices', [wr_rot, wheel_right_pts(:,3)], ...
                'Faces', [1 2 3 4]);
            
            % Sensor beams
            robot_center = [pose(1), pose(2), 0.02];
            for s = 1:5
                tip = [sg_x(s), sg_y(s), 0.02];
                set(hBeams(s), 'XData', [robot_center(1), tip(1)], ...
                                'YData', [robot_center(2), tip(2)], ...
                                'ZData', [robot_center(3), tip(3)], ...
                                'Color', [0.3 0.3 0.8]);
            end
            % Sensor points
            cols = repmat([0.85 0.1 0.1], 5, 1);
            cols(sensor_states,:) = repmat([0.1 0.85 0.1], sum(sensor_states), 1);
            set(hSensors, 'XData', sg_x, 'YData', sg_y, ...
                'ZData', zeros(1,5)+0.02, 'CData', cols);
            
            % Telemetry plots
            set(hS, 'XData', t_hist, 'YData', s_hist);
            set(hH, 'XData', t_hist, 'YData', h_hist);
            set(hE, 'XData', t_hist, 'YData', e_hist);
            set(hD, 'XData', t_hist, 'YData', d_hist);
            xlim(ax1, [0 max(2, t_hist(end)+1)]);
            xlim(ax2, [0 max(2, t_hist(end)+1)]);
            xlim(ax3, [0 max(2, t_hist(end)+1)]);
            xlim(ax4, [0 max(2, t_hist(end)+1)]);
            
            % Sensor bar chart
            for s = 1:5
                set(hBars(s), 'YData', sensor_distances(s));
                if sensor_distances(s) <= sensor_thresh
                    set(hBars(s), 'FaceColor', [0.2 0.8 0.2]);
                else
                    set(hBars(s), 'FaceColor', [0.8 0.2 0.2]);
                end
            end
            drawnow;
            pause(dt * 0.6);
        end
        
        % Completion
        if m_idx >= N-10 && t_idx*dt > 2.0
            fprintf('Lap complete at t = %.1f s\n', t_idx*dt);
            break;
        end
        if t_idx*dt > 60, break; end
    end
    fprintf('Simulation finished | Mean dev: %.4f m | Max dev: %.4f m\n', mean(d_hist), max(d_hist));
end

%% 3. Start simulation (forward direction)
run_simulation(false);