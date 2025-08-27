% Calibrate the scale parameter and wheel track of the robot
addpath("simulator/"); % Add the simulator to the MATLAB path.
% pb = piBotSim("floor_spiral.jpg");
% pb.place([2.5;2.5], 0.6421);
pb = PiBot('192.168.50.1'); % Use this command instead if using PiBot.

% --- Visualisation: camera view, landmark radar, and robot path
figure('Name','PiBot Camera, Landmark Radar & Path','NumberTitle','off');
tiledlayout(1,3,'Padding','compact','TileSpacing','compact');
camAxes = nexttile; title(camAxes,'Bottom-ROI binarised'); axis(camAxes,'ij');
landAxes = nexttile; hold(landAxes,'on'); grid(landAxes,'on'); axis(landAxes,'equal');
xlabel(landAxes,'x forward (m)'); ylabel(landAxes,'y left (m)'); title(landAxes,'Landmark Radar (Robot at [0,0])');
xlim(landAxes,[-0.5 1.5]); ylim(landAxes,[-1 1]); % tweak as needed
robotDot = plot(landAxes,0,0,'ko','MarkerFaceColor','k','MarkerSize',6);
landPts  = scatter(landAxes,nan,nan,'filled');
landTxt  = []; % handles to text labels
pathAxes = nexttile; hold(pathAxes,'on'); grid(pathAxes,'on'); axis(pathAxes,'equal');
xlabel(pathAxes,'x (m)'); ylabel(pathAxes,'y (m)'); title(pathAxes,'Robot Path & World Landmarks');
pathLine = plot(pathAxes,0,0,'b-','LineWidth',2);
currentPos = plot(pathAxes,0,0,'ro','MarkerFaceColor','r','MarkerSize',8);
worldLandmarks = scatter(pathAxes,nan,nan,100,'filled','MarkerFaceColor','g','MarkerEdgeColor','k');
worldLandmarkTxt = [];  % handles to world landmark text labels

% --- ArUco & camera params
addpath('arucoDetector/include');
addpath('arucoDetector');
addpath("arucoDetector/dictionary");
ArucoDict = load("arucoDict.mat");
addpath("CamCal");
CamParam = load("CamParam.mat");

% --- Line following control state
prev_err = 0; 
err_valid = false;

% --- Path tracking state (starting at origin in body-fixed frame)
robot_state = [0; 0; 0];  % [x; y; theta] starting at origin
path_x = 0;               % path history
path_y = 0;

% --- World-frame landmark tracking
world_landmarks = containers.Map('KeyType', 'int32', 'ValueType', 'any');  % ID -> [x, y, count]
landmark_merge_threshold = 0.15;  % merge landmarks within this distance (meters)
min_observations = 3;             % minimum observations before displaying landmark

% Controller gains
Kp_turn = -0.5;    % P gain
Kd_turn = -0.1;    % D gain (start ~20% of Kp)

u_base = 0.2;      % nominal forward speed
u_min  = 0.05;     % do not drop below this

% --- Robust dt estimation: tic/toc per loop + EMA smoothing & clamping
dt_ema = 0.1;      % initial guess
ema_alpha = 0.2;   % smoothing factor (0..1) – higher = quicker response
dt_min = 0.01;     % 100 Hz cap
dt_max = 0.2;      % 5 Hz floor

loop_tic = tic;
while true
    % Measure instantaneous dt and smooth it
    dt_inst = toc(loop_tic);
    if isfinite(dt_inst) && dt_inst > 0
        dt_inst = max(min(dt_inst, dt_max), dt_min);      % clamp outliers
        dt_ema  = ema_alpha*dt_inst + (1-ema_alpha)*dt_ema;
    end
    loop_tic = tic;  % reset timer for next loop
    
    % --- Get current camera frame
    img = pb.getImage();

    % --- Detect landmarks (IDs and their centres relative to camera/robot)
    % NOTE: detectArucoPoses returns [marker_nums, landmark_centres, marker_corners]
    % where landmark_centres is Nx3 in metres in the camera frame.
    [marker_nums, landmark_centres, marker_corners] = detectArucoPoses( ...
        img, 0.07, CamParam.cameraParams, ArucoDict.arucoDict);

    % --- Binarise and find line centre using bottom ROI
    gray_img = rgb2gray(img);
    bin_img  = ~imbinarize(gray_img);                % dark line on light floor
    [H, W]   = size(bin_img);
    roi      = bin_img(round(0.6*H):H, :);           % bottom 40%
    imshow(roi, "Parent", camAxes); title(camAxes,'Bottom-ROI binarised');

    [r,c] = find(roi==1);
    if isempty(c)
        % No line detected in ROI -> gentle search behaviour
        err = 0;                 % centre assumption
        err_derivative = 0;
        err_valid = false;
        q = 0.3;                 % slow, bias a gentle turn to reacquire line
        u = u_min;               % creep forward slowly while searching
    else
        med_c = median(c);
        err   = (med_c - (W)/2) / (W/2);    % approx [-1,1]
        if err_valid
            err_derivative = (err - prev_err) / dt_ema;
        else
            err_derivative = 0;  % first valid sample, no D term
            err_valid = true;
        end
        q = Kp_turn * err + Kd_turn * err_derivative;

        % speed scales down as |err| grows; never below u_min
        u = max(u_min, u_base * (1 - min(abs(err), 1)));
    end
    prev_err = err;

    % --- Convert (u, q) to wheel speeds and command robot
    [wl, wr] = inverse_kinematics(u, q);
    pb.setVelocity(wl, wr);
    
    % --- Update robot path using kinematics integration
    robot_state = integrate_kinematics(robot_state, dt_ema, u, q);
    path_x(end+1) = robot_state(1);
    path_y(end+1) = robot_state(2);
    
    % --- Transform landmarks from camera frame to world frame and update map
    if ~isempty(landmark_centres)
        for i = 1:size(landmark_centres, 1)
            % Camera frame landmark position (x forward, y left, z up)
            cam_x = landmark_centres(i, 1);
            cam_y = landmark_centres(i, 2);
            
            % Transform to world frame using current robot pose
            cos_theta = cos(robot_state(3));
            sin_theta = sin(robot_state(3));
            
            % Rotation matrix from camera frame to world frame
            world_x = robot_state(1) + cos_theta * cam_x - sin_theta * cam_y;
            world_y = robot_state(2) + sin_theta * cam_x + cos_theta * cam_y;
            
            % Update landmark map with deduplication
            landmark_id = marker_nums(i);
            if isKey(world_landmarks, landmark_id)
                % Existing landmark - update with exponential moving average
                current_data = world_landmarks(landmark_id);
                current_x = current_data(1);
                current_y = current_data(2);
                count = current_data(3) + 1;
                
                % Use exponential moving average for position update
                alpha = 0.3;  % smoothing factor
                new_x = alpha * world_x + (1 - alpha) * current_x;
                new_y = alpha * world_y + (1 - alpha) * current_y;
                
                world_landmarks(landmark_id) = [new_x, new_y, count];
            else
                % New landmark
                world_landmarks(landmark_id) = [world_x, world_y, 1];
            end
        end
    end

    % --- Update the landmark radar plot
    % Assumption: landmark_centres is in the robot/camera coordinate frame
    % with x forward, y left (if your detector returns a different convention,
    % swap/negate axes below accordingly).
    if ~isempty(landmark_centres)
        xs = landmark_centres(:,1);
        ys = landmark_centres(:,2);
        set(landPts,'XData',xs,'YData',ys);

        % Refresh text labels for IDs
        if ~isempty(landTxt); delete(landTxt); end
        landTxt = gobjects(0);
        for i = 1:numel(marker_nums)
            landTxt(end+1) = text(landAxes, xs(i), ys(i), sprintf('%d', marker_nums(i)), ...
                'HorizontalAlignment','center','VerticalAlignment','bottom','FontWeight','bold');
        end
    else
        set(landPts,'XData',nan,'YData',nan);
        if ~isempty(landTxt); delete(landTxt); landTxt = []; end
    end

    % Optional: show loop timing/FPS in the radar title
    title(landAxes, sprintf('Landmark Radar (dt=%.3f s, ~%.1f FPS)', dt_ema, 1/max(dt_ema,eps)));
    
    % --- Update robot path plot
    set(pathLine, 'XData', path_x, 'YData', path_y);
    set(currentPos, 'XData', robot_state(1), 'YData', robot_state(2));
    
    % --- Update world landmarks visualization
    landmark_ids = keys(world_landmarks);
    if ~isempty(landmark_ids)
        % Filter landmarks with sufficient observations
        valid_landmarks = [];
        valid_ids = [];
        for j = 1:length(landmark_ids)
            landmark_data = world_landmarks(landmark_ids{j});
            if landmark_data(3) >= min_observations  % count >= threshold
                valid_landmarks(end+1, :) = landmark_data(1:2);  % [x, y]
                valid_ids(end+1) = landmark_ids{j};
            end
        end
        
        if ~isempty(valid_landmarks)
            set(worldLandmarks, 'XData', valid_landmarks(:,1), 'YData', valid_landmarks(:,2));
            
            % Update text labels for world landmarks
            if ~isempty(worldLandmarkTxt); delete(worldLandmarkTxt); end
            worldLandmarkTxt = gobjects(0);
            for j = 1:length(valid_ids)
                worldLandmarkTxt(end+1) = text(pathAxes, valid_landmarks(j,1), valid_landmarks(j,2), ...
                    sprintf('%d', valid_ids(j)), 'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'bottom', 'FontWeight', 'bold', 'Color', 'white');
            end
        else
            set(worldLandmarks, 'XData', nan, 'YData', nan);
        end
    else
        set(worldLandmarks, 'XData', nan, 'YData', nan);
    end
    
    % Auto-scale path plot to show full trajectory and landmarks
    all_x = path_x;
    all_y = path_y;
    if ~isempty(valid_landmarks)
        all_x = [all_x, valid_landmarks(:,1)'];
        all_y = [all_y, valid_landmarks(:,2)'];
    end
    
    if length(all_x) > 1
        margin = 0.2;
        x_range = [min(all_x) - margin, max(all_x) + margin];
        y_range = [min(all_y) - margin, max(all_y) + margin];
        xlim(pathAxes, x_range);
        ylim(pathAxes, y_range);
    end

    drawnow();
end

% Save the trajectory of the robot to a file.
% Don't use this if you are using PiBot.
% pb.saveTrail();
