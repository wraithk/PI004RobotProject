% Calibrate the scale parameter and wheel track of the robot
addpath("simulator/"); % Add the simulator to the MATLAB path.
pb = piBotSim("floor_spiral.jpg");

% Start by placing your robot at the start of the line
pb.place([2.5;2.5], 0.6421);

%pb = PiBot("192.168.50.1"); % Use this command instead if using PiBot.

% Create a window to visualise the robot camera
figure;
camAxes = axes();

% Follow the line in a loop
while true
    
    % First, get the current camera frame
    img = pb.getImage();
    imshow(img, "Parent", camAxes); % Check the video
    
    % Detect any visible landmarks and record their ids
    % Find the centre of the line to follow
    % Binarise the image by picking some sensible threshold
    % --- Find line centre using a bottom ROI, then compute a normalised error (-1..1)
    gray_img = rgb2gray(img);
    bin_img  = ~imbinarize(gray_img);  % assume dark line on light floor; use Otsu
    
    [H, W] = size(bin_img);
    roi = bin_img(round(0.7*H):H, :);      % bottom 30% of the image
    col_sum = sum(roi, 1);
    
    if any(col_sum)
        cols = find(col_sum > 0);
        line_centre_px = (min(cols) + max(cols)) / 2;    % in pixels
        % normalised lateral error: negative if line is left, positive if right
        err = (line_centre_px - (W+1)/2) / (W/2);        % range approx [-1, 1]
    else
        % no line seen in ROI – keep going straight (or slow/stop if you prefer)
        err = 0;
    end
    

    Kp_turn = -2;              
    q = Kp_turn * err;             
    
    u_base = 0.12;                 
    u_min  = 0.04;
    u = max(u_min, u_base * (1 - min(abs(err), 1)));
    

    [wl, wr] = inverse_kinematics(u, q);
    wmax = 10;                     % rad/s (example)
    wl = max(min(wl, wmax), -wmax);
    wr = max(min(wr, wmax), -wmax);
    
    pb.setVelocity(wl, wr);
    
    drawnow;
end


% Save the trajectory of the robot to a file.
% Don't use this if you are using PiBot.
pb.saveTrail();
