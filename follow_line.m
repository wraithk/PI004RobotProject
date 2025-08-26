% Calibrate the scale parameter and wheel track of the robot
addpath("simulator/"); % Add the simulator to the MATLAB path.
%pb = piBotSim("floor_spiral.jpg");

% Start by placing your robot at the start of the line
%pb.place([2.5;2.5], 0.6421);

pb = PiBot('192.168.50.1'); % Use this command instead if using PiBot.

% Create a window to visualise the robot camera
figure;
camAxes = axes();

% Follow the line in a loop
while true
    
    % First, get the current camera frame
    img = pb.getImage();
    
    
    % Detect any visible landmarks and record their ids
    % Find the centre of the line to follow
    % Binarise the image by picking some sensible threshold
    % --- Find line centre using a bottom ROI, then compute a normalised error (-1..1)
    gray_img = rgb2gray(img);
    bin_img  = ~imbinarize(gray_img);  % assume dark line on light floor; use Otsu
    % Check the video
    [H, W] = size(bin_img);
    roi = bin_img(round(0.6*H):H, :);      % bottom 30% of the image
    imshow(roi, "Parent", camAxes); 
    
    [r,c] = find(roi==1);
    centre_of_mass = [mean(c),mean(r)];
    
    err = (centre_of_mass(1) - (W)/2) / (W/2);        % range approx [-1, 1]
    Kp_turn = -0.5;              
    q = Kp_turn * err;             
    
    u_base = 0.2;                 
    u_min  = 0.05;
    u = max(u_min, u_base * (1 - min(abs(err), 1)));
    

    [wl, wr] = inverse_kinematics(u, q);
    
    pb.setVelocity(wl, wr);
    drawnow();
end


% Save the trajectory of the robot to a file.
% Don't use this if you are using PiBot.
%pb.saveTrail();
