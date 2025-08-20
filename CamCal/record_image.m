

addpath("C:\Users\seabr\Documents\RobotProject\PI004RobotProject\lab1_kinematics"); % 
pb = PiBot('192.168.50.1');

for i = 1:20
    input('');
    img = pb.getImage();
    imwrite(img, [num2str(i) '.png']); % save as PNG
end