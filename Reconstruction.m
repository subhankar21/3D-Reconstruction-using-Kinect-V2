%SUBHANKAR CHATTORAJ -3DMT
clear all; close all; clc;
%%
%Checkboard Camera Calibration Parameters Extracted using Checkboard calibration technique
% Left RGB Intrinsic Camera Parameters
fx_RGB_left = 512.7662; 
fy_RGB_left = 520.6098;  
cx_RGB_left = 300.6089; 
cy_RGB_left = 223.8618; 
% Right RGB Intrinsic Camera Parameters
fx_RGB_right = 512.3432;  
fy_RGB_right = 509.5246; 
cx_RGB_right = 320; % 274.6973
cy_RGB_right = 240; %208.3558
%%
% The parameters obtained from constant Kinect Depth camera parameters
fx_DEPTH =  5.7616540758591043e+02;
fy_DEPTH = 5.7375619782082447e+02;
cx_DEPTH = 3.2442516903961865e+02;
cy_DEPTH = 2.3584766381177013e+02;

% Extrinsic parameters between RGB and Depth camera for Kinect V2
% Relative transform between the sensors (in meters)
% Rotation matrix
Rotation_Vec =  inv([  9.9998579449446667e-01, 3.4203777687649762e-03, -4.0880099301915437e-03;
    -3.4291385577729263e-03, 9.9999183503355726e-01, -2.1379604698021303e-03;
    4.0806639192662465e-03, 2.1519484514690057e-03,  9.9998935859330040e-01]);

% Translation vector.
Translation_Vec = -[  2.2142187053089738e-02, -1.4391632009665779e-04, -7.9356552371601212e-03 ]';
%% Reading the both RGB and the corresponding depth image for calibration 
% RGB and Depth left image
left_RGB_img = imread('LeftRgbobjects_scene21.png');
left_depth_img = imread('LeftDepthobjects_scene21.png');
%% RGB and Depth right image
right_RGB_img = imread('RightRgbobjects_scene21.png');
right_depth_img = imread('RightDedpthobjects_scene21.png');
%% Ploting the image for visulization
% Left RBG image
figure();
imshow(left_RGB_img );set(gca,'FontSize',13)
title('Left RGB Image')
% Left depth image
figure();
imshow(left_depth_img,[0.8,3.0],'Colormap', parula(255));set(gca,'FontSize',13)
title('Left Depth Image')
% Right RBG image
figure();
imshow(right_RGB_img); set(gca,'FontSize',20); set(gca,'FontSize',13)
title('Right RGB Image')
% Right depth image
figure();
imshow(right_depth_img,[0.8,3.0],'Colormap', parula(255)); set(gca,'FontSize',13)
title('Right Depth Image')
%% All the 2D point from depth image back-projecting it into 3D space
% 2-D ponts of Left Camera:
[n_row, n_col] = size(left_depth_img);
left_depth_mapping = zeros(n_row, n_col, 3);  
for x = 1:n_row
    for y = 1:n_col
        % 2D point from depth image back-projecting it into 3D space
        left_depth_mapping(x, y, 1) = (y - cx_DEPTH) * double(left_depth_img(x, y)) / fx_DEPTH;
        left_depth_mapping(x, y, 2) = (x - cy_DEPTH) * double(left_depth_img(x, y)) / fy_DEPTH;
        left_depth_mapping(x, y, 3) = double(left_depth_img(x, y));
       
    end
end
%% All the 2D point from depth image back-projecting it into 3D space
% 2-D points Right Camera:
[n_row, n_col] = size(right_depth_img);
right_depth_mapping = zeros(n_row, n_col, 3);
for x = 1:n_row
    for y = 1:n_col
        % 2D point from depth image back-projecting it into 3D space
        right_depth_mapping(x, y, 1) = (y - cx_DEPTH) * double(right_depth_img(x, y)) / fx_DEPTH;
        right_depth_mapping(x, y, 2) = (x - cy_DEPTH) * double(right_depth_img(x, y)) / fy_DEPTH;
        right_depth_mapping(x, y, 3) = double(right_depth_img(x, y));
    end
end
%% Projecting all transformed 3D point into the RGB image
% Projecting the Left Camera:
index = 0;
pc_RGB_left = zeros(n_row * n_col, 3);
pc_RGB_left_col = zeros(n_row * n_col, 3);
for x = 1:n_row
    for y = 1:n_col
        if left_depth_mapping(x, y, 3) > 0
            % Applying transformation between Depth and RGB camera
            XYZ_depth = [left_depth_mapping(x, y, 1), left_depth_mapping(x, y, 2), left_depth_mapping(x, y, 3)];
            XYZ_RGB = Rotation_Vec * XYZ_depth' + Translation_Vec;
            % Projecting every transformed 3D point into the RGB image
            x_RGB = round(fx_RGB_left * XYZ_RGB(1) / XYZ_RGB(3) + cx_RGB_left);
            y_RGB = round(fy_RGB_left * XYZ_RGB(2) / XYZ_RGB(3) + cy_RGB_left);

            if x_RGB > 0 && y_RGB > 0 && x_RGB < n_col  && y_RGB < n_row
                % look up color corrsponding to a given 3D point
                color = left_RGB_img(y_RGB, x_RGB, :);
                index = index + 1;
                % creating new point cloud with color
                pc_RGB_left(index,:) =  XYZ_RGB;
                pc_RGB_left_col(index, :) = color;
            end
        end
    end
end
pc_RGB_left = pc_RGB_left(1:index,:);
pc_RGB_left_col = pc_RGB_left_col (1:index,:);
figure()
pcshow(left_depth_mapping);set(gca,'FontSize',13)
title('Left reconstructed using colormap')
figure()
pcshow(pc_RGB_left, pc_RGB_left_col / 256.0); set(gca,'FontSize',13)
title('Left reconstructed using colorpixels')
%% Projecting all transformed 3D point into the RGB image
% Projecting the Right camera:
index = 0;
pc_RGB_right = zeros(n_row * n_col, 3);
pc_RGB_right_col = zeros(n_row * n_col, 3);
for x = 1:n_row
    for y = 1:n_col
         if right_depth_mapping(x, y, 3) > 0
            % Applying transformation between Depth and RGB camera
            XYZ_depth = [right_depth_mapping(x, y, 1), right_depth_mapping(x, y, 2), right_depth_mapping(x, y, 3)];
            XYZ_RGB = Rotation_Vec * XYZ_depth' + Translation_Vec;
            % Projecting every transformed 3D point into the RGB image
            x_RGB = round(fx_RGB_right * XYZ_RGB(1) / XYZ_RGB(3) + cx_RGB_right);
            y_RGB = round(fy_RGB_right * XYZ_RGB(2) / XYZ_RGB(3) + cy_RGB_right);

            if x_RGB > 0 && y_RGB > 0 && x_RGB < n_col  && y_RGB < n_row
                % looking up color corrsponding to a given 3D point
                color = right_RGB_img(y_RGB, x_RGB, :);
                index = index + 1;
                % creating new point cloud with color
                pc_RGB_right(index,:) =  XYZ_RGB;
                pc_RGB_right_col(index, :) = color;
            end
        end
    end
end
pc_RGB_right = pc_RGB_right(1:index,:);
pc_RGB_right_col = pc_RGB_right_col (1:index,:);
figure()
pcshow(right_depth_mapping); set(gca,'FontSize',13)
title('Right reconstructed using colormap')
figure()
pcshow(pc_RGB_right, pc_RGB_right_col / 256.0); set(gca,'FontSize',13)
title('Right reconstructed using colorpixels')
%% Transforming from left 3D to right 3D coordinate camera
% Left and right RBG cameras
Trans_RGB = [ -954.57277, 99.75455, 462.56569] - [66.49332, 16.62784, 128.72005*0.5];
Right_RGB = [ -0.03190 0.73381 -0.15668 ];
Right_RGB = rotationVectorToMatrix(Right_RGB);
Right_RGB = inv(Right_RGB);
%%
% Calculate corresponding XYZ translated from left to right
len = length(pc_RGB_left);
pc_RGB_left_right = zeros(len, 3);
for i=1:len
    pc_RGB_left_right(i, :) = (Right_RGB *  pc_RGB_left(i, :)' + Trans_RGB')';
end
%%
% Merging 3D point cloud generated from both both cameras:
figure()
pcshow([pc_RGB_left_right; pc_RGB_right], [pc_RGB_left_col/256.0; pc_RGB_right_col/256]); set(gca,'FontSize',13)
title('Final Reconstructed Image')