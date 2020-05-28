%%SUBHANKAR CHATTORAJ-3DMT
%%
clear all; close all; clc;
%% Calibrate the two RGB cameras using the checkerboard calibration technique
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1.1.	Detect the corner points of the checkerboard
% Select the Camera (out of the one by commenting or un-commenting)
path = 'RGB\Right'; % Right Camera
% path = 'RGB\Left'; % Left Camera
allFiles = dir(path); imageFileNames = { allFiles.name }; 
for k = 1:size(imageFileNames,2)-2
    FileName = [path,'\',imageFileNames{1,k+2}]; I = imread(FileName); k
    figure; imshow(I); hold on;
    [imagePoints,boardSize] = detectCheckerboardPoints(I);
    plot(imagePoints(:,1),imagePoints(:,2),'ro');
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% At first convert all the RGB to grayscale images
path = 'RGB\Left'; 
allFiles = dir(path); imageFileNames = { allFiles.name };
for k = 1:size(imageFileNames,2)-2
    FileName = [path,'\',imageFileNames{1,k+2}]; I = imread(FileName);
    NewFileName = strrep(FileName,'RGB','grayscale'); 
    imwrite(rgb2gray(I),NewFileName); k
end
path = 'RGB\Right';
allFiles = dir(path); imageFileNames = { allFiles.name };
for k = 1:size(imageFileNames,2)-2
    FileName = [path,'\',imageFileNames{1,k+2}]; I = imread(FileName);
    NewFileName = strrep(FileName,'RGB','grayscale'); 
    imwrite(rgb2gray(I),NewFileName); k
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ***************************** Note *************************************
%% Replace your grayscale converted images in the below folders
% 'C:\Program Files\MATLAB\R2019b\toolbox\vision\visiondata\calibration\stereo\left'
% 'C:\Program Files\MATLAB\R2019b\toolbox\vision\visiondata\calibration\stereo\right'

%% ***************************** Note *************************************
%% Run (Evaluate Section) each of these code segments in the command window

% Specify calibration images.
leftImages = imageDatastore(fullfile(toolboxdir('vision'),'visiondata', ...
    'calibration','stereo','left'));
rightImages = imageDatastore(fullfile(toolboxdir('vision'),'visiondata', ...
    'calibration','stereo','right'));

% Detect the checkerboards.
[imagePoints,boardSize] = ...
  detectCheckerboardPoints(leftImages.Files,rightImages.Files);

% Specify the world coordinates of the checkerboard keypoints. Square size is in millimeters.
squareSize = 108;
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

% Calibrate the stereo camera system. Both cameras have the same resolution.
I = readimage(leftImages,1); 
imageSize = [size(I,1),size(I,2)];
params = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize);

% Visualize the calibration accuracy.
figure, showReprojectionErrors(params);
title('Projection Error')

% Visualize camera extrinsics.
figure; showExtrinsics(params);
figure; showExtrinsics(params,'patternCentric');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1.2.	Calibrate each camera separately (to get intrinsic parameters)
% intrinsic parameters (inherent to individual cameras) 
%% Camera-1
C1 = params.CameraParameters1; C1_Intrinsics = C1.Intrinsics;
C1_FocalLength = C1_Intrinsics.FocalLength;
C1_PrincipalPoint = C1_Intrinsics.PrincipalPoint;
C1_ImageSize = C1_Intrinsics.ImageSize;
C1_RadialDistortion = C1_Intrinsics.RadialDistortion;
C1_TangentialDistortion = C1_Intrinsics.TangentialDistortion;
C1_Skew = C1_Intrinsics.Skew;
C1_IntrinsicMatrix = C1_Intrinsics.IntrinsicMatrix;
         
%% Camera-2
C2 = params.CameraParameters2; C2_Intrinsics = C2.Intrinsics;
C2_FocalLength = C2_Intrinsics.FocalLength;
C2_PrincipalPoint = C2_Intrinsics.PrincipalPoint;
C2_ImageSize = C2_Intrinsics.ImageSize;
C2_RadialDistortion = C2_Intrinsics.RadialDistortion;
C2_TangentialDistortion = C2_Intrinsics.TangentialDistortion;
C2_Skew = C2_Intrinsics.Skew;
C2_IntrinsicMatrix = C2_Intrinsics.IntrinsicMatrix;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1.3.	Calibrate one camera with respect of each other(to get extrinsic parameters)
% extrinsic parameters = Inter-camera Geometry = [R|T]
R = params.RotationOfCamera2;
T = params.TranslationOfCamera2;
Fundamental_Matrix = params.FundamentalMatrix;
Essential_Matrix = params.EssentialMatrix;
%%
R
T
Fundamental_Matrix
Essential_Matrix