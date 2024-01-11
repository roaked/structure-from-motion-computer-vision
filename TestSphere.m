
%Add images folder to path
folder=strcat(pwd,'\Images\');
addpath(folder)

%% Read a Pair of Images
%Load a pair of images into the workspace.

I1 = imread('DSC_1.jpg');
I2 = imread('DSC_2.jpg'); 

figure (1)
imshowpair(I1, I2, 'montage');
title('Original Images');

%% Load Camera Parameters
%This example uses the camera parameters calculated by the cameraCalibrator app. 
%The parameters are stored in the cameraParams object, and include the
%camera intrinsics and lens distortion coefficients.

load('camParamsXiaomi.mat')

%% Remove Lens Distortion
%Lens distortion can affect the accuracy of the final reconstruction. 
%You can remove the distortion from each of the images using the 
%undistortImage function. This process straightens the lines that are bent 
%by the radial distortion of the lens.

I1 = undistortImage(I1, cameraParams);
I2 = undistortImage(I2, cameraParams);

figure (2)
imshowpair(I1, I2, 'montage');
title('Undistorted Images');

%% Find Point Correspondences Between The Images
%Detect good features to track. Reduce 'MinQuality' to detect fewer points,
%which would be more uniformly distributed throughout the image. 
%If the motion of the camera is not very large, then tracking using the 
%KLT algorithm is a good way to establish point correspondences.

% Detect feature points
imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'MinQuality', 0.04);

% Visualize detected points
figure (3)
imshow(I1, 'InitialMagnification', 50);
title('150 Strongest Corners from the First Image');
hold on
plot(selectStrongest(imagePoints1, 150));

% Create the point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);

% Track the points
[imagePoints2, validIdx] = step(tracker, I2);
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);

% Visualize correspondences
figure (4)
showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
title('Tracked Features');


%% Estimate the Essential Matrix
%Use the estimateEssentialMatrix function to compute the essential matrix 
%and find the inlier points that meet the epipolar constraint.

% Estimate the fundamental matrix
[E, epipolarInliers] = estimateEssentialMatrix(...
    matchedPoints1, matchedPoints2, cameraParams, 'Confidence', 99.99);

% Find epipolar inliers
inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);

% Display inlier matches
figure (5)
showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
title('Epipolar Inliers');


%% Compute the Camera Pose
%Compute the location and orientation of the second camera relative to 
%the first one. Note that t is a unit vector, because translation can only 
%be computed up to scale.

[orient, loc] = relativeCameraPose(E, cameraParams, inlierPoints1, inlierPoints2);


%% Reconstruct the 3-D Locations of Matched Points
%Re-detect points in the first image using lower 'MinQuality' to get 
%more points. Track the new points into the second image. 
%Estimate the 3-D locations corresponding to the matched points using 
%the triangulate function, which implements the Direct Linear Transformation 
%(DLT) algorithm [1]. Place the origin at the optical center of the camera 
%corresponding to the first image.

% Detect dense feature points. Use an ROI to exclude points close to the
% image edges.
roi = [30, 30, size(I1, 2) - 30, size(I1, 1) - 30];
imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'ROI', roi, ...
    'MinQuality', 0.001);

% Create the point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);

% Track the points
[imagePoints2, validIdx] = step(tracker, I2);
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);

% Compute the camera matrices for each position of the camera
% The first camera is at the origin looking along the Z-axis. Thus, its
% rotation matrix is identity, and its translation vector is 0.
camMatrix1 = cameraMatrix(cameraParams, eye(3), [0 0 0]);

% Compute extrinsics of the second camera
[R, t] = cameraPoseToExtrinsics(orient, loc);
camMatrix2 = cameraMatrix(cameraParams, R, t);

% Compute the 3-D points
points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);

% Get the color of each reconstructed point
numPixels = size(I1, 1) * size(I1, 2);
allColors = reshape(I1, [numPixels, 3]);
colorIdx = sub2ind([size(I1, 1), size(I1, 2)], round(matchedPoints1(:,2)), ...
    round(matchedPoints1(:, 1)));
color = allColors(colorIdx, :);

% Create the point cloud
ptCloud = pointCloud(points3D, 'Color', color);

%% Display the 3-D Point Cloud
%Use the plotCamera function to visualize the locations and orientations 
%of the camera, and the pcshow function to visualize the point cloud.

% Visualize the camera locations and orientations
cameraSize = 0.3;
figure (6)
plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
hold on
grid on
plotCamera('Location', loc, 'Orientation', orient, 'Size', cameraSize, ...
    'Color', 'b', 'Label', '2', 'Opacity', 0);

% Visualize the point cloud
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);

% Rotate and zoom the plot
camorbit(0, -30);
camzoom(1.5);

% Label the axes
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis')

title('Up to Scale Reconstruction of the Scene');

%% Fit a Sphere to the Point Cloud to Find the Ball
%Find the ball in the point cloud by fitting a sphere to the 3-D points 
%using the pcfitsphere function.

% Detect the ball
sphere = pcfitsphere(ptCloud, 0.1);

% Display the surface of the ball
plot(sphere);
title('Estimated Location and Size of the Ball');
xlim([-30 30])
ylim([-30 30])
zlim([0 60])
hold off


%% Metric Reconstruction of the Scene
%The actual radius of the globe is 10.5cm. You can now determine the 
%coordinates of the 3-D points in centimeters.

% Determine the scale factor
scaleFactor = 10.5 / sphere.Radius;


% Scale the point cloud
ptCloudScaled = pointCloud(points3D * scaleFactor, 'Color', color);
locScaled = loc * scaleFactor;

% Visualize the point cloud in centimeters
cameraSize = 2;

figure (7)
plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
set(gca,'Color','w')
hold on
grid on
plotCamera('Location', locScaled, 'Orientation', orient, 'Size', cameraSize, ...
    'Color', 'b', 'Label', '2', 'Opacity', 0);
set(gca,'Color','w')


% Visualize the point cloud
pcshow(ptCloudScaled, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
camorbit(0, -30);
camzoom(1.5);

% Label the axes
xlim([-40 40])
ylim([-40 40])
zlim([0 80])
xlabel('x-axis (cm)');
ylabel('y-axis (cm)');
zlabel('z-axis (cm)')
title('Metric Reconstruction of the Scene');


%Summary
%This example showed how to recover camera motion and reconstruct the 
%3-D structure of a scene from two images taken with a calibrated camera.
