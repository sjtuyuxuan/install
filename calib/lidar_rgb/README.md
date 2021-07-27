## Camera intrinsic calibration

use Matlab Camera Calibrator

save the result into camera_instrinsic.mat

```matlab
save('camera_instrinsic.mat')
```

## Lidar - Camera extrinsic calibration

### Dependency

add  toolbox **lidar**

Click Add-Ons(附加功能) - get Add-Ons 


https://blog.csdn.net/qq_27339501/article/details/110224436

```matlab
clear; clc; clear all;

imagePath = fullfile('/home/chenyu/Desktop/lidar_calib/data_temp','cam'); 
ptCloudPath = fullfile('/home/chenyu/Desktop/lidar_calib/data_temp','lidar');
cameraParamsPath = fullfile('/home/chenyu/Desktop/lidar_calib/camera_instrinsic.mat');
% change the file path

intrinsic = load(cameraParamsPath); % Load camera intrinscs
imds = imageDatastore(imagePath); % Load images using imageDatastore
pcds = fileDatastore(ptCloudPath, 'ReadFcn', @pcread); % Loadr point cloud files

imageFileNames = imds.Files;
ptCloudFileNames = pcds.Files;

squareSize = 40 % Square size of the checkerboard in mm

% Set random seed to generate reproducible results.
rng('default');

% Extract Checkerboard corners from the images
[imageCorners3d, checkerboardDimension, dataUsed] = ...
    estimateCheckerboardCorners3d(imageFileNames, intrinsic.cameraParams, squareSize);

imageFileNames = imageFileNames(dataUsed); % Remove image files that are not used


% Filter point cloud files corresponding to the detected images
ptCloudFileNames = ptCloudFileNames(dataUsed);

imageCorners3d_tmp = imageCorners3d;
% Extract ROI from the detected image corners
% roi = helperComputeROI(imageCorners3d, 5);
tolerance = 0.5;
x = reshape(imageCorners3d_tmp(:, 1, :), [], 1);
y = reshape(imageCorners3d_tmp(:, 2, :), [], 1);
z = reshape(imageCorners3d_tmp(:, 3, :), [], 1);

xMax = max(z) + tolerance;
xMin = min(z) - tolerance;

yMax = max(x) + tolerance;
yMin = min(x) - tolerance;

zMax = max(y) + tolerance;
zMin = min(y) - tolerance;

roi = [xMin, xMax, yMin, yMax, zMin, zMax];


%Extract Checkerboard in lidar data
[lidarCheckerboardPlanes, framesUsed, indices] = detectRectangularPlanePoints(...
    ptCloudFileNames, checkerboardDimension);
imageCorners3d = imageCorners3d(:, :, framesUsed);
% Remove ptCloud files that are not used
ptCloudFileNames = ptCloudFileNames(framesUsed);
% Remove image files 
imageFileNames = imageFileNames(framesUsed);
[tform, errors] = estimateLidarCameraTransform(lidarCheckerboardPlanes, ...
    imageCorners3d, 'CameraIntrinsic', intrinsic.cameraParams);
helperFuseLidarCamera(imageFileNames, ptCloudFileNames, indices,...
    intrinsic.cameraParams, tform);
helperShowError(errors)
```

注意：

imagePath, ptCloudPath,cameraParamsPath, squareSize 需要修改。
