% 创建一组校准图像
images = imageSet(fullfile(toolboxdir('vision'),'visiondata',...
            'calibration','mono'));
disp(fullfile(toolboxdir('vision'),'visiondata',...
            'calibration','mono'));
% 所有图像文件名
imageFileNames = images.ImageLocation;

% 检测校准模式
[imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);

% 生成正方形角的世界坐标
squareSizeInMM = 29;
worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);

% 校准相机
I = readimage(images,1); 
imageSize = [size(I, 1),size(I, 2)];
params = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize);

% 可视化校准精度
showReprojectionErrors(params);

% 可视化相机外部
figure;
showExtrinsics(params);
drawnow;

% 绘制检测和重新投影的点
figure; 
imshow(imageFileNames{1}); 
hold on;
plot(imagePoints(:,1,1), imagePoints(:,2,1),'go');
plot(params.ReprojectedPoints(:,1,1),params.ReprojectedPoints(:,2,1),'r+');
legend('Detected Points','ReprojectedPoints');
hold off;


