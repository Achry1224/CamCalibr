disp("===== begin =====");

% ===== 输入参数 =====
images_io = imageSet('.\intr_gs_40_5_1\img_example'); % 校准图像文件夹
imageSize_io = [1024, 1280]; % 校准图像分辨率（倒置）
Corners = load('.\intr_gs_40_5_1\corners_info\Corners.mat');  % 输入校准参数
num_cor = 144;  % 总角点数量
num_xy = 2; % 角点坐标数量
num_img = 100;  % 校准图像数量
imagePoints_input = NaN(num_cor, num_xy, num_img);  % 初始化校准矩阵
% =================
disp1 = ['Corners的前三个元素：', num2str(Corners.CornersIdx(1,1)), ', ', ...
    num2str(Corners.CornersIdx(2,1)), ', ',num2str(Corners.CornersIdx(3,1))];
disp(disp1);

% 图像计数器
img_count = 0;
for index = 1:length(Corners.CornersIdx) %遍历所有命名
    if isnan(Corners.CornersIdx(index))
        img_count = img_count + 1;
    else
        % Xdisp = [' index: ', index,' Corners.CornersIdx(index): ', Corners.CornersIdx(index)];
        % disp(Corners.CornersIdx(index));
        imagePoints_input(Corners.CornersIdx(index)+1, 1, img_count) = Corners.CornersImageFrame(index, 1);
        imagePoints_input(Corners.CornersIdx(index)+1, 2, img_count) = Corners.CornersImageFrame(index, 2);
    end
    
    % disp(Corners.CornersIdx(name));
    % index = CornersIdx(name);
    % data_name = strcat('TestData.',index);
    % data = eval(data_name{1}) %打印所有数据
end

disp('imagePoints_input(120,1,1)为');
disp(imagePoints_input(120,1,1));


% 校准相机
% I = readimage(images,1); 
% imageSize = [size(I, 1),size(I, 2)];
% params_io = estimateCameraParameters(imagePoints,worldPoints, ...
%                                  'ImageSize',imageSize);
% params_io_pinhole = estimateCameraParameters(imagePoints_input, Corners.AprilGridPoint, ...
%                                  'ImageSize',imageSize_io, EstimateTangentialDistortion = true);
params_io_fisheye = estimateFisheyeParameters(imagePoints_input, Corners.AprilGridPoint, ...
                                 imageSize_io);


% 所有图像文件名
imageFileNames_io = images_io.ImageLocation;

% 可视化校准精度
showReprojectionErrors(params_io_fisheye);

% 可视化相机外部
figure;
showExtrinsics(params_io_fisheye);
drawnow;

% 绘制检测和重新投影的点
figure; 
imshow(imageFileNames_io{1}); 
hold on;
plot(imagePoints_input(:,1,1), imagePoints_input(:,2,1),'go');
plot(params_io_fisheye.ReprojectedPoints(:,1,1),params_io_fisheye.ReprojectedPoints(:,2,1),'r+');
legend('Detected Points','ReprojectedPoints');
hold off;




