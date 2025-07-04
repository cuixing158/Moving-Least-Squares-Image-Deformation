function Example
% Example  MLS图像变形交互演示
%
% 本示例演示如何使用 mlsImageWarp 类进行基于控制点的图像变形。
% 用户可在图像上交互式添加控制点，并拖动这些点实时观察变形效果。
%
% 步骤:
%   1. 读取并显示一张图片
%   2. 鼠标点击添加控制点（红色圆点，带编号标签）
%   3. 按ESC键结束控制点添加
%   4. 拖动任意一个控制点，图像实时变形
%
% 依赖（test in R2025a），其它版本或许也可行:
%   - MATLAB
%   - Image Processing Toolbox
%

% 读取图片
% img = imread('kobi.png');
img = imread('images/Lenna.jpg');

% 创建图形窗口和坐标轴
f=figure;
ax = axes(f);
hold(ax,"on");
hObj = imshow(img,Parent=ax);% 显示原图
num = 1;% 控制点编号

p = [];% 控制点坐标，num x 2

% 交互式添加控制点
while true
    % 在图像上添加一个可拖动的点
    rois(num) = drawpoint(ax,Color='r',Label=num2str(num),LabelAlpha=0.5);
    addlistener(rois(num),'MovingROI',@allevents);% 添加拖动监听
    p = [p;
        rois(num).Position];% 记录点坐标

    % 检查是否按下ESC键，若是则退出循环
    key = get(gcf, 'CurrentCharacter');
    if strcmp(key, char(27)) % 27是ESC的ASCII码
        break;
    end

    num = num + 1;
end

% 新控制点初始化为原始点
q = p;

% 构建MLS变形对象
mlsObj = mlsImageWarp(p,size(img,1),size(img,2),Type="rigid");


%% 控制点拖动回调函数
    function allevents(src, evt)
        % allevents  控制点拖动时的回调
        % src: 当前被拖动的点对象
        % evt: 事件数据，包含新坐标
        idx = str2double(src.Label); % 获取点编号
        q(idx,:) = evt.CurrentPosition; % 更新对应控制点的新位置

        % 对原图进行变形
        tic;
        deformImg = mlsObj.warpImage(img, q);
        toc;
        hObj.CData = deformImg; % 实时更新显示
        imwrite(deformImg, "deformImg.jpg"); % 保存变形结果
    end

end