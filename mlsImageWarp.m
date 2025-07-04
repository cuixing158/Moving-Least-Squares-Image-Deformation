classdef mlsImageWarp < handle %#codegen
    % Brief: mlsImageWarp Class for MLS image warping
    % Details:
    %  This class implements Moving Least Squares (MLS) deformation for images
    %  using control points. It supports affine, similar, and rigid transformations.
    %
    % Example:
    %    img = imread("kobi.png"); % matlab built-in source image
    % % fixed points
    % p = [648.44       428.55
    %        767.76       587.64
    %        908.95       736.79
    %        1000.4       887.93
    %        1131.7       915.77
    %        1211.2       810.37
    %        1147.6       627.41
    %        1145.6       438.49
    %        1101.8       183.95
    %        819.46       231.68
    %        877.13       589.63
    %          1076       502.13];
    %  % moving points
    % q = [648.44       428.55
    %        767.76       587.64
    %        908.95       736.79
    %        1000.4       887.93
    %        1131.7       915.77
    %        1211.2       810.37
    %        1147.6       627.41
    %        1145.6       438.49
    %        1101.8       183.95
    %        819.46       231.68
    %        831.39       625.43
    %        1105.8       486.22];
    % % image warping using MLS
    % mls = mlsImageWarp(p,size(img,1),size(img,2),Type="rigid");
    % outImg = mls.warpImage(img,q);
    %
    % % show result
    % figure;
    % nexttile;
    % imshow(img);
    % hold on;
    % plot(p(:,1), p(:,2), 'r.', 'MarkerSize', 10, 'DisplayName', 'Original Points');
    %
    % nexttile;
    % imshow(outImg);
    % hold on;
    % plot(q(:,1), q(:,2), 'b.', 'MarkerSize', 10, 'DisplayName', 'Deformed Points');
    %
    % See also: imwarp

    % Author:                          cuixingxing
    % Email:                           cuixingxing150@gmail.com
    % Created:                         03-Jul-2025 19:16:49
    % Version history revision notes:
    %                                  None
    % Implementation In Matlab R2025a
    % Copyright © 2025 xingxing cui.All Rights Reserved.
    %

    properties
        p       % Source control points (size: Mx2)
        type    % Deformation type ('affine','similar','rigid')
        alpha   % Weight exponent (default=2)
    end

    properties(Hidden)
        v       % Target points(flatten grid points) to deform (Nx2)

        gridX   % X grid points
        gridY   % Y grid points

        w       % Weights for MLS deformation (size: M x N, M is number of control points, N is number of grid points)
        data    % Precomputed deformation data based on type
    end

    methods
        function obj = mlsImageWarp(p, imageHeight, imageWidth, options)
            % mlsImageWarp Constructor
            % obj = mlsImageWarp(p, imageHeight, imageWidth) creates rigid MLS deformation
            % obj = mlsImageWarp(p, imageHeight, imageWidth, Name=Value) specifies options:
            %   - Type: 'affine'|'similar'|'rigid' (default)
            %   - Alpha: weight exponent (default=2)

            arguments
                p (:,2) double    % Source points coordinates in pixel (Mx2)
                imageHeight (1,1) {mustBeInteger,mustBePositive} % Image height
                imageWidth (1,1) {mustBeInteger,mustBePositive}  % Image width
                options.Type (1,1) string {mustBeMember(options.Type, ["affine", "similar", "rigid"])} = "rigid"
                options.Alpha (1,1) double {mustBePositive} = 2
            end

            delta = 15; % Step size for grid points, can be adjusted

            % Create a meshgrid for the image
            [gridX, gridY] = meshgrid(1:delta:imageWidth, 1:delta:imageHeight);
            obj.p = p; % Mx2
            obj.v = [gridX(:), gridY(:)]; % Flatten grid to Nx2
            obj.type = options.Type;
            obj.alpha = options.Alpha;

            % Precompute weights and deformation data
            obj.w = obj.precomputeWeights();
            obj.data = obj.precomputeDeformationData();

            % gridX and gridY are used for interpolation later
            obj.gridX = gridX;  % Store grid for later use
            obj.gridY = gridY;  % Store grid for later use
        end

        function [imgo, mapX, mapY, fv] = warpImage(obj, img, q, options)
            %WARPIMAGE Warp an image using MLS deformation
            %
            % Syntax:
            %  imgo = warpImage(img, q)
            %  [imgo, ...] = warpImage(..., Name=Value)
            %
            % Parameters:
            %  img    - Input image (HxWxC)
            %  q      - New control points (Mx2),must same size as obj.p
            %
            % Options:
            %  Mode   - Interpolation mode ('linear'|'nearest'|'cubic')
            %  Fill   - Fill value for out-of-bound pixels (default=0)
            %
            % Outputs:
            %  imgo   - Warped image
            %  mapX   - X deformation field
            %  mapY   - Y deformation field
            %  fv     - Deformed grid points (optional)

            arguments
                obj
                img  {mustBeNumeric}           % 输入图像
                q (:,2) double  % 新控制点
                options.Mode (1,1) string {mustBeMember(options.Mode, ["linear","nearest","cubic"])} = "linear"
                options.Fill = 0                 % 填充值
            end

            assert(size(q, 1) == size(obj.p, 1), 'New control points q must have the same number of points as p');

            % 计算变形后的网格点
            sfv = obj.deform(q);  % sfv is NX2 matrix of deformed points

            % 计算位移场
            dxy = obj.v - sfv;% Nx2
            [H, W, ~] = size(img);
            [TX, TY] = meshgrid(1:W, 1:H);

            % 插值位移场到图像网格
            dxT = interp2(obj.gridX, obj.gridY, reshape(dxy(:,1), size(obj.gridX)), TX, TY, options.Mode, 0); % HxW
            dyT = interp2(obj.gridX, obj.gridY, reshape(dxy(:,2), size(obj.gridY)), TX, TY, options.Mode, 0);% HxW

            % 计算逆向映射
            mapX = TX + dxT;
            mapY = TY + dyT;

            % 可选：计算正向变形点
            if nargout > 3
                fXT = TX - dxT;
                fYT = TY - dyT;
                fv = [fXT(:), fYT(:)]; % (HxW)x2
            end

            % 执行图像变形
            imgo = imageInterp(img, mapX, mapY, FillValues=options.Fill);
        end
    end

    methods (Access = private)
        function deformed = deform(obj, q)
            %DEFORM Apply deformation using new control points q
            arguments
                obj
                q (:,2) double    % New control points (must match obj.p size)
            end

            switch obj.type
                case "affine"
                    deformed = obj.applyAffineDeformation(q);% Nx2
                case "similar"
                    deformed = obj.applySimilarDeformation(q);% Nx2
                case "rigid"
                    deformed = obj.applyRigidDeformation(q);% Nx2
            end
        end

        function w = precomputeWeights(obj)
            %PRECOMPUTEWEIGHTS Compute weights for MLS
            w = zeros(size(obj.p, 1), size(obj.v, 1)); % MxN size

            for i = 1:size(obj.p, 1)
                diff = obj.v - obj.p(i,:); % Nx2
                norms_2 = sum(diff.^2, 2); % Nx1
                w(i, :) = 1 ./ (norms_2.^obj.alpha + 1e-8)';
            end
        end

        function data = precomputeDeformationData(obj)
            %PRECOMPUTEDEFORMATIONDATA Precompute based on deformation type
            switch obj.type
                case "affine"
                    data = obj.precomputeAffineData();
                case "similar"
                    data = obj.precomputeSimilarData();
                case "rigid"
                    data = obj.precomputeRigidData();
            end
        end

        function data = precomputeAffineData(obj)
            %PRECOMPUTEAFFINEDATA Precompute for affine deformation
            p_star = obj.computeWeightedCentroids(obj.p, obj.w);% Nx2
            M1 = obj.v - p_star;% Nx2

            % Initialize matrix components
            a = zeros(size(p_star, 1),1);% Nx1
            b = a;% Nx1
            d = a;% Nx1
            p_hat = cell(1, size(obj.p, 1));% 1xM

            for i = 1:size(obj.p, 1)
                p_hat{i} = obj.p(i,:) - p_star;% Nx2
                a = a + obj.w(i, :)' .* p_hat{i}(:,1).^2;% Nx1
                b = b + obj.w(i, :)' .* p_hat{i}(:,1) .* p_hat{i}(:,2);% Nx1
                d = d + obj.w(i, :)' .* p_hat{i}(:,2).^2;% Nx1
            end

            det = a .* d - b.^2;% Nx1
            Ia = d ./ det;% Nx1
            Ib = -b ./ det;% Nx1
            Id = a ./ det;% Nx1

            F1 = [sum(M1 .* [Ia, Ib], 2), sum(M1 .* [Ib, Id], 2)];% Nx2

            A = zeros(size(obj.p, 1), size(p_star, 1));% MxN
            for j = 1:size(obj.p, 1)
                A(j, :) = obj.w(j, :).*(sum(F1 .* p_hat{j}, 2))'; %1xN
            end

            data.A = A;
        end

        function data = precomputeSimilarData(obj)
            %PRECOMPUTESIMILARDATA Precompute for similarity deformation
            p_star = obj.computeWeightedCentroids(obj.p, obj.w);% Nx2
            mu = zeros(size(p_star, 1),1);% Nx1
            p_hat = cell(1, size(obj.p, 1));% 1xM

            for i = 1:size(obj.p, 1)
                p_hat{i} = obj.p(i,:) - p_star;% Nx2
                mu = mu + obj.w(i, :)' .* sum(p_hat{i}.^2, 2); % Nx1
            end

            [A, ~] = obj.precomputeAComponents(p_star, p_hat);

            % Normalize by mu
            for i = 1:numel(A)
                A{i}.a = A{i}.a ./ mu;% Nx1
                A{i}.b = A{i}.b ./ mu;% Nx1
                A{i}.c = A{i}.c ./ mu;% Nx1
                A{i}.d = A{i}.d ./ mu;% Nx1
            end

            data.A = A;
        end

        function data = precomputeRigidData(obj)
            %PRECOMPUTERIGIDDATA Precompute for rigid deformation
            p_star = obj.computeWeightedCentroids(obj.p, obj.w);% Nx2 matrix
            p_hat = cell(1, size(obj.p, 1)); % 1xM

            for i = 1:size(obj.p, 1)
                p_hat{i} = obj.p(i,:) - p_star;% 1XM个cell,每个cell是Nx2矩阵
            end

            [A, v_p_star] = obj.precomputeAComponents(p_star, p_hat);
            norm_v_p_star = sqrt(sum(v_p_star.^2, 2));% Nx1

            data.A = A;
            data.norm_v_p_star = norm_v_p_star; % Nx1
        end

        function centroids = computeWeightedCentroids(~, points, weights)
            %COMPUTEWEIGHTEDCENTROIDS Compute weighted centroids
            centroids = (weights' * points) ./ sum(weights, 1)'; % Nx2 matrix
        end

        function [A, R1] = precomputeAComponents(obj, p_star, p_hat)
            %PRECOMPUTEACOMPONENTS Compute A matrix components
            A = cell(1, numel(p_hat)); % 1xM
            R1 = obj.v - p_star; % Nx2
            R2 = [R1(:,2), -R1(:,1)]; % Nx2

            for i = 1:numel(p_hat)
                L1 = p_hat{i}; % Nx2
                L2 = [L1(:,2), -L1(:,1)];% Nx2

                A{i}.a = obj.w(i, :)' .* sum(L1 .* R1, 2);% Nx1
                A{i}.b = obj.w(i, :)' .* sum(L1 .* R2, 2);% Nx1
                A{i}.c = obj.w(i, :)' .* sum(L2 .* R1, 2);% Nx1
                A{i}.d = obj.w(i, :)' .* sum(L2 .* R2, 2);% Nx1
            end
        end

        function deformed = applySimilarDeformation(obj, q)
            %APPLYSIMILARDEFORMATION Apply similarity deformation
            arguments
                obj
                q (:,2) double
            end

            % 计算加权质心
            q_star = obj.computeWeightedCentroids(q, obj.w); % Nx2
            deformed = q_star;% Nx2

            % 添加相似变换部分
            for i = 1:size(q, 1)
                q_hat = q(i,:) - q_star; % Nx2
                deformed = deformed + [
                    sum(q_hat .* [obj.data.A{i}.a, obj.data.A{i}.c], 2),sum(q_hat .* [obj.data.A{i}.b, obj.data.A{i}.d], 2)];% Nx2
            end
        end

        function deformed = applyRigidDeformation(obj, q)
            %APPLYRIGIDDEFORMATION Apply rigid deformation (rotation only)
            arguments
                obj
                q (:,2) double
            end

            % 计算加权质心
            q_star = obj.computeWeightedCentroids(q, obj.w);% Nx2

            % 计算初始变形
            fv2 = zeros(size(q_star));% Nx2
            for i = 1:size(q, 1)
                q_hat = q(i,:) - q_star;% Nx2
                fv2 = fv2 + [
                    sum(q_hat .* [obj.data.A{i}.a, obj.data.A{i}.c], 2),sum(q_hat .* [obj.data.A{i}.b, obj.data.A{i}.d], 2)];% Nx2
            end

            % 计算归一化因子（保持长度不变）
            norm_fv2 = sqrt(sum(fv2.^2, 2));% Nx1
            norm_fv2(norm_fv2 == 0) = 1e-8;  % 避免除以零
            norm_factor = obj.data.norm_v_p_star ./ norm_fv2;% Nx1

            % 生成最终输出（标准化后加平移）
            deformed = fv2 .* norm_factor + q_star;% Nx2
        end

        function deformed = applyAffineDeformation(obj, q)
            %APPLYAFFINEDEFORMATION Apply affine deformation
            % 计算加权质心
            q_star = obj.computeWeightedCentroids(q, obj.w);% Nx2
            deformed = q_star;% Nx2

            % 添加仿射部分（不含平移）
            for j = 1:size(q, 1)
                q_hat = q(j,:) - q_star;% Nx2
                deformed = deformed + q_hat .* obj.data.A(j, :)';% Nx2
            end
        end
    end
end


%% Utility function for image interpolation
function outImage = imageInterp(inImage,mapX,mapY,options)
% Brief: Interpolate image based on mapping coordinates
%
% Syntax:
%     outImage = imageInterp(inImage,mapX,mapY)
%     outImage = imageInterp(inImage,mapX,mapY,name=value)
%
% Inputs:
%   inImage   - 输入图像，大小为 [h, w, c]，支持灰度或彩色
%   mapX      - 目标像素的 X 坐标映射，大小为 [oH, oW]，类型为 double/single
%   mapY      - 目标像素的 Y 坐标映射，大小为 [oH, oW]，类型为 double/single
%   options   - 结构体或参数集，包含以下字段:
%       FillValues   - 边界填充值（double，默认0），用于 BORDER_CONSTANT
%       BorderMode   - 边界模式（string），可选:
%                         "BORDER_CONSTANT"   : 超出边界用 FillValues 填充
%                         "BORDER_REPLICATE"  : 超出边界用最近边界像素值填充
%                         "BORDER_REFLECT"    : 超出边界做镜像反射
%       SmoothEdges  - 是否对边缘做平滑/反锯齿处理（logical，默认 false）
%
% Outputs:
%   outImage  - 变形后的输出图像，大小与mapX/mapY一致
%
% See also: interp2, imwarp

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         03-Jul-2025 20:32:13
% Version history revision notes:
%                                  None
% Implementation In Matlab R2025a
% Copyright © 2025 TheMatrix.All Rights Reserved.
%
arguments
    inImage {mustBeNumeric,mustBeFinite} % 输入图像
    mapX (:,:) double {mustBeNumeric,mustBeFinite} % 输入图像的 X 坐标映射，大小为 [oH, oW]，类型为 double/single
    mapY (:,:) double {mustBeNumeric,mustBeFinite} % 输入图像的 Y 坐标映射，大小为 [oH, oW]，类型为 double/single
    options.FillValues (1,1) double {mustBeInRange(options.FillValues,0,255)}=0
    options.BorderMode (1,1) string {mustBeMember(options.BorderMode,["BORDER_CONSTANT","BORDER_REPLICATE","BORDER_REFLECT"])} = "BORDER_CONSTANT"
    options.SmoothEdges logical = false; % 是否对边缘反锯齿/光滑处理
end
[h,w,~] = size(inImage);

switch options.BorderMode
    case "BORDER_CONSTANT"
        % 使用固定值填充边界
        outImage = images.internal.interp2d(inImage,mapX,mapY,"linear",options.FillValues,options.SmoothEdges);

    case "BORDER_REPLICATE"
        % 限制坐标到图像范围内，实现边界复制
        mapX_clamped = max(1, min(w, mapX));
        mapY_clamped = max(1, min(h, mapY));
        % out = interp2(1:w, 1:h, inImage, mapX_clamped, mapY_clamped, 'linear');
        outImage = images.internal.interp2d(inImage,mapX_clamped,mapY_clamped,"linear",options.FillValues,options.SmoothEdges);

    case "BORDER_REFLECT"
        % 反射坐标处理
        mapX_reflect = reflectCoordinates(mapX, w);
        mapY_reflect = reflectCoordinates(mapY, h);
        % out = interp2(1:w, 1:h, inImage, mapX_reflect, mapY_reflect, 'linear');
        outImage = images.internal.interp2d(inImage,mapX_reflect,mapY_reflect,"linear",options.FillValues,options.SmoothEdges);
    otherwise
        error('Invalid border mode: %s', borderMode);
end
end

% 反射坐标处理
function coord = reflectCoordinates(coord, maxVal)
% 将坐标反射到1到maxVal的范围内
coord = 2*maxVal - coord;
coord = mod(coord - 1, 2*(maxVal - 1)) + 1;
coord = maxVal - abs(maxVal - coord);
coord = max(1, min(maxVal, coord));
end