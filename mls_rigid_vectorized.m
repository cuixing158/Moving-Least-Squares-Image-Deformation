function [Xd, Yd] = mls_rigid_vectorized(p, q, H, W)
% Vectorized implementation of MLS rigid deformation
% Based on: S. Schaefer et al., "Image Deformation Using Moving Least Squares", TOG 2006

% 输入控制点数量
K = size(p, 1);

% 所有图像网格点
[XX, YY] = meshgrid(1:W, 1:H);

% XX = gpuArray(XX);
% YY = gpuArray(YY);

X = XX(:); % N x 1
Y = YY(:); % N x 1
N = numel(X);

% 所有像素点与控制点的差值向量 v - p_i (N x K x 2)
Vx = reshape(X, [N, 1]) - reshape(p(:,1), [1, K]);
Vy = reshape(Y, [N, 1]) - reshape(p(:,2), [1, K]);

% 权重矩阵 W (N x K)
w = 1 ./ (Vx.^2 + Vy.^2 + eps);

% 计算 p*, q* (加权中心): N x 1
sw = sum(w, 2); % N x 1
p_star_x = (w * p(:,1)) ./ sw;
p_star_y = (w * p(:,2)) ./ sw;
q_star_x = (w * q(:,1)) ./ sw;
q_star_y = (w * q(:,2)) ./ sw;

% 减去中心后: p_hat, q_hat (K x 2)
p_hat = p - [mean(p_star_x), mean(p_star_y)];% Kx2
q_hat = q - [mean(q_star_x), mean(q_star_y)];% Kx2

% 垂直向量 p_hat_perp: (K x 2)
p_hat_perp = [p_hat(:,2), -p_hat(:,1)];
q_hat_perp = [q_hat(:,2), -q_hat(:,1)];

% 初始化结果坐标
Xd = zeros(N,1);
Yd = zeros(N,1);

% 主计算循环（外层 N 可以保留向量化的内部）
for i = 1:N
    vi = [X(i), Y(i)]; % 1x2

    % 当前点的控制点加权中心
    pi_star = [p_star_x(i), p_star_y(i)];% 1x2
    qi_star = [q_star_x(i), q_star_y(i)];% 1x2

    % 构建 M 矩阵: K x 2 -> (p_hat; p_hat_perp)
    M = [p_hat; p_hat_perp]; % 2K x 2
    W_i = diag(repmat(w(i,:)', 2, 1));% 2K x 2K diagnal matrix

    % 构建 Q 矩阵: K x 2 -> (q_hat; q_hat_perp)
    Q = [q_hat; q_hat_perp];% 2K x 2

    % fs = (v - p*) * M * W * Q' * inv(M * W * M')
    vp = vi - pi_star;% 1x2
    A = M' * W_i * M;% 2x2
    B = M' * W_i * Q;% 2x2

    fs = (vp * B) / A;%1x2
    fs_len = norm(fs);% 1x1
    vp_len = norm(vp);% 1x1

    fs_scaled = fs * (vp_len / (fs_len + eps)) + qi_star;% % 1x2
    Xd(i) = fs_scaled(1);
    Yd(i) = fs_scaled(2);
end

% Reshape 回图像
Xd = reshape(Xd, H, W);
Yd = reshape(Yd, H, W);
end
