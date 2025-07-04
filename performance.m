%% 此脚本用于测试Moving Least Squares图像变形的性能
% 通过比较预计算和传统计算的时间消耗，评估不同方法的效率

%% 用户控制的参数，可以根据需要修改
width = 512;
num = 60;

%% 生成随机控制点和目标点
H = width;
W = width;
p = rand(num,2);
q = rand(num,2);
mls = mlsImageWarp(p,H,W,Type="rigid");

%% 使用timeit函数测试性能
T1 = timeit(@()mls.warpImage(rand(H,W,3), q))
T2 = timeit(@()mls_rigid_vectorized(p, q, H, W))



