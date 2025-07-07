# Moving Least Squares Image Deformation

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=cuixing158/Moving-Least-Squares-Image-Deformation)
[![View Moving-Least-Squares-Image-Deformation on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/181409-moving-least-squares-image-deformation)

![Demo GIF](images/output.gif)

**高性能移动最小二乘(MLS)图像变形库** - 基于控制点的实时图像变形工具，适用于图像编辑、图像配准、面部表情动画，计算机视觉、数据增强等应用领域。以类的方式设计，在初始化阶段智能预计算权重和变形数据，相比传统逐帧计算MLS变形，性能提升高达数十倍。

## Features

- **实时变形**：通过控制点交互，实时更新图像变形。
- **高性能**：预计算权重和变形数据，显著提升变形速度。
- **简洁API**：通过类的方式封装MLS变形逻辑，提供简洁的API接口。
- **支持多种变形类型**："Affine"、"Similarity"、"Rigid"。
- **部署支持**： 支持嵌入式C/C++代码生成，便于集成到其他应用中。

## Usage

`p`,`q`是同大小（Mx2）的二维控制点集坐标，分别表示源图像和目标图像的控制点位置。`imgHeight`和`imgWidth`是图像的高度和宽度。

```matlab
% 三步完成复杂变形
mls = mlsImageWarp(p, imgHeight, imgWidth); % 初始化
deformed = mls.warpImage(img, q);           % 执行变形
imshow(deformed);                           % 可视化
```

## Example

1. run `Example.m` ，弹出交互窗口。
2. 在弹出的图像窗口中，鼠标**点击添加控制点**（红色圆点，自动编号），每点击一次添加一个点。
3. 添加完所有控制点后，按下 **ESC 键** 结束控制点添加。
4. 拖动已添加的控制点，图像会根据控制点的新位置自动实时变形。

## Performance

| 方法 | 512×512图像 | 2000x2000图像 | 控制点 |
|------|-------------|-----------|----------------|
| 本库(预计算) | 8.8 ms      | 12.2 ms   | 10个      |
| 本库(预计算) | 9.4 ms      | 13.2 ms   | 30个      |
| 本库(预计算) | 10.3 ms     | 15.1 ms   | 60个      |
| 传统计算 | 830.0 ms     |  13662.1 ms  | 10个       |
| 传统计算 | 1303 ms     | 24771.5 ms  | 30个        |
| 传统计算 | 2678.9 ms     | 39582.1 ms  | 60个        |

*测试环境：MATLAB R2025a, Intel Core i5-13400F，变形类型：“Rigid”*

## Requirement

- MATLAB
- Image Processing Toolbox™

## References

[1] [Schaefer, Scott, et al. "Image deformation using moving least squares." ACM Transactions on Graphics (TOG) 25.3 (2006): 533-540.](https://people.engr.tamu.edu/schaefer/research/mls.pdf)
