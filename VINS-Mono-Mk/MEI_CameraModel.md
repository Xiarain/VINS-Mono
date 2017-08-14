###VINS-Mono 相机校准部分

####说明
* 1 VINS-Mono 相机校准部分是用了ETH的源码[Github](https://github.com/hengli/camodocal)。
* 2

####工程结构介绍

* camera_model
  * calib
    * CameraCalibration.c 封装了各个相机模型
  * camera_models
    * camera.c 构造camera类，为各种相机模型提供虚函数接口
    * PinholeCamera.c 针孔相机模型
    * ScaramuzzaCamera.c SCARAMUZZA相机模型
    * CataCamera.c MEI相机模型
    * EquidistantCamera.c KANNALA_BRANDT相机模型
    * CostFunctionFactory.c 代价函数，用于优化相机模型中的参数
  * chessboard 用于检测棋盘格特征点
  * gpl 经纬度变换（好像并没用调用）
  * sparse_graph
    * Transform.c 实现数据变量的操作
  * intrinsic_calib.cc 相机矫正模块主函数，提供了人机交互接口


####函数流程
* intrinsic_calib.cc main()函数
  * camodocal::Chessboard chessboard 查找棋盘格特征点
    * opencv库
    * Chessboard::findChessboardCornersImproved 增强版
  * camodocal::CameraCalibration calibration.calibrate()函数
    * CameraCalibration::calibrate
      * CameraCalibration::calibrateHelper
        * camera->estimateIntrinsics (各个相机模型类中实现)
        * camera->estimateExtrinsics （camera.c）
        * CameraCalibration::optimize
          *  CostFunctionFactory::instance() 投影误差,添加观测数据
      * 计算相机测量协方差（从重投影误差出发）

####张氏相机矫正法
* 在这个相机矫正软件包中使用的张氏相机矫正方法和论文中是不一样的。
* 下面是整个矫正算法的流程：
1. 根据张氏标定法，令标定景物平面为$Z=0$，那么有
$$
s\begin{bmatrix} u\\v\\1 \end{bmatrix}
= A\begin{bmatrix}\mathbf r_1 &  \mathbf r_2 & \mathbf r_3  & \mathbf t\end{bmatrix}\begin{bmatrix} X\\Y\\Z\\1 \end{bmatrix}
=A\begin{bmatrix}\mathbf r_1 &  \mathbf r_2 & \mathbf t\end{bmatrix}\begin{bmatrix} X\\Y\\1 \end{bmatrix}\tag{1}
$$
考虑相机外参矩阵为
$$
A=\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 &  0 &  1 \\
\end{bmatrix}\tag{2}
$$
则
$$
s\begin{bmatrix} u\\v\\1 \end{bmatrix}
=\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 &  0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
r_{11} & r_{12} & r_{14} \\
r_{21} & r_{22} & r_{24} \\
r_{31} & r_{32} & r_{34} \\
\end{bmatrix}
\begin{bmatrix} X\\Y\\1 \end{bmatrix}
\tag{3} \\
$$
从一个平面到另外一个平面的映射关系就是单应矩阵，把内参矩阵和外参矩阵相乘就可以获得一个单应矩阵（注意3D点平面中的Z轴为0，所以才可以这么做）。
2. 理想图像坐标减去图像偏移量
$$
\left[ \begin{matrix}
u - u_0 \\
v - v_0 \\
1  
\end{matrix} \right] =
\left[ \begin{matrix}
1 & 0 & - u_0 \\
0 & 1 & - v_0 \\
0 & 0 & 1 \\
\end{matrix} \right]
\left[ \begin{matrix}
u \\
v \\
1  
\end{matrix} \right] \tag{4}
$$
把公式（3）带入公式（4）进行转换得到公式（5）
$$
s\left[ \begin{matrix}
u - u_0 \\
v - v_0 \\
1  
\end{matrix} \right] =
\left[ \begin{matrix}
1 & 0 & - u_0 \\
0 & 1 & - v_0 \\
0 & 0 & 1 \\
\end{matrix} \right]
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 &  0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
r_{11} & r_{12} & r_{14} \\
r_{21} & r_{22} & r_{24} \\
r_{31} & r_{32} & r_{34} \\
\end{bmatrix}
\begin{bmatrix} X\\Y\\1 \end{bmatrix}
\tag{5}
$$
简化得到公式（6）
$$
s\left[ \begin{matrix}
u - u_0 \\
v - v_0 \\
1  
\end{matrix} \right] =
\begin{bmatrix}
f_x & 0 &0 \\
0 & f_y & 0 \\
0 &  0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
r_{11} & r_{12} & r_{14} \\
r_{21} & r_{22} & r_{24} \\
r_{31} & r_{32} & r_{34} \\
\end{bmatrix}
\begin{bmatrix} X\\Y\\1 \end{bmatrix}
\tag{6}
$$

继续简化得到公式（7）
$$
s\left[ \begin{matrix}
u - u_0 \\
v - v_0 \\
1  
\end{matrix} \right] =
\begin{bmatrix}
f_x r_{11} & f_x r_{12} &f_x r_{14} \\
f_y r_{21} & f_yr_{22} & f_y r_{24} \\
r_{31} & r_{32} & r_{34} \\
\end{bmatrix}
\begin{bmatrix} X\\Y\\1 \end{bmatrix}
\tag{7}
$$

由公式（7）可以得到单应矩阵$H$，在齐次坐标系下第九个元素的值为1。
$$
H =
\begin{bmatrix}
h_1 & h_2 & h_2 \\  
h_4 & h_5 & h_6 \\
h_7 & h_8 & h_9 \\
\end{bmatrix}
=
\frac{1}{r_{34}}
\begin{bmatrix}
f_x r_{11} & f_x r_{12} &f_x r_{14} \\
f_y r_{21} & f_yr_{22} & f_y r_{24} \\
r_{31} & r_{32} & r_{34} \\
\end{bmatrix}
\tag{7}
$$
在程序中$H$矩阵是已经测量得到了，所以需要求解的是公式（7）中的$r_{11} ... r_{34}$。
3. 旋转矩阵在构造中是相互正交的，即$r_1$和$r_2$相互正交$r_1^T r_2 = 0$，而且旋转向量长度相等（旋转不改变尺度）$||r_1|| = ||r_2|| = 1$。
$$
\begin{cases}
r_{11}^2+r_{21}^2+r_{31}^2=r_{12}^2+r_{22}^2+r_{32}^2=1\\
r_{11}r_{12}+r_{21}r_{22}+r_{31}r_{32}=0
\end{cases}\tag{8}
$$

4. 构造$Ax=b$方程组求解相机内参中的$f_x,f_y$。由公式（7）和公式（8）可得：
$$
\begin{cases}
(\frac{r_{34}h_1}{f_x})^2 + (\frac{r_{34}h_4}{f_x})^2 + (r_{34}h_7)^2 =
(\frac{r_{34}h_2}{f_y})^2 + (\frac{r_{34}h_5}{f_y})^2 + (r_{34}h_8)^2 \\
\frac{h_1}{f_x} \frac{h_2}{f_x} + \frac{h_4}{f_y} \frac{h_5}{f_y} + h_{7}h_{8} = 0

\end{cases}\tag{9}
$$

5. SVD求解内参
$$
\begin{bmatrix}
h_1^2-h_2^2 & h_4^2-h_5^2\\
h_1h_2 & h_4h_5
\end{bmatrix}
\begin{bmatrix}
\frac{1}{f_x^2}\\
\frac{1}{f_y^2}
\end{bmatrix}
=\begin{bmatrix}
h_8^2-h_7^2\\
-h_7h_8
\end{bmatrix}\tag{11}
$$
####MEI相机矫正法（鱼眼相机）
![](picture\MEI_camera.png)

1. 鱼眼矫正模型

$$
h(\chi_{s}) = m = (\frac{X_s}{Z_s+\xi}, \frac{Y_s}{Z_s+\xi},1) \tag{1}
$$

这个过程是可逆的，因为每一个球面点唯一地对应一个鱼眼畸变模型，而每一个鱼眼图像点也唯一地对应一个球面点。
$$
h^{-1}(m) =
\begin{bmatrix}
\frac{\xi + \sqrt{1 + (1 - \xi^2)(x^2 + y^2)}}{x^2 + y^2 +1}x \\
\frac{\xi + \sqrt{1 + (1 - \xi^2)(x^2 + y^2)}}{x^2 + y^2 +1}y \\
\frac{\xi + \sqrt{1 + (1 - \xi^2)(x^2 + y^2)}}{x^2 + y^2 +1} - \xi \\
\end{bmatrix}
\tag{2}
$$

通过公式（2）可以将鱼眼图像上所有图像点都映射到球面上，而这些球面点必须满足投影约束。

2. 当 $\xi = 1$，可以将公式简化得到公式（3）， 注意这里是约等于。
$$
h^{-1}(m) \backsim
\begin{bmatrix}
x \\
y \\
f(x,y)
\end{bmatrix},
f(x,y) = \frac{1}{2} - \frac{1}{2}(x^2 + y^2)
\tag{3}
$$
$$


\begin{cases}
m = (x, y, 1), \
p_c = (u_c, v_c), \
\gamma _{i} = f_{i} \eta, \
p_c = \gamma m \\
p = Km=
\begin{bmatrix}
f_1 \eta & f_1 \eta \alpha & u_0 \\
0 & f_2 \eta & v_0 \\
0 & 0 \eta & 1 \\
\end{bmatrix} m
= k(m)
\end{cases}\tag{4}
$$
通过公式（4）展开可以得到公式（5）
$$
\begin{cases}
\gamma_{1} x + u_{0} = u_{c} \\
\gamma_{2} y + v_{0} = v_{c}
\end{cases} \tag{5}
$$
将公式（5）中 $u_0, v_0$忽略（在程序中是将原始图像先减去这个两个值） 代入公式（3）同时将三个公式通分 $\gamma$ 可以得到
$$
h^{-1}(m) \backsim
\begin{bmatrix}
u_c \\
v_c \\
g(x,y)
\end{bmatrix},
g(x,y) = \frac{\gamma}{2} - \frac{1}{2\gamma}(u_{c}^2 + v_{c}^2)
\tag{6}
$$
3. 通过点在平面上构建方程 $\pi ^{T} X=0$，点$N$在$h^{-1}(m)$ 平面上也就是在Fig.4图中的球形镜面表面上。
$$
\begin{cases}
N =
\begin{bmatrix}
n_x & n_y & n_z
\end{bmatrix} \\
h^{-1}(m)^{T} N = 0 \iff n_x u_c + n_y v_c + \frac{a}{2} -b\frac{u_{c}^2 + v_{c}^2}{2} = 0
\end{cases}
\tag{7}
$$
其中$a = \gamma n_z, b = \frac{n_z}{\gamma}$ \
如果有$n$个点的话，可以得到下面公式
$$
P_{4 \times 4} C_{4 \times 1} = 0,P =
\begin{bmatrix}
u_{c1} & v_{c1} & \frac{1}{2} & -\frac{u_{c1}^2 + v_{c1}^2}{2} \\
\vdots & \vdots & \vdots & \vdots \\
u_{cn} & v_{cn} & \frac{1}{2} & -\frac{u_{cn}^2 + v_{cn}^2}{2} \\
\end{bmatrix}
\tag{8}
$$

4. 通过SVD分解$P = U \Sigma V^{T}$，最小二乘法的解就是 $V$向量最后一列，$C = \begin{bmatrix} c_1 & c_2 & c_3 & c_4\end{bmatrix}$。

5. 最后求解 $\gamma$，具体求解方法参考论文，这个地方比较简单。
