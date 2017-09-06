### 标定

根据张氏标定法，令标定景物平面为$Z=0$，那么有
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
\tag{3}
$$
不考虑齐次因子$s$，上式可以转换为如下等式
$$
\begin{bmatrix} u\\v \end{bmatrix}=
\begin{bmatrix}
\frac{r_{11}X+r_{12}Y+r_{14}}{r_{31}X+r_{32}Y+r_{34}}f_x+c_x\\
\frac{r_{21}X+r_{22}Y+r_{24}}{r_{31}X+r_{32}Y+r_{34}}f_y+c_y
\end{bmatrix}
\tag{4}
$$
我们把图像点的坐标系原点变换到图像中心，假设$(u_0,v_0)^T$为图像中心，则根据式$(4)$，把透视模型$(3)$改写为
$$
s\begin{bmatrix} u-u_0\\v-v_0\\1 \end{bmatrix}=
\begin{bmatrix}
f_x r_{11} & f_x r_{12} & f_x r_{14} \\
f_y r_{21} & f_y r_{22} & f_y r_{24} \\
r_{31} & r_{32} & r_{34} \\
\end{bmatrix}
\begin{bmatrix} X\\Y\\1 \end{bmatrix}
\tag{5}
$$
可以得点$\begin{bmatrix}u-u_0&v-v_0&1\end{bmatrix}^T$和点$\begin{bmatrix}X&Y&1\end{bmatrix}^T$之间的单应矩阵$H$，则有
$$
H=\begin{bmatrix}
h_1 & h_2 & h_3 \\
h_4 & h_5 & h_6 \\
h_7 & h_8 & 1
\end{bmatrix}
=
\frac{1}{r_{34}}\begin{bmatrix}
f_x r_{11} & f_x r_{12} & f_x r_{14} \\
f_y r_{21} & f_y r_{22} & f_y r_{24} \\
r_{31} & r_{32} & r_{34} \\
\end{bmatrix}\tag{6}
$$
把$H$代入$(5)$式，改写为
$$
s\begin{bmatrix} u-u_0\\v-v_0\\1 \end{bmatrix}=
r_{34}\begin{bmatrix}
h_1 & h_2 & h_3 \\
h_4 & h_5 & h_6 \\
h_7 & h_8 & 1
\end{bmatrix}
\begin{bmatrix} X\\Y\\1 \end{bmatrix}
\tag{7}
$$


不考虑齐次因子$s$和$r_{34}$，对上式再进行处理，转化为$A\mathbf h=\mathbf b$的形式，$\mathbf h=(h_1,h_2,h_3,h_4,h_5,h_6,h_7,h_8)^T$有
$$
\begin{bmatrix}
X & Y & 1 & 0 & 0 & 0 & -X(u-u_0) & -Y(u-u_0)\\
0 & 0 & 0 & X & Y & 1 & -X(v-v_0) & -Y(v-v_0)\\
\end{bmatrix}
\begin{bmatrix}
h_1\\h_2\\h_3\\h_4\\h_5\\h_6\\h_7\\h_8
\end{bmatrix}=
\begin{bmatrix}
u-u_0\\
v-v_0
\end{bmatrix}\tag{8}
$$
根据向量$\mathbf r_i$的性质，有$\mathbf r_1^T\mathbf r_2=0$，$\mathbf r_1^T\mathbf r_1=\mathbf r_2^T\mathbf r_2=1$，有等式
$$
\begin{cases}
r_{11}^2+r_{21}^2+r_{31}^2=r_{12}^2+r_{22}^2+r_{32}^2=1\\
r_{11}r_{12}+r_{21}r_{22}+r_{31}r_{32}=0
\end{cases}\tag{9}
$$
通过$(6)$式可以得到如下关系


$$
\begin{bmatrix}
r_{11} \\ r_{12} \\ r_{14} \\
r_{21} \\ r_{22} \\ r_{24} \\
r_{31} \\ r_{32} \\ r_{34}
\end{bmatrix}
=r_{34}\begin{bmatrix}
\frac{1}{f_x}h_1 \\ \frac{1}{f_x}h_2 \\ \frac{1}{f_x} h_3 \\
\frac{1}{f_y}h_4 \\ \frac{1}{f_y}h_5 \\ \frac{1}{f_y} h_6 \\
h_7 \\ h_8 \\ 1
\end{bmatrix}\tag{10}
$$
代入式$(9)$可得
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
