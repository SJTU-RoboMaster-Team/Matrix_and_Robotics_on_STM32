# Matrix Calculation and Robotics toolbox on STM32

## Matrix

基于arm_math矩阵运算库实现和封装，使用方法类似于Eigen库的矩阵运算。
直接引入Eigen库在STM32上的运行速度不能满足需求：引入Eigen库会导致编译后的执行文件体积严重膨胀，接近用满MCU内存，同时烧写速度非常缓慢，几乎不可用。

### Class Matrix_Base

矩阵基类，用于完成所有矩阵的底层运算和类型转换，重载[],=,+,-,*,/等运算符。

[Note] 使用Matrix_Base类时需特别注意内存管理，对于连续初始化的Matrix_Base对象（例如定义对象数组或在结构体内使用）重新赋值，会进入HardFault_Handler（暂时未查明准确原因，可能和STM32库函数的内存分配方式有关，目前通过模板解决该问题）。

### Class Matrix<_rows,_cols>:Matrix_Base

矩阵模板类，将数据储存的指针重定向到数组。

### Class Vector<_size>:Matrix_Base

向量模板类，将数据储存的指针重定向到数组。重载[]运算符，成员函数包含取模、归一化。

### 其他函数

点乘，转置，求迹

特殊矩阵：0矩阵，1矩阵，单位矩阵，对角矩阵；

3维向量运算：叉乘，与反对称矩阵的相互转换。

### 线性方程Ax=b求解solve(A,b)

使用高斯消元法求解。

### QR分解

Householder变换实现。

### SVD

QR分解迭代实现

### 性能参考

平台：STM32F407IGHx

矩阵乘法运算耗时约为纯数组计算的1.8倍，约为Eigen库的1/40。

6*6矩阵QR分解耗时0.95ms。

## Robotics

参考matlab robotics toolbox实现了基础的机器人学功能，包含位姿变换，DH法建模，运动学，动力学

### 位姿变换

齐次变换矩阵、旋转矩阵、位移向量、欧拉角、旋转向量、四元数、旋量坐标向量间的互相转换。

### struct DH_t

DH法建模结构体

### class Link

单刚体连杆建模，包含

运动学参数：DH参数，关节初始偏移，关节限位，关节类型

动力学参数：质量，质心，惯量张量

### class Serial_Link<_n>

多刚体串联建模，实现正逆运动学，动力学求解

Serial_Link<_n>::fkine(q) -- 正运动学，返回 $T_n^0(q)$

Serial_Link<_n>::fkine(q,k) -- 正运动学，返回 $T_k^0(q)$

Serial_Link<_n>::T(q,k-1) -- 返回刚体k的齐次变换矩阵 $T_k^{k-1}(q)$

Serial_Link<_n>::jacob(q) -- 雅可比矩阵 $J(q)$

Serial_Link<_n>::ikine(T, q, thres, max_iter) -- 逆运动学数值求解（牛顿法迭代），经过奇异位形时通过SVD求伪逆，返回 $q(T,q_0)$

*Serial_Link<_n>::ikine_analytic(T) -- 逆运动学解析求解接口，由用户实现,返回 $q(T)$

Serial_Link<_n>::idyn(q,qv,qa,he) -- 动力学求解（牛顿-欧拉法），返回关节力/力矩 $\tau_i(q,\dot q,\ddot q,h_e)$，( $h_e=[f_e^T,\mu_e^T]^T$，为末端负载)

### 性能参考

平台：STM32F407IGHx

6轴机械臂正运动学计算耗时0.14ms，雅可比矩阵计算耗时0.59ms，逆运动学求解耗时3ms~10ms。

## 例程

example文件夹包含UR，PUMA560，SCARA3种机械臂的*_main.cpp文件和对应的matlab例程，将*_main.cpp中的内容复制到Src文件夹中的main.cpp中编译后即可烧录调试。

## Reference

[1] SJTU ME385-2, Robotics, Y.Ding.

[2] SJTU ME391-1, Numerical analysis (Matlab), Y.Ding

[3] Bruno Siciliano, et al., Robotics: Modelling, Planning and Control, Springer, 2010.

[4] R.Murry, Z.X.Li, and S.Sastry, A Mathematical Introduction to Robotic Manipulation, CRC Press, 1994.

[5] Timothy Sauer, Numerical analysis (2nd ed.), Pearson Education, 2012.

[6] Golub G, Kahan W. Calculating the singular values and pseudo-inverse of a matrix. 1965.

[7] Peter Corke, MATLAB Robotics Toolbox <http://petercorke.com>.

[8] Paul Godfrey (2022). Simple SVD <https://www.mathworks.com/matlabcentral/fileexchange/12674-simple-svd>
