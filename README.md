# Matrix and Robotics toolbox on STM32

## 概述

本工具库基于arm_math优化数学库，通过C++模板实现和封装了矩阵的各种基本运算，在此基础上实现了机器人学的基本功能函数，包括位姿描述、DH建模、运动学和逆向动力学。上海交通大学交龙战队在RoboMaster2023赛季基于本工具库开发了工程机器人的机械臂控制算法。

## 索引

**src**目录下包含了矩阵运算工具库（matrix.cpp/h），机器人学工具库（robotics.cpp/h）和两个工具库使用到的基础数学运算函数。

**demo**目录下包含了一个可在RoboMaster C型开发板（STM32F407）上运行的单片机程序，矩阵运算和机器人学工具库相关代码存放在**demo/toolbox**目录下，**demo/Src/main.cpp**中给出了使用3种不同构型机械臂（PUMA560，UR，SCARA）的例程，**demo/matlab**目录下包含了对应的matlab程序。例程中包括了机械臂的建模方法以及运动学、动力学、雅可比矩阵等计算。

## matrix-矩阵运算库

基于arm_math矩阵运算库实现和封装，对常用的基本运算进行了重载，使用方法类似于Eigen的矩阵运算。

### 为什么要专门编写一个矩阵运算工具库？

目前STM32平台缺少高效且易用的开源矩阵库，直接使用arm_math的库函数会使得代码较为臃肿，对于机械臂控制这样包含大量矩阵运算的程序来说非常影响编程体验和调试效率。而在STM32上直接引入Eigen库的运行速度不能满足需求，且会编译后的执行文件体积非常大。

### 性能参考

本工具库直接使用arm_math矩阵运算库函数进行封装，矩阵乘法运算耗时约为纯数组计算的1.6倍，约为Eigen库的1/40。

## robotics-机器人学工具库

参考matlab robotics toolbox (Peter Corke)实现了基础的机器人学功能函数，包含位姿变换，DH建模，运动学，雅可比矩阵，动力学等。使用方法也可大致参考robotics toolbox的说明文档。

### 位姿变换

齐次变换矩阵、旋转矩阵、位移向量、欧拉角、旋转向量、四元数、旋量坐标向量间的互相转换。

### struct DH_t

DH建模参数结构体

### class Link

单刚体连杆建模，包含：
- 运动学参数：DH参数，关节初始偏移，关节限位，关节类型  
- 动力学参数：质量，质心，惯量张量

### class Serial_Link<_n>

串联多刚体建模，实现正逆运动学，动力学求解等。

- Serial_Link<_n>::fkine(q) -- 正运动学，返回 $T_n^0(q)$
- Serial_Link<_n>::fkine(q,k) -- 正运动学，返回 $T_k^0(q)$
- Serial_Link<_n>::T(q,k-1) -- 返回刚体k的齐次变换矩阵 $T_k^{k-1}(q)$
- Serial_Link<_n>::jacob(q) -- 雅可比矩阵 $J(q)$
- Serial_Link<_n>::ikine(T, q, tol, max_iter) -- 逆运动学数值求解（牛顿法迭代），返回 $q(T,q_0)$
- *Serial_Link<_n>::ikine_analytic(T) -- 逆运动学解析求解接口，由用户实现,返回 $q(T)$
- Serial_Link<_n>::rne(q,qv,qa,he) -- 逆动力学求解（牛顿-欧拉法），返回关节力/力矩 $\tau_i(q,\dot q,\ddot q,h_e)$，( $h_e=[f_e^T,\mu_e^T]^T$，为末端负载)

### 性能参考

6轴机械臂正运动学计算耗时0.14ms，雅可比矩阵计算耗时0.59ms，逆运动学求解耗时3~20ms。

## 补充说明

1. 本工具库最主要的用途是实现机械臂控制，由于自制的机械臂与产品级的机械臂不同，缺少完善的安全保护功能，因此**在编写程序时要尤其关注安全措施例如紧集停止、断电等，在实际对机械臂进行控制前应确保每个电机在发生意外状况时能通过软件或硬件方式断电**。
2. 要实现机械臂的控制，在此工具库的基础上仍需要实现逆运动学的解析解、奇异位形处理、姿态插值、轨迹规划等功能。不同构型、机构参数的机械臂在这方面的处理不完全相同。如果处理不当可能存在一定危险性（例如奇异位形下电机突然高速运行）。
3. 机器人学工具库中的逆运动学使用迭代求解，求解结果数值不稳定且效率较低，推荐自行实现解析逆解。
4. 工具库作者本人非计算机、软件工程相关专业，代码编写方面没有经过系统的训练，同时用C++在STM32上开发目前也不算特别成熟，程序和代码规范可能有些不完善的地方，欢迎指出和提出修改意见。

## 参考资料

[1] SJTU ME385-2, Robotics, Y.Ding.  
[2] SJTU ME391-1, Numerical analysis, Y.Ding.  
[3] Bruno Siciliano, et al., Robotics: Modelling, Planning and Control, Springer, 2010.  
[4] R.Murry, Z.X.Li, and S.Sastry, A Mathematical Introduction to Robotic Manipulation, CRC Press, 1994.  
[5] Timothy Sauer, Numerical analysis (2nd ed.), Pearson Education, 2012.  
[6] Peter Corke, MATLAB Robotics Toolbox <http://petercorke.com>.  
