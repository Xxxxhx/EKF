# Homework 10

11612232 黄旭

## Problem 1

![1558148027374](./Homework 10.assets/1558148027374.png)

### 1. Known landmark location and known associations

Since the location of the landmark is known, the EKF SLAM problem can be transfer to EKF location problem

State vector is 
$$
\mu = \begin{bmatrix} x & y \end{bmatrix}
$$
State transfer matrix is 
$$
A = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}
$$
Control vector is
$$
u = \begin{bmatrix} v_x & v_y \end{bmatrix}
$$
Control transfer matrix is
$$
B = \begin{bmatrix} t & 0 \\ 0 & t \end{bmatrix}
$$
So, the prediction part of EKF location is:
$$
\begin{align}
\bar\mu_t &= A\mu_{t-1} + Bu_t \\
\bar\Sigma_t &= A\Sigma_{t-1}A^T + Q
\end{align}
$$
where 
$$
Q = \begin{bmatrix} \sigma_{vx} & 0 \\ 0 & \sigma_{vy} \end{bmatrix}
$$


For the observation, the observation matrix is


$$
z = \begin{bmatrix} d_1 & d_2 & d_3 & d_4 \end{bmatrix}
$$
the observation state transfer function is
$$
h(\mu) = 
\begin{bmatrix}
1 & 0 \\
1 & 0 \\
0 & -1 \\
0 & -1 \\
\end{bmatrix}
\mu + 
\begin{bmatrix}
-x_0 \\
-y_0 \\
x_0 + a \\
y_0 + a \\
\end{bmatrix}
$$
Thus, the Jacobian matrix is
$$
H = 
\begin{bmatrix}
\frac{\partial h_1(\bar\mu)}{\partial x} & \frac{\partial h_1(\bar\mu)}{\partial y} \\
\frac{\partial h_2(\bar\mu)}{\partial x} & \frac{\partial h_2(\bar\mu)}{\partial y} \\
\frac{\partial h_3(\bar\mu)}{\partial x} & \frac{\partial h_3(\bar\mu)}{\partial y} \\
\frac{\partial h_4(\bar\mu)}{\partial x} & \frac{\partial h_4(\bar\mu)}{\partial y} \\
\end{bmatrix} =
\begin{bmatrix} 1 & 0 \\ 0 & 1 \\ -1 & 0 \\ 0 & -1 \end{bmatrix}
$$
So, the correction part of EKF location is:
$$
\begin{align}
K_t &= \bar\Sigma_tH^T(H\bar\Sigma_tH^T + R)^{-1} \\
\mu_t &= \bar\mu_t + K_t(z - h(\bar\mu_t)) \\
\Sigma_t &= (I - K_tH)\bar\Sigma_t
\end{align}
$$
where
$$
R = 
\begin{bmatrix} 
\sigma_{d1} & 0 & 0 & 0 \\
0 & \sigma_{d2} & 0 & 0 \\
0 & 0 & \sigma_{d3} & 0 \\
0 & 0 & 0 & \sigma_{d4} \\
\end{bmatrix}
$$

### 2. Unknown landmark location and known associations

For SLAM problem, the corresponding state vector is:
$$
\mu = \begin{bmatrix} x & y & x_0 & y_0 & a\end{bmatrix}
$$
State transfer matrix is 
$$
A = 
\begin{bmatrix} 
1 & 0 & 0 & 0 & 0 \\ 
0 & 1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1 \\
\end{bmatrix}
$$
Control vector is
$$
u = \begin{bmatrix} v_x & v_y \end{bmatrix}
$$
Control transfer matrix is
$$
B = 
\begin{bmatrix}
t & 0 \\
0 & t \\
0 & 0 \\
0 & 0 \\
0 & 0 \\
\end{bmatrix}
$$
So, the prediction part of EKF location is:
$$
\begin{align}
\bar\mu_t &= A\mu_{t-1} + Bu_t \\
\bar\Sigma_t &= A\Sigma_{t-1}A^T + Q
\end{align}
$$
where 
$$
Q = \begin{bmatrix} \sigma_{vx} & 0 \\ 0 & \sigma_{vy} \end{bmatrix}
$$
For the observation, the observation matrix is


$$
z = \begin{bmatrix} d_1 & d_2 & d_3 & d_4 \end{bmatrix}
$$
the observation state transfer function is
$$
h(\mu) = 
\begin{bmatrix}
1 & 0 & -1 & 0 & 0 \\
0 & 1 & 0 & -1 & 0 \\
-1 & 0 & 1 & 0 & 1 \\
0 & -1 & 0 & 1 & 1 \\
\end{bmatrix}
\mu
$$
Thus, the Jacobian matrix is
$$
H = 
\begin{bmatrix}
\frac{\partial h_1(\bar\mu)}{\partial x} & \frac{\partial h_1(\bar\mu)}{\partial y} & \frac{\partial h_1(\bar\mu)}{\partial x_0} & \frac{\partial h_1(\bar\mu)}{\partial y_0} & \frac{\partial h_1(\bar\mu)}{\partial a} \\
\frac{\partial h_2(\bar\mu)}{\partial x} & \frac{\partial h_2(\bar\mu)}{\partial y} & \frac{\partial h_2(\bar\mu)}{\partial x_0} & \frac{\partial h_2(\bar\mu)}{\partial y_0} & \frac{\partial h_2(\bar\mu)}{\partial a} \\
\frac{\partial h_3(\bar\mu)}{\partial x} & \frac{\partial h_3(\bar\mu)}{\partial y} & \frac{\partial h_3(\bar\mu)}{\partial x_0} & \frac{\partial h_3(\bar\mu)}{\partial y_0} & \frac{\partial h_3(\bar\mu)}{\partial a} \\
\frac{\partial h_4(\bar\mu)}{\partial x} & \frac{\partial h_4(\bar\mu)}{\partial y} & \frac{\partial h_4(\bar\mu)}{\partial x_0} & \frac{\partial h_4(\bar\mu)}{\partial y_0} & \frac{\partial h_4(\bar\mu)}{\partial a} \\
\end{bmatrix} =
\begin{bmatrix}
1 & 0 & -1 & 0 & 0 \\
0 & 1 & 0 & -1 & 0 \\
-1 & 0 & 1 & 0 & 1 \\
0 & -1 & 0 & 1 & 1 \\
\end{bmatrix}
$$
So, the correction part of EKF location is:
$$
\begin{align}
K_t &= \bar\Sigma_tH^T(H\bar\Sigma_tH^T + R)^{-1} \\
\mu_t &= \bar\mu_t + K_t(z - h(\bar\mu_t)) \\
\Sigma_t &= (I - K_tH)\bar\Sigma_t
\end{align}
$$
where
$$
R = 
\begin{bmatrix} 
\sigma_{d1} & 0 & 0 & 0 \\
0 & \sigma_{d2} & 0 & 0 \\
0 & 0 & \sigma_{d3} & 0 \\
0 & 0 & 0 & \sigma_{d4} \\
\end{bmatrix}
$$

### 3. Unknown landmark location and unknown associations





## Problem 2

### 1. Known landmark location and known associations



### 2. Unknown landmark location and known associations

### 3. Unknown landmark location and unknown associations





