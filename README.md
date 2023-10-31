## Electrical Train Optimal Control

The electrical motor steady-state equation is  <br/>
$V_a = RI_a+\omega (M_{af}I_f)$  <br/>

$V_a$: drum motor voltage <br/>
R: resistance of the coil  <br/>
$\omega$: angular velocity  <br/>
$M_{af}$: mutual induce stator-rotor  <br/>
$I_f$: rotor current

Motor torque <br/>
$T = (M_{af}I_f)I_a$

From the equations above, with the addition of a square friction-air drag term, the dynamics of the train can be described from the equations below 

$$
\begin{aligned}
& \dot{x}_1=x_2 \\
& \dot{x}_2=-k_1 x_2-k_2 x_2^2+k_3 u
\end{aligned}
$$

where, <br/>
$x_1$: the position of the train <br/>
$x_2$: the velocity of the train <br/>
$u$: the rotor current of the motor

Cost function <br/>
$J=c_1\left(x_1(T)-x_{1 f}\right)^2+c_2 x_2^2(T)+\int_0^T k_4 x_2 u+R u^2 d t$

The theoretical optimal solution is

$$\dot{\mathbf{x}}=\left[\begin{array}{l}x_1 \\ x_2\end{array}\right]=\left[\begin{array}{c}x_2 \\ -k_1 x_2-k_2 x_2^2+k_3 u\end{array}\right], \mathbf{x}(0)=\mathbf{0}$$

 $$H=L+\mathbf{p} \cdot(A \mathbf{x}+B u)=k_4 x_2 u+R u^2+p_1 x_2+p_2\left(-k_1 x_2-k_2 x_2^2+k_3 u\right)$$

 $$\begin{array}{r}\dot{\mathbf{p}}=-\frac{\partial H}{\partial \mathbf{x}} \Rightarrow\left\{\begin{array}{l}\dot{p}_1=0 \\ \dot{p}_2=-k_4 u+k_1 x_2-p_1+2 k_2 x_2 p_2\end{array}\right. \\ \qquad \frac{\partial \varphi}{\partial \mathbf{x}}-\mathbf{p}\left(t_f\right)=\mathbf{0} \Rightarrow\left\{\begin{array}{l}p_1\left(t_f\right)=2 c_1\left(x_1\left(t_f\right)-x_{1 f}\right) \\ p_2\left(t_f\right)=2 c_2 x_2\left(t_f\right)\end{array}\right.\end{array}$$

 $u^*=\frac{-\left(k_3 p_2+k_4 x_2\right)}{2 R}$, $u \in\left[I_{\min }, I_{\max }\right]$

 $x_{1f} = fixed$

Computatinally

For $k_1=0,5, k_2=0,1, k_3=1, c_1=c_2=1000, x_{1 f}=10, R=0,3, k_4=10, I_{\min }=-2, I_{\max }=2, T=10$, the solution $u$ to the optimal control problem is given the graph

![image](https://github.com/steltze/Electrical-Train-Optimal-Control/assets/79508119/139e8b29-f3de-4306-95ab-7869ce81e844)

If the initial state was to mildly change, for example x(0) = [0.4, 0.6]

![image](https://github.com/steltze/Electrical-Train-Optimal-Control/assets/79508119/e9461fe2-cf39-4f52-8695-23046f84c511)

The system develops a steady-state error. In order to mitigate that error, 
