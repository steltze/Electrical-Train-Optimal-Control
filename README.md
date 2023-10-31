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

```math
\begin{aligned}
& \dot{\mathbf{p}}=-\frac{\partial H}{\partial \mathbf{x}} \Rightarrow\left\{\begin{array}{l}
\dot{p}_1=0 \\
\dot{p}_2=-k_4 u+k_1 x_2-p_1+2 k_2 x_2 p_2
\end{array}\right. \\
& \frac{\partial \varphi}{\partial \mathbf{x}}-\mathbf{p}\left(t_f\right)=\mathbf{0} \Rightarrow\left\{\begin{array}{l}
p_1\left(t_f\right)=2 c_1\left(x_1\left(t_f\right)-x_{1 f}\right) \\
p_2\left(t_f\right)=2 c_2 x_2\left(t_f\right)
\end{array}\right.
\end{aligned}
```

 $u^*=\frac{-\left(k_3 p_2+k_4 x_2\right)}{2 R}$, $u \in\left[I_{\min }, I_{\max }\right]$

 $x_{1f} = fixed$

Computationally, the open-loop solution is

For $k_1=0,5, k_2=0,1, k_3=1, c_1=c_2=1000, x_{1 f}=10, R=0,3, k_4=10, I_{\min }=-2, I_{\max }=2, T=10$, the solution $u$ to the optimal control problem is given the graph

![image](https://github.com/steltze/Electrical-Train-Optimal-Control/assets/79508119/139e8b29-f3de-4306-95ab-7869ce81e844)

If the initial state was to mildly change, for example x(0) = [0.4, 0.6]

![image](https://github.com/steltze/Electrical-Train-Optimal-Control/assets/79508119/e9461fe2-cf39-4f52-8695-23046f84c511)

The system develops a steady-state error. In order to mitigate that error, we will introduce a feedback correction term (closed-loop solution)

```math
A(t)=\left.\frac{\partial f}{\partial x}\right|_{(x(t), u(t))} \quad B(t)=\left.\frac{\partial f}{\partial u}\right|_{(x(t), u(t))}
```

```math
A(t) = \left[\begin{array}{cc}0 & 1 \\ 0 & -k_1-2 k_2 x_2(t)\end{array}\right]$
```

```math
B(t) = \left[\begin{array}{cc}0 \\ k_3\end{array}\right]
```

```math
\begin{array}{r}J=\mathbf{y}^T\left(t_f\right) S_f \mathbf{y}\left(t_f\right)+\int_0^{t_f}\left(\mathbf{y}^T(t) Q \mathbf{y}(t)+v^2(t)\right) d t \\ S_f=\left[\begin{array}{cc}20 & 0 \\ 0 & 20\end{array}\right],  Q=\left[\begin{array}{cc}2 & 0 \\ 0 & 2\end{array}\right]\end{array}
```

After solving the Riccati equation

```math
\begin{aligned}
& \dot{P}+P A+A^T P-P B B^T P+Q=0 \\
& \Rightarrow\left\{\begin{array}{l}
\dot{P}_{11}=k_3^2 P_{12}^2-2 \\
\dot{P}_{12}=k_3^2 P_{12} P_{22}-P_{11}-a(t) P_{12} \\
\dot{P}_{22}=-k_3^2 P_{22}^2-2-2\left(P_{12}+a(t) P_{22}\right)
\end{array}\right. \\
& v(y)=-k_3\left(P_{12} y_1(t)+P_{22} y_2(t)\right)
\end{aligned}
```

By substituting $u \rightarrow u+v$, the position and speed steady-state errors vanish

![image](https://github.com/steltze/Electrical-Train-Optimal-Control/assets/79508119/055f0dde-cf07-4db4-b514-49318c2775bd)

![image](https://github.com/steltze/Electrical-Train-Optimal-Control/assets/79508119/d12759ba-cd6a-44b9-983e-3e89fdba0493)




