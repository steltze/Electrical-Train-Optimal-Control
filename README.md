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

For 
$$
\begin{aligned} & k_1=0,5, k_2=0,1, k_3=1, c_1=c_2=1000, x_{1 f}=10, R=0,3, k_4=10 \quad I_{\min }=-2, \quad I_{\max }=2 \text{ and } \mathrm{l} \\ & T=10\end{aligned}
$$
