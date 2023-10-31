## Electrical Train Optimal Control

The electrical motor steady-state equation is  <br/>
$V_a = RI_a+\omega (M_{af}I_f)$  <br/>

`V_a: drum motor voltage` <br/>
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
$x_1$:
