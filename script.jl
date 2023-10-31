using Symbolics, OptControl
using SciMLBase
using LinearAlgebra
using DifferentialEquations
using Plots

k1 = 0.5
k2 = 0.1
k3 = 1
c1 = 1000
c2 = 1000
x1f = 10
R = 0.3
k4 = 10
T = 10.0
Imin = -2
Imax = 2


###################################
#####  Open-Loop Solution     #####
###################################

@variables t u x[1:2]
f = [x[2], -k1*x[2]-k2*x[2]^2+k3*u]
L = k4*x[2]*u + R*u^2
t0 = [0, 0]
tf = [x1f, nothing]
tspan = (0.0, T)
uub = [Imax]
ulb = [Imin]
N = 50
Φ = c1*(x[1] - x1f)^2 + c2*x[2]^2
saveat = LinRange(0, T, N)
opt1_sol = generateJuMPcodes(L, f, x, u, tspan, t0, tf, Φ, nothing; N=N, u_ub=uub, u_lb=ulb)

x1_opt = opt1_sol[1][1:N]
x2_opt = opt1_sol[1][N+1:2*N]
u_opt = opt1_sol[2]
plot(saveat, [x1_opt, x2_opt, u_opt], label=["x1" "x2" "u"], title="x(0)=[0, 0], State and Optimal Input with respect to time")

cost1 = c1*(x1_opt[N] - x1f)^2 + c2*x2_opt[N]^2 + sum(k4*x2_opt.*u_opt + R*u_opt.*u_opt)*T/N

function state!(dX,X,input,t)
    dX[1] = X[2]
    if t == 0
        dX[2] = -k1*X[2]-k2*X[2]^2+k3*input[1]
    else 
        dX[2] = -k1*X[2]-k2*X[2]^2+k3*input[Int(ceil(t*N/T))]
    end
end

X0 = [0.4; 0.6]

prob = ODEProblem(state!,X0,tspan, u_opt)
opt2_sol = solve(prob, saveat=saveat)

x1_opt_dist = [opt2_sol[i][1] for i in 1:lastindex(opt2_sol)]
x2_opt_dist = [opt2_sol[i][2] for i in 1:lastindex(opt2_sol)]
plot(saveat, [x1_opt_dist, x2_opt_dist], label=["x1" "x2"], title="x(0)=[0.4, 0.6], State and Optimal Input with respect to time")


#####################################################
#####  Calculation of the Closed-Lopp Solution  #####
#####################################################

function dre!(dP, P, (Q, R, A, B), t)
    dP .= -P*A(t) - A(t)'*P + P*B*(R\B')*P - Q
end

function At(t)
    if t == 0
        return [0 1; 0 -k1]
    else 
        return [0 1; 0 -k1-2*k2*x2_opt[Int(ceil(t*N/T))]]
    end
end

B = [0; k3]
Q = [2 0; 0 2]
R = 1

tf = T
P0 = [20 0; 0 20]

prob = ODEProblem(dre!, P0, (T, 0), (Q, R, At, B))

sol = solve(prob, saveat=saveat)

P11 = reverse([sol[i][1] for i in 1:lastindex(sol)])
P12 = reverse([sol[i][2] for i in 1:lastindex(sol)])
P21 = reverse([sol[i][3] for i in 1:lastindex(sol)])
P22 = reverse([sol[i][4] for i in 1:lastindex(sol)])
plot(saveat, [P11, P12, P21, P22], label=["P11" "P12" "P21" "P22"], title="Riccati Matrix P")

################################################################
#####  System response after the feedback correction term  #####
################################################################

function diff!(dY,Y,(P12,P22),t)
    dY[1] = Y[2]
    if t == 0
        dY[2] = (-k1-2*k2*x2_opt[1])*Y[2] - (k3^2)*(P12[1]*Y[1] + P22[1]*Y[2]) 
    else 
        dY[2] = (-k1-2*k2*x2_opt[Int(ceil(t*N/T))])*Y[2] - (k3^2)*(P12[Int(ceil(t*N/T))]*Y[1] + P22[Int(ceil(t*N/T))]*Y[2])
    end
end

Y0 = [0.4; 0.6]
tspan = (0.0, T)
prob = ODEProblem(diff!,Y0,tspan, (P12, P22))
Y_d = solve(prob, saveat=saveat)

Y1 = [Y_d[i][1] for i in 1:lastindex(Y_d)]
Y2 = [Y_d[i][2] for i in 1:lastindex(Y_d)]

plot(saveat, [Y1, Y2], label=["Y1" "Y2"], title="Position and Speed Errors")

v = - k3*(P12.*Y1 + P22.*Y2)
opt_input = v + u_opt

function state!(dX,X,opt_input,t)
    dX[1] = X[2]
    if t == 0
        dX[2] = -k1*X[2]-k2*X[2]^2+k3*opt_input[1]
    else 
        dX[2] = -k1*X[2]-k2*X[2]^2+k3*opt_input[Int(ceil(t*N/T))]
    end
end

X0 = Y0

prob = ODEProblem(state!,X0,tspan, opt_input)
X_opt = solve(prob, saveat=saveat)

x1_final = ([X_opt[i][1] for i in 1:lastindex(X_opt)])
x2_final = ([X_opt[i][2] for i in 1:lastindex(X_opt)])
plot(saveat, [x1_final, x2_final, v, opt_input], label=["x1" "x2" "v" "v+u"], title="x(0)=[0.4, 0.6], State and Optimal Input icluding v, with respect to time")
