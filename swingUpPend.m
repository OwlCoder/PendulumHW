function swingUpPend

%% set params
modelParams=setParams();
assert(modelParams.x_init(1)==0,'change setParams.m');

%% gains for simple pendulum
k_e=3;

%% initial push
x=modelParams.x_init;
if ~all(x)
    u=0.1;
    [xdot,~]=simplePendDynamics(x,u,modelParams);
    x=x+modelParams.dt*xdot;
end

%% energy shaping
while abs(x(1)-pi)>0.5
    E=modelParams.m*modelParams.length*((0.5*modelParams.length*x(2)^2)-...
        modelParams.g*cos(x(1)));
    E_desired=modelParams.m*modelParams.g*modelParams.length;
    E_diff=E-E_desired;
    u=-k_e*x(2)*E_diff;
    [xdot,~]=simplePendDynamics(x,u,modelParams);
    x=x+modelParams.dt*xdot;
end

actual_traj=inf_LQR(x);
end