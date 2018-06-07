function actual_traj=inf_LQR(x)
close all

%% set params
modelParams=setParams();
if nargin==1
    modelParams.x_init=x;
end
%% linearize over unstable fixed point
A=[0 1;modelParams.g/modelParams.length -modelParams.c/modelParams.m];
B=[0 ;1/(modelParams.m*modelParams.length^2)];

%% LQR for infinite horizon
[K,S] = lqr(A,B,modelParams.Qt,modelParams.Rt);

%% dynamics-forward propagation
x=zeros(2,modelParams.N);
x(:,1)=modelParams.x_init;
u=zeros(modelParams.N,1);
for dyn_iter=1:modelParams.N-1
    xdiff=[pi;0]-x(:,dyn_iter);
    u(dyn_iter)=K*xdiff;
%     if abs(u(dyn_iter))>modelParams.u_lim
%         u(dyn_iter)=sign(u(dyn_iter))*modelParams.u_lim;
%     end
    xdot=-A*xdiff+B*u(dyn_iter);
    x(:,dyn_iter+1)=x(:,dyn_iter)+xdot*modelParams.dt;
end
actual_traj.x=x;
actual_traj.u=u;
plot_figs(actual_traj.x,actual_traj.u);
end