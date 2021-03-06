function ft_LQR_discrete()
close all
modelParams=setParams();
%% linearizing dynamics at (pi,0)
A=[0 1;modelParams.g/modelParams.length -modelParams.c/modelParams.m];
B=[0 ;1/(modelParams.m*modelParams.length^2)];
M=[A B;zeros(1,3)]*modelParams.dt;
M=expm(M);
A=M(1:2,1:2);
B=M(1:2,3);
%% ricatti equations-backprop
S=zeros(2,2,modelParams.N);
K=zeros(modelParams.N,2);
S(:,:,end)=modelParams.Qf;

for i=modelParams.N-1:-1:1
    S(:,:,i)=modelParams.Qt+A'*S(:,:,i+1)*A-A'*S(:,:,i+1)*B*inv(modelParams.Rt...
        +B'*S(:,:,i+1)*B)*B'*S(:,:,i+1)*A;
end

for i =1:modelParams.N
    K(i,:)=inv(modelParams.Rt+B'*S(:,:,i)*B)*B'*S(:,:,i)*A;
end
%% dynamics-forward propagation
x=zeros(2,modelParams.N);
x(:,1)=modelParams.x_init;
u=zeros(modelParams.N,1);
for dyn_iter=1:modelParams.N-1
    xdiff=[pi;0]-x(:,dyn_iter);
    u(dyn_iter)=K(dyn_iter,:)*xdiff;
    if abs(u(dyn_iter))>modelParams.u_lim
        u(dyn_iter)=sign(u(dyn_iter))*modelParams.u_lim;
    end
    x(:,dyn_iter+1)=x(:,dyn_iter)+A*xdiff +B*u(dyn_iter);
%     x(:,dyn_iter+1)=A*x(:,dyn_iter) + B*u(dyn_iter);
end
plot_figs(x,u);
end