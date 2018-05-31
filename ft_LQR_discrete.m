function ft_LQR_discrete()
close all
modelParams=setParams();
%% linearizing dynamics at (pi,0)
A=[0 1;modelParams.g/modelParams.length -modelParams.c/modelParams.m];
B=[0 ;1];

%% ricatti equations-backprop
S=zeros(2,2,modelParams.N);
K=zeros(modelParams.N,2);
S(:,:,end)=modelParams.Qf;

for i=modelParams.N-1:-1:1
    K(i,:)=inv(modelParams.Rt+B'*S(:,:,i+1)*B)*B'*S(:,:,i+1)*A;
    S(:,:,i)=modelParams.Qt+K(i,:)'*modelParams.Rt*K(i,:)+(A-B*K(i,:))'...
        *S(:,:,i+1)*(A-B*K(i,:));
end

%% dynamics-forward propagation
x=zeros(2,modelParams.N);
x(:,1)=modelParams.x_init;
u=zeros(modelParams.N,1);
for dyn_iter=1:modelParams.N-1
    xdiff=x(:,dyn_iter)-[pi;0];
    xdiff(1)=wrapToPi(xdiff(1));
    u(dyn_iter)=-K(dyn_iter,:)*xdiff;
    xdot=A*xdiff+B*u(dyn_iter);
    x(:,dyn_iter+1)=x(:,dyn_iter)+xdot*modelParams.dt;
end
end