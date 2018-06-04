function linear_pid
close all;
modelParams=setParams();
kp = -200;
kd = -60;
x=modelParams.x_init;
x_desired=[pi*0.98;0];
for time=1:modelParams.N
    error  = wrapToPi(x(:,time))-x_desired;
    u(time)= [kp,kd]*error;
    [~,xNext]=simplePendDynamics(x(:,time),u(time),modelParams);
    x(:,time+1)=xNext;
end
figure(1);
subplot(2,1,1)
plot([1:length(u)+1],abs(wrapToPi(x(1,:))-x_desired(1)))
subplot(2,1,2)
plot([1:length(u)],u)
end