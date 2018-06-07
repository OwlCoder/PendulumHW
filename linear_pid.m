function linear_pid
close all;
modelParams=setParams();
kp = -20;
kd = -4;
x=modelParams.x_init;
x_desired=[pi;0];
for time=1:modelParams.N
    error  = wrapToPi(x(:,time))-x_desired;
    u(time)= [kp,kd]*error;
%     if abs(u(time))>modelParams.u_lim
%         u(time)=sign(u(time))*modelParams.u_lim;
%     end        
    [~,xNext]=simplePendDynamics(x(:,time),u(time),modelParams);
    x(:,time+1)=xNext;
end
figure(1);
subplot(2,2,1)
plot([1:length(u)+1],abs(wrapToPi(x(1,:))-x_desired(1)),'b','LineWidth',2)
title('error versus time');
subplot(2,2,2)
plot([1:length(u)],u,'r','LineWidth',2)
title('torque versus time');
subplot(2,2,3)
plot([1:length(u)+1],x(1,:),'b','LineWidth',2);
title('position versus time');
subplot(2,2,4)
plot([1:length(u)+1],x(2,:),'b','LineWidth',2);
title('velocity versus time');
end