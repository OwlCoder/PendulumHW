function ft_LQR_conti
close all
modelParams=setParams();
%% linearizing dynamics at (pi,0)
A=[0 1;modelParams.g/modelParams.length -modelParams.c/modelParams.m];
B=[0 ;1];

%% ricatti equation backprop
[t_,S]=ode45(@(t,S)ricattiEqn(t,S,A,B,modelParams.Qt,modelParams.Rt),...
    [modelParams.T 0], reshape(modelParams.Qf,4,1));

%% plugging u back into the dynamcis and solving its ODE
%do interpolation to find S(t)
[t,x]=ode45(@(t,x)dynamics(t,x,A,B,modelParams.Rt...
    ,linearInter(t,flipud(t_),flipud(S))),[0 modelParams.T], modelParams.x_init);


    function xdot=dynamics(t,x,A,B,R,S)
        S_=reshape(S,2,2);
        u=(-R\B'*S_)*(x-[pi;0]);
        xdot=A*(x-[pi;0])+B*u;
    end

    function SInterp=linearInter(t,t_,S)
        if find(t_==t)
            SInterp=S(find(t_==t),:);
        else
            smaller=find(t_<t);
            greater=find(t_>t);
            
            ti=smaller(end);
            tiplus=greater(1);
            
            if abs(ti-t)<abs(tiplus-t)
                SInterp=S(ti,:);
            else
                SInterp=S(tiplus,:);
            end
        end
    end
    
    function dsdt=ricattiEqn(t,S,A,B, Q, R)
        S_=reshape(S,2,2);
        dsdt_=-A'*S_-S_*A+S_*B*(R\B')*S_-Q;
        dsdt=reshape(dsdt_,4,1);
    end

end