function modelParams=setParams()

    modelParams.g=1;%9.8
    modelParams.m=1;
    modelParams.length=1;
    modelParams.c=0.0;
    
    modelParams.dt=0.1;
    modelParams.T=10; %N=T/dt
    modelParams.N=modelParams.T/modelParams.dt+1;
    
    modelParams.Qt=diag([1,1]);
    modelParams.Qf=diag([100,100]);
    modelParams.Rt=1;
    
    modelParams.x_init=[pi-0.5;0];
    modelParams.u_lim=1;
    
    modelParams.gen_traj=1;
    modelParams.viz=1;
    
end