function modelParams=setParams()

    modelParams.g=1;%9.8
    modelParams.m=1;
    modelParams.length=10;
    modelParams.c=0;
    
    modelParams.dt=0.01;
    modelParams.T=10; %N=T/dt
    modelParams.N=modelParams.T/modelParams.dt+1;
    
    modelParams.Qt=diag([1,1]);
    modelParams.Qf=diag([10,10]);
    modelParams.Rt=1;
    
    modelParams.x_init=[0;0];
    modelParams.u_lim=1;
    
    modelParams.gen_traj=1;
    modelParams.viz=1;
    
end