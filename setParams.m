function modelParams=setParams()

    modelParams.g=9.8;%9.8
    modelParams.m=0.5;
    modelParams.length=1;
    modelParams.c=2.5;
    
    modelParams.dt=0.01;
    modelParams.T=4; %N=T/dt
    modelParams.N=modelParams.T/modelParams.dt+1;
    
    modelParams.Qt=diag([1,1]);
    modelParams.Qf=diag([100,100]);
    modelParams.Rt=10;
    
    modelParams.x_init=[0;0];
    modelParams.u_lim=5;
    
    modelParams.gen_traj=1;
    modelParams.viz=1;
    
end