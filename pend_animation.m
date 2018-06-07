function pend_animation(u,modelParams)
O=[0 0];
axis(gca,'equal');
axis([-0.7 0.7 0.7 0.2]);
grid on;

% Loop for animation

for i=1:length(t)
    %Mass Point
    P=L*[sin(x(i,1))-cos(x(i,1))];
    
    %circle in origin
    O_circ=viscircles(O,0.01);
    
    % pendulum
    pend=line([O(1) P(1)],[O(2) P(2)]);
    
    %ball
    ball=viscircles(P,0.05);
    
    pause(0.001);
    
    if i<length(t)
        delete(pend);
        delete(ball);
        delete(O_circ);
    end
end