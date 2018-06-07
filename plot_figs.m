function plot_figs(x,u)
    figure;
    subplot(2,2,1)
    plot(x(1,:),x(2,:),'b','LineWidth',2);
    title('state-space')
    subplot(2,2,2)
    plot([1:1:length(u)],u,'r','LineWidth',2);
    title('control input');
    subplot(2,2,3)
    plot([1:1:length(x(1,:))],x(1,:),'b','LineWidth',2);
    title('Position')
    subplot(2,2,4)
    plot([1:1:length(x(2,:))],x(2,:),'b','LineWidth',2);
    title('velocity')
end