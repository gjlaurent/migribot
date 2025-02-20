close all;
clear all;

l = 5.8;
c = 1;
homePosition = -l:0.01:0;

X=homePosition;
Z=homePosition*0;
for lig=1:length(homePosition)
    z0=homePosition(lig);
    
    if (sqrt(l^2 - z0^2)-l>-c/2)
        zmax=0;
    else
        zmax=-sqrt(z0^2 - c.*sqrt(l^2 - z0^2) - (c/2)^2);
    end

    if (sqrt(l^2 - z0^2)>c/2)
        zmin=-sqrt(z0^2 + c.*sqrt(l^2 - z0^2) - (c/2)^2);
    else
        zmin=-l;
    end
    
    Y(lig)=zmax-zmin;
end



figure(1);
set(gcf,'Color',[1 1 1]);


plot(X,Y,'LineWidth',2);
set(gca,'FontSize',12);
grid on;
xlabel('$$z_{0}$$ (mm)','Interpreter','latex','FontSize',20);
ylabel('$$h$$ (mm)','Interpreter','latex','FontSize',20);
hold on;


plot([-2.36 -2.36],[0 3.25],'--','LineWidth',2);
hold off;


