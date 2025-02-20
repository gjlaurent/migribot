close all;
clear all;

l = 5.8;
stroke = 0:0.01:2*l;
homePosition = -l:0.01:0;

[X,Y] = meshgrid(stroke,homePosition);
Z = X*0;

for col=1:length(stroke)
    c=stroke(col);
    for lig=1:length(homePosition)
        z0=homePosition(lig);
        %if (z0^2 - c*sqrt(l.^2 - z0.^2) - (c/2)^2>=0)
        if (sqrt(l^2 - z0^2)-l>-c/2)% || sqrt(l^2 - z0^2)+l<c/2)
            zmax=0;%-sqrt(z0^2 - c.*sqrt(l^2 - z0^2) - (c/2)^2);
        else
            zmax=-sqrt(z0^2 - c.*sqrt(l^2 - z0^2) - (c/2)^2);
        end
        
        %if (z0^2 + c*sqrt(l.^2 - z0.^2) - (c/2)^2>0)
        if (sqrt(l^2 - z0^2)>c/2)
            zmin=-sqrt(z0^2 + c.*sqrt(l^2 - z0^2) - (c/2)^2);
        else
            zmin=-l;
        end
        Z(lig,col)=zmax-zmin;
    end
end


figure(1);
set(gcf,'WindowState','maximized','Color',[1 1 1]);


surf(X/l,Y/l,Z/l,'EdgeColor','none');
set(gca,'FontSize',12);
colormap parula 
%caxis([0 1]);
view(2)

xlabel('$$\tilde{c}$$ (unitless)','Interpreter','latex','FontSize',20);
ylabel('$$\tilde{z}_{0}$$ (unitless)','Interpreter','latex','FontSize',20);

c=colorbar('Location','southoutside');
c.Label.String='$$\tilde{h}$$ (unitless)';
c.Label.Interpreter='latex'
c.Label.FontSize=20;


axis square


hold on;
X=homePosition*0;
Y=homePosition;
for lig=1:length(homePosition)
    z0=homePosition(lig);
    if (2*(sqrt(l.^2-z0.^2))>l)
        c=2*(sqrt(l.^2-z0.^2));
    else
        c=2*(l-sqrt(l.^2-z0.^2));
    end
    X(lig)=c;
end

plot3(X/l,Y/l,X*0+1,':','Color',[0 0 0],'LineWidth',2);

for lig=1:length(homePosition)
    z0=homePosition(lig);
    
        c=2*(l-sqrt(l.^2-z0.^2));
   
    X(lig)=c;
end

plot3(X/l,Y/l,X*0+1,':','Color',[0 0 0],'LineWidth',2);

text(1, -0.3, 1, '$\textcircled{a}$', 'Interpreter', 'latex','FontSize',28)
text(0.4, -0.8, 1, '$\textcircled{b}$', 'Interpreter', 'latex','FontSize',28)
text(1.8, -0.8, 1, '$\textcircled{c}$', 'Interpreter', 'latex','FontSize',28)
