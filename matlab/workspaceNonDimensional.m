%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Code pour la génération de la minpulabilite en translation de la strcutre
% micro. Le code permet de générer un gif avec variation de l'angle.

close all
clear all

folderName ='workspace';
mkdir(folderName);
GifName = 'evolution_manipulabilite_theta.gif';
fileNameGif=strcat(folderName,'/',GifName);
figure(1);
set(gcf,'WindowState','maximized','Color',[1 1 1]);
debut=deg2rad(-0); 
fin=deg2rad(0);
courseactionneur=2;%5.8*4;
poseinit=-1;
colormap jet    
k=0;
%for k=debut:(fin-debut)/20:fin
leg = 5.8;

for col = 1:3
    for lig = 1:3

    courseactionneur=leg*col/4;
    poseinit=-leg*lig/4;
        
    figurename=sprintf('%.2f',k);
    fileFigureName=strcat(folderName,'/',figurename,'.fig');
    taillePolice=20;
    
    PoseI1=[-5.8 -5.8 0 k];
    PoseF1=[5.8 5.8 -6 k];
    %PoseZ = -5.1;
    PoseZ = -4;
    
    
    %[xXZ,zXZ,yYZ,zYZ,xXY,yXY,xXZbutee,zXZbutee,yYZbutee,zYZbutee,xXYbutee,yXYbutee,M1,M2,M3,X,Y,Z]=manipulabilite_plan_det(PoseI1,PoseF1,300,0,0,PoseZ,courseactionneur,poseinit);
    [xXZ,zXZ,yYZ,zYZ,xXY,yXY,xXZbutee,zXZbutee,yYZbutee,zYZbutee,xXYbutee,yXYbutee,M1,M2,M3,X,Y,Z]=manipulabilite_plan(PoseI1,PoseF1,300,0,0,-5.1,courseactionneur,poseinit);
    
    
    %st=sgtitle(['$Manipulability \ index \ for \ the \ angle \ platform \ \theta$'],'Interpreter','latex')
    %st=sgtitle([sprintf('$Manipulability\\ index\\, \\mu (J)\\ for \\, \\theta=%.2f \\ rad$',k)],'FontSize',taillePolice,'Interpreter','latex')%'Angle of the platform ',num2str(k),' rad')
    %st=sgtitle([sprintf('$Inverse\\ conditon\\ number \\, \\kappa \\ for \\, \\theta=%.2f \\ rad$',k)],'FontSize',taillePolice,'Interpreter','latex')%'Angle of the platform ',num2str(k),' rad')
    
   % st=sgtitle([sprintf('Manipulability index for $\\theta=$%.1f deg',k*180/pi)],'FontSize',taillePolice,'Interpreter','latex')%'Angle of the platform ',num2str(k),' rad')
    
    s1=subplot(3,3,col+(lig-1)*3);
    hold on
  
    surf(X/leg,Z/leg,M1,'EdgeColor','none');
    %mesh(X,Z,M1);
    caxis([0 1]);
    planXZ=boundary(xXZ,zXZ,1);
    %altitude=ones(length(xXZ(planXZ)));
    plot(xXZ(planXZ)/leg,zXZ(planXZ)/leg,'Color',[0 0 0.5],'LineWidth',2);
    plot(0,poseinit/leg,'+','Color',[0 0 0.5],'MarkerSize',10,'LineWidth',2);
    plot([-courseactionneur/2 -courseactionneur/2]/leg,[-leg 0]/leg,'--','Color',[0.5 0 0],'LineWidth',1);
    plot([courseactionneur/2 courseactionneur/2]/leg,[-leg 0]/leg,'--','Color',[0.5 0 0],'LineWidth',1);
    %plot3(xXZ(planXZ),zXZ(planXZ),altitude,'Color',[0 0 0.5],'LineWidth',2)
    %plot([0 0],[-6 6],'k--');
    %plot([-6 6],[PoseZ PoseZ],'k--');
    axis([-leg/2 leg/2 -leg 0]/leg);
    grid on;
    axis square
    set(s1,'DataAspectRatio',[1 1 100000],'FontSize',taillePolice-8);
    
    title([sprintf('$\\tilde{c}=%i/4$ and $\\tilde{z}_0=%i/4$',col, lig)],'FontSize',taillePolice,'Interpreter','latex')
    %title([sprintf('$c=%.1f$ and $z_0=%.1f$',courseactionneur, poseinit)],'FontSize',taillePolice,'Interpreter','latex')
    xlabel(['$\tilde{x}_p$ (unitless)'],'FontSize',taillePolice-2,'Interpreter','latex')
    ylabel(['$\tilde{z}_p$ (unitless)' ],'FontSize',taillePolice-2,'Interpreter','latex')
    hold off
    
%     s1=subplot(3,3,col+lig*3);
%     hold on
%     
%     %surf(Y,Z,M2,'EdgeColor','none');
%     %surf(Y,-Z,M2,'EdgeColor','none');
%     %mesh(Y,Z,M2);
%     caxis([0 1])
%     planYZ=boundary(yYZ,zYZ,1);
%     altitude=ones(length(yYZ(planYZ)));
%     plot3(yYZ(planYZ),zYZ(planYZ),altitude,'Color',[0 0 0.5],'LineWidth',2)
%     plot([0 0],[-6 6],'k--');
%     plot([-6 6],[PoseZ PoseZ],'k--');
%     
% 
%     axis square
%     set(s2,'DataAspectRatio',[1 1 100000],'FontSize',taillePolice-8);
%     %title(['$O \mathbf{y_w}\mathbf{z_w}\ with\  x_p=0$'],'FontSize',taillePolice-2,'Interpreter','latex')
%     title(['$x_p=0$ cross section'],'FontSize',taillePolice-2,'Interpreter','latex')
%     xlabel(['$y_p \ (mm)$'],'FontSize',taillePolice-2,'Interpreter','latex')
%     ylabel(['$z_p \ (mm)$'],'FontSize',taillePolice-2,'Interpreter','latex')
%     hold off
%     
%     s3=subplot(1,3,3);
%     hold on
%     surf(X,Y,M3,'EdgeColor','none');
%     %mesh(X,Y,M3);
%     caxis([0 1])
%     planXY=boundary(xXY,yXY,1);
%     altitude=ones(length(xXY(planXY)));
%     %contours ?
%     plot3( xXY(planXY),yXY(planXY),altitude,'Color',[0 0 0.5],'LineWidth',2)
%     plot([0 0],[-6 6],'k--');
%     plot([-6 6],[0 0],'k--');
% 
%     
%     axis square
%     set(s3,'DataAspectRatio',[1 1 10000],'FontSize',taillePolice-8);
%     %title(['$Projection \ in\ O \mathbf{x_w}\mathbf{y_w}\ with\  z_p=-5.1$'],'FontSize',taillePolice-2,'Interpreter','latex')
%     title(['$z_p=-4$ cross section'],'FontSize',taillePolice-2,'Interpreter','latex')
%     xlabel(['$x_p \ (mm)$'],'FontSize',taillePolice-2,'Interpreter','latex')
%     ylabel(['$y_p \ (mm)$'],'FontSize',taillePolice-2,'Interpreter','latex')
%     hold off
    
%     c=colorbar('Location','southoutside','Position',[0.118890765765765,0.076867015340886,0.798141891891892,0]);
%     c.Label.String='$\kappa \ (unitless)$';
%     c.Label.Interpreter='latex'
%     c.Label.FontSize=taillePolice;
% 
%     frame = getframe(1);
%       im = frame2im(frame);
%       [imind,cm] = rgb2ind(im,256);
%       if k == debut;
%           imwrite(imind,cm,fileNameGif,'gif', 'Loopcount',inf);
%       else
%           imwrite(imind,cm,fileNameGif,'gif','WriteMode','append');
%       end
%     savefig(fileFigureName)
%     cla(s1)
%     cla(s2)
%     cla(s3)
%     colorbar('off')
%     cla(st)
end
end

savefig([folderName '/workspace2.fig']);


hold off;
figure(2);
set(gcf,'Color',[1 1 1]);
c=colorbar('Location','southoutside');%,'Position',[0.118890765765765,0.076867015340886,0.798141891891892,0]);
c.Label.String='$\kappa \ (unitless)$';
c.Label.Interpreter='latex'
c.Label.FontSize=taillePolice;
colormap jet;
caxis([0 1]);
savefig([folderName '/jetColorMap.fig']);


% 
% PoseaTester=generation_Pose_espace_travail(PoseI1,PoseF1, 10)
% 
% size(PoseaTester,1)
% Result=[];
% Result1=[];
% Result2=[];
% n=10;
% 
% 
% for i=1:1:size(PoseaTester,1)
% [theta2,manip]=calcul_indice_manip_theta(PoseaTester(i,1),PoseaTester(i,2),PoseaTester(i,3),pi/n);
% Result2=[Result2;theta2 manip];
% end
% 
% 
% 
% figure
% hold on
% for k=1:1:size(PoseaTester,1)
%     plot(Result2(n*(k-1)+k:k*n+k,1),Result2(n*(k-1)+k:k*n+k,2),'k')
%     xticks([-pi/2 -pi/4 0 pi/4 pi/2])
%     xticklabels( {'-\pi/2','-\pi/4','0','\pi/4','\pi/2'})
% end
% plot([-pi/2 pi/2],[0 0],'r','LineWidth',1 )
% ylabel('$ manipulabilte $','Interpreter','latex');
% xlabel('$Angle\ of\  the\ platform\ \theta\  (rad)$','Interpreter','latex');
% hold off
