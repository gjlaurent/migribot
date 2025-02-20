close all


% Code pour la génération de la minpulabilite en translation de la structure
% micro. Le code permet de générer un gif avec variation de l'angle.

clear all
close all


%% Creation du repertoire d'enregistrement et du GIF
folderName ='workspace_manipulabilite_kappa';
mkdir(folderName);
GifName = 'evolution_kappa_theta.gif';
fileNameGif=strcat(folderName,'/',GifName);
%figure(1)=figure('WindowState','maximized','Color',[1 1 1]);
%figure
debut=deg2rad(-5); 
fin=deg2rad(5);
courseactionneur=4*5.8;
poseinit=0;


figure(1)=figure('WindowState','maximized','Color',[1 1 1]')
colormap jet   
k=0


%% Calcul de la manipulabilite pour differents angles
for k=debut:(fin-debut)/20:fin
    figurename=sprintf('%.2f',k)
    fileFigureName=strcat(folderName,'/',figurename,'.fig');
    taillePolice=24;
    
    PoseI1=[-5.8 -5.8 0 k];
    PoseF1=[5.8 5.8 -5.8 k];
    [xXZ,zXZ,yYZ,zYZ,xXY,yXY,xXZbutee,zXZbutee,yYZbutee,zYZbutee,xXYbutee,yXYbutee,M1,M2,M3,X,Y,Z]=manipulabilite_plan(PoseI1,PoseF1,300,0,0,-5.1,courseactionneur,poseinit);
    %                                                                                                                                  le paramètre a chnager est le troisieme pour affiner l'image
    %st=sgtitle(['$Manipulability \ index \ for \ the \ angle \ platform \ \theta$'],'Interpreter','latex')
    st=sgtitle([sprintf('$Inverse\\ conditon\\ number \\, \\kappa (G)\\ for \\, \\theta=%.2f \\ rad$',k)],'FontSize',taillePolice,'Interpreter','latex')%'Angle of the platform ',num2str(k),' rad')
    %st=sgtitle([sprintf('$Inverse\\ conditon\\ number \\, \\kappa \\ for \\, \\theta=%.2f \\ rad$',k)],'FontSize',taillePolice,'Interpreter','latex')%'Angle of the platform ',num2str(k),' rad')
    
    s1=subplot(1,3,1)
    hold on
  
    surf(X,Z,M1,'EdgeColor','none');
    %mesh(X,Z,M1);
    caxis([0 1])
    planXZ=boundary(xXZ,zXZ,1);
    altitude=ones(length(xXZ(planXZ)));
   plot3(xXZ(planXZ),zXZ(planXZ),altitude,'Color',[0 0 0.5],'LineWidth',2)

    axis square
    set(s1,'DataAspectRatio',[1 1 1],'FontSize',taillePolice-8);
    title(['$Projection \ in\ O \mathbf{x_w}\mathbf{z_w}\ with\  y_p=0$'],'FontSize',taillePolice-2,'Interpreter','latex')
    xlabel(['$x_p \ (mm)$'],'FontSize',taillePolice-2,'Interpreter','latex')
    ylabel(['$z_p \ (mm)$'],'FontSize',taillePolice-2,'Interpreter','latex')
    hold off
    
    s2=subplot(1,3,2)
    hold on
    
    surf(Y,Z,M2,'EdgeColor','none');
    %mesh(Y,Z,M2);
    caxis([0 1])
    planYZ=boundary(yYZ,zYZ,1);
    altitude=ones(length(yYZ(planYZ)));
    plot3(yYZ(planYZ),zYZ(planYZ),altitude,'Color',[0 0 0.5],'LineWidth',2)

    axis square
    set(s2,'DataAspectRatio',[1 1 1],'FontSize',taillePolice-8);
    title(['$Projection \ in\ O \mathbf{y_w}\mathbf{z_w}\ with\  x_p=0$'],'FontSize',taillePolice-2,'Interpreter','latex')
    xlabel(['$y_p \ (mm)$'],'FontSize',taillePolice-2,'Interpreter','latex')
    ylabel(['$z_p \ (mm)$'],'FontSize',taillePolice-2,'Interpreter','latex')
    hold off
    
    s3=subplot(1,3,3)
    
    hold on
    surf(X,Y,M3,'EdgeColor','none');
    %mesh(X,Y,M3);
    caxis([0 1])
    planXY=boundary(xXY,yXY,1);
    altitude=ones(length(xXY(planXY)));
   plot3( xXY(planXY),yXY(planXY),altitude,'Color',[0 0 0.5],'LineWidth',2)

    axis square
    set(s3,'DataAspectRatio',[1 1 1],'FontSize',taillePolice-8);
    title(['$Projection \ in\ O \mathbf{x_w}\mathbf{y_w}\ with\  z_p=-5.1$'],'FontSize',taillePolice-2,'Interpreter','latex')
    xlabel(['$x_p \ (mm)$'],'FontSize',taillePolice-2,'Interpreter','latex')
    ylabel(['$y_p \ (mm)$'],'FontSize',taillePolice-2,'Interpreter','latex')
    hold off
    
    c=colorbar('Location','southoutside','Position',...
    [0.118890765765765,0.176867015340886,0.798141891891892,0.05607813549765]);
    c.Label.String='$\kappa \ (unitless)$';
    c.Label.Interpreter='latex'
    c.Label.FontSize=taillePolice;
    frame = getframe(1);
      im = frame2im(frame);
      [imind,cm] = rgb2ind(im,256);
      if k == debut;
          imwrite(imind,cm,fileNameGif,'gif', 'Loopcount',inf);
      else
          imwrite(imind,cm,fileNameGif,'gif','WriteMode','append');
      end
    savefig(fileFigureName)
    cla(s1)
    cla(s2)
    cla(s3)
    colorbar('off')
    cla(st)
end

