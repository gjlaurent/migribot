function[xXZ,zXZ,yYZ,zYZ,xXY,yXY,xXZbutee,zXZbutee,yYZbutee,zYZbutee,xXYbutee,yXYbutee,M1,M2,M3,X,Y,Z]=manipulabilite_plan(poseDepart,poseFinale, pas, yplanXZ, xplanYZ,zplanXY,courseActionneur,poseInit)

    X=[poseDepart(1):(poseFinale(1)-poseDepart(1))/pas:poseFinale(1)];
    Y=[poseDepart(2):(poseFinale(2)-poseDepart(2))/pas:poseFinale(2)];
    Z=[poseDepart(3):(poseFinale(3)-poseDepart(3))/pas:poseFinale(3)];

    % extraction des données pour valeur dans le plan
    xXZ=[];
    zXZ=[];
    xXZbutee=[];
    zXZbutee=[];

    yYZ=[];
    zYZ=[];
    yYZbutee=[];
    zYZbutee=[];

    xXY=[];
    yXY=[];
    xXYbutee=[];
    yXYbutee=[];

    %matrice des indices de manipulabilité
    M1=zeros(length(X),length(Z));
    M2=zeros(length(Y),length(Z));
    M3=zeros(length(X),length(Y));
    for i=1:1:length(X)
        for j=1:1:length(Z)
            [q1,q2,q3,q4]=mgi(X(i),yplanXZ,Z(j),poseDepart(4),courseActionneur,poseInit);
            if q1~=9999 && q2~=9999 && q3~=9999  && q4~=9999 
                xXZ=[xXZ;X(i)];
                zXZ=[zXZ;Z(j)];
                M1(j,i)=indice_manipulabilite(X(i),yplanXZ,Z(j),poseDepart(4),q1,q2,q3,q4);
                
            else
                M1(j,i)=NaN;
            end
        end
    end 
    for i=1:1:length(Y)
        for j=1:1:length(Z)
            [q1,q2,q3,q4]=mgi(xplanYZ,Y(i),Z(j),poseDepart(4),courseActionneur,poseInit);
            if q1~=9999 && q2~=9999 && q3~=9999  && q4~=9999 
                yYZ=[yYZ;Y(i)];
                zYZ=[zYZ;Z(j)];
                M2(j,i)=indice_manipulabilite(xplanYZ,Y(i),Z(j),poseDepart(4),q1,q2,q3,q4);
                 
            else
                M2(j,i)=NaN;
            end
        end
    end
  
    for i=1:1:length(X)
        for j=1:1:length(Y)
            [q1,q2,q3,q4]=mgi(X(i),Y(j),zplanXY,poseDepart(4),courseActionneur,poseInit);
            if q1~=9999 && q2~=9999 && q3~=9999  && q4~=9999 
                xXY=[xXY;X(i)];
                yXY=[yXY;Y(j)];
                M3(j,i)=indice_manipulabilite(X(i),Y(j),zplanXY,poseDepart(4),q1,q2,q3,q4);
            else
                M3(j,i)=NaN;
            end
        end
    end
end
