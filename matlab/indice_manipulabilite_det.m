function mu = indice_manipulabilite_det(x,y,z,theta,q1,q2,q3,q4)

H=-2.35;
l=5.8;
d1=1.45+l*cos(asin(H/l));
d2=2.9+l*cos(asin(H/l));
u=1.45;
w=2.9;
v=w/2;

ouverture_initiale=0.03; %ouverture de la pince pour theta=0
n=2.37;%longueur normale des doigts
ouverture=ouverture_initiale+2*n*theta;
    
%     A=[x+u-d1-q1 y z+v*theta v*(z+v*theta);
%        x y+w-d2-q2 z+w*theta w*(z+w*theta);
%        x-u+d1-q3 y z+v*theta v*(z+v*theta);
%        x y-w+d2-q4 z+w*theta w*(z+w*theta)];
      A=[x+u-d1-q1 y z+v*(ouverture/(2*n)) (v/2*n)*(z+v*(ouverture/(2*n)));
       x y+w-d2-q2 z+w*(ouverture/(2*n)) (w/2*n)*(z+w*(ouverture/(2*n)));
       x-u+d1-q3 y z+v*(ouverture/(2*n)) (v/2*n)*(z+v*(ouverture/(2*n)));
       x y-w+d2-q4 z+w*(ouverture/(2*n)) (w/2*n)*(z+w*(ouverture/(2*n)))];
    
     B=[x+u-d1-q1 0 0 0;
        0 y+w-d2-q2 0 0;
        0 0 x-u+d1-q3 0;
        0 0 0 y-w+d2-q4];
    
    if rank(A)==4
        invA=inv(A);
        J=invA*B;
        %J=J(4:4,1:4);
        %mu=J*transpose(J);
        %mu=1/sqrt(mu);
         mu=sqrt(det(J*transpose(J)));
         mu=1/(1+mu);
         %mu=exp(-mu);
         %mu=sqrt(0.25*trace(J*transpose(J)));
    else
        mu=0;
    end
end
