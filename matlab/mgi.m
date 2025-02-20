function [q1,q2,q3,q4] = mgi(x,y,z,theta,courseActionneur,poseInit )

%Programme permettant de calculer le modèle géomètrique inverse de la
%structure robotique
%%Définition des paramètres
CA=courseActionneur;
x=x;
y=y;
z=z;
theta=theta;

H=poseInit;
l=5.8;
L=5.8;
d1=1.45+l*cos(asin(H/l));
d2=2.9+l*cos(asin(H/l));
u=1.45;
w=2.9;
v=w/2;

q1=x+u-d1+sqrt(L^2-y^2-(z+v*theta)^2);
q2=y+w-d2+sqrt(L^2-x^2-(z+w*theta)^2);
q3=x-u+d1-sqrt(L^2-y^2-(z+v*theta)^2);
q4=y-w+d2-sqrt(L^2-x^2-(z+w*theta)^2);

 indi=isreal(q1)*isreal(q2)*isreal(q3)*isreal(q4);
 if indi~=0 && (abs(q1)<=CA/2 )&& (abs(q2)<=CA/2)&& (abs(q3)<=CA/2) && (abs(q4)<=CA/2) && z<=0
    
 else 
      q1=9999;
      q2=9999;
      q3=9999;
      q4=9999;
 end
end