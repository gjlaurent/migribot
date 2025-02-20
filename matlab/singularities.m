
u = sym('u', 'real');
v = sym('v', 'real');
w = sym('w', 'real');
q1 = sym('q_1', 'real');
q2 = sym('q_2', 'real');
q3 = sym('q_3', 'real');
q4 = sym('q_4', 'real');

xp =  (q1+q3)/2;%sym('x_p', 'real');
yp = (q2+q4)/2;%sym('y_p', 'real');
zp = sym('z_p', 'real');
th = sym('theta', 'real');


Jx = [ xp+u-q1, yp,      zp+v*th, v*(zp+v*th)
       xp,      yp+w-q2, zp+w*th, w*(zp+w*th)
       xp-u-q3, yp,      zp+v*th, v*(zp+v*th)
       xp,      yp-w-q4, zp+w*th, w*(zp+w*th)];

Jq = [xp+u-q1, 0,       0,       0
      0,       yp+w-q2, 0,       0
      0,       0,       xp-u-q3, 0
      0,       0,       0,       yp-w-q4];
      
   
detJx = simplify(det(Jx))

detJq = simplify(det(Jq))

detinvJqJx = simplify(det(inv(Jq)*Jx))
