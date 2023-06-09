function J=Jacobian(t)
% Links length
a1=[2; 0.0]; a2=[1.5;0.0]; a3=[1;0.0];
r3=R(t(1)+t(2)+t(3))*a3;
r2=R(t(1)+t(2))*a2+r3;
r1=R(t(1))*a1+r2;
E=[0 -1; 1 0];  % Rotate by pi/2
J=E*[r1 r2 r3]; % Jacobian is a 2 x 3 matrix
end