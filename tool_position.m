function p=tool_position(t)
  % Links length
  a1=[2; 0.0]; a2=[1.5;0.0]; a3=[1;0.0];
  p=R(t(1))*(a1+R(t(2))*(a2+R(t(3))*a3));
end