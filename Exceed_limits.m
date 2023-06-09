% Check if the manipulator is outside its joint limits
% Return 1 to 7 indicating the joints that are out of limits
% Return 0   if within the joint limits
function s = Exceed_limits(t)
  global debug tmin tmax;
  
  s=0;    % Default is within the joint limits
  if t(1)<tmin(1) | tmax(1)<t(1)
      s=s+1;
  end
  if t(2)<tmin(2) | tmax(2)<t(2)
      s=s+2;
  end
  if t(3)<tmin(3) | tmax(3)<t(3)
      s=s+4;
  end

  if debug & s>0
      z=sprintf('Exceed_limits: %d',s);
      disp(z);
  end
end