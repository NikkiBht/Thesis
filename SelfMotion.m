% Self displacement from t (joint) to f (joint)
% mode = u : Up Self Motion with t1+ t2- t3+
%      = d : Down Self Motion with t1- t2+ t3-
% s    = 0 in the workspace limit
%      = 1 to 7 as a binary number indicating the joints out of limits
function R =SelfMotion(t,mode)
  global tmax tmin;
  n=50;       % Maximum number of iteration
  e=1;        % Very large initial erreur (1 meter)
  a=0.75;     % Damping factor to avoid overshooting
  
  s = Exceed_limits(t);
  pf=tool_position(t);
  tr  = (tmax-tmin);                     % joint range
  W   = diag([1/tr(1) 1/tr(2) 1/tr(3)]); % Weighting matrix
  h=W*[+0.1; -0.1; +0.1];
  if mode=='d'
    h = -1.*h;
  end
  
  p=tool_position(t);     % Compute the actual tool position
  while(n>0 & s==0)       % While e is greater than 0.1 mm (and n>0)
    dp=a*(pf-p);          % Required Cartesian displacement
    J=Jacobian(t);        % Evaluate the Jacobian
    dt=pinv(J)*dp;        % Compute the minimum joint rotation
    dt=dt+(diag([1 1 1])-pinv(J)*J)*h; % Add to the joint displacement
    f=norm(dt);
    t=t+a*dt;             % Perform the joint rotation with damping
    p=tool_position(t);   % Compute the resulting tool position
    e=norm(pf-p);         % Error between p et the required pf
    n=n-1;                % Another iteration done
    s=Show_Manip(t);      % Show the manipulator at the joint t position
  end
  R = [t' s];             % Result is a 1x4 row
end

  