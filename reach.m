% Iteratively reach the required pf position from the t joint position
% mode = c : use the minimum joint displacement solution
%      = r : use the nearst mid-joint optimization 
function t=reach(pf,t,mode)
  global tmin tmax;
  n=25;       % Maximum number of iteration
  e=1;        % Very large initial erreur (1 meter)
  a=0.75;     % Damping factor to avoid overshooting

  % Compute the mid-joint position and set the weighting matrix
  if mode=='r'
    tmid= (tmin+tmax)/2;                   % Mid-joint position
    tr  = (tmax-tmin);                     % joint range
    W   = diag([1/tr(1) 1/tr(2) 1/tr(3)]); % Weighting matrix
  end

  p=tool_position(t);     % Compute the actual tool position
  while(n>0 & e>0.00001)  % While e is greater than 0.1 mm (and n>0)
    dp=a*(pf-p);          % Required Cartesian displacement
    J=Jacobian(t);        % Evaluate the Jacobian
    dt=pinv(J)*dp;        % Compute the minimum joint rotation

    % Add the projection on the null space of the gradient h
    if mode=='r'          % Mid-joint objective
      h=W*(tmid-t);       % Gradient of the objective function
      dt=dt+(diag([1 1 1])-pinv(J)*J)*h; % Add to the joint displacement
    end
    
    t=t+a*dt;             % Perform the joint rotation with damping
    p=tool_position(t);   % Compute the resulting tool position
    e=norm(pf-p);         % Error between p et the required pf
    n=n-1;                % Another iteration done 
  end

  if n==0                 % Show  warning with the erreur
     disp('Reach: Maximum number of iteration with erreur ='); e
  end
  
end