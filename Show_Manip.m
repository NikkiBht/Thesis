% Show the manipulator at the t joint position
% Show the path Cartesian path Ctrj
% Show the manipulator in blue (within) or in red (out of joint limits)
function s=Show_Manip(t)
  global a tmin tmax;  % Geometry of the manipulator
  global Ctrj;         % Cartesian path
  
  % Compute the 3 joints and the end-effector position
  tmid=(tmin+tmax)/2;
  r1=R(t(1))*a(:,2);
  r2=r1+R(t(1)+t(2))*a(:,3);
  r3=r2+R(t(1)+t(2)+t(3))*a(:,4);
  mj = [0.0 0.0; r1'; r2'; r3'];
  
  % Compute the limits of 3 joints
  e = [0.5; 0.0];       % Half unit vector along x axis as limit
  % Limits of joint 1
  e1 = R(tmin(1))*e;    % Limits of joint 1
  e2 = R(tmid(1))*e;
  e3 = R(tmax(1))*e;
  EA = [e1'; 0 0; e3']; % min and max limits
  EB = [0 0; e2'];      % Mid limit
  % Limits of joint 2
  f1 = R(t(1)+tmin(2))*e+r1;       % Limits of joint 2
  f2 = R(t(1)+tmid(2))*e+r1;
  f3 = R(t(1)+tmax(2))*e+r1;
  FA = [f1'; r1'; f3']; % min and max limits
  FB = [r1'; f2'];      % Mid limit
  % Limits of joint 3
  g1 = R(t(1)+t(2)+tmin(3))*e+r2;  % Limits of joint 3
  g2 = R(t(1)+t(2)+tmid(3))*e+r2;
  g3 = R(t(1)+t(2)+tmax(3))*e+r2;
  GA = [g1'; r2'; g3']; % min and max limits
  GB = [r2'; g2'];      % Mid limit
  
  % Show the manipualtor in blue or red based in/out of limits
  [l,c]=size(Ctrj);
  mcolor='-ob';         % Show it in blue by default (in joint limits)
  s=Exceed_limits(t);
  if s>0   
      mcolor='-or';     % Show it in red (out of joint limits)
  end
  
  % Swith off and redraw the manipulator and path
  hold off;                          % Restart with next plot
  plot(mj(:,1),mj(:,2),mcolor,'LineWidth',2);  % Show the manipulator
  hold on; grid on;                  % Continue with next plot
  plot(mj(:,1),mj(:,2),'or','MarkerSize',5,'MarkerFaceColor','r');
  plot(EA(:,1),EA(:,2),'-r');        % Show limits joint 1
  plot(EB(:,1),EB(:,2),'-g');        % Show mid joint 1
  plot(FA(:,1),FA(:,2),'-r');        % Show limits joint 2
  plot(FB(:,1),FB(:,2),'-g');        % Show mid joint 2
  plot(GA(:,1),GA(:,2),'-r');        % Show limits joint 3
  plot(GB(:,1),GB(:,2),'-g');        % Show mid joint 3
  if l>1
    plot(Ctrj(:,1),Ctrj(:,2),'--k'); % Show the Cartesian trajectory
  end
  axis([-3 3 -1.5 4.5]);             % Frame the workspace
  pbaspect([1 1 1]);                 % Equal x and y axes
  pause(0.1);
end