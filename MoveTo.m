% Displacement from t (joint) to f (joint or positon)
% mode = j : Joint space displacement
%      = c : Cartesian straight line minimum joint displacement
%      = r : Cartesian straight line near mid-joint position
% Interpolation with 5<= n <=50 segments
function t=MoveTo(f,t,mode)
  global debug csv;      % Flag 0 or 1
  global Ctrj Jtrj;      % Carteisan path and recoprded Joint trajectory
  step=0.010;
  
  % ===== Straight line in joint space ('j')
  if mode=='j'
    e=f-t; t0=t;
    pl=sqrt(e'*e);
    n = min(max(5,round(0.5+pl/step)),50);  % number of segment
    if pl>0.00001
      e=e/n;              % Split the displacement in n segments
      for i=1:n
        t=t0+i*e;         % Go step by step toward f
        if csv            
          [l,c]=size(Jtrj);
          Jtrj(l+1,:)=t'; % Record the joint position
        end        
        Show_Manip(t);    % Show the manipulator at the joint t position
      end
    end
  % ===== Straight line in Cartesian space ('c' and 'r')
  elseif mode=='c' | mode=='r'
    p0=tool_position(t);  % Initial position
    [l,c]=size(Ctrj);
    Ctrj(l+1,:)=f';       % Set the path to be shown
    e=f-p0;               % Overall displacement
    pl=sqrt(e'*e);
    n = min(max(5,round(0.5+pl/step)),50);  % numbre of segment
    if pl>0.00001
      e=e/n;              % Split the displacement in n segments
      for i=1:n
        p(:,i)=p0+i*e;    % Go step by step toward f
        t=reach(p(:,i),t,mode);  % Compute the joint position
        if csv
          [l,c]=size(Jtrj);
          Jtrj(l+1,:)=t'; % Record the joint position
        end
        Show_Manip(t);    % Show the manipulator at the joint t position
      end
    end
  end
  if debug
    t
  end
end