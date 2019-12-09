function state_new = getState(dt)
%state: X = (x,y,v,phi,w, a)
TIME = 5;
% we only return the first DIMX values (there might be more...)
DIMX = 5;

persistent s;
if(isempty(s))
  s = 0;
end;


persistent sp;
persistent traj;
persistent state_old;
if(isempty(traj))
  state_old = [8 8 10 pi/6 0 0]';
  points = [8 12 12 8 4  4 8
            8 12 4  8 12 4 8];
  pd = points - (circshift(points',1)');  
  sp = cumsum(sqrt(sum(pd.* pd)));
  sp = sp ./max(sp) * TIME;  
  traj = spline(sp, [points]);
end;

x_old = state_old(1);
y_old = state_old(2);
v_old = state_old(3);
phi_old = state_old(4);


s = s + dt;
if(s > TIME-dt)
  s = 0;  
end;



pos = fnval(traj,s);
x = pos(1);
y = pos(2);

v = sqrt((x-x_old)^2 + (y-y_old)^2)/dt;
a = (v - v_old)/dt;

dxdy = fnval(fnder(traj,1),s);
phi = atan2(dxdy(2), dxdy(1));

w = normalizeAngle(phi - phi_old)/dt;


state_new = [x,y,v,phi,w,a]';
state_old = state_new;
% only take the first DIMX values
state_new = state_new(1:DIMX);
