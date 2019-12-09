function uebung5
%measurement z = (x,y,phi)
%state: X = (x,y,v,phi,w)
close all;

%history size: the number of measurements, estimations,... to store for
%visualization
global HIST_SIZE;
HIST_SIZE = 100;

%dimensions of state vector and measurement vector
DIMX = 5;
DIMZ = 3;

%the time betweeen to measurements we get
T = 0.02;

% Kalman-Filter variables
x_pred = [];                    % predicted state
x_est  = [];         % state estimation x_est = (x,y,phi,v,w)
P_pred = [];                    % predicted error covariance
P_est  = [];                    % estimated error covariance

X_Hist = [];                    % state history
X_est_Hist = [];                % estimation history
Z_Hist = [];                    % measurement history

% performance variables
NEES_Hist = zeros(1,HIST_SIZE);     % NEES - history
NIS_Hist = zeros(1,HIST_SIZE);      % NIS - history


% measurement noise = ?
R = [0.1 0 0;
     0 0.1 0;
     0 0 0.01];

% linear measurement matrix = ?
H = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 0 1 0];

%process noise = ?
sigma2a = [10000];
sigma2wa = [5000];

while (1)  % simulation loop
  if (isempty(x_est))
      loops = 2;
  else
      loops = 1 ;
  end;
  
  for(i=1:loops)
    x_true = getState(T);
    % update state history
    X_Hist = addHistory(X_Hist, x_true);
    z = getMeasurement(x_true(1:4));
    % update measurement history
    Z_Hist = addHistory(Z_Hist, z);
  end;
  
  % filter initialization (once per simulation)
  if (isempty(x_est))    
    x_est = H' * z;
    P_est = 100 * eye(5);
    continue;
  end

  % use process modell: CV/CW (VC/VO)
  psi = x_est(4);
  x_pred = unfuck_angle(f(x_est, T), 4);
  
  
  Q = [0.25 * T^4 * cos(psi)^2,          0.25 * T^4 * sin(psi) * cos(psi), 0.5 * T^3 * cos(psi), 0,          0;
       0.25 * T^4 * sin(psi) * cos(psi), 0.25 * T^4 * sin(psi)^2,            0.5 * T^3 * sin(psi), 0,          0;
       0.5 * T^3 * cos(psi),             0.5 * T^3 * sin(psi),             T^2                 , 0,          0;
       0,                                0,                                0,                    0.25 * T^4, 0.5 * T^3;
       0,                                0,                                0,                    0.5 * T^3,  T^2];
   Q(1:3, 1:3) = sigma2a * Q(1:3, 1:3);
   Q(4:5, 4:5) = sigma2wa * Q(4:5, 4:5);
   
   deriv = f_x(x_est, T);
   P_pred = deriv * P_est * deriv' + Q;
   z_pred = H * x_pred;
   S = H * P_pred * H' + R;
   K = P_pred * H' / S;
  
   z_diff = unfuck_angle(z - z_pred, 3);
   
   x_est = unfuck_angle(x_pred + K * z_diff, 4);
   
   stuff = eye(5) - K * H;
   P_est = stuff * P_pred * stuff' + K * R * K';

  X_est_Hist = addHistory(X_est_Hist, x_est);
  
  %-------------NEES---------
  x_error = unfuck_angle(x_true - x_est, 4);
  eps = x_error' / P_est * x_error;
  NEES_Hist = addHistory(NEES_Hist, eps);

  % degrees of fisnan(eig)reedom for NEES: dimension of x_est
  DOF_NEES = size(x_est,1);
  
  %one-sided confidence interval of 95% from the chi-square-distribution
  P95_NEES = chi2inv(0.95,DOF_NEES);

   %------------NIS-----------
  z_error = unfuck_angle(z - z_pred, 3);
  eps_nis = z_error' / S * z_error;
  NIS_Hist = addHistory(NIS_Hist, eps_nis);
  
  % degrees of freedom for NIS: dimension of z
  DOF_NIS = size(z,1);
  P95_NIS = chi2inv(0.95,DOF_NIS);
  
  %=======================================================================%
  %     Visualisation
  %=======================================================================%
  
  % true and estimated trajectory
  subplot(2,2,1)
  plot(x_true(1),x_true(2),'b.','MarkerSize',25);
  hold on;
  vFactor = 0.5;
  plotDirVec(x_true(1),x_true(2),x_true(4),x_true(3)*vFactor,'gr');
  plot(X_Hist(1,:),X_Hist(2,:),'r-');
  
  if(~isempty(x_est))
    plot(x_est(1),x_est(2),'c.','MarkerSize',25);
    plot(X_est_Hist(1,:),X_est_Hist(2,:),'g-');
    plotDirVec(x_est(1),x_est(2),x_est(4),x_est(3)*vFactor,'b');
    axis([0 15 0 15]);

    %draw 3 sigma ellipse
    [evecs, evals] = eig(P_est(1:2, 1:2));
    c = rsmak('circle',50);
    ellipse = fncmb(c,diag(diag(evals).*3));
    ellipse = fncmb(ellipse, evecs);
    ellipse = fncmb(ellipse, x_est(1:2));
    fnplt(ellipse);
  end
  
  hold off;
  
  axis([-5 20 0 15]);
  daspect([1 1 1]);
  grid on
  title 'true trajectory'
  
  % measurement trajectory
  subplot(2,2,2);
  plot(z(1),z(2),'r.','MarkerSize',25);
  plotDirVec(z(1),z(2),z(3),8,'b');
  hold on;
  plot(Z_Hist(1,:),Z_Hist(2,:),'r-', 'LineWidth', 1);
  axis([-5 20 0 15]);
  daspect([1 1 1]);
  hold off;
  grid on
  title 'measurement'
  
  % NEES
  subplot(2,2,3)
  plot(NEES_Hist,'LineWidth',3);
  hold on;
  line([0 size(NEES_Hist,2)], [P95_NEES P95_NEES], 'Color', 'r');
  hold off;
  title 'normalized estimation error squared (NEES)'
  
  % NIS
  subplot(2,2,4)
  plot(NIS_Hist,'LineWidth',3);
  hold on;
  title 'normalized innovation squared (NIS)'
  line([0 size(NIS_Hist,2)], [P95_NIS P95_NIS], 'Color', 'r');
  hold off;
  
  drawnow
  pause(0.05);
end

function Hist = addHistory(Hist, val)
global HIST_SIZE;
if(isempty(Hist))
  for(i=1:HIST_SIZE)
    Hist(:,i) = val;
  end;
end;

Hist = circshift(Hist',1)';
Hist(:,1) = val;



function plotDirVec(x,y,phi,l,col)
%draw direction vector
endpX = x + l * cos(phi);
endpY = y + l * sin(phi);
l = line([x endpX], [y; endpY]);
set(l, 'Color', col);


function state = normalizeVelocity(state, iV,iP)
% helper function: normalizes velocity.
% if velocity is lower than zero, the orientation of the object will be
% changed by pi
if(state(iV)<0)
  state(iV) = -state(iV);
  state(iP) = normalizeAngle(state(iP)+pi);
end;
