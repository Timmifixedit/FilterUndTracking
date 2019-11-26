function uebung3
close all;
%history size: the number of measurements, estimations,... to store for
%visualization
global HIST_SIZE;
HIST_SIZE = 100;

%constant velocity, no acceleration if set to a value > 0
V_CONST = 0;

%the time betweeen to measurements we get
T = 0.02;



% Kalman-Filter variables
x_pred = [];                    % predicted state          
x_est  = [];                    % state estimation x_est = (y,z,vy,vz)
P_pred = [];                    % predicted error covariance
P_est  = [];                    % estimated error covariance
F = [1 0 T 0 0.5*T^2 0;
     0 1 0 T    0  0.5*T^2;
     0 0 1 0    T    0;
     0 0 0 1    0    T;
     0 0 0 0    1    0;
     0 0 0 0    0    1];
 

Q = 1500 *[0.25 * T^4 0 0.5 * T^3 0 0.5 * T^2 0;
     0 0.25 * T^4 0 0.5 * T^3 0 0.5 * T^2;
     0.5 * T^3 0 T^2 0 T 0;
     0 0.5 * T^3 0 T^2 0 T;
     0.5 * T^2 0 T 0 1 0;
     0 0.5 * T^2 0 T 0 1];

H = [1 0 0 0 0 0;
     0 1 0 0 0 0];
R = eye(2);

X_Hist = [];                    % state history
X_est_Hist = [];                % estimation history
Z_Hist = [];                    % measurement history

% performance variables
NEES_Hist = zeros(1,HIST_SIZE);     % NEES - history
NIS_Hist = zeros(1,HIST_SIZE);      % NIS - history



% process noise = ?
% measurement noise = ?

x_true = [];
while (1)   
  % simulation loop  
  x_true = getStateRect(x_true,T, V_CONST);      

  % update state history
  X_Hist = addHistory(X_Hist, x_true);


  z = getMeasurement(x_true);
  % update measurement history  
  Z_Hist = addHistory(Z_Hist, z);

  %=============== insert your kalman filter ======================================%
  % filter initialization (once per simulation)  
  if (isempty(x_est))   
    x_est = [0 0 0 0 0 0]';
    P_est = 10 * eye(6);
  end;
  
  
  
  
  %================================================================================%
  %Prediction
  x_pred = F * x_est;
  P_pred = F * P_est * F' + Q;
  z_pred = H * x_pred;
  S = H * P_pred * H' + R;
  
  %Innovation
  K = P_pred * H' / S;
  x_est = x_pred + K * (z - z_pred);
  P_est = P_pred - K * S * K';
  
  
  % x_est = ??;

  % update estimation history
  X_est_Hist = addHistory(X_est_Hist, x_est);

  P95_NEES = 12.59;
  P95_NIS = 12.59;
  
  
  %NEES-----
  x_error = x_true - x_est;
  eps = x_error' / P_est * x_error;
  NEES_Hist = addHistory(NEES_Hist, eps);
  
  %NIS----
  z_error = z - z_pred;
  eps_nis = z_error' / S * z_error;
  NIS_Hist = addHistory(NIS_Hist, eps_nis);
  
  %=======================================================================%
  %     Visualisation
  %=======================================================================%
 
  % true and estimated trajectory
  pos = H * x_est;
  pos_cov = H * P_est * H';
  x1 = -40:.2:40;
  x2 = 10:.2:80;
  [X1,X2] = meshgrid(x1,x2);
  X = [X1(:) X2(:)];
  y = mvnpdf(X, pos', pos_cov);
  y = reshape(y,length(x2),length(x1));
  
  subplot(2,2,1)
  plot(x_true(1),x_true(2),'b.','MarkerSize',25);  
  hold on;
  plot(X_Hist(1,:),X_Hist(2,:),'r-'); 
  plot(x_est(1),x_est(2),'c.','MarkerSize',25); 
  plot(X_est_Hist(1,:),X_est_Hist(2,:),'g-'); 
  
  contour(x1, x2, y, [0.0001, 0.001, 0.01, 0.05, 0.15, 0.25, 0.35]);
  xlabel('x');
  ylabel('y');
  
  hold off;
  daspect([1 1 1]);
  axis([-40,40,10,80]);
  grid on
  title 'true trajectory'

  % measurement trajectory
  subplot(2,2,2);
  plot(z(1),z(2),'r.','MarkerSize',25);  
  hold on;  
  plot(Z_Hist(1,:),Z_Hist(2,:),'r-', 'LineWidth', 1); 
  daspect([1 1 1]);
  axis([-40,40,10,80]);
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
