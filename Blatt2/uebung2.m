function uebung2
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

X_Hist = [];                    % state history
X_est_Hist = [];                % estimation history
Z_Hist = [];                    % measurement history

% performance variables
NEES_Hist = zeros(1,HIST_SIZE);     % NEES - history
NIS_Hist = zeros(1,HIST_SIZE);      % NIS - history



% process noise = ?
% measurement noise = ?

R = [1, 0;
     0, 1];

Q = 15000 * [0.25 * T^4, 0, 0.5 * T^3, 0;
     0, 0.25 * T^4, 0, 0.5 * T^3;
     0.5 * T^3, 0, T^2, 0;
     0, 0.5 * T^3, 0, T^2];

F = [1, 0, T, 0;
     0, 1, 0, T;
     0, 0, 1, 0;
     0, 0, 0, 1];
 
 H = [1, 0 ,0 ,0;
      0, 1, 0, 0];

x_true = [];
iteration = 1;
k_pos = zeros(500, 1);
k_v = zeros(500, 1);
P_hist = zeros(500, 4, 4);
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
    x_est = [0 0 0 0]';
    P_est = 10 * eye(4);
  end;
  
  %-----Prediction------
  x_pred = F * x_est;
  P_pred = F * P_est * F' + Q;
  z_pred = H * x_pred;
  S = H * P_pred * H' + R;
  
  %-----Innovation-----
  K = P_pred * H' / S;
  x_est = x_pred + K * (z - z_pred);
  P_est = P_pred - K * S * K';
  
  
  k_pos(iteration) = K(1, 1);
  k_v(iteration) = K(3, 1);
  P_hist(iteration, :, :) = P_est;
  iteration = iteration + 1;
  if iteration >= 500
     figure;
     plot(k_pos);
     hold on;
     plot(k_v);
     legend("pos", "v");
     figure;
     plot(P_hist(:, 1, 1));
     hold on;
     plot(P_hist(:, 2, 2), '--');
     plot(P_hist(:, 3, 3), ':');
     plot(P_hist(:, 4, 4), '-.');
     plot(P_hist(:, 3, 1));
     plot(P_hist(:, 2, 4));
     legend("sigma y", "sigma z", "sigma v_y", "sigma v_z", "cov y", "cov z");
     
     gedafzr
  end
  %================================================================================%
 
  % x_est = ??;

  % update estimation history
  X_est_Hist = addHistory(X_est_Hist, x_est);

  P95_NEES = 9.49;
  P95_NIS = 9.49;
  
  %-------------NEES---------
  x_error = x_true(1:4) - x_est;
  eps = x_error' / P_est * x_error;
  NEES_Hist = addHistory(NEES_Hist, eps);
  
  %------------NIS-----------
  z_error = z - z_pred;
  eps_nis = z_error' / S * z_error;
  NIS_Hist = addHistory(NIS_Hist, eps_nis);
  
  %=======================================================================%
  %     Visualisation
  %=======================================================================%
 
  % true and estimated trajectory
  subplot(2,2,1)
  plot(x_true(1),x_true(2),'b.','MarkerSize',25);  
  hold on;
  plot(X_Hist(1,:),X_Hist(2,:),'r-'); 
  plot(x_est(1),x_est(2),'c.','MarkerSize',25); 
  plot(X_est_Hist(1,:),X_est_Hist(2,:),'g-'); 
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
