function [Xpred, Ppred] = KF_Predict(Xk,Pk,Q,dt)

% % Constant acceleration state transition function
% A = [1 dt 1/2*dt^2 0 0 0
%      0 1 dt 0 0 0
%      0 0 1 0 0 0
%      0 0 0 1 dt 1/2*dt^2
%      0 0 0 0 1 dt
%      0 0 0 0 0 1];
 
%  % Process noise
% Q = eye(6);
 
 % Constant velocity state transition function
 
 A = [1 dt 0 0
      0 1 0 0
      0 0 1 dt
      0 0 0 1];

%% Prediction (Time Update)

Xpred = A*Xk;
Ppred = A*Pk*transpose(A) + Q;

end

