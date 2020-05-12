function [Xest, Pest, Sk] = KF_Correct(Xpred,Ppred,Zk,R)
% X = [x vx y vy]'
% Y = [x vx y vy]'

% Measurementt matrix
H = eye(4);

%% Correction (Measurement Update)

Sk = H*Ppred*transpose(H) + R;               % Innovation Covariance
K = Ppred*transpose(H)/Sk;                   % Kamlan Gain
Xest = Xpred + K*(Zk - H*Xpred);            % State estimation
Pest = (eye(4) - K*H)*Ppred;                % Error covariance update
 
end

