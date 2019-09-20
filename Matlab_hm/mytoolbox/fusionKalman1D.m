% Inspired from from
% https://fr.mathworks.com/matlabcentral/fileexchange/5377-learning-the-kalman-filter
%
% SYSTEM DYNAMICS:
% x = Fx + Gu + w  meaning the state vector x evolves during one time
%                  step by premultiplying by the "state transition
%                  matrix" F. There is optionally (if nonzero) an input
%                  vector u which affects the state linearly, and this
%                  linear effect on the state is represented by
%                  premultiplying by the "input matrix" G. There is also
%                  gaussian process noise w.
% z = Hx + v       meaning the observation vector z is a linear function
%                  of the state vector, and this linear relationship is
%                  represented by premultiplication by "observation
%                  matrix" H. There is also gaussian measurement
%                  noise v.
% where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
%       v ~ N(0,R) meaning v is gaussian noise with covariance R
%
%
% VECTOR VARIABLES:
% x (nx1) = state vector estimate. In the input struct, this is the
%       "a priori" state estimate (prior to the addition of the
%       information from the new observation). In the output struct,
%       this is the "a posteriori" state estimate (after the new
%       measurement information is included).
% z (mx1) = observation vector
% u (px1) = input control vector, optional (defaults to zero).
% w (nx1) = process noise
% v (mx1) = measurement noise
%
% MATRIX VARIABLES:
% F (nxn) = state transition matrix.
% P (nxn) = covariance of the state vector estimate. In the input struct,
%       this is "a priori," and in the output it is "a posteriori."
%       (required unless autoinitializing as described below).
% G (nxp) = input matrix, optional (defaults to zero).
% Q (nxn) = process noise covariance (defaults to zero).
% R (mxm) = measurement noise covariance (required).
% H (mxn) = observation matrix (defaults to identity).
% K (nxm) = Kalman filter gain
%
% NORMAL OPERATION:
% (1) define all state definition fields: F,G,H,Q,R
% (2) define intial state estimate: x,P
% (3) obtain observation and control vectors: z,u
% (4) call the filter to obtain updated state estimate: x,P
% (5) return to step (3) and repeat


function [x, P] = fusionKalman1D(x, P, u, z, s_acc, s_pos, deltat)
% Local variables
F = [1 deltat; 0 1]; % transition matrix
G = [deltat^2/2; deltat]; % input matrix
H = [1 0]; % measurement matrix
% s_acc = 0.2; % acceleration noise (m/sec^2)
% s_pos = 2.5; % position measurement noise (m)
Q = s_acc^2 * [deltat^4/4 deltat^3/2; deltat^3/2 deltat^2]; % process noise covariance matrix
R = s_pos^2; % observation noise covariance

% == PREDICT ==
% Prediction for state vector and covariance
x = F * x + G * u;
P = F * P * F' + Q;

% == UPDATE ==
% Compute Kalman gain factor
K = P * H' * (H * P * H' + R)^(-1);
% Correction based on observation
x = x + K * (z - H * x);
P = P - K * H * P;

end
