% Inspired from from
% https://fr.mathworks.com/matlabcentral/fileexchange/5377-learning-the-kalman-filter
% http://publications.lib.chalmers.se/records/fulltext/159412.pdf
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


function [x_upd, P_upd] = fusionKalman2D(x, P, z, H, R, Q, dt)

% == INIT ==
% Linearized process model matrix F, jacobian of the non-linear process f with respect to x
F = [1, 0, -sin(x(3))*(x(4)*dt+x(6)*dt^2/2)-cos(x(3))*(x(4)*x(5)*dt^2/2+2*x(6)*x(5)*dt^3/6), cos(x(3))*dt-sin(x(3))*x(5)*dt^2/2, -sin(x(3))*(x(4)*dt^2/2+2*x(6)*dt^3/6), cos(x(3))*dt^2/2-sin(x(3))*2*x(5)*dt^3/6;
    0, 1, cos(x(3))*(x(4)*dt+x(6)*dt^2/2)+sin(x(3))*(x(4)*x(5)*dt^2/2+2*x(6)*x(5)*dt^3/6), sin(x(3))*dt+cos(x(3))*x(5)*dt^2/2, cos(x(3))*(x(4)*dt^2/2+2*x(6)*dt^3/6), sin(x(3))*dt^2/2+cos(x(3))*2*x(5)*dt^3/6;
    0, 0, 1, 0, sign(x(4))*dt, 0;
    0, 0, 0, 1, 0, dt;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1];

% Jacobian of the non-linear process f with respect to w
GAM = [-sin(x(3))*(x(4)*dt^3/6+x(6)*dt^4/8), cos(x(3))*dt^3/3-sin(x(3))*x(5)^4/8;
    cos(x(3))*(x(4)*dt^3/6+x(6)*dt^4/8), sin(x(3))*dt^3/3+cos(x(3))*x(5)^4/8;
    sin(x(3))*dt^2/2, 0;
    0, dt^2/2;
    dt, 0;
    0, dt];
    

% == PREDICT ==
% Prediction for state vector and covariance
x_pred = F * x;

% GAM = [-sin(x(3))*(x(4)*dt^3/6+x(6)*dt^4/8+x_pred(6)*dt^5/20), cos(x(3))*dt^3/3-sin(x(3))*(x(5)^4/8+x_pred(5)*dt^5/20);
%     cos(x(3))*(x(4)*dt^3/6+x(6)*dt^4/8+x_pred(6)*dt^5/20), sin(x(3))*dt^3/3+cos(x(3))*(x(5)^4/8+x_pred(5)*dt^5/20);
%     sin(x(3))*dt^2/2, 0;
%     0, dt^2/2;
%     dt, 0;
%     0, dt];

P_pred = F * P * F' + GAM * Q * GAM';

% == UPDATE ==
% Compute Kalman gain factor
K = P_pred * H' * (H * P_pred * H' + R)^(-1);
% Correction based on observation
x_upd = x_pred + K * (z - H * x_pred);
P_upd = P_pred - K * H * P_pred;    

end