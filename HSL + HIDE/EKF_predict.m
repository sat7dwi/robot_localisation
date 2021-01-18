function [x,P]= EKF_predict(x,P,v,g,Q,dt)
% Inputs:
%   x, P - state and covariance
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   dt - timestep
%
% Outputs: 
%   x, P - predicted state and covariance
 
% <------------------------- TO DO -------------------------->
 

 

% jacobians   
J = [1, 0, -v*dt*sin(g+x(3)); ...
     0, 1,  v*dt*cos(g+x(3)); ...
     0, 0,       1          ]; 
F = [dt*cos(g+x(3)), -v*dt*sin(g+x(3)); ... 
     dt*sin(g+x(3)),  v*dt*cos(g+x(3)); ...
          0,               1          ]; 

% predict state
x = [ x(1) + v*dt*cos(g+x(3)) ; ... 
      x(2) + v*dt*sin(g+x(3)) ; ...
         pi_to_pi(g+x(3))     ];  

% predict covariance
P = J*P*J' + F*Q*F';

 
  

 
