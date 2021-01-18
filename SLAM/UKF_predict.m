function [x,P]= predict(x,P,v,g,Q,dt)
% Inputs:
%   x, P - state and covariance
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   dt - timestep
%
% Outputs: 
%   x, P - predicted state and covariance
 
% <------------------------- TO DO -------------------------->
 
F = [dt*cos(g+x(3)), -v*dt*sin(g+x(3)); ... 
     dt*sin(g+x(3)),  v*dt*cos(g+x(3)); ...
          0,               1          ]; 
n = 3;
l = 1;
X = zeros(n,2*n+1);
X2 = zeros(n,2*n+1);
W = ones(2*n+1,1)*0.5/(n+l);
X(:,1) = x;
W(1) = l/(n+l);
C = sqrtm(P);
for i = 1:n
    X(:,i+1) = x + sqrt(n+l)*C(:,i);
    X(:,n+i+1) = x - sqrt(n+l)*C(:,i);
end

for i = 1 : 2*n+1
    X2(:,i) = [ X(1,i) + v*dt*cos(g+X(3,i)) ; ...
                X(2,i) + v*dt*sin(g+X(3,i)) ; ...
                         g+X(3,i)           ];  
end

x = X2*W;
P = zeros(n,n);
for i = 1 : 2*n+1
    P = P + W(i)*(X2(:,i)-x)*(X2(:,i)-x)';
end
x(3) = pi_to_pi(x(3));
P = P + F*Q*F';

 
  

 
