function [x,P]= update(x,P,z,R,idf,lm)
% correcting the predicted pose using Kalman gain.

% Inputs:
%   x, P -  state and covariance
%   z, R - range-bearing measurements and covariances
%   idf - indecies of each landmark from which measurements have arrived 
 
% Outputs:
%   x, P - updated state and covariance


% <---------------------------- TO DO -------------------------->

n = 3;
l = 1;
X = zeros(n,2*n+1);
Z = zeros(2,2*n+1);
W = ones(2*n+1,1)*0.5/(n+l);
X(:,1) = x;
W(1) = l/(n+l);
C = sqrtm(P);
for i = 1:n
    X(:,i+1) = x + sqrt(n+l)*C(:,i);
    X(:,n+i+1) = x - sqrt(n+l)*C(:,i);
end

for j=1:size(z,2) % loop over each landmark    
  
    for i = 1 : 2*n+1
        Z(:,i) = [   sqrt((X(1,i)-lm(1,idf(j)))^2 + (X(2,i)-lm(2,idf(j)))^2)   ; ...
                   atan2((lm(2,idf(j))-X(2,i)),(lm(1,idf(j))-X(1,i))) - X(3,i) ];  
    end 
    z_hat = Z*W;

    Sigma = zeros(3,2);
    S = zeros(2,2);
    for i = 1 : 2*n+1
        S = S + W(i)*(Z(:,i)-z_hat)*(Z(:,i)-z_hat)';
        Sigma = Sigma + W(i)*(X(:,i)-x)*(Z(:,i)-z_hat)';
    end
    S = S + R;
    z_hat(2) = pi_to_pi(z_hat(2));
    
     
    % Compute Kalman Gain
    K = Sigma/S;     
    
    % Update pose x 
    x = x + K*(z(:,j)-z_hat);
    x(3) = pi_to_pi(x(3));     

    % Update covariance P
    P = P - K*S*K';
    
    [V, D] = eig(P);
    for i = 1:3
        if(D(i,i)<=0)
            D(i,i)=1e-8;
        end
    end
    P = V*D/V;
end        


     
    
    
  

         