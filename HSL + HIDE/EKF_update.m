function [x,P]= EKF_update(x,P,z,R,idf,lm)
% correcting the predicted pose using Kalman gain.

% Inputs:
%   x, P -  state and covariance
%   z, R - range-bearing measurements and covariances
%   idf - indecies of each landmark from which measurements have arrived 
 
% Outputs:
%   x, P - updated state and covariance


% <---------------------------- TO DO -------------------------->

for i=1:size(z,2) % loop over each landmark    
     
%      Compute expected measurement and Jacobian of measurement model representeda as H       
     z_hat = [       sqrt((x(1)-lm(1,idf(i)))^2 + (x(2)-lm(2,idf(i)))^2)       ; ...
               pi_to_pi(atan2((lm(2,idf(i))-x(2)),(lm(1,idf(i))-x(1))) - x(3)) ];
      
     H = [ (x(1)-lm(1,idf(i)))/sqrt((x(1)-lm(1,idf(i)))^2 + (x(2)-lm(2,idf(i)))^2) ,   (x(2)-lm(2,idf(i)))/sqrt((x(1)-lm(1,idf(i)))^2 + (x(2)-lm(2,idf(i)))^2) ,    0 ; ...
             (lm(2,idf(i))-x(2))/((x(1)-lm(1,idf(i)))^2 + (x(2)-lm(2,idf(i)))^2)   ,     -(lm(1,idf(i))-x(1))/((x(1)-lm(1,idf(i)))^2 + (x(2)-lm(2,idf(i)))^2)  ,   -1 ];

         
%      Compute Kalman Gain
     K = P*H'/(H*P*H'+R);
     
     % Update pose x 
     x = x + K*(z(:,i)-z_hat);
     x(3) = pi_to_pi(x(3));     
     
     % Update covariance P
     P = (eye(3) - K*H)*P;
end        


     
    
    
  

         