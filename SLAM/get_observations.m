function [obsScan, actScan] = get_observations(X, map, RMAX)

idx = rangesearch(map, X(1:2)', RMAX);
idx = idx{1};
map = map(idx,:);
actScan = map;

T = [ cos(X(3))    -sin(X(3))   X(1) ;
      sin(X(3))     cos(X(3))   X(2) ;
         0             0         1   ];
obsScan = [map ones(size(map,1),1)]*inv(T)'; 
obsScan = obsScan(:, 1:2);

%
%
% 
% function [lm,idf]= get_visible_landmarks(x,lm,idf,rmax)
% % Select set of landmarks that are visible within vehicle's semi-circular field-of-view
% dx= lm(1,:) - x(1);
% dy= lm(2,:) - x(2);
% phi= x(3);
% 
% % incremental tests for bounding semi-circle
% ii= find(abs(dx) < rmax & abs(dy) < rmax ... % bounding box
%       & (dx*cos(phi) + dy*sin(phi)) > 0 ...  % bounding line
%       & (dx.^2 + dy.^2) < rmax^2);           % bounding circle
% % Note: the bounding box test is unnecessary but illustrates a possible speedup technique
% % as it quickly eliminates distant points. Ordering the landmark set would make this operation
% % O(logN) rather that O(N).
%   
% lm= lm(:,ii);
% idf= idf(ii);
% 
% %
% %
% 
% function z= compute_range_bearing(x,lm)
% % Compute exact observation
% dx= lm(1,:) - x(1);
% dy= lm(2,:) - x(2);
% phi= x(3);
% z= [sqrt(dx.^2 + dy.^2);
%     atan2(dy,dx) - phi];
%     