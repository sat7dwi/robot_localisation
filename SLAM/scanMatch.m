function fitness = scanMatch(X, map, scan)
    fitness = zeros(size(X,1),1);
    
    for i = 1 : size(X,1)
        T = [ cos(X(i,3))    -sin(X(i,3))   X(i,1) ;
              sin(X(i,3))     cos(X(i,3))   X(i,2) ;
                  0                0           1   ];
        S = [scan ones(size(scan,1),1)]*T'; 
        S = S(:, 1:2);
        
        [~, dist] = dsearchn(map, S);

        dist = exp(-dist);
        fitness(i) = sum(dist)/size(dist,1);                    
    end
end