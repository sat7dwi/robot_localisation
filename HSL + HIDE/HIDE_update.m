function newPos = HIDE_update(pos, R, map, scan)
    HRT = 4;
    NFE = 20;
    L = 0.7;
    lambda = 0.44;

    HMS = 30;
    HMCR = 0.88;
    PARmax = 0.98;
    PARmin = 0.55;
    bw_max = 8;
    bw_min = 0.2;
    dim = 3;

    pos = pos';
    err = [sqrt(R(1,1)) sqrt(R(2,2)) sqrt(R(3,3))];

    HM = ones(HMS,1)*(pos-err) + rand(HMS,dim).*(2*ones(HMS,1)*err);
    HM(1,:) = pos;
    F = scanMatch(HM, map, scan);

    for iter = 1 : NFE
        X = zeros(1, dim);

        PAR = PARmin + (PARmax - PARmin)*iter/NFE;
        bw = bw_max*exp(log(bw_min/bw_max)*iter/NFE);

        for i = 1 : dim
           if rand <= HMCR
               X(i) = HM(randi(HMS),i);
               if rand <= PAR;
                   X(i) = X(i) + 1 - 2*rand*bw;
               end
           else
               X(i) = pos(i) - err(i) + 2*rand*err(i);
           end
        end
        X(3) = pi_to_pi(X(3));

        temp = scanMatch(X, map, scan);
        if temp > min(F)
            HM(F == min(F), :) = ones(size(HM(F == min(F), :),1),1)*X;
            F(F == min(F)) = temp;
        end

        if mod(iter,HRT) == 0
           best = HM(F == max(F), :);
           best = best(1, :);
           for i = 1 : HMS
               X_hat = HM(i,:) + L*(best - HM(i,:)) + lambda*(HM(randi(HMS),:)-HM(randi(HMS),:));
               temp = scanMatch(X_hat, map, scan);
               if(temp > F(i))
                   HM(i,:) = X_hat;
                   F(i) = temp;
               end       
           end
        end
    end
    
    newPos = HM(F == max(F), :)';
    disp(max(F));
    
end
