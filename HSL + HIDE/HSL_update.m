function newPos = HSL_update(pos, R, map, scan)

HMS = 30;
HMCR = 0.88;
PARmax = 0.98;
PARmin = 0.55;
bw_max = 8;
bw_min = 0.2;
NI = 20;
dim = 3;

pos = pos';
err = [sqrt(R(1,1)) sqrt(R(2,2)) sqrt(R(3,3))];

HM = ones(HMS,1)*(pos-err) + rand(HMS,dim).*(2*ones(HMS,1)*err);
HM(1,:) = pos;
F = scanMatch(HM, map, scan);
    
for iter = 1 : NI
    X = zeros(1, dim);

    PAR = PARmin + (PARmax - PARmin)*iter/NI;
    bw = bw_max*exp(log(bw_min/bw_max)*iter/NI);

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
  
end

newPos = HM(F == max(F), :)';
disp(max(F));

% i=1;
% T = [ cos(X(i,3))    -sin(X(i,3))   X(i,1) ;
%       sin(X(i,3))     cos(X(i,3))   X(i,2) ;
%           0                0           1   ];
% S = [scan ones(size(scan,1),1)]*T'; 
% S = S(:, 1:2);
% 
% figure
% scatter(map(:,1),map(:,2),'r','.');
% hold on
% scatter(S(:,1),S(:,2),'b','.');
% axis([-3 4 -3.5 2])
% title('After Registration')
% legend('scan1','scan2');
% hold off;
% drawnow;

