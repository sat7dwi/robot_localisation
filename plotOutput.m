load('HSL_notOdo.mat')
Algo = 'HSL';
path = output.path;
tpath = output.true;
tm = output.i;
t = 0.025*(1:tm);

delta = tpath(1,:) - path(1,:);
errX = sqrt(delta*delta'/size(delta,2));
figure;
plot(t, tpath(1,:), 'g', t, path(1,:), 'r');
title(strcat(Algo, '- X coordinate'));
grid on;
grid minor;

delta = tpath(2,:) - path(2,:);
errY = sqrt(delta*delta'/size(delta,2));
figure;
plot(t, tpath(2,:), 'g', t, path(2,:), 'r');
title(strcat(Algo, '- Y coordinate'));
grid on;
grid minor;

delta = tpath(3,:) - path(3,:);
errQ = sqrt(delta*delta'/size(delta,2));
figure;
plot(t, tpath(3,:), 'g', t, path(3,:), 'r');
title(strcat(Algo, '- Heading Angle'));
grid on;
grid minor;

disp(strcat(Algo, ' Algorithm'));
disp(strcat('mean square error in X = ', num2str(errX)));
disp(strcat('mean square error in Y = ', num2str(errY)));
disp(strcat('mean square error in theta = ', num2str(errQ)));