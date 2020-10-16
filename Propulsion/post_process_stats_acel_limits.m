close all

%% *********************************************************************
% Generate Plot of Acceleration Torque current vs. Acceleration Limit
% setting for a range of limit setpoints.
%
acels = [20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80, 90, 100, ...
        125, 150, 175, 200, 250, 300];   % Array of acel limit's tested
num_files = size(acels,2);

num_stats = 10;
mcur = zeros(num_files,1);
cur_max_ave = mvel;
for ii = 1:num_files
  temp =load(['140_', num2str(acels(ii)), '_stats']);
  cur_max_ave(ii) = temp.cur_max_ave;
end

torque_i = cur_max_ave-mcur;
coefs = polyfit(acels(1:8)', torque_i(1:8), 1);
acels_L = zeros(size(acels,2)+1,1);
acels_L(2:end) = acels;
torque_L = polyval(coefs,acels_L);
figure, plot(acels, torque_i, '-r*', acels_L, torque_L, '--b')
grid('on')
ylabel('Acceleration Torque (Amps)'), 
xlabel('Acceleration Limit Setting (RPM/sec)')
ylim([0, 12])
title('Acceleration Torque Current (Amps) vs. RPM setting')
legend('measured', 'linear fit')



