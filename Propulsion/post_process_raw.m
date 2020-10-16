close all

%% *********************************************************************
% Load in all Motor Test Results raw data (profiles)
% and replot the profiles
%

RPM = (20:20:300);                     % Array of RPM's tested
num_files = size(RPM,2);
ACL = [20,   25,  30,  35,  40,  45, 50, 55, 60, 70, 80, 90, 100, ...
       125, 150, 175, 200, 250, 300]; % Array of Accels tested
num_file2 = size(ACL,2);

%{
for ii = 1:num_files
  load([num2str(RPM(ii)),'_20_profiles']);
     % acel_date   contains the 3D accelerometer vibration time series
     % x, y, z, time
     % data_record contains the RPM and Current time series
     % time, RPM, AMPs
  figure, yyaxis left
  plot(data_record(:,1), data_record(:,2), '-b')
  title('RPM Measured vs. Time')
  xlabel('Time (sec)'), 
  ylabel('RPM Measured')
  yyaxis right
  plot(data_record(:,1), data_record(:,3), '-r')
  grid('on')
  ylabel('Torque(amps) Measured')
end
%}

figure, hold 'on';
for ii = 1:num_file2
  load(['140_',num2str(ACL(ii)),'_profiles']);
     % acel_date   contains the 3D accelerometer vibration time series
     % x, y, z, time
     % data_record contains the RPM and Current time series
     % time, RPM, AMPs
  strt = ceil(size(data_record,1)*0.025);
  stop = ceil(size(data_record,1)*0.250);  % only show aceleration
  yyaxis left
  plot(data_record(strt:stop,1), data_record(strt:stop,2))
  title('Measured RPM and Total Current vs. Time for 19 Accel Limits')
  ylabel('RPM Measured')
  ylim([-25, 150])
  yyaxis right
  plot(data_record(strt:stop,1), data_record(strt:stop,3))
  grid('on')
  ylabel('Torque(amps) Measured')
  xlabel('Time (sec)'),
  ylim([-2.5, 15])
end

for ii=1:num_file2
  leg{ii} = num2str(ACL(ii)); 
end
legend(leg)


