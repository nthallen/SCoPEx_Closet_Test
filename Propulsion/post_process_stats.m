close all

%% *********************************************************************
% Load in all Motor Test Results summary stats, not raw data
% and generate 7 charts as follows
% 1) Motor RPM vs. Phase Vpeak (From Manufacturer Motor Test Result Sheet)
% 2) Settled RPM and Resulting Torque (amps) Meausred vs. RPM setting
% 3) Motor Peak Voltage and 100v DC Power Supply RMS Power vs. RPM setting
% 4) Absolute Noise in Settled RPM and Torque (amps) vs.RPM Setting
% 5) SNR in Settled RPM and Torque (amps) vs. RPM Setting
% 6) Peak Torque(amps) and Accel. (@20 RPM/sec) Torque(amps) vs. RPM set
% 7) Structural Vibrational Energy vs. RPM setting

% From Motor Manufacture's Motor Test Results Data Sheet
% supplied with motor
%
RPM_data = [0,88,175,350,525,700,875,1050,1225,1400,1575,1750];
Vpk_data = [19.7,23.8,27.9,36,44.2,52.3,60.5,68.6,76.7,84.8,92.9,101];
Vrms_data = Vpk_data/sqrt(2);
Ke_coefs = polyfit(Vpk_data,RPM_data,1);

figure,
plot(Vpk_data, RPM_data, '-b*'),grid('on')
title('RPM vs. Vpeak (From Manufacturer Motor Test Result Sheet)')
xlabel('Vpeak (volts)'), ylabel('Motor Angular Velocity (RPM)')
ylim([0, max(RPM_data)*1.05])

RPM = (20:20:300);                   % Array of RPM's tested
num_files = size(RPM,2);
num_stats = 10;
mvel = zeros(num_files,1);
nvel = mvel;
mcur = mvel;
ncur = mvel;
mtime = mvel;
ntime = mvel;
xrms = mvel;
yrms = mvel;
zrms = mvel;
cur_max_ave = mvel;
data_path='C:\Users\mpl055\Documents\ScopeX\Propulsion\Closet Test Results\20201010_good_acel_mount\';
for ii = 1:num_files
  temp =load([data_path,num2str(RPM(ii)),'_20_stats']);
  mvel(ii) = temp.mvel;
  nvel(ii) = temp.nvel;
  mcur(ii) = temp.mcur;
  ncur(ii) = temp.ncur;
  mtime(ii) = temp.mtime;
  ntime(ii) = temp.ntime;
  xrms(ii) = temp.xrms;
  yrms(ii) = temp.yrms;
  zrms(ii) = temp.zrms;
  cur_max_ave(ii) = temp.cur_max_ave;
end
arms = (xrms+yrms+zrms)./3;
Vpk_meas = (mvel-Ke_coefs(2))/Ke_coefs(1);
Pwr_meas = Vpk_meas.*mcur./(sqrt(2)*sqrt(2));
Sup_cur  = Pwr_meas./100;

figure, yyaxis left
plot(RPM, mvel, '-b*'),grid('on')
title('Settled RPM and Resulting Torque (Nm) Meausred vs. RPM setting')
xlabel('RPM Set'), ylabel('RPM Measured')
ylim([0, mvel(num_files)*1.1])
yyaxis right
plot(RPM, mcur*0.55, '-r*'),grid('on')
ylabel('Torque(amps) Measured')
ylim([0, mcur(num_files)*1.05])

figure, yyaxis left
plot(RPM, Vpk_meas, '-b*'),grid('on')
title('Motor Phase Peak Voltage and 100v DC Power Supply RMS Power vs. RPM setting')
xlabel('RPM Set'), ylabel('Motor Phase Peak Voltage (volts)')
ylim([min(Vpk_meas)*0.9, max(Vpk_meas)*1.1])
yyaxis right
plot(RPM, Pwr_meas, '-r*'),grid('on')
ylabel('100V DC Supplied RMS power (watts)')
ylim([min(Pwr_meas)*0.95, max(Pwr_meas)*1.05])

figure, yyaxis left
plot(RPM, nvel, '-b*'),grid('on')
title('Absolute Noise in Settled RPM and Torque (amps) vs.RPM Setting')
xlabel('RPM Set'), ylabel('RPM Noise (in RPM)')
ylim([0, nvel(num_files)*1.1])
yyaxis right
plot(RPM, ncur, '-r*'),grid('on')
ylabel('Torque noise(amps)')
ylim([0, ncur(num_files)*1.05])

figure, yyaxis left
plot(RPM, mvel./nvel, '-b*'),grid('on')
title('SNR in Settled RPM and Torque (amps) vs. RPM Setting')
xlabel('RPM Set'), ylabel('RPM SNR')
ylim([0, mvel(num_files)/nvel(num_files)*1.1])
yyaxis right
plot(RPM, mcur./ncur, '-r*'),grid('on')
ylabel('Torque SNR')
ylim([0, mcur(num_files)/ncur(num_files)*1.05])

figure, yyaxis left
plot(RPM, cur_max_ave, '-b*'),grid('on')
title('Peak Torque (amps) and Acceleration (@ 20 RPM/sec) Torque (amps) vs. RPM setting')
xlabel('RPM Set'), ylabel('Peak Torque (amps)')
ylim([cur_max_ave(1)*0.95, cur_max_ave(num_files)*1.05])
yyaxis right
plot(RPM, cur_max_ave-mcur, '-r*'),grid('on')
ylabel('Acceleration Torque')
ylim([min(cur_max_ave-mcur)*0.95, max(cur_max_ave-mcur)*1.05])

figure, 
plot(RPM, xrms, '-r*', RPM, yrms, '-g+', RPM, zrms, '-bo'),
title('Structural Vibrational Energy vs. RPM setting')
xlabel('RPM Set'), ylabel('Vibrational Energy (au)')
grid('on')
energy_lo = min([min(xrms),min(yrms),min(zrms)]);
energy_hi = max([max(xrms),max(yrms),max(zrms)]);
ylim([energy_lo*0.95, energy_hi*1.05])

% Extend out the RPM and plot anticipated results
Kt       = 0.55;                       % Motor Torque (Nm/amp_rms)
coefs_t  = polyfit(RPM', mcur*Kt,2);
coefs_r  = polyfit(RPM', mvel   ,1);
RPM_pad  = (0:20:500);
coefs_ft = polyval(coefs_t,RPM_pad);
coefs_fr = polyval(coefs_r,RPM_pad);

figure, yyaxis left
plot(RPM, mvel, '-b*', RPM_pad, coefs_fr, '--b')
title('Settled RPM and Resulting Torque (Nm) Meausred vs. RPM setting')
xlabel('RPM Set'), ylabel('RPM Measured')
ylim([0, RPM_pad(end)*1.1])
yyaxis right
plot(RPM, mcur*Kt, '-r*',RPM_pad, coefs_ft, '--r')
ylabel('Torque(amps) Measured')
ylim([0, coefs_ft(end)])
grid('on')