%% ***********************************************************************
% Code for controlling Motor / Props during closet testing
%
% Talks to Advanced Motion Control's (AMC) DZRAlTE-025L200 Driver Board.
% Driver must be fully setup and commissioned via AMC's Driveware Software.
% Driver must be in velocity mode (only commands supported in this rev).
%
% The following Parameters can be modified via this tool:
%
%     Bridge Enable     Off or On,         Default off
%     Motor Vel         in RPM,            Default 0 RPM
%     + Vel Accel       in RPM/sec,        Default 60RPM/sec
%     + Vel Decel       in RPM/sec,        Default 60RPM/sec
%
% Process - 
%   1) Set Velocity Parameters
%        a) Steady state velocity (in RPM)
%        b) Accel                 (in RPM/sec)
%        c) Decel                 (in RPM/sec)
%   2) Set Pulse length Parameters (all in seconds)
%        a) start to Dwell,
%        b) Dwell time,
%        c) Dwell to stop
%   3) Hit go
%
% On Hitting go, the code will:
%   Enable the bridge
%   start collecting data (measured vel and measured current)
%   command the velocity
%   wait until the Go to Dwell + Dwell time has expired
%   command to 0 velocity
%   wait until the Dwell to stop time has expired
%   stop collecting data
%   Disable the bridge

close all

%% **********************************************************************
% Set test Parameters
%
mtr_port = 'COM10'; % COMM Port attached to motor driver
vib_port = 'COM12'; % COMM Port attached to vibe sensor

RPM_set  = 140;     % (RPM Desired steady state motor
Acl_set  = 400;     % (RPM/sec)
Dcl_set  = 020;     % (RPM/sec)

zer_time = 002;     % dwell time with bridge on at 0RPM prior to step (sec)
dwl_time = 010;     % dwell time at commanded RPM                     (sec)
end_time = 010;     % dwell time after returning to 0 RPM post step   (sec)

num_vars = 2;       % # of vars to read from drive
F_sample = 40;      % Motor Driver Sampling Freq in                    (Hz)

%% *********************************************************************
% Calc various elapsed times and array size based on test parameters
%
acl_time = ceil(RPM_set/Acl_set*1.1);    % estimated acceleration time
dcl_time = ceil(RPM_set/Dcl_set*1.1);    % estimated deceleration time

stp_time = zer_time + acl_time + dwl_time;    % time to cmd back to 0 vel
run_time = stp_time + dcl_time + end_time;    % total run time

num_start= round(zer_time*F_sample/num_vars); %# of samples to issue x RPM
num_stop = round(stp_time*F_sample/num_vars); %# of samples to issue 0 RPM
num_tot  = round(run_time*F_sample/num_vars); %# of samples to end acquire

rate     = robotics.Rate(F_sample);           % for setting a fixed rate
rate.OverrunAction = 'drop';

%% ***********************************************************************
% Define Driver and Motor constants 
%   These constants MUST BE constitent with settings in Driver set
%   via AMC's Driveware prior to using this code.
%
% Encoder related
lin_per_rev = 2000;             % encoder lines per revolution  (lines/rev)
cts_per_rev = lin_per_rev*4;    % Quadrature (A, B, Index)     (counts/rev)

% RPM related
Drv_Freq    = 20e3;             % Driver Switching Frequency        (1/sec)
RPM_FS      = 2^17;             % Driver's Maximum RPM reading   (unitless)
RPM_scale   = (RPM_FS/Drv_Freq)*(cts_per_rev/60);     %        (counts/RPM)

Acl_FS      = 2^34;                                   %          (unitless)
Acl_scale   = (Acl_FS/(Drv_Freq^2))*(cts_per_rev/60); %    (counts/RPM/sec)

% Current related
Drv_I_Max   = 25;               % Driver's Peak Current              (Amps)
Amp_FS      = 2^13;             % Driver's Max  Current reading,   (counts)
Cur_peak    = Drv_I_Max/Amp_FS; % Reported as Peak of sinewave (counts/Amp)
Cur_scale   = Cur_peak/sqrt(2); % Reported as  RMS of sinewave (counts/Amp)

%% *********************************************************************** 
% Command Headers in Hex (as strings)
% Commands must be converted to binary prior to fwrite to serial port
%
% These commands have thier payloads and payload crc's hardcoded
cmd_wr_access_h   = 'A53F02070001B3E7FF0003FF';     % always allow access
cmd_wr_brdg_on_h  = 'A53F02010001014700000000';     % always  enable
cmd_wr_brdg_off_h = 'A53F02010001014701003331';     % always disable]
cmd_wr_vel_0_h    = 'A53F02450002F049000000000000'; % always set to 0 RPM
cmd_rd_vel_h      = 'A53F011102028FF9';             % read RPM measured
cmd_rd_Ims_h      = 'A53F01100301BB9B';             % read I measured
cmd_rd_Idm_h      = 'A53F0110020188AA';             % read I demand

cmd_wr_access_d   = cnvrt_cmd(cmd_wr_access_h);
cmd_wr_brdg_on_d  = cnvrt_cmd(cmd_wr_brdg_on_h);
cmd_wr_brdg_off_d = cnvrt_cmd(cmd_wr_brdg_off_h);
cmd_wr_vel_0_d    = cnvrt_cmd(cmd_wr_vel_0_h);
cmd_rd_vel_d      = cnvrt_cmd(cmd_rd_vel_h);
cmd_rd_Ims_d      = cnvrt_cmd(cmd_rd_Ims_h);
cmd_rd_Idm_d      = cnvrt_cmd(cmd_rd_Idm_h);

% These commands have payloads which must be calculated and appended to
% string prior to converting to binary for fwrite to serial port
cmd_wr_vel_h      = 'A53F02450002F049';    % set velocity header
cmd_wr_acl_h      = 'A53F023C0003A6F1';    % set acceleration ramp
cmd_wr_dcl_h      = 'A53F023C0303F3A2';    % set deceleration ramp

%% **********************************************************************
% Process Parameters for use during pulse test
%
% Generate RPM, Acel and Decel commands
RPM_cnt   = round(RPM_set*RPM_scale); % command in decimal counts
RPM_hex   = dec2hex(RPM_cnt);         % command in string as hex
pay_siz   = 8;                        % payload size for command in nibbles
payload   = gen_pay(pay_siz, RPM_hex);% generate payload
cmd_vel_h = [cmd_wr_vel_h, payload];  % concatenate command with payload
cmd_vel_d = cnvrt_cmd(cmd_vel_h);     % convert to binary for fwrite

Acl_cnt   = round(Acl_set*Acl_scale); % command in decimal counts
Acl_hex   = dec2hex(Acl_cnt);         % command in string as hex
pay_siz   = 12;                       % payload size for command in nibbles
payload   = gen_pay(pay_siz, Acl_hex);% generate payload
cmd_acl_h = [cmd_wr_acl_h, payload];  % concatenate command with payload
cmd_acl_d = cnvrt_cmd(cmd_acl_h);     % convert to binary for fwrite

Dcl_cnt   = round(Dcl_set*Acl_scale); % command in decimal counts
Dcl_hex   = dec2hex(Dcl_cnt);         % command in string as hex
pay_siz   = 12;                       % payload size for command in nibbles
payload   = gen_pay(pay_siz, Dcl_hex);% generate payload
cmd_dcl_h = [cmd_wr_dcl_h, payload];  % concatenate command with payload
cmd_dcl_d = cnvrt_cmd(cmd_dcl_h);     % convert to binary for fwrite

%% *********************************************************************
% Open the COM ports, s1 for talking to the Motor Driver
%                     s2 for talking to the acelerometer
% Get the steady state acelerometer offsets for zeroing out

close_sx;    % Find all open COM ports and close them

s1 = serial(mtr_port,'BaudRate',115200,'InputBufferSize',3000);
s2 = serial(vib_port,'BaudRate',115200,'InputBufferSize',3000, ...
                     'Timeout',0.1,    'Terminator',"CR/LF");
fopen(s1);   % Motor Driver COM Port
fopen(s2);   % Vibe Sensor  COM Port

acel_offset = 50;                        % # samples to average
acel_off = zeros(acel_offset,4);
tic
for ii = 1:acel_offset
    acel_off(ii,:) = get_acel_line(s2);  % get accelerometer readings
end
a_off=mean(acel_off);
a_off_std = std(acel_off);


%% **********************************************************************
% Execute the Selected Velocity Pulse Sequence
%
fwrite(s1, cmd_wr_access_d);      % enable register modification
fread(s1, 8);                     % read back response

fwrite(s1, cmd_wr_vel_0_d);       % set velocity to zero
fread(s1, 8);      

fwrite(s1, cmd_acl_d);            % set acelleration pos. pos.
fread(s1, 8);

fwrite(s1, cmd_dcl_d);            % set decelleration pos. pos.
fread(s1, 8);

fwrite(s1, cmd_wr_brdg_on_d);     % enable bridge
fread(s1, 8);
pause(1)

blk         = 1;                  % starting block to animate
num_pnts    = 0;                  % # of points per block
data_record = zeros(num_tot,3);   % initialize recording buffer
acel_data   = zeros(num_tot,4);

run_time=run_time*1.2;
subplot(2,1,1)
yyaxis left
plot(data_record(:,1), data_record(:,2))
ylabel('Motor Speed (RPM)'), xlabel('Time (sec)');
xlim([0 run_time]), ylim([-RPM_set*0.2 RPM_set*1.2]); % Optimize plot time
yyaxis right
plot(data_record(:,1), data_record(:,3))
ylabel('Motor Current (Amp)');
xlim([0 run_time]),                                

a_max = 2.5;
subplot(2,1,2)
plot(acel_data(:,4), acel_data(:,1)-a_off(1), '-r', ...
     acel_data(:,4), acel_data(:,2)-a_off(2), '-g', ...
     acel_data(:,4), acel_data(:,3)-a_off(3), '-b')
xlabel('Time (sec)'), ylabel('Acceleration in m/sec^2)');
legend('X', 'Y', 'Z'), grid('on');
xlim([0 run_time]), ylim([-a_max a_max]); 

% Here we go !!!!    
zz = 1;                       % index for acceleration data
tic                           % get data record start time
for ii=1:num_tot              % start data acquisiton
  fwrite(s1, cmd_rd_vel_d);   % request velocity
  vel = fread(s1, 14);        % read    velocity
  if (vel(12) > 128)          % check sign bit
      temp = vel(12)-128;
      temp = (temp*2^24+vel(11)*2^16+vel(10)*2^8+vel(9))-2^31;
      data_record(ii,2) = temp/RPM_scale;
  else
      data_record(ii,2)=(vel(12)*2^24+vel(11)*2^16+vel(10)*2^8+vel(9))/RPM_scale;
  end
  waitfor(rate);                        % wait for driver sample period
  
  fwrite(s1, cmd_rd_Ims_d);    % request current
  cur = fread(s1, 12);         % read    current
  if (cur(10) > 128)           % check sign bit
      temp = cur(10)-128;
      temp = (temp*2^8+cur(9))-2^15;
      data_record(ii,3)= temp*Cur_scale;
  else
      data_record(ii,3)=(cur(10)*2^8+cur(9))*Cur_scale;
  end
  data_record(ii,1) = toc;     % save time stamp for vel and current
  
  acel_data(ii,:) = get_acel_line(s2);  % get accelerometer readings
  waitfor(rate);     % wait for sample period
  
  if(num_start == ii)          % dwell time at 0 RPM period expired?
    fwrite(s1, cmd_vel_d);     % set the velocity
    fread(s1, 8); 
  end

  if(num_stop == ii)            % dwell time at Stepped RPM period expired?
      fwrite(s1, cmd_wr_vel_0_d);
      fread(s1, 8);
  end
  
  num_pnts = num_pnts+1;       % points collected since last animation
  if(num_pnts == F_sample)     % update display once per second
    subplot(2,1,1)
    yyaxis left
    plot(data_record(:,1), data_record(:,2))
    ylabel('Motor Speed (RPM)'), xlabel('Time (sec)');
    xlim([0 run_time]);
    yyaxis right
    plot(data_record(:,1), data_record(:,3))
    ylabel('Motor Current (Amp)');
    xlim([0 run_time]);
    grid on
 %   title({'Preliminary Velocity Loop Tuning Result';...
 %          'Step Response to a 0 - 250 RPM Step, 20RPM/sec acel/decel limits'; ...
 %          'Unfiltered Data, Sample rate 12.5Hz, 80msec'}) 
    blk = blk+F_sample;        % point to next block
    num_pnts = 0;
  end
  
  if(mod(zz, F_sample+1) == 0)
    subplot(2,1,2)
    plot(acel_data(:,4), acel_data(:,1)-a_off(1), '-r', ...
         acel_data(:,4), acel_data(:,2)-a_off(2), '-g', ...
         acel_data(:,4), acel_data(:,3)-a_off(3), '-b')
    xlabel('Time (sec)'), ylabel('Acceleration in m/sec^2)');
    legend('X', 'Y', 'Z'), grid('on'); 
    xlim([0 run_time]), ylim([-a_max a_max]); 
  end
end

fwrite(s1, cmd_wr_brdg_off_d);    % turn off bridge
fread(s1, 8);
fclose(s1);
fclose(s2);

%% ******************************************************************** 
% Generate a lowpass (~0.5 Hz) lowpass filtered version of
% velocity and current vs. time
%
RPM_flt = smooth(data_record(:,2),5);
AMP_flt = smooth(data_record(:,3),5);

figure, subplot(2,1,1)
yyaxis left
plot(data_record(:,1), RPM_flt)
ylabel('Motor Speed (RPM)'), xlabel('Time (sec)');
xlim([0 run_time]), ylim([-RPM_set*0.2 RPM_set*1.2])
yyaxis right
plot(data_record(:,1), AMP_flt)
ylabel('Motor Current (Amp)');
xlim([0 run_time])
grid on
title(['Low Pass Filtered (~0.5 Hz) 0 - ', num2str(RPM_set), ...
       ' RPM step response'])
subplot(2,1,2)
plot(acel_data(:,4), acel_data(:,1)-a_off(1), '-r', ...
     acel_data(:,4), acel_data(:,2)-a_off(2), '-g', ...
     acel_data(:,4), acel_data(:,3)-a_off(3), '-b')
xlabel('Time (sec)'), ylabel('Acceleration in m/sec^2)');
legend('X', 'Y', 'Z'), grid('on'); 
xlim([0 run_time]), ylim([-a_max a_max]); 
title('Unfiltered vibrations during step')

%% **********************************************************************
% Get some stats from record and 
% generate the Vel and Current vs. time
%
% Velocity consistency during dwell period
v_strt = round((zer_time+acl_time)*F_sample*1.1/num_vars);
v_stop = round(stp_time*F_sample*0.9/num_vars);
nvel   =  std(data_record(v_strt:v_stop,2));  % noise in the velocity
mvel   = mean(data_record(v_strt:v_stop,2));  % average velocity
SNRvel = 20*log10(mvel/nvel);                 % SNR velocity in db

% Current consistency during dwell period
ncur   =  std(data_record(v_strt:v_stop,3));  % noise in the velocity
mcur   = mean(data_record(v_strt:v_stop,3));  % average velocity
SNRcuur = 20*log10(mcur/ncur);                % SNR velocity in db

% Current peak during acel
cur_max = max(AMP_flt);
cur_idx = find(AMP_flt == cur_max,1);
width = 2;
cur_max_ave = mean(AMP_flt(cur_idx-width:cur_idx+width));

% aceleration rms values during steady state
xrms   = sqrt(mean((acel_data(v_strt:v_stop,1)-a_off(1)).^2));
yrms   = sqrt(mean((acel_data(v_strt:v_stop,2)-a_off(2)).^2));
zrms   = sqrt(mean((acel_data(v_strt:v_stop,3)-a_off(3)).^2));

% Sample time consistency Stats
dtime = diff(data_record(:,1));              % time between samples
ntime = std(dtime);                          % noise in the sample time
mtime = median(dtime);                       % median sample time

filename=[num2str(RPM_set),'_',num2str(Acl_set),'_stats'];
save(filename,'cur_max_ave','mcur',  'ncur',  ...
                            'mvel',  'nvel',  ...
                            'mtime', 'ntime', ...
                    'xrms', 'yrms',  'zrms');
filename=[num2str(RPM_set),'_',num2str(Acl_set),'_profiles'];
save(filename,'acel_data','data_record');
savefig(filename)

figure,plot(dtime);

%% ************************************************************************
%
%                            Subroutines
%
% *************************************************************************

%% ***********************************************************************
% Convert a Hex formatted command into its binary equivalent for
% use with fwrite out the serial port
%
function cmd_d = cnvrt_cmd(cmd_h)
  cmd_sz = length(cmd_h)/2;
  cmd_d  = ones(1,cmd_sz);
  jj=1;
  for ii=1:cmd_sz
    cmd_d(ii)=hex2dec(cmd_h(jj:jj+1));
    jj=jj+2;
  end
end

%% ***********************************************************************
% Given a Big Endian Hex valued payload, generate a Little Endian one
% with a CRC appended
%
function payload = gen_pay(num_nibbs, data_in)
  append = num_nibbs - size(data_in,2); % pad as appropriate
  for ii=1:append
      data_in = ['0',data_in];
  end
  payload = data_in;                  % payload must be in littel endian
  jj = 1;
  for ii=1:num_nibbs/2                % swap endianess, by byte not nibble
      payload(jj)   = data_in(num_nibbs-jj);
      payload(jj+1) = data_in(num_nibbs-jj+1);
      jj = jj+2;
  end
  crc = crc16(payload);               % generate CRC16-CCITT (XMODEM)
  append = 4 - size(crc,2); % pad as appropriate
  for ii=1:append
      crc = ['0',crc];
  end
  payload = [payload, crc];           % append CRC to little Endian Pay
end

function dummy = get_acel_line(com_port)
  for ii=1:3
    data = fgetl(com_port);
    if isempty(data)
      fprintf(1, 'Empty Line!\n');
    elseif data(1) == 'X'
      dummy(1) = str2double(data(2:end));
    elseif data(1) == 'Y'
      dummy(2) = str2double(data(2:end));
    elseif data(1) == 'Z'
      dummy(3) = str2double(data(2:end));
    else
      fprintf(1, 'Accelrometer Vibration Data test ICM20948\n');
    end
  end
  dummy(4) = toc; 
end