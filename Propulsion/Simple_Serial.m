lin_per_rev = 2000;            % encoder lines per revolution
cts_per_rev = lin_per_rev*4;   % Quadrature encoding (A, B, Index)
RPM_FS      = 2^17;            % Driver's Maximum RPM reading in counts
Drv_Freq    = 20e3;            % Driver Frequency
Drv_I_Max   = 25;              % Driver Peak Current
Amp_FS      = 2^13;            % Driver's MAximum Current reading in counts

RPM_scale   = (Drv_Freq/RPM_FS)*(60/cts_per_rev);  % units RPMs/count
Cur_scale   = Drv_I_Max/Amp_FS;                    % units Amps/count
RPM_set     = 800;                       % Desired motor RPM
Decel       = 60;                        % RPM/sec
RPM_cnt     = round(RPM_set*RPM_scale);  % command in counts
RPM_hex     = dec2hex(RPM_cnt);          % command to hex

run_time    = 40;                        % Duration of time to sample over
stop_time   = 25;                        % must be <= to run_time
F_sample    = 25;                        % Sampling Rate in Hz
rate        = robotics.Rate(F_sample);   % set rate
rate.OverrunAction = 'drop';

num_vars = 2;                            % # of vars to read from drive
num_samp = run_time*F_sample/num_vars;   % # of samples to acquire

[crc, hex] = crc16(cmd_wr_access_d);
s1 = serial('COM1','BaudRate',115200,'InputBufferSize',3000);
fopen(s1);
%                     S A D I O Z 1 2 P P 1 2
cmd_wr_access_h   = 'A53F02070001B3E7FF0003FF';
cmd_wr_brdg_on_h  = 'A53F02010001014700000000';
cmd_wr_brdg_off_h = 'A53F02010001014701003331';
cmd_wr_vel_100_h  = 'A53F02450002F049555501004F71';
cmd_wr_vel_800_h  = 'A53F02450002F049ABAA0A0061FF';
cmd_wr_vel_000_h  = 'A53F02450002F049000000000000';

cmd_wr_access_d   = cnvrt_cmd(cmd_wr_access_h);
cmd_wr_brdg_on_d  = cnvrt_cmd(cmd_wr_brdg_on_h);
cmd_wr_brdg_off_d = cnvrt_cmd(cmd_wr_brdg_off_h);
cmd_wr_vel_100_d  = cnvrt_cmd(cmd_wr_vel_100_h);
cmd_wr_vel_800_d  = cnvrt_cmd(cmd_wr_vel_800_h);
cmd_wr_vel_000_d  = cnvrt_cmd(cmd_wr_vel_000_h);

cmd_rd_vel_h = 'A53F011102028FF9';
cmd_rd_cur_h = 'A53F01100301BB9B';
cmd_rd_vel_d = cnvrt_cmd(cmd_rd_vel_h);
cmd_rd_cur_d = cnvrt_cmd(cmd_rd_cur_h);

fwrite(s1,cmd_wr_access_d);
fread(s1,8);

fwrite(s1,cmd_wr_vel_000_d);
fread(s1,8);
%pause(20);

fwrite(s1,cmd_wr_brdg_on_d);
fread(s1,8);
pause(1)

%% ********************************************************************
fwrite(s1,cmd_wr_vel_800_d);
fread(s1,8);
vel_rpm = zeros(num_samp,3);
tic
for ii=1:num_samp
  fwrite(s1,cmd_rd_vel_d);
  vel = fread(s1,14);
  vel_rpm(ii,2)=int32((vel(12)*2^24+vel(11)*2^16+vel(10)*2^8+vel(9)))*RPM_scale;
  waitfor(rate);
  
  fwrite(s1,cmd_rd_cur_d);
  vel = fread(s1,12);
  vel_rpm(ii,3)=(vel(10)*2^8+vel(9))*Cur_scale;
  vel_rpm(ii,1)=toc;
  waitfor(rate);
  
  if(round(stop_time*F_sample/2) == ii)
      fwrite(s1,cmd_wr_vel_000_d);
      fread(s1,8);
  end
end

fwrite(s1,cmd_wr_brdg_off_d);
[resp, count] = fread(s1,8);
fclose(s1);

dtime = diff(vel_rpm(:,1));          % time between samples
ntime = std(dtime);                  % noise in the sample time
mtime = median(dtime);               % median sample time

nvel  = std(vel_rpm(100:end,2));     % noise in the velocity
mvel  = mean(vel_rpm(100:end,2));    % average velocity
SNRvel = 20*log10(mvel/nvel);        % SNR velocity in db


yyaxis left
plot(vel_rpm(:,1), vel_rpm(:,2))
ylabel('Motor Speed (RPM)'), xlabel('Time (sec)'), ylim([0,900]);
yyaxis right
plot(vel_rpm(:,1), vel_rpm(:,3))
ylabel('Motor Current (Amp)'), ylim([0,2])
hw=instrfind;




function cmd_d = cnvrt_cmd(cmd_h)
  cmd_sz = length(cmd_h)/2;
  cmd_d = ones(1,cmd_sz);
  jj=1;
  for ii=1:cmd_sz
    cmd_d(ii)=hex2dec(cmd_h(jj:jj+1));
    jj=jj+2;
  end
end