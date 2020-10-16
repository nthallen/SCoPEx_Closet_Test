%  ICM20948 vibration data receive/plot
%
%  11:49 AM 9/21/2020  Maroc - Initial release
%  13:00   10/06/2020  Litch - Modified for longer duration data holding 
%%
close all
serial_port_clear();
port = 'COM5';
s = serial(port,'BaudRate',115200,'InputBufferSize',3000, ...
                'Timeout',0.1,    'Terminator',"CR/LF");
fopen(s);

%% *************************************************************
% Get the acelerometer data
%
num_samps = 100;
jj = 1; kk=1;
acel = zeros(num_samps,4);
fprintf(1, 'Starting Sampling');
plot(acel(:,4), acel(:,1)+9.4715, '-r',...
     acel(:,4), acel(:,2)+2.0304, '-g',...
     acel(:,4), acel(:,3)-1.3311, '-b')
xlabel('Time (sec)'), ylabel('Acceleration in m/sec^2)');
legend('X', 'Y', 'Z'), grid('on')
xlim([0 num_samps*0.06]), ylim([-1 1]);              % Optimizing plot time
tic
for ii=1:num_samps*3
  [data,count] = fgetl(s);
  if isempty(data)
    fprintf(1, 'Empty Line!\n');
  elseif data(1) == 'X'
    acel(jj,1) = str2double(data(2:end));
  elseif data(1) == 'Y'
    acel(jj,2) = str2double(data(2:end));
  elseif data(1) == 'Z'
    acel(jj,3) = str2double(data(2:end));
    acel(jj,4) = toc;
    jj = jj+1;
    if (mod(jj,50) == 0)
      plot(acel(:,4), acel(:,1)-9.4715, '-r',...
           acel(:,4), acel(:,2)-2.0304, '-g',...
           acel(:,4), acel(:,3)+1.3311, '-b')
      xlabel('Time (sec)'), ylabel('Acceleration in m/sec^2)');
      legend('X', 'Y', 'Z'), grid('on')
      xlim([0 num_samps*0.06]), ylim([-1 1]);  % Optimizing plot time
      pause(0.001)
    end
 else
    fprintf(1, 'Accelrometer Vibration Data test ICM20948\n');
  end
end
plot(acel(:,4), acel(:,1)-9.4715, '-r',...
     acel(:,4), acel(:,2)-2.0304, '-g',...
     acel(:,4), acel(:,3)+1.3311, '-b')
xlabel('Time (sec)'), ylabel('Acceleration in m/sec^2)');
legend('X', 'Y', 'Z'), grid('on')

fclose(s);

acel_ave=mean(acel,1);
samp_time = diff(acel(:,4));
samp_ave = mean(samp_time)
figure, grid('on')
plot(acel(:,4), acel(:,1)-acel_ave(1), '-r',...
     acel(:,4), acel(:,2)-acel_ave(2), '-g',...
     acel(:,4), acel(:,3)-acel_ave(3), '-b')
xlabel('Time (sec)'), ylabel('Acceleration in m/sec^2)');
legend('X', 'Y', 'Z'), grid('on')

figure, grid('on')
plot(samp_time)
xlabel('sample number, (unitless)');
ylabel('Time (sec)');



