%%
cd C:\Users\nort\Documents\Documents\Exp\SCoPEx\SW\SCoPEx_Closet_Test\vib_subbus_matlab
%%
serial_port_clear();
%%
[s,port] = serial_port_init('COM5');
% set(s,'BaudRate',57600);
% set(s,'BaudRate',115200);
%
% First check that the board is a vibration sensor
[subfunc,desc] = get_subfunction(s);
if subfunc ~= 16
  error('Expected BdID 16. Reported %d', BdID);
end
BoardID = read_subbus(s,2);
Build = read_subbus(s,3);
[SerialNo,SNack] = read_subbus(s,4);
[InstID,InstIDack] = read_subbus(s,5);

fprintf(1, 'Attached to Vibration Sensor S/N %d Build # %d\n', SerialNo, Build);
fprintf(1, 'The description is "%s"\n', desc);
vib_base = 5*16;
%
vib_report_status(s);
%
SampleRate = 100; % Hz
SamplesPerRead = 10;
ReadPeriod = SamplesPerRead/SampleRate; % seconds
RunDuration = 30; % seconds
NReads = RunDuration/ReadPeriod + 1;
RecordLength = 4; % x, y, z, dT
%
% Initialize the sample duration:
dur = 10000/SampleRate;
write_subbus(s, vib_base+1, dur); % 10 Hz sample
rb = read_subbus(s, vib_base+1);
if rb == dur
  fprintf(1,'Readback of sample duration %.1f msecs confirmed\n', dur*1e-1);
else
  fprintf(1,'Readback of sample duration expected %d, received %d\n', ...
    dur, rb);
end
%
% Initialize the sample duration:
preq = SamplesPerRead+1;
write_subbus(s, vib_base+2, preq); % 10 Hz sample
rb = read_subbus(s, vib_base+2);
if rb == preq
  fprintf(1,'Readback of PREQDEPTH %d confirmed\n', preq);
else
  fprintf(1,'Readback of PREQDEPTH expected %d, received %d\n', ...
    preq, rb);
end
%
if false
  fifodep = read_subbus(s, vib_base+3);
  fprintf(1,'FIFO Depth is currently %d\n', fifodep);
  if fifodep == 0
    [val,ack] = read_subbus(s, vib_base+4);
    if ack == 0
      fprintf(1, 'Received expected NACK reading from empty FIFO\n');
    elseif ack == 1
      fprintf(1, 'ERROR: Received unexpected ACK reading from empty FIFO\n');
    else
      fprintf(1, 'ERROR: Ack was %d reading from empty FIFO\n', ack);
    end
  else
    % empty it?
    rm_obj = read_multi_prep([vib_base+3,4,vib_base+4,0]);
    while fifodep > 0
      [vals,ack] = read_multi(s,rm_obj);
      if length(vals) == 5
        fprintf(1, 'N:%d X:%d Y:%d Z:%d dT:%d\n', vals);
      end
      fifodep = vals(1);
    end
  end
  vib_report_status(s);
end
%
% Issue preset command and verify status and fifo depth
write_subbus(s, vib_base, 1); % preset command
vib_report_status(s);
%
% Now see if we are fast enough to sample at a sample 
% Assuming we are in PRESET based on the previous section
RunSamples = RunDuration * SampleRate + 20; % a little extra
RunSteps = RunDuration * SampleRate / SamplesPerRead;
Data = zeros(RunSamples,RecordLength);
MetaData = zeros(NReads,3); % status, FIFOdep, dT
nsamples = 0;
n_reads = 0;
% Basic plan is to empty the FIFO on each read
rm_obj = read_multi_prep(vib_base, [vib_base+3,SamplesPerRead*2*RecordLength,vib_base+4,0], vib_base+5);

tic;
while true
  T0 = toc;
  if T0 > RunDuration
    break;
  end
  [vals,ack] = read_multi(s,rm_obj);
  nvals = length(vals);
  
  if nvals == 0 || mod(nvals-3,RecordLength) ~= 0
    error('Unexpected length of read: %d-1 not divisible by %d', nvals, RecordLength);
  end
  n_reads = n_reads+1;
  MetaData(n_reads,:) = vals([1 2 end]);
  nrows = (nvals-3)/RecordLength;
  X = reshape(vals(3:end-1),4,[])';
  X1 = X(:,1:3);
  X1 = X1 - (X1>=32768)*65536;
  X(:,1:3) = X1;
  Data(nsamples+1:nsamples+nrows,:) = X;
  nsamples = nsamples + nrows;
  fprintf(1,'T0:%.3f S:%02X N:%d/%d dT:%d\n', T0, vals(1), vals(2), nsamples, vals(end));
  T1 = toc - T0;
  if T1 < ReadPeriod
    pause(ReadPeriod - T1);
  end
end
write_subbus(s, vib_base, 0); % Reset to stop collection
%DRaw = Data(:,1:3);
%Data(:,1:3) = DRaw - (DRaw >= 32786)*65536;
PreData = Data(1:preq,:);
Data = Data(preq+1:nsamples,:);
%
figure;
plot(Data(:,1:3));
%
% Reset:
if false
  write_subbus(s, vib_base, 0); % reset command
  vib_report_status(s);
  %%
  vib_report_status(s);
  %%
end
%
flush_input(s);
serial_port_clear();
delete(s);
clear s
