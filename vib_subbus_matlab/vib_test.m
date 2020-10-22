%%
cd C:\Users\nort\Documents\Documents\Exp\SCoPEx\SW\SCoPEx_Closet_Test\vib_subbus_matlab
%%
serial_port_clear();
%%
[s,port] = serial_port_init('COM5');
% set(s,'BaudRate',57600);
% set(s,'BaudRate',115200);
%%
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
%%

rm_obj = read_multi_prep([8,40,9,0]);
[vals,ack] = read_multi(s,rm_obj);
even = mod(vals(2:end),256);
odd = floor(vals(2:end)/256);
il = [even odd]';
il = il(:)';
nc = find(il == 0,1);
il = il(1:(nc-1));
desc = char(il);
fprintf(1,'Description from FIFO is: %s\n', desc);
flush_input(s);
%%
rm_obj = read_multi_prep([20,1,37]);
%
% while true
for iadc=1:10
  %%
  [vals,ack] = read_multi(s,rm_obj);
  fprintf(1,'---------\n');
  fprintf(1,'%04X %d\n', vals(1),vals(end));
  hdr = floor(vals(3:2:end-1)/256);
  adc = bitand(vals(3:2:end-1),255)*65536 + vals(2:2:end-2);
  V = adc >= 2^23;
  sadc = adc - (adc>=2^23)*2^24;
  vref = 2.5;
  vadc = vref * sadc / (2^23);
  for i=1:length(hdr)
    fprintf(1,'%02X %8X %8d %10f V\n',hdr(i), adc(i), sadc(i), vadc(i));
  end
  %%
  pause(1);
end
flush_input(s);
% peek/poke interface to the AD7770 registers:
%[value,ack] = read_subbus(s,39); % General read register
%fprintf(1,'ack=%d value=%04X\n', ack,value);
%
%write_subbus(s, 17, 0);
%% Timer section
rm_obj = read_multi_prep([64,1,68]); % 0x40
%
T0 = -1;
N = 1000;
fprintf(1,'Collecting %.1f seconds of timer data\n', N/10);
ielp = 1;
Tres = zeros(N,4);
while ielp <= N
  [vals,ack] = read_multi(s,rm_obj);
  T1 = vals(1) + vals(2)*65536;
  if T0 > 0
    dT = T1-T0;
    Tres(ielp,:) = [dT*1e-5, vals(3)*1e-2, vals(4)*1e-2, vals(5)];
    % fprintf(1, 'Elapsed/Loop/Max/State: %.5f %.3f %.3f %.0f\n', dT*1e-5, vals(3)*1e-2, vals(4)*1e-2, vals(5));
    % fprintf(1, '  Elapsed/Loop/Max/State: %.5f %.3f %.3f %.0f\n', Tres(ielp,:));
    ielp = ielp+1;
  end
  T0 = T1;
  pause(.1);
end
flush_input(s);
fprintf(1,'  Elapsed: %.5f +/- %.5f\n', mean(Tres(:,1)), std(Tres(:,1)));
fprintf(1,'  Loop T:  %.5f +/- %.5f\n', mean(Tres(:,2)), std(Tres(:,2)));
fprintf(1,'  Max T:   %.5f +/- %.5f\n', mean(Tres(:,3)), std(Tres(:,3)));
fprintf(1,'  Unique states: %s\n', sprintf(' %d', unique(Tres(:,4))));
%
Tf = figure;
ax = [ nsubplot(2,1,1) nsubplot(2,1,2) ];
X = 1:N;
plot(ax(1), X, Tres(:,1),'.');
ylabel(ax(1),'elapsed sec');
plot(ax(2), X, Tres(:,2), '.', X, Tres(:,3), '*');
ylabel(ax(2),'Loop msec');
set(ax(1),'YAxisLocation','Right','XTickLabels',[]);
%% Timer statistics: takes N/10 seconds
% N = 100;
% fprintf(1,'Collecting %.1f seconds of timer data\n', N/10);
% curlooptime = zeros(N,1);
% for i=1:N
%   val = read_subbus(s,66);
%   curlooptime(i) = val*1e-2;
%   pause(.1);
% end
% %
% figure; plot(curlooptime,'.');
% ylabel('msec');
% flush_input(s);
%% Skip this section unless BoardID is 2
% Honeywell Pressure Sensor Readings:
Honeybase = hex2dec('50');
%%
% NWpsdesc = read_subbus(s,Honeybase+9);
if BoardID ~= 2
  fprintf(1,'Skipping Honeywell Pressure Sensor tests: not uDACS B\n');
else
  rm_obj = read_multi_prep([Honeybase+9,50,Honeybase+10,0]);
  [vals,~] = read_multi(s,rm_obj);
  if isempty(vals) || vals(1) ~= 46 || vals(1)+1 ~= length(vals)
    error('vals length was %d, expected 46', length(vals));
  end
  %
  PSdef = struct( ...
    'PartNumber', { words2txt(vals(2:10)) words2txt(vals(23+(2:10))) }, ...
    'SerialNumber', { words2txt(vals(11:16)) words2txt(vals(23+(11:16))) }, ...
    'PressureUnit', { words2txt(vals(17:19)) words2txt(vals(23+(17:19))) }, ...
    'PressureRef', { words2txt(vals(20)) words2txt(vals(23+20)) }, ...
    'PressureRange', { typecast(uint32(vals(21)+65536*vals(22)),'single'), ...
    typecast(uint32(vals(23+21)+65536*vals(23+22)),'single') }, ...
    'PressureMin', { typecast(uint32(vals(23)+65536*vals(24)),'single'), ...
    typecast(uint32(vals(23+23)+65536*vals(23+24)),'single') });
  %
  for i=1:2
    fprintf(1, 'Sensor %d:\n', i);
    fprintf(1, '  Part: %s\n', PSdef(i).PartNumber);
    fprintf(1, '  S/N:  %s\n', PSdef(i).SerialNumber);
    fprintf(1, '  Unit: %s %s\n', PSdef(i).PressureUnit, PSdef(i).PressureRef);
    fprintf(1, '  Min/Range: %f/%f\n', PSdef(i).PressureMin, PSdef(i).PressureRange);
  end
  
  conv2Torr = struct('PSI', 51.7149, 'inH2O', 1.86645);
  rm_obj = read_multi_prep([Honeybase,1,Honeybase+8]); % 0x40
  %%
  [vals,ack] = read_multi(s, rm_obj);
  status = vals(1);
  fprintf(1,'Status: 0x%X\n', status);
  PSread = struct( ...
    'T', { typecast(uint32(vals(2)+65536*vals(3)),'single'), ...
    typecast(uint32(vals(6)+65536*vals(7)),'single') }, ...
    'P', { typecast(uint32(vals(4)+65536*vals(5)),'single'), ...
    typecast(uint32(vals(8)+65536*vals(9)),'single') });
  for i=1:2
    fprintf(1,'P%d: %7.3f %5s %s   T%d: %7.3f\n', i, ...
      PSread(i).P, PSdef(i).PressureUnit, PSdef(i).PressureRef, ...
      i, PSread(i).T);
    fprintf(1,'P%d: %7.3f %5s %s   T%d: %7.3f\n', i, ...
      PSread(i).P*conv2Torr.(PSdef(i).PressureUnit), 'Torr', PSdef(i).PressureRef, ...
      i, PSread(i).T);
  end
end

%% Fail circuit
write_subbus(s,6,0);
tick_subbus(s);
tic;
val = read_subbus(s,6);
if val ~= 0
  error('Read back 0x%04X from fail, expected 0', val);
end
fprintf(1,'Verified initial value after tick\n');
write_subbus(s,6,1);
val = read_subbus(s,6);
if val ~= 1
  error('Read back 0x%04X from fail, expected 1 (via software)', val);
end
fprintf(1,'Verified ability to set and readback fail in software\n');
write_subbus(s,6,0);
val = read_subbus(s,6);
if val ~= 0
  error('Read back 0x%04X from fail, expected 0', val);
end
fprintf(1,'Verified ability to reset software fail\n');
fprintf(1,'Waiting for 120 second timeout\n');
for i=1:124
  val = read_subbus(s,6);
  if val > 0
    break;
  end
  pause(1);
end
toc;
if val > 0
  fprintf(1,'Observed fail on timeout\n');
  write_subbus(s,6,2);
  val = read_subbus(s,6);
  if val == 3
    fprintf(1,'Verified fail still set when writing additional bits\n');
  else
    fprintf(1,'ERROR: Read 0x%04X, expected 3 after writing 2\n', val);
  end
  write_subbus(s,6,0);
  val = read_subbus(s,6);
  if val == 1
    fprintf(1,'Verified fail still asserted after writing 0\n');
  else
    fprintf(1,'ERROR: Read 0x%04X, expected 1 after writing 0\n', val);
  end
else
  fprintf(1,'ERROR: Did not observe fail after timeout\n');
end
%%
flush_input(s);
serial_port_clear();
delete(s);
clear s
