%  ICM20948 vibration data receive/plot
%  Marco Rivero
%  11:49 AM 9/21/2020

cd C:\huarp\ElecCore\IMU\code\Matlab
%%
serial_port_clear();
%%

%[s,port] = serial_port_init('COM9');
% Shortcut to []=serial_port_inti()
port = 'COM9';
% s = serialport(port,'BaudRate',115200,'InputBufferSize',3000,'Timeout',0.1,'Terminator',"CR/LF");
s = serialport(port,115200);

configureTerminator(s,"CR/LF");

% remove old serial data ?
flush(s);

% Serialport UserData property saves vibration data and point count
s.UserData = struct("VibX",[],"VibY",[],"VibZ",[],"Count",1);


% Begin receiving continuous stream from ICM20948
fprintf(1, 'Accelrometer Vibration Data test ICM20948\n');

configureCallback(s,"terminator",@read_vibdata);

figure('Name','XYZ Vibration Data');
axis([0,20,-5,5]);

