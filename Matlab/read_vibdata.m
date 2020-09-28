function read_vibdata(s, ~)
% callback function to read Vibration x, Y, Z, and plot every 100 points
% s = serialport object
% ~ = Not Used
%

% Read the ASCII data from the serialport object 
%   and cast to char to allow parse
data = char(readline(s));

% Parse and convert data to numeric type and save it in the UserData
% property of the serialport object.
if isempty(data)
  fprintf(1, 'Empty Line!\n');
elseif data(1) == 'X'
  s.UserData.VibX(end+1) = str2double(data(2:end));
elseif data(1) == 'Y'
  s.UserData.VibY(end+1) = str2double(data(2:end));
elseif data(1) == 'Z'
  s.UserData.VibZ(end+1) = str2double(data(2:end))-9.8;
else
  fprintf(1, '-- Unknown Error --\n');
end

% Update the Count value of the serialport object.
s.UserData.Count = s.UserData.Count + 1;

points = 60;	% remember: 20 points x 3 axes


% If 'points' data points have been collected, switch off the
% callbacks and plot the data.
if s.UserData.Count > points % (3 axes) * (# points)

    plot(s.UserData.VibX(1:end));
	hold on
    plot(s.UserData.VibY(1:end));
	hold on
    plot(s.UserData.VibZ(1:end));
	hold off
    
% After plot, clear data
 	s.UserData.VibX = [];
 	s.UserData.VibY = [];
 	s.UserData.VibZ = [];

	s.UserData.Count = 1;
end



