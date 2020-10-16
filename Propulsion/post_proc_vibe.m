%% **********************************************************************
% Get information on the time and frequency content of the
% system vibrational data runninig at various commanded motor RPMs
%
close all

STEP     = (20:20:300);             % List of commanded RPMs to study
FREQ     = STEP/60;                 % RPM in Hz
rms_acel = zeros(size(STEP,2),3);   % to store rms value of vibrations

peak_i   = zeros(size(STEP,2),3);   % index of peaking in frequency domain
peak_f   = peak_i;                  % frequency at index of peak
peak_a   = peak_i;                  % amplitude at index of peak
peak_f_a = zeros(size(STEP,2),1);   % store peak ampl in FFT across 3 dims

for ii = 1:size(STEP,2)
  load([num2str(STEP(ii)),'_20_profiles.mat']); % load the data
  dat_sz   = size(acel_data);                   % how many samples
  atime    = mean(diff(data_record(:,1)));      % ave time between samples
  Fs       = 1/atime;                           % Effective Sampling Freq
  off_acel = mean(acel_data,1);                 % ave (gravity) value
  cor_acel = acel_data-off_acel;                % offset corrected samples
  
  % get the rms value for the 30% to 70% range of the time data
  % i.e. after aceleration is done, before decelration starts
  % i.e. during the steady state RPM portion of the step
  
  start    = floor(size(cor_acel,1)*0.3);
  stop     = floor(size(cor_acel,1)*0.7);
  rms_acel = sqrt(mean((cor_acel(start:stop,:)).^2));

  figure,            
  subplot(2,1,1)      % plot all offset corrected values vs time
  yyaxis 'left'
  plot(acel_data(:,4), cor_acel(:,1), '-m', ...
       acel_data(:,4), cor_acel(:,2), '-g', ...
       acel_data(:,4), cor_acel(:,3), '-b');
  grid('on'),
  title(['Vibration and RPM for 0 to ',num2str(STEP(ii)), ' RPM step']);
  xlabel('Time(sec)'), ylabel('Acceleration (m/s^2)')
  ylim([-15, 15])

  yyaxis 'right'      % plot all RPM step response vs. time
  plot(data_record(:,1), data_record(:,2), '-r');
  ylim([-(STEP(ii)*0.1), STEP(ii)*1.1]), ylabel('RPM')
  legend('X axis', 'Y-axis', 'Z-axis', 'RPM');

  % Now get the frequency content of the samples
  % 1st re-interpolate data onto evenly sampled time samples
  %
  zz       = 1;  % oversampling factor for interpolation prior to FFT
  etime    = (data_record(1,1):atime/zz:data_record(end,1));
  int_acel = interp1(acel_data(:,4), cor_acel(:,1:3), etime, 'PCHIP');

  % Now take FFT of the interpolated, evenly time spaced data
  % using only samples between 30% and 70% of the run
  % i.e. after the acceleration is complete and before deceleration begins
  % i.e. only the steady state, settled RPM samples
  
  start = floor(size(int_acel,1)*0.3);
  stop  = floor(size(int_acel,1)*0.7);
  xoft  = int_acel(start:stop,:); % functions for FFT
  
  L     = size(xoft,1);           % number of samples in function
  Fmax  = Fs/2;                   % Max Freq = 1/2 the Sample Freq
  Fstep = Fs/L;                   % Freq step size = Fsample/(number of samples)
  freq  = Fstep*(0:ceil(L/2));    % Make the appropriate frequency vector
  
  Y  = fft(xoft);                 % complex two-sided FFT
  P2 = abs(Y/L);                  % normalize magnitude by # of samples
  P1 = P2(1:ceil(L/2)+1,:);       % get postive half plus DC value
  P1(2:end-1,:)= 2*P1(2:end-1,:); % double it for single sided but not DC value
  
  % Now find the peak of the FFT along all 3 acelerometer axis's
  
  for jj=1:3    
    P1(:,jj)      = smooth(P1(:,jj),5);             % smooth the response
    peak_i(ii,jj) = find(P1(:,jj)==max(P1(:,jj)));  % find the peak index
    peak_f(ii,jj) = freq(peak_i(ii,jj));            % find the peak freq
    peak_a(ii,jj) =   P1(peak_i(ii,jj),jj);         % find the peak amp
  end
  peak_f_ind = find(peak_a(ii,:)==max(peak_a(ii,:)));
  peak_f_a(ii) = peak_f(ii,peak_f_ind);
  % Now plot the FFT results
  
  subplot(2,1,2)
  plot(freq(1:round(end/zz)),P1(1:round(end/zz),1), '-r', ... 
       freq(1:round(end/zz)),P1(1:round(end/zz),2), '-g', ...
       freq(1:round(end/zz)),P1(1:round(end/zz),3), '-b')
  title({'Single-Sided Amplitude Spectrum of Vibrations',; ...
        [' expected peak = ', num2str(FREQ(ii),2), ' Hz',  ...
         ' measured peak = ', num2str(peak_f_a(ii),2), ' Hz']})
  xlabel('f (Hz)'), ylabel('|P1(f)|'), grid('on')
  legend('X-axis', 'Y-axis', 'Z-axis');
end

