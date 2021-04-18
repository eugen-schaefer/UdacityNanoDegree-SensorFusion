close all
clear
clc;


%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation / carrier freq = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
carrier_frequency = 77e9;
max_range = 200;
range_resolution = 1;
max_velocity = 100;


%% User defined range and velocity of target
%  define the target's initial position and velocity. Note : Velocity
%  remains contant
target_range = 110;
target_velocity = -20;
 

%% FMCW Waveform Generation
reset_chirp = true;

% Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of
% the FMCW chirp using the requirements above.
speed_of_light = physconst('LightSpeed');
bandwidth = speed_of_light / (2 * range_resolution);
chirp_time = 5.5 * 2 * max_range / speed_of_light;
slope = bandwidth / chirp_time;

                                                          
% Its ideal to have 2^ value for the ease of running the FFT for Doppler Estimation. 
number_chirps=128;
samples_per_chirp=1024;

% Timestamp for running the displacement scenario for every sample on each
% chirp
time_span = linspace(0, number_chirps*chirp_time, ...
                        samples_per_chirp*number_chirps);
sample_time = median(diff(time_span));

% Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(time_span)); %transmitted signal
Rx=zeros(1,length(time_span)); %received signal
Mix = zeros(1,length(time_span)); %beat signal

frequency_tx = zeros(1,length(time_span));
frequency_rx = zeros(1,length(time_span));

if reset_chirp
    max_roundtrip_time = 2*max_range/speed_of_light;
    truncate_samples = ceil(max_roundtrip_time / sample_time);
else
    truncate_samples = 0;
end


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(time_span)         
    t = time_span(i);

    %For each time stamp update the Range of the Target for constant velocity.
    target_range = target_range + target_velocity * sample_time;

    %For each time sample we need update the transmitted signal
    if reset_chirp
        delta_freq_tx = 0.5*slope*mod(t, chirp_time);
    else
        delta_freq_tx = 0.5*slope*t;
    end
    frequency_tx(i) = carrier_frequency + delta_freq_tx;
    Tx(i) =  cos(2*pi*frequency_tx(i)*t);
    
    %For each time sample we need update the received signal
    roundtrip_time = 2*target_range/speed_of_light;
    if reset_chirp
        delta_freq_rx = 0.5*slope*mod((t-roundtrip_time), chirp_time);
    else
        delta_freq_rx = 0.5*slope*(t-roundtrip_time);
    end
    frequency_rx(i) = carrier_frequency + delta_freq_rx;
    Rx(i) =  cos(2*pi*frequency_rx(i)*(t-roundtrip_time));
    
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmiter and
    %Receiver Signal
    Mix(i) = Tx(i).*Rx(i);
end


%% RANGE MEASUREMENT

% Reshape the vector into samples_per_chirp*number_chirps array. 
% samples_per_chirp and number_chirps here would also define the size of
% Range and Doppler FFT respectively.
mixed_signal_matrix = reshape(Mix, [samples_per_chirp, number_chirps]);

% For every chirp, truncate mixed signal on chirp boundaries such that we 
% can estimate correct beat frequency
mixed_signal_matrix = mixed_signal_matrix(truncate_samples+1:end, :);

Ts = median(diff(time_span));
Fs = 1/Ts;                    
L = samples_per_chirp - truncate_samples;
f = Fs*(0:(L/2))/L;
d = f*chirp_time*speed_of_light/(2*bandwidth);


% Run the FFT on the beat signal along the range bins dimension 
% (samples_per_chirp) and normalize.
range_fft = fft(mixed_signal_matrix(:,1));


% Take the absolute value of FFT output
P2 = abs(range_fft/L);

% Output of FFT is double sided signal, but we are interested in only one 
% side of the spectrum. Hence we throw out half of the samples.
P1 = P2(1:floor(L/2)+1);

% Plot
figure ('Name', 'Range-FFT (1D)', 'NumberTitle', 'off');
tiledlayout(2,1);

nexttile
plot(d, P1);
title('Single-Sided Amplitude Spectrum of X(t)');
xlabel('Radial range (m)');
ylabel('|1D-FFT|');
axis ([0 200 0 1]);

nexttile
plot(time_span, frequency_tx);
hold on;
plot(time_span, frequency_rx);
title('Chirp frequency over time');
xlabel('Time [s]');
ylabel('Frequency [Hz]');
legend('TX-chirp', 'RX-chirp')


%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map. You will implement CFAR on the generated RDM

% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

% 2D FFT using the FFT size for both dimensions.
reduced_samples_per_chirp = samples_per_chirp-truncate_samples;
sig_fft2 = fft2(mixed_signal_matrix, reduced_samples_per_chirp, number_chirps);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:floor(reduced_samples_per_chirp/2),1:number_chirps);
sig_fft2 = fftshift(sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

% use the surf function to plot the output of 2DFFT and to show axis in both
% dimensions
doppler_axis = linspace(-100, 100, number_chirps);
range_axis = linspace(-200, 200, floor(reduced_samples_per_chirp/2))*(floor(reduced_samples_per_chirp/2)/400);

figure ('Name', 'Range-Doppler-FFT (2D)', 'NumberTitle', 'off');
title('Amplitude spectrum of X(t)');
surf(doppler_axis, range_axis, RDM);
xlabel('Radial velocity (m/s)');
ylabel('Radial range (m)');
zlabel('|2D-FFT| in dB');


%% CFAR implementation

% Select the number of Training Cells in both the dimensions.
Tr = 10;
Td = 10;

% Select the number of Guard Cells in both dimensions around the Cell under 
% test (CUT) for accurate estimation
Gr = 4;
Gd = 4;

% offset the threshold by SNR value in dB
offset = 10;


% Design a loop such that it slides the CUT across range doppler map by
% giving margins at the edges for Training and Guard Cells.
% For every iteration sum the signal level within all the training
% cells. To sum convert the value from logarithmic to linear using db2pow
% function. Average the summed values for all of the training
% cells used. After averaging convert it back to logarithimic using pow2db.
% Further add the offset to it to determine the threshold. Next, compare the
% signal under CUT with this threshold. If the CUT level > threshold assign
% it a value of 1, else equate it to 0.

% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR
RDM_CFAR = zeros(size(RDM));
for range_idx = 1:(size(RDM, 1) - (2*Tr+2*Gr+1))
    for doppler_idx = 1:(size(RDM, 2) - (2*Td+2*Gd+1))
        
        T_span_range = range_idx:range_idx+2*Tr;
        T_span_doppler = doppler_idx:doppler_idx+2*Td;
        
        G_span_range = (range_idx+Tr-Gr):(range_idx+2*Tr-(Tr-Gr));
        G_span_doppler = (doppler_idx+Td-Gd):(doppler_idx+2*Td-(Td-Gd));
        
        guard_cells_sum = sum(db2pow(RDM(G_span_range, G_span_doppler)), 'all');
        guard_cells_num = numel(RDM(G_span_range, G_span_doppler));
        
        training_cells_sum = sum(db2pow(RDM(T_span_range, T_span_doppler)), 'all') - guard_cells_sum;
        training_cells_num = numel(RDM(T_span_range, T_span_doppler)) - guard_cells_num;
        
        training_cells_avg = pow2db(training_cells_sum/training_cells_num);
        
        range_idx_CUT = range_idx + Tr;
        doppler_idx_CUT = doppler_idx + Td;
        
        CUT = RDM(range_idx_CUT, doppler_idx_CUT);
        
        if(CUT > training_cells_avg + offset)
            RDM_CFAR(range_idx_CUT, doppler_idx_CUT) = 1;
        end
    end
end


% Display the CFAR output
figure ('Name', 'CA-CFAR', 'NumberTitle', 'off');
surf(doppler_axis,range_axis, RDM_CFAR);
title('Cell Averaging CFAR (CA-CFAR)');
xlabel('Radial velocity (m/s)');
ylabel('Radial range (m)');
zlabel('0/1 Detection ');