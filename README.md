# SFND_Radar_Target_Generation_and_Detection


## FP1. FMCW Configuration
Configure the FMCW waveform based on the system requirements. 
### Radar System Requirements
![Radar System Requirements](/Radar_Sysyem_Requirements.png)

**Radar System Requirements**
```
%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.
B = c / (2*d_res);
Tchirp = 5.2*2*R_max/c;
slope = B/Tchirp;

%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq
                                                      
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));

```

## FP.2 Moving Target Generation and Signal Propogation
Define the range and velocity of target and simulate its displacement. For the same simulation loop process the transmit and receive signal to determine the beat signal
```
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
 R_target = 110;
 v_target = -20;

...

%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = R_target + v_target*t(i);
    td(i) = 2*r_t(i)/c;
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i) + slope*t(i)^2/2));
    Rx(i) = cos(2*pi*(fc*(t(i)-td(i)) + slope*(t(i)-td(i))^2/2));
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i).*Rx(i);
    
end

```

## FP.3 Range/Doppler FFT
Perform Range FFT on the received signal to determine the Range. Then generate a Range Doppler Map.  
![Fast Fourier Transform](/FFT2.png)

**Fast Fourier Transform**
```
%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix = reshape(Mix, [Nr, Nd]);

% *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
signal_fft = fft(Mix, Nr, 1);

 % *%TODO* :
% Take the absolute value of FFT output
P2 = abs(signal_fft/Nr);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
P1 = P2(1:Nr/2,:);


%plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)

 % *%TODO* :
 % plot FFT output
plot(P1(:,1));
axis([0 200 0 0.5]);

%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.
 
% Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

```
![Range from First FFT](/FIgure1.png)

**Range from First FFT**
![2DFFT](/Figure2.jpg)

**2DFFT**

## FP.4 CFAR Detection
Towards the end, perform the CFAR processing on the output of 2nd FFT to display the target.
![2D CFAR](/2DCFAR.png)
I set

|     Tr      |      Td   |     Gr     |     Gd   |
|:----------:|:------------:|:-------------:|:--------------:|
|  16    |        8    |     2     |      4     | 

**Implementig 2D CFAR steps**
1. Determine the number of Training cells for each dimension. Similarly, pick the number of guard cells.
2. Slide the cell under test across the complete matrix. Make sure the CUT has margin for Training and Guard cells from the edges.
3. For every iteration sum the signal level within all the training cells. To sum convert the value from logarithmic to linear using db2pow function.
4. Average the summed values for all of the training cells used. After averaging convert it back to logarithmic using pow2db.
5. Further add the offset to it to determine the threshold.
6. Next, compare the signal under CUT against this threshold.
7. If the CUT level > threshold assign it a value of 1, else equate it to 0.

and the last, To keep the map size same as it was before CFAR, 
8. equate all the non-thresholded cells to 0.

```
%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr = 16;
Td = 8;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 2;
Gd = 4;

% *%TODO* :
% offset the threshold by SNR value in dB
offset = 3;
% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);


% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.

RDM = RDM/max(RDM(:)); %normalize

   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR
   Nr = Nr/2;

numTrainingCell = (2*Td+2*Gd+1)*(2*Tr+2*Gr+1) - (2*Gd+1)*(2*Gr+1);
for i = Tr+Gr+1:Nr-(Tr+Gr)
    for j = Td+Gd+1:Nd-(Td+Gd)
        avgTraining = (sum( db2pow(RDM( i-(Gr+Tr):i+(Gr+Tr),j-(Gd+Td):j+(Gd+Td) )), 'all') ...
        - sum(db2pow(RDM(i-Gr:i+Gr,j-Gd:j+Gd)), 'all'))/numTrainingCell;
        noise_level = pow2db(avgTraining);
        threshold = noise_level*offset;
        CUT = RDM(i,j);
        if (CUT > threshold)
            RDM(i,j) = 1;
        else
            RDM(i,j) = 0;
        end
    end
end


% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 

RDM(RDM ~= 1 & RDM ~= 0) = 0;
```
![2DFFT](/Figure2.jpg)

**2DFFT**
![2DCFAR](/Figure3.jpg)

**2DCFAR**

Comparing those two results, we can see that noise is eliminated successfully.
