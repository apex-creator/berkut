%% Parse ADC Data
% This MATLAB script is to parse the ADC data collected via DCA1000 
clear variables;
figure(1); figure(2); figure(3); figure(4); 
clf(1); clf(2); clf(3); clf(4);

DeviceName = 'xWRLx432';
adcLogging = 2; % 1-DCA1000, 2-SPI based capture
sideband_data_present = 0; % 0=ADC_ONLY data, 1=ADC + sideband data
rdif_scrambler_en = 0; % 0=RDIF scrambler disabled, 1=RDIF scrambler enabled

fid = fopen('adc_data_spi.cfg');
tline = fgetl(fid);

while ischar(tline)
    cmd = split(tline,' ');
    tline = fgetl(fid);
    switch cmd{1}
        case 'channelCfg'
            chirpCfg.numRx = 3;
            chirpCfg.numTx = 2;
            if(str2num(cmd{2}) == 3 || str2num(cmd{2}) == 5 || str2num(cmd{2}) == 6)
                chirpCfg.numRx = 2;   
            elseif(str2num(cmd{2}) == 1 || str2num(cmd{2}) == 2 || str2num(cmd{2}) == 4)
                chirpCfg.numRx = 1;
            end
            if(str2num(cmd{3}) == 1 || str2num(cmd{3}) == 2)
                chirpCfg.numTx = 1;
            end 
        case 'chirpComnCfg'
            chirpCfg.samplingRate = 100/str2double(cmd{2});
            chirpCfg.numAdcSamples = str2num(cmd{5});
            chirpCfg.chirpTxMimoPatSel = str2num(cmd{6});
            chirpCfg.rampTime = str2num(cmd{7});
            
        case 'chirpTimingCfg'
            chirpCfg.idleTime = str2num(cmd{2});
            chirpCfg.slope = str2num(cmd{5});
            
        case 'frameCfg'
            chirpCfg.numChirps = str2num(cmd{2});
            chirpCfg.chirpsPerTx = chirpCfg.numChirps/chirpCfg.numTx;
            chirpCfg.numFrames = str2num(cmd{7});
            chirpCfg.numBursts = str2num(cmd{5});
    end
end



n_rx_chan = chirpCfg.numRx;                     
n_tx_chan = chirpCfg.numTx;                      
n_samp_per_chirp = chirpCfg.numAdcSamples;             
n_chirps_per_frame = chirpCfg.chirpsPerTx;            
n_frames = chirpCfg.numFrames; 
n_bursts = chirpCfg.numBursts;

Frame_num = 10;                     % Frame to Process
Chirp_count = n_chirps_per_frame;   % Chrips count to process
cp=[];
cq=[];
crc=[];
if adcLogging == 1
    adc_file_name = 'adc_data_Raw_0.bin';
    [adc_dca, cp, cq, crc] = ar_convertAdcData_xWRLx432(adc_file_name, n_rx_chan, ...
        n_tx_chan, n_samp_per_chirp, (n_chirps_per_frame*n_bursts*n_tx_chan), n_frames, sideband_data_present, rdif_scrambler_en); % n_rx_chan, n_samp_per_chirp, n_tx_chan, n_chirps_per_frame, n_frames
    adcOut = permute(adc_dca, [5, 3, 1, 4, 2]);
elseif adcLogging == 2
    %% Read ADC log file
    fid = fopen('adc_data_spi.txt');
    testadc = fscanf(fid, '%d');

    fclose(fid);
    %testAdcreshape = reshape(testadc,[numAdcSamples,numRx,numChirps*numBursts,numFrames]);
    adc_spi = reshape(testadc,[n_samp_per_chirp,n_rx_chan,n_tx_chan,(n_chirps_per_frame*n_bursts),n_frames]);
    adcOut = permute(adc_spi, [5, 3, 2, 4, 1]);
end


    %-------------------------------------------------------------------------- 
    adcOut_woDC = adcOut-mean(adcOut); % Remove DC components
    figure(1);
    sgtitle(strcat(DeviceName, ' ADC Output'))
    for j = 1:n_tx_chan
        for k = 1:n_rx_chan
            subplot(n_tx_chan, n_rx_chan, (j-1)*n_rx_chan+k)
            plot(squeeze(real(adcOut(Frame_num,j,k,1:Chirp_count,:)))');
            xlim([0 128])
            xticks([0 64 128]) 
            ylim([-150 150])
            yticks([-150 -100 -50 0 50 100 150])
            xlabel('Sample #')
            ylabel('ADC Out (code)')
            title(strcat('TX',int2str(j), '\rightarrowRX',int2str(k)))
            grid on
        end
    end

    %--------------------------------------------------------------------------
    % 1D FFT: Range FFT
    fftSize1D = size(adcOut,5);
    radar_data_1dFFT = fft(adcOut,[], 5);
    radar_data_1dFFTMag = abs(radar_data_1dFFT);
    figure (2);
    sgtitle(strcat(DeviceName, ' 1D FFT'))
    for j = 1:n_tx_chan
        for k = 1:n_rx_chan
            subplot(n_tx_chan, n_rx_chan, (j-1)*n_rx_chan+k)
            plot(10*log10(squeeze(abs(radar_data_1dFFTMag(Frame_num,j,k,1:Chirp_count,:)))'));
            xlim([0 128])
            xticks([0 64 128]) 
            ylim([-10 60])
            yticks([-10 0 10 20 30 40 50 60])
            xlabel('Bin #')
            ylabel('Mag (dBcode)')
            title(strcat('TX',int2str(j), '\rightarrowRX',int2str(k)))
            grid on
        end
    end

    %--------------------------------------------------------------------------
    % 2D FFT: Doppler FFT
    fftSize2D = size(adcOut, 4);
    radar_data_2dFFT = fft(radar_data_1dFFT,[],4);
    % non-coherent combination cross antenna to get ready for peak detection.
    radar_data_2dFFTMag_allAnt = squeeze(sum(abs(radar_data_2dFFT).^2, [2 3])); 
    % Plot 2D FFT output
    figure(4);
    plot(10*log10(squeeze(abs(radar_data_2dFFTMag_allAnt(Frame_num,:,:))))');
    xlim([0 128]) 
    xticks([0 32 64 96 128]) 
    ylim([0 120])
    yticks([0 20 40 60 80 100 120])
    xlabel('Bin #')
    ylabel('\Sigma Mag^2 (dBcode)')
    title('Sigma Power Spectrum of ALL TX\rightarrowRX Combinations')
    
    grid on
    % Single Channel 2D FFT Plot
    radar_data_2dFFTMag = abs(radar_data_2dFFT).^2; 
    figure(3);
    sgtitle(strcat(DeviceName, ' 2D FFT'))
    for j = 1:n_tx_chan
        for k = 1:n_rx_chan
            subplot(n_tx_chan, n_rx_chan, (j-1)*n_rx_chan+k)
            plot(squeeze(10*log10(abs(radar_data_2dFFTMag(Frame_num,j,k,:,:))))');
            xlim([0 128]) 
            xticks([0 64 128])
            ylim([0 120])
            yticks([0 20 40 60 80 100 120])
            xlabel('Bin #')
            ylabel('Mag^2 (dBcode)')
            title(strcat('TX',int2str(j), '\rightarrowRX',int2str(k)))
            grid on
        end
    end 
    
    
   