function [radar_data_all, cp, cq, crc] = ar_convertAdcData_xWRLx432(adc_file_name, num_rx_channels, num_tx_channels, num_samples_per_chirp, num_chirps_per_frame, num_frames, sideband_data_present, rdif_scrambler_en)
    % Read data from file
    fileID = fopen(adc_file_name, 'r');
    readFmt = 'uint16';
    if (sideband_data_present == 1)
        % Sideband data present
        num_samples_cp = 3;  % number of 12-bit samples
        num_samples_cq = 3;  % number of 12-bit samples (equal to max number of RX channels)
        num_samples_crc = 6;  % number of 12-bit samples
        total_sideband_samples = num_samples_cp + num_samples_cq + num_samples_crc;
    else
        total_sideband_samples = 0;
    end
    total_num_samples_in_file = ((num_rx_channels * num_samples_per_chirp) + total_sideband_samples) * num_chirps_per_frame * num_frames;
    radar_data_all = fread(fileID, total_num_samples_in_file, readFmt);

    % 64xx/14xx RDIF processing (radar_data_all is uint16 datatype)
    radar_data_all = radar_data_all(1:floor(length(radar_data_all) / 4) * 4);  % make sure data is multiple of 64 bits
    radar_data_all = reshape(radar_data_all, [4 (length(radar_data_all) / 4)]);  % break into chunks of 64 bits

    % De-scramble data if RDIF scrambler was enabled
    if (rdif_scrambler_en == 1)
        % RDIF scrambler was enabled
        num_64bitchunks = size(radar_data_all, 2);
        radar_data_all_unscrambled = radar_data_all;
        M = num_64bitchunks; % 1000; % num of samples to process per iter. Can take any value from 2 to num_64bitchunks.
                  % Larger M will run faster but will require more memory
        for lane = 1:4
            for block64_index = 1:(num_64bitchunks-M+1)
                if (mod(block64_index,M) == 1) || (block64_index == (num_64bitchunks-M+1)) % Every Mth iteration or last iteration
                    switch block64_index
                        case 1 % First sample
                            preceding_18bits = zeros(1,18);
                            preceding_23bits = [zeros(1,18) ones(1,5)];
                        case 2 % Second sample
                            preceding_18bits = [zeros(1,6) bin_mat_ln_MSB_to_LSB(1:12)];
                            preceding_23bits = [zeros(1,6) ones(1,5) bin_mat_ln_MSB_to_LSB(1:12)];
                        case (num_64bitchunks-M+1) % If last iteration which is not 1 or 2
                            bin_mat_ln_MSB_to_LSB = dec2bin(radar_data_all(lane,(num_64bitchunks-M-1):(num_64bitchunks-M)),12)-'0'; % Convert previous 2 samples to binary (24 bits)
                            bin_mat_ln_MSB_to_LSB = reshape(bin_mat_ln_MSB_to_LSB.',1,[]);
                            preceding_18bits = reshape(bin_mat_ln_MSB_to_LSB(end-17:end),1,[]); % Extract the last 18 bits from previous block of 2 samples (24 bits)
                            preceding_23bits = reshape(bin_mat_ln_MSB_to_LSB(end-22:end),1,[]); % Extract the last 23 bits from previous block of 2 samples (24 bits)
                        otherwise % Middle iterations
                            preceding_18bits = reshape(bin_mat_ln_MSB_to_LSB(end-17:end),1,[]); % Extract the last 18 bits from previous block of M samples
                            preceding_23bits = reshape(bin_mat_ln_MSB_to_LSB(end-22:end),1,[]); % Extract the last 23 bits from previous block of M samples
                    end
                    bin_mat_ln_MSB_to_LSB = dec2bin(radar_data_all(lane,block64_index:(block64_index+M-1)),12)-'0'; %Convert a block of M samples to binary (12M bits)
                    bin_mat_ln_MSB_to_LSB = reshape(bin_mat_ln_MSB_to_LSB.',1,[]);
                    dlayed_ver1 = [preceding_18bits bin_mat_ln_MSB_to_LSB(1:end-18)];
                    dlayed_ver2 = [preceding_23bits bin_mat_ln_MSB_to_LSB(1:end-23)];
                    tmp1 = bitxor(bin_mat_ln_MSB_to_LSB,dlayed_ver1);
                    tmp2 = bitxor(tmp1,dlayed_ver2);
                    tmp3 = reshape(tmp2,12,[]);
                    str_x = num2str((tmp3.'));
                    str_x(strcmp(str_x,' ')) = '';
                    radar_data_all_unscrambled(lane,block64_index:(block64_index+M-1)) = bin2dec(str_x).';
                end
            end
        end
        radar_data_all = reshape(radar_data_all_unscrambled, 4, []);  % break into chunks of 64 bits
    end

    % Remaining 64xx/14xx RDIF processing (radar_data_all is uint16 datatype)
    for block64_index = 1:size(radar_data_all, 2)
        bit_vector = zeros(4, 12);
        for block16_index = 1:4
            % Re-arrange the bits as per RDIF Swizzling Mode 2 (output pattern)
            bit_vector(block16_index,:) = bitget(radar_data_all(block16_index, block64_index), [10:12 7:9 4:6 1:3]);
        end
        bit_vector = reshape(bit_vector, [12, 4]);  % partition into 12-bit values
        radar_data_all(:, block64_index) = 2.^(0:11) * bit_vector;  % convert bits to sample values
    end
    radar_data_all = reshape(radar_data_all, 1, []);  % 1-D array with index: samples
    radar_data_all = single(radar_data_all);
    
    % Separate CQ/CP/CRC from ADC data
    if (sideband_data_present == 1)
        % Sideband data present
        radar_data_all = reshape(radar_data_all, (num_rx_channels * num_samples_per_chirp) + total_sideband_samples, num_chirps_per_frame * num_frames);
        sideband_data = radar_data_all(num_rx_channels * num_samples_per_chirp + (1:total_sideband_samples), :);

        cp = sideband_data(1:num_samples_cp, :);  % FRAME_CNT, BURST_CNT, CHIRP_CNT
        cp = reshape(cp, num_samples_cp, num_chirps_per_frame, num_frames);

        cq = sideband_data(num_samples_cp + (1:num_samples_cq), :);  % SAT_CNT_CH3, SAT_CNT_CH2, SAT_CNT_CH1
        cq = flip(cq, 1);  % Reverse channels so order is now SAT_CNT_CH1, SAT_CNT_CH2, SAT_CNT_CH3
        cq = reshape(cq, num_samples_cq, num_chirps_per_frame, num_frames);

        crc = sideband_data(num_samples_cp + num_samples_cq + (1:num_samples_crc), :);  % 2 samples padding, LSB sample, MSB sample, 2 samples padding
        crc = crc(3, :) + (2^12 * crc(4, :));  % Combine LSB 12-bits with MSB 12-bits (actual CRC-16 is only 16 bits)
        crc = reshape(crc, num_chirps_per_frame, num_frames);

        % Remove sideband data to get ADC_ONLY data
        radar_data_all = radar_data_all(1:num_rx_channels * num_samples_per_chirp, :);
    else
        % Return empty arrays if sideband data was not present
        cp = [];
        cq = [];
        crc = [];
    end
    
    
    % Rearrange the ADC data into a 4D matrix with dimensions: channels x samples x chirps x frames
    radar_data_all = reshape(radar_data_all, num_rx_channels, num_samples_per_chirp, num_tx_channels, (num_chirps_per_frame/num_tx_channels), num_frames);

    % Convert from 2's complement
    l_max = 2^(12 - 1) - 1;
    radar_data_all(radar_data_all > l_max) = radar_data_all(radar_data_all > l_max) - 2^12;
end



