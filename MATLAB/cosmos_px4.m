%% convert baro data
clear baro_data;
last_time = 0;
output_index = 1;
timestamp = baro_ts;
for source_index = 1:length(timestamp)
    baro_timestamp = timestamp(source_index);
    if (baro_timestamp ~= last_time)
        baro_data.time_us(output_index,1) = baro_timestamp;
        baro_data.height(output_index) = baro_alt_meter(source_index);
        last_time = baro_timestamp;
        output_index = output_index + 1;
    end
end

%% convert IMU data to delta angles and velocities
% Note: these quatntis were converted from delta angles and velocites using
% the integral_dt values in the PX4 sensor module so we only need to
% multiply by integral_dt to convert back
clear imu_data;
timestamp = imu_ts;
n_samples = length(timestamp);
imu_data.time_us = timestamp;
imu_data.gyro_dt = gyro_integral_dt ./ 1e6;
imu_data.del_ang = [gyro_rad0.*imu_data.gyro_dt, gyro_rad1.*imu_data.gyro_dt, gyro_rad2.*imu_data.gyro_dt];

imu_data.accel_dt = accelerometer_integral_dt ./ 1e6;
imu_data.del_vel = [accelerometer_m_s20.*imu_data.accel_dt, accelerometer_m_s21.*imu_data.accel_dt, accelerometer_m_s22.*imu_data.accel_dt];

%% convert magnetomer data
clear mag_data;
last_time = 0;
output_index = 1;
timestamp = imu_ts;
for source_index = 1:length(timestamp)
    mag_timestamp = timestamp(source_index);
    if (mag_timestamp ~= last_time)
        mag_data.time_us(output_index,1) = mag_timestamp;
        mag_data.field_ga(output_index,:) = [magnetometer_ga0(source_index),magnetometer_ga1(source_index),magnetometer_ga2(source_index)];
        last_time = mag_timestamp;
        output_index = output_index + 1;
    end
end


%% convert GPS data
clear gps_data;
timestamp = gps.timestamp;
gps_data.time_us = gps.timestamp * 1000;
gps_data.pos_error = gps.h_acc / 1000;
gps_data.spd_error = gps.s_acc / 1000;
gps_data.hgt_error = gps.v_acc / 1000;
lat = gps.lat;
lon = gps.lon;
alt = gps.hmsl;
vel_n_m_s = gps.vel_n / 1000;
vel_e_m_s = gps.vel_e / -1000;
vel_d_m_s = gps.vel_d / -1000;
fix_type = gps.fixtype;

% set reference point used to set NED origin when GPS accuracy is sufficient
gps_data.start_index = max(min(find(gps_data.pos_error < 5.0)),min(find(gps_data.spd_error < 1.0)));
if isempty(gps_data.start_index)
    gps_data.start_index = 1;
    gps_data.refLLH = [1e-7*median(lat);1e-7*median(lon);0.001*median(alt)];
else
    gps_data.refLLH = [1e-7*lat(gps_data.start_index);1e-7*lon(gps_data.start_index);0.001*alt(gps_data.start_index)];
end

% convert GPS data to NED
for index = 1:length(timestamp)
    if (fix_type(index) >= 3)
        gps_data.pos_ned(index,:) = LLH2NED([1e-7*lat(index);1e-7*lon(index);0.001*alt(index)],gps_data.refLLH);
        gps_data.vel_ned(index,:) = [vel_n_m_s(index),vel_e_m_s(index),vel_d_m_s(index)];
    else
        gps_data.pos_ned(index,:) = [0,0,0];
        gps_data.vel_ned(index,:) = [0,0,0];
    end
end

%% save data and clear workspace
clearvars -except baro_data imu_data mag_data gps_data;

save baro_data.mat baro_data;
save imu_data.mat imu_data;
save mag_data.mat mag_data;
save gps_data.mat;
