% Convert my readings to PX4 readings
% The output of this file can be replayed using the Ardupilot EKF library


%% File Paths
radio_path = '.\Logs\RADIO.txt';
gps_path = '.\Logs\GPS.txt';
imu_path = '.\Logs\IMU.txt';
baro_path = '.\Logs\BARO.txt';

%% Radio Data Reduction
radio = radio_import(radio_path, 3);

%% GPS Data Reduction
gps = brett_gps_import(gps_path, 4);
toDelete = find(gps.timestamp > gps.timestamp(end));
gps(toDelete,:) = [];
toDelete = find(gps.timestamp < gps.timestamp(1));
gps(toDelete,:) = [];

%% IMU Data Reduction
clear imu
imu = brett_imu_import(imu_path, 4);

% Delete readings with erroneous timestamps
toDelete = find(imu.timestamp > imu.timestamp(end));
imu(toDelete,:) = [];
toDelete = find(imu.timestamp < imu.timestamp(1));
imu(toDelete,:) = [];

% Convert timestamp to microseconds
imu_ts = imu.timestamp * 1000;

% Integrate accelerometer and gyro readings
accelerometer_integral_dt = diff(imu_ts);
accelerometer_integral_dt = cat(1, accelerometer_integral_dt(1), accelerometer_integral_dt);
gyro_integral_dt = diff(imu_ts);
gyro_integral_dt = cat(1, gyro_integral_dt(1), gyro_integral_dt);

% Smooth accel and gyro data
g = 9.87;
accelerometer_m_s20 = smoothdata((imu.accel_x) * g, 'rlowess', 20);
accelerometer_m_s21 = smoothdata((imu.accel_y)  * g, 'rlowess', 20);
accelerometer_m_s22 = smoothdata((imu.accel_z) * g, 'rlowess', 20);

gyro_rad0 = smoothdata((imu.gyro_x - 0.08)*((pi)/180), 'rlowess', 20);
gyro_rad1 = smoothdata((imu.gyro_y - 1.02)*((pi)/180), 'rlowess', 20);
gyro_rad2 = smoothdata((imu.gyro_z - 1.08)*((pi)/180), 'rlowess', 20);

% Calibrate magnetometer measurements using std calibration
m_center = [0.142905, 0.095924, -0.072416];
m_comp = [0.9767, 0.0400, 0.0135;, 0.0400, 0.9114, 0.0142; 0.0135, 0.0142, 0.9225];

mag_vals = [imu.mag_x, imu.mag_y, imu.mag_z];

% Hard iron correction (offset)
mag_vals = mag_vals - m_center;

% Soft iron correction (rotation)
mag_vals = ((m_comp) * (mag_vals)')';

% Smooth mag readings
magnetometer_ga0 = smoothdata((-1 * mag_vals(:,2)), 'rlowess', 20);
magnetometer_ga1 = smoothdata((-1 * mag_vals(:,1)), 'rlowess', 20);
magnetometer_ga2 = smoothdata((-1 * mag_vals(:,3)), 'rlowess', 20);


%% Barometer Data Reduction
clear baro
baro = brett_baro_import(baro_path, 4);

% Remove any baro readings with erroneous timestamps
toDelete = find(baro.timestamp > baro.timestamp(end));
baro(toDelete,:) = [];
toDelete = find(baro.timestamp < baro.timestamp(1));
baro(toDelete,:) = [];

% Place hard limits on barometer altitude readings
baro_ts = baro.timestamp * 1000;
K = 273.15;
t_offset = 100.0;
SeaPress = 101325;

% Calculate altitude from pressure
baro_alt_meter = altcalc(SeaPress, baro.temp ./ t_offset + K, baro.pressure);
range_limits = [500,1000];

lower = find(baro_alt_meter < range_limits(1));
upper = find(baro_alt_meter > range_limits(2));

% Interpolate erroneous readings to previous valid readings
for i = lower
    prev = baro_alt_meter(i - 1);
    next = baro_alt_meter(i + 1);
    avg = (prev + next)/2;
    baro_alt_meter(i) = avg;
end

for i = upper
    if i < 2
        prev = baro_alt_meter(i + 1);
    else
        prev = baro_alt_meter(i - 1);
    end
    next = baro_alt_meter(i + 1);
    avg = (prev + next)/2;
    baro_alt_meter(i) = avg;
end

lower = find(baro_alt_meter < range_limits(1));
upper = find(baro_alt_meter > range_limits(2));

assert(isempty(lower), "Data has values less than range limits")
assert(isempty(upper), "Data has values greater than range limits")

baro_alt_meter = smoothdata(baro_alt_meter, 'rlowess', 3);

clear i K lower next prev range_limits SeaPress upper avg toffset


function baro_alt_meter = altcalc(a, k, i)
    M = 0.0289644;
    g = 9.80665;
    R = 8.31432;
    d = -0.0065;
    e = 0;
    k = 273.15 + 26.5; % Force to constant
    j = (i ./ a) .^ ((R * d) ./ (g * M));
    baro_alt_meter = e + ((k .* ((1 ./ j) - 1)) / d);
end
