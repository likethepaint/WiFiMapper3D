% Color Plotting Test

import_cosmos_data
convert_cosmos_px4


% Limit barometer measurement window to GPS window
interp_baro = baro_data;
toDelete = find(interp_baro.time_us < gps_data.time_us(1));
interp_baro.time_us(toDelete) = [];
interp_baro.height(toDelete) = [];
toDelete = find(interp_baro.time_us > gps_data.time_us(end));
interp_baro.time_us(toDelete) = [];
interp_baro.height(toDelete) = [];

% Grab NED coordinates from GPS data
north = gps_data.pos_ned(:,1);
east = gps_data.pos_ned(:,2);
down = gps_data.pos_ned(:,3);

% Interpolate GPS data to barometer timestep
north_interp = interp1(gps_data.time_us, north, interp_baro.time_us);
east_interp = interp1(gps_data.time_us, east, interp_baro.time_us);
down_interp = interp1(gps_data.time_us, down, interp_baro.time_us);

% Interpolate Radio data to barometer timestep
radio_interp = interp1(radio.timestamp * 1000, radio.avg_rssi, interp_baro.time_us);


%% First Set
one_third = floor(length(north_interp) / 3);
two_third = floor(one_third * 2) - 650;
one_third = one_third + 1000;

north_1 = north_interp(1:one_third);
east_1 = east_interp(1:one_third);

down_interp = smooth(interp_baro.height);
down_interp = down_interp(1) - down_interp;

down_1 = down_interp(1:one_third);
radio_1 = radio_interp(1:one_third);

x = north_1;
y = east_1;
z = down_1;

% Generate gridded data from NED
grid_x = linspace(min(x), max(x), 50);
grid_y = linspace(min(y), max(y), 50);
[xx,yy] = meshgrid(grid_x, grid_y);

% Height data
zz = griddata(x,y,z,xx,yy);

% Radio data
rr = griddata(x,y,radio_1, xx, yy);

% Generate surface plot for first layer
s1 = surf(xx,yy,zz);

% Set color of first layer to RSSI
s1.CData = rr;
hold on;

%% Second Set
north_2 = north_interp(one_third:two_third);
east_2 = east_interp(one_third:two_third);
down_2 = down_interp(one_third:two_third);
radio_2 = radio_interp(one_third:two_third);

x = north_2;
y = east_2;
z = down_2;

grid_x = linspace(min(x), max(x), 50);
grid_y = linspace(min(y), max(y), 50);
[xx,yy] = meshgrid(grid_x, grid_y);

zz = griddata(x,y,z,xx,yy);
rr = griddata(x,y,radio_2, xx, yy);

s2 = surf(xx,yy,zz);
s2.CData = rr;

%% Thrid Set
north_3 = north_interp(two_third:end);
east_3 = east_interp(two_third:end);
down_3 = down_interp(two_third:end);
radio_3 = radio_interp(two_third:end);

x = north_3;
y = east_3;
z = down_3;

grid_x = linspace(min(x), max(x), 50);
grid_y = linspace(min(y), max(y), 50);
[xx,yy] = meshgrid(grid_x, grid_y);

zz = griddata(x,y,z,xx,yy);
rr = griddata(x,y,radio_3, xx, yy);

s3 = surf(xx,yy,zz);
s3.CData = rr;

colorbar
hold off

%% KML layer generation
north_interp = interp1(gps_data.time_us, north, interp_baro.time_us);
lat = interp1(gps_data.time_us, gps.lat, interp_baro.time_us) .* 1e-7;
lon = interp1(gps_data.time_us, gps.lon, interp_baro.time_us) .* 1e-7;

lat_t = lat(one_third:two_third);
lon_t = lon(one_third:two_third);
down_t = down_interp(one_third:two_third);
radio_t = radio_interp(one_third:two_third);

grid_lat = linspace(min(lat_t), max(lat_t), 50);
grid_lon = linspace(min(lon_t), max(lon_t), 50);
[llat,llon] = meshgrid(grid_lat, grid_lon);
hh = griddata(lon_t,lat_t,-1*down_t,llon,llat);
rr = griddata(lon_t,lat_t,radio_t, llon,llat);

cLimLow = min(radio_interp(:));
cLimHigh = max(radio_interp(:));

kmlFileName = 'middle.kml';

s4 = surf(llon, llat, hh);
s4.CData = rr;

kmlStr1 = ge_surf(grid_lon,grid_lat,hh,rr,...
            'polyAlpha','FF',...
            'lineColor','0000CCFF',...
            'lineWidth',0.1,...
              'cLimLow',cLimLow,...
             'cLimHigh',cLimHigh,...
            'vertExagg',1,...
          'altRefLevel',1,...
              'extrude',false);

              
kmlStr2 = ge_colorbar(llon(end),llat(1),hh,...
                       'numClasses',20,...
                          'cLimLow',cLimLow,...
                         'cLimHigh',cLimHigh,...
                    'cBarFormatStr','%+07.4f');

ge_output(kmlFileName,[kmlStr1,kmlStr2],...
               'name','middle layer');
           
%% KML 2nd layer generation
lat_t = lat(1:one_third);
lon_t = lon(1:one_third);
down_t = down_interp(1:one_third);
radio_t = radio_interp(1:one_third);

grid_lat = linspace(min(lat_t), max(lat_t), 50);
grid_lon = linspace(min(lon_t), max(lon_t), 50);
[llat,llon] = meshgrid(grid_lat, grid_lon);
hh = griddata(lon_t,lat_t,-1*down_t,llon,llat);
rr = griddata(lon_t,lat_t,radio_t, llon,llat);

cLimLow = min(radio_interp(:));
cLimHigh = max(radio_interp(:));

kmlFileName = 'low.kml';

s4 = surf(llon, llat, hh);
s4.CData = rr;

kmlStr1 = ge_surf(grid_lon,grid_lat,hh,rr,...
            'polyAlpha','FF',...
            'lineColor','0000CCFF',...
            'lineWidth',0.1,...
              'cLimLow',cLimLow,...
             'cLimHigh',cLimHigh,...
            'vertExagg',1,...
          'altRefLevel',1,...
              'extrude',false);

              
kmlStr2 = ge_colorbar(llon(end),llat(1),hh,...
                       'numClasses',20,...
                          'cLimLow',cLimLow,...
                         'cLimHigh',cLimHigh,...
                    'cBarFormatStr','%+07.4f');

ge_output(kmlFileName,[kmlStr1,kmlStr2],...
               'name','bottom layer');
           
%% KML 3rd layer generation
lat_t = lat(two_third:end);
lon_t = lon(two_third:end);
down_t = down_interp(two_third:end);
radio_t = radio_interp(two_third:end);

grid_lat = linspace(min(lat_t), max(lat_t), 50);
grid_lon = linspace(min(lon_t), max(lon_t), 50);
[llat,llon] = meshgrid(grid_lat, grid_lon);
hh = griddata(lon_t,lat_t,-1*down_t,llon,llat);
rr = griddata(lon_t,lat_t,radio_t, llon,llat);

cLimLow = min(radio_interp(:));
cLimHigh = max(radio_interp(:));

kmlFileName = 'high.kml';

s4 = surf(llon, llat, hh);
s4.CData = rr;

kmlStr1 = ge_surf(grid_lon,grid_lat,hh,rr,...
            'polyAlpha','FF',...
            'lineColor','0000CCFF',...
            'lineWidth',0.1,...
              'cLimLow',cLimLow,...
             'cLimHigh',cLimHigh,...
            'vertExagg',1,...
          'altRefLevel',1,...
              'extrude',false);

              
kmlStr2 = ge_colorbar(llon(end),llat(1),hh,...
                       'numClasses',20,...
                          'cLimLow',cLimLow,...
                         'cLimHigh',cLimHigh,...
                    'cBarFormatStr','%+07.4f');

ge_output(kmlFileName,[kmlStr1,kmlStr2],...
               'name','top layer');
