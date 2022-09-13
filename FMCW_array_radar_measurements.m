function Z = FMCW_array_radar_measurements(points,plat_state,plot_on)
% function simulates the measurements obtained from a FMCW radar defined
% according to radar_system at a position plat_state for a scene defined by
% points.
% Inputs:
%       points          - Description of the scene, N by 4 matrix, where N
%                           is the number of points in the scene and 4
%                           represents (x,y,z,rcs)
%       plat_state      - Current platform state (x,y,z,heading)
%       plot_on         - binary variable, 1 = plots radar iamge and CFAR 
%                           detections on figure, 0 = no plots.
% 
% Outputs:
%       Z               - M by 3 measurement matrix, where M is the number
%                           of detections and each detection is
%                           (range, azimuth, power)
% Comments:
%       1) Only simulates the IF signal model and does not include the 
%       residual video phase (RVP) term
%       2) The simulation does not take into account the change of position
%       within a waveform (i.e. it uses the "stop and go" assumption)
%       3) The radar systems comprises multiple phased array units that are
%       uniformly spaced around the platform and the platform is assumed to
%       be a point.
%       4) Limited to 2D geometry currently 
%       5) Transmitter is assumed to be at (0,0) of the body frame and
%       omidirectional
%       6) Radar system parameters defined in inbuilt function
%
% References:
% "Stop and Go" assumption:
% J. Mũnoz-Ferreras and R. Gomez-Garcıa. Beyond the stop-and-go assumption in pulse-doppler radar sensors. IEEE Sensors Journal, Sep. 2014
% FMCW signal processing and IF signal model:
% A. Meta, P. Hoogeboom, and L. P. Ligthart. Signal processing for FMCW SAR. IEEE Transactions on Geoscience and Remote Sensing, Nov 2007.
%

% load radar system parameters:
radar_system = set_Radar_system;

% define phyiscal constants and parameters:
c = physconst('LightSpeed');    % Speed of light

% define wavelength:
lambda = c/radar_system.fc;

% define angular position of radar units in rads:
Rx_angle = (0:(radar_system.Unit_num-1)).*(2*pi./radar_system.Unit_num);

% define receiver spatial positions (uniform array with spacing lambda/2):
Rx_pos = [zeros(1,radar_system.array_Num); (((radar_system.array_Num-1)/2):-1:(-(radar_system.array_Num-1)/2))*lambda/2];

% define rotation matrix function (angle input in rads):
rot_fn = @(theta) [cos(theta), sin(theta); -sin(theta), cos(theta)];

% rotate and translate scene to body frame (simplify to 2D):
scene_pt = rot_fn(plat_state(end))*(points(:,1:2).' - plat_state(1:2).');
N_scene = length(scene_pt(1,:));    % number of points in scene

% range from points to transmitter in body frame:
Range_Tx = sqrt(scene_pt(1,:).^2 + scene_pt(2,:).^2);

% initalise IF model data arrays:
N_range = radar_system.fs*radar_system.T_chirp;
Data = zeros(N_range,radar_system.array_Num,radar_system.Unit_num);

% define time axis:
t_fast = (0:(N_range-1))./radar_system.fs;

% noise power:
Pstd = sqrt(radar_system.Noise_power);       % Std of complex Gaussian noise

%% Generate IF model data:
for unit_index = 1:radar_system.Unit_num
    % rotate scene to unit frame:
    scene_pt_hat = scene_pt;

    % rotate receiver array:
    Rx_pos_hat = rot_fn(-Rx_angle(unit_index))*Rx_pos;

    % for loop for each point:
    for point_index = 1:N_scene

        % local rcs in number:
        rcs_local =  10.^((points(point_index,4))./10);

        % determine angle between point and phase centre of the radar array: 
        phi = angle_estimation(scene_pt_hat(1:2,point_index),Rx_angle(unit_index));

        % check if point is visible (i.e. inside radar beam and within range):
        if abs(phi) <= radar_system.gamma/2 && Range_Tx(point_index) <= radar_system.range_max
            % for loop for each receiver
            for Rx_index = 1:radar_system.array_Num
                % range to receiver:
                Range_Rx = sqrt((scene_pt_hat(1,point_index) - Rx_pos_hat(1,Rx_index))^2 + (scene_pt_hat(2,point_index) - Rx_pos_hat(2,Rx_index))^2);
    
                % Bistatic range:
                targetRange = Range_Tx(point_index) + Range_Rx;

                % compute amplitude using target's RCS and radar system
                % characteristics (equation is the radar range equation for
                % power)
                amp = sqrt(2).*sqrt(rcs_local * radar_system.Ptx_W * radar_system.Gtx * radar_system.Gtx * radar_system.GRx * (lambda ^ 2) ./ (Range_Tx(point_index).^2*Range_Rx.^2.*((4 * pi) ^ 3) ));
        
                % Generate IF FMCW signal based on the target's range and the system
                % set up:
                Data(:,Rx_index,unit_index) = Data(:,Rx_index,unit_index) + amp.*exp(1i*2*pi*radar_system.fc.*(targetRange./c)).*exp(1i*2*pi*radar_system.Chirp_rate.*(targetRange./c).*(t_fast.'));
            end
        end
    end
    % add noise to each unit:
    Data(:,:,unit_index) = Data(:,:,unit_index) + randn(size(Data(:,:,1)))*(Pstd/sqrt(2)) + 1i.*randn(size(Data(:,:,1)))*(Pstd/sqrt(2));
end

%% Perform range and azimuth compression to obtain radar image:
% note that the method below is quite basic, i.e. spectral analysis using 
% kaiser windows to limit spectral spreading

% setting for FMCW signal processing:
% N_range_ZP = 2^nextpow2(radar_system.fs*radar_system.T_chirp);      % number of points in range after zero padding
% N_phi_ZP = 361;                                                     % number of points in phi (approximately 0.5 degrees resolution assuming 180 degree range);
N_range_ZP = ceil(radar_system.fs*radar_system.T_chirp);      % number of points in range after zero padding
N_phi_ZP = 181;                                                     % number of points in phi (approximately 1 degrees resolution assuming 180 degree range);

% initalise radar image:
Radar_image = zeros(N_phi_ZP,N_range_ZP,radar_system.Unit_num);

% Window function in the range domain:
Window_range = kaiser(N_range,3);
Window_range = Window_range./norm(Window_range);

% Window function in the azimuth domain:
Window_azimuth = kaiser(radar_system.array_Num,3).';
Window_azimuth = Window_azimuth./norm(Window_azimuth);

% perform compression:
for unit_index = 1:radar_system.Unit_num

    % Perform range compression:
    Data_RangeComp = fft(squeeze(Data(:,:,unit_index)).*(Window_range*ones(1,radar_system.array_Num)),N_range_ZP,1);

    % initial output variable:
    Radar_image(:,:,unit_index) =  fftshift(ifft(Data_RangeComp.*(ones(N_range_ZP,1)*Window_azimuth),N_phi_ZP,2),2).'.*N_phi_ZP;

end
clear('Data','Data_RangeComp');

% define range and phi axis (in unit frame):
range_axis = (c*(0:(radar_system.fs/N_range_ZP):(radar_system.fs - 1/N_range_ZP))./(2*radar_system.Chirp_rate));
% phi_axis = asin(-1:(1/((N_phi_ZP-1)/2)):1)*180./pi;                % phi is based on actually obtaining sin(phi) in the radar image
phi_axis = asin((floor(-(N_phi_ZP-1)/2):floor((N_phi_ZP-1)/2))./N_phi_ZP*2);

%% CFAR algorithm:

% Threshold in dB:
thres = radar_system.CFAR_thres; 

% CFAR algorithm parameters:
Guard_size = radar_system.Guard_size;         % number of Guard cells
Outer_size = radar_system.Outer_size;         % number of cells

% Use Cell averaging smallest for range
Filter_Range = [ones(1,Outer_size(1)), zeros(1,Guard_size(1)*2+1),zeros(1,Outer_size(1))].';
Filter_Range = Filter_Range./sum(Filter_Range);

% use standard Cell averaging for azimuth:
% adjust azimuth Guard based on azimuth resolution:
Guard_size(2) = max(round(radar_system.Azimuth_res*pi/180/(2/N_phi_ZP)),1);

Filter_Az = [ones(1,Outer_size(2)), zeros(1,Guard_size(2)*2+1),ones(1,Outer_size(2))].';
Filter_Az = Filter_Az./sum(Filter_Az);


% initialise measurements:
Z = [];

% determine per unit detections:
for unit_index = 1:radar_system.Unit_num

    % Run CFAR filter over data:
    Px = abs(squeeze(Radar_image(:,:,unit_index)))./sqrt(radar_system.fs);

    % Range Px:
    Px_range = mean(Px);

    % compute noise floor:
    N_fl1 = 20*log10(imfilter(Px_range,Filter_Range.','circular'));
    N_fl2 = 20*log10(imfilter(Px_range,flipud(Filter_Range).','circular'));

    % compute value of cell under test and compare to threshold:
    Detections = logical(20*log10(Px_range) >=(thres + min(N_fl1,N_fl2)));

    % Extract range and azimuth:
    Num_target = sum(Detections(:));
    range_index = find(Detections(:)==1);       % Range points
    N_fl_Az = 20*log10(imfilter(Px(:,range_index), Filter_Az,'circular')); % noise floor estimate across Azimuth
    for index = 1:Num_target
        % determine detections in Azimuth for given range
        Detects_Az = logical(20*log10(Px(:,range_index(index))) >= (thres + N_fl_Az(:,index)) ).*(logical(abs(phi_axis) <= radar_system.gamma/2).');
        
        % Find azimuth points (filter based on beamwidth)
        pt_az = find(Detects_Az == 1);
        pt_targ = range_index(index)*ones(length(pt_az),1);

        % concaterate measurements:
        Z = [Z; range_axis(pt_targ).',-phi_axis(pt_az).' + Rx_angle(unit_index), 20*log10(Px(sub2ind(size(Px),pt_az,pt_targ)))];

        % remove any obvious errors:

    end
end

% convert azimuth to (-pi,pi]:
Z(Z(:,2)>pi,2) = Z(Z(:,2)>pi,2) - 2*pi;

if plot_on == 1
    % Plot detections and radar image:
    test = (Z(:,1).*cos(Z(:,2))).';
    test(2,:) = (Z(:,1).*sin(Z(:,2))).';
    Plot_Radar_images(radar_system,range_axis,phi_axis,Rx_angle,Radar_image,test);
end

% end of main function
end


% Inbuilt functions:
function theta_out = angle_estimation(target_pos, Rx_angle)
% function determines the angle between a line connecting the target and 
% antenna position and a line defining the direction of the antenna (i.e. a
% normal to the phase center).

% define unit vector normal to the phased array:
AntTrg_vec = [cos(Rx_angle); sin(Rx_angle)];

% determine angle between v_vec and ant_norm_vec:
theta_out = acos(sum(target_pos.*AntTrg_vec)./sqrt(sum(abs(target_pos).^2)));
end

function Plot_Radar_images(radar_system,range_axis,phi_axis,Rx_angle,Radar_image,scene_pt)
% function plots the radar image obtained form the sensor in the body
% frame. The functions plots the image in both the range-azimuth domain and
% the cartesian domain

% Find points within field of view:
index1 = find(abs(phi_axis) <= radar_system.gamma/2);

% construct image and phi array:
holder = [];
h = [];
for index = 1:length(Rx_angle)
    h = [fliplr(phi_axis(index1))+Rx_angle(index), h];
    holder = [(squeeze(Radar_image(index1,:,index)));holder];
end

% convert phi to (-180,180]
h(h>180) = h(h>180) - 360;

% % Plot Range-Azimuth image:
% figure; surface(range_axis, h,20*log10(abs(squeeze(holder))/sqrt(radar_system.fs)),'edgecolor','interp');
% title('Radar image in Range-Azimuth domain in body frame');
% xlabel('Range (m)');
% ylabel('Azimuth (degrees)');
% c_label = colorbar;
% c_label.Label.String = 'Power (dB)';
% ylim([-181,181]);
% xlim([-0.1,radar_system.range_max+0.1]);

% Plot cartesian image:
[r,theta] = meshgrid(range_axis, h);
xx=r.*cos(theta);
yy=r.*sin(theta);
figure(2); clf; surface(xx,yy,20*log10(abs(squeeze(holder))/sqrt(radar_system.fs)),'edgecolor','interp');
axis square
hold on; scatter3(scene_pt(1,:),scene_pt(2,:),-70*ones(length(scene_pt(1,:)),1),'r*');
title('Radar image in cartesian domain in body frame');
xlabel('x coordinate (m)');
ylabel('y coordinate (m)');
c_label = colorbar;
c_label.Label.String = 'Power (dB)';
xlim([-radar_system.range_max,radar_system.range_max]);
ylim([-radar_system.range_max,radar_system.range_max]);
caxis([-160,-85]);
box on;
hold off;
end

function radar_system = set_Radar_system
% functions to convert dBm and dB into linear
dBm2num = @(x_in) 10.^((x_in-30)./10);
dB2num = @(x_in) 10.^((x_in)./10);

% Radar Hardware Description:
% note numbers are roughly realistic.
radar_system.Ptx_W = dBm2num(12);           % Transmit Power (conversion from dBm to number)
radar_system.Gtx = dB2num(10);              % Antenna Gain (conversion from dB to number)
radar_system.GRx = dB2num(48);              % Reciever gain (conversion from dB to number)
radar_system.Fnoise = dB2num(15);           % Noise Figure of the receiver (conversion from dB to number)
radar_system.fc = 77e9;                     % Centre operating frequency
radar_system.gamma = 90*pi/180;             % Azimuth beamwidth of antenna (conversion from degrees to rads)
radar_system.array_Num = 35;                % Size of array (i.e. number of receivers in array)
radar_system.Chirp_rate = 45e6/1e-6;        % Chirp rate for FMCW waveform in Hz/second
radar_system.T_chirp = 50e-6;               % Chirp duration for FMCW waveform in seconds
radar_system.fs = 8e6;                      % sampling rate in Hz
radar_system.Unit_num = 4;                  % Number of radar units

% range and azimuth performance:
c = physconst('LightSpeed');
kB = physconst('Boltzmann');     % Boltzman constant
Temp = 295;                     % Temperature in Kelvin (295K ~ 20 degrees C)
radar_system.range_res = c/(2*radar_system.Chirp_rate*radar_system.T_chirp);                        % range resolution in meters
radar_system.range_max = c*radar_system.fs/(2*radar_system.Chirp_rate);                             % Maximum range in meters
radar_system.Azimuth_res = 2/radar_system.array_Num*180/pi;                                         % Azimuth resolution in degrees
radar_system.Noise_power = kB * Temp * radar_system.Fnoise * radar_system.GRx *(radar_system.fs);

% Define CFAR parameters:
radar_system.CFAR_thres = 12;           % CFAR threshold in dB
radar_system.Guard_size = [2,2];        % number of Guard cells, first number corresponds to range and second to azimuth
radar_system.Outer_size = [4,4];        % number of cells, first number corresponds to range and second to azimuth
end