%% Clear past plots, variables and console prints
close all; clear all; clc;

% Load data 
load 'UAV.mat';

% acx = x-axis accelerometer reading
% acy = y-axis accelerometer reading
% acz = z-axis accelerometer reading
% 
% phi = Roll angle computed by the drone's on-board computer
% tht = Pitch angle computed by the drone's on-board computer
% psi = Yaw angle computed by the drone's on-board computer 
% 
% fix = GPS position fix signal 
% eph = GPS horizontal variance 
% epv = GPS vertical variance 
% lat = GPS Latitude (deg)
% lon = GPS Longitude (deg)
% alt = GPS altitude
% gps_nSat = Number of GPS satellites
% 
% out1 = Motor 1 signal
% out2 = Motor 2 signal
% out3 = Motor 3 signal
% out4 = Motor 4 signal
%
% a = Equatorial radius (semi-major axis in meters)
% b = Polar radius (semi-minor axis in meters)
% e2 = Square of the first numerical eccentricity of ellipsoid
%
% lat_0 = Latitude Reference (initial UAV pos.)
% lon_0 = Latitude Reference (initial UAV pos.)
% alt_0 = Altitude Reference (initial UAV pos.)
%
% x_e0 = x-axis ECEF reference position
% y_e0 = y-axis ECEF reference position
% z_e0 = z-axis ECEF reference position
% x_e = x-axis of ECEF Coordinate Frame
% y_e = y-axis of ECEF Coordinate Frame
% z_e = z-axis of ECEF Coordinate Frame
% N = Radius of curvature
% 
% x_n = NED Coordinate frame pointing towards geodetic North
% y_n = NED Coordinate frame pointing towards geodetic East
% z_n = NED Coordinate frame pointing Down along ellipsoid normal
%
% x_0 = initial state
% P0 = initial covariance
% R = Measurement noise covariance
% x_hat = EKF estimate
% dt = uniform sampling time interval
% R_NEDtoGround = Rotation matrix from NED to Ground
% F = State Transition Matrix
% g = gravity
% G = Control Input Matrix
% Q = Process Noise Covariance Matrix
% x_pred = state prediction step
% P_pred = covarance prediction step
% y = measurement model of GPS
% H = output matrix (measurement matrix)
% S = Residual Covariance
% W = Optimal Kalman Gain



 %% Accelerometer plot
figure; set(gcf,'numbertitle','off','name','Acceleration');  
subplot(3,1,1); plot(t, acx, 'b'); ylim([-2 2]); ylabel('acx (m/s^2)'); grid on; 
subplot(3,1,2); plot(t, acy, 'b'); ylim([-2 2]); ylabel('acy (m/s^2)'); grid on; 
subplot(3,1,3); plot(t, acz, 'b'); ylabel('acz (m/s^2)'); xlabel('time (s)'); grid on; 

%% Euler angles plot
figure; set(gcf,'numbertitle','off','name','Euler Angles');  
subplot(3,1,1); plot(t, rad2deg(phi), 'b'); ylabel('Roll (degree)'); grid on; 
subplot(3,1,2); plot(t, rad2deg(tht), 'b'); ylabel('Pitch (degree)'); grid on; 
subplot(3,1,3); plot(t, rad2deg(psi), 'b'); ylabel('Yaw (degree)'); xlabel('time (s)'); grid on; 


%% GPS plot
figure; set(gcf,'numbertitle','off','name','GPS');  
subplot(3,2,1); plot(t, lon); ylabel('Longitude'); grid on;
subplot(3,2,3); plot(t, lat); ylabel('Latitude'); grid on;
subplot(3,2,5); plot(t, alt); ylabel('Altitude'); grid on; xlabel('time (s)');

subplot(3,2,2); plot(t, gps_nSat, '.'); ylabel('Sat'); grid on;
subplot(3,2,4); plot(t, eph); ylabel('Eph'); grid on; ylim([0 5]);
subplot(3,2,6); plot(t, epv); ylabel('Epv'); grid on; ylim([0 5]); xlabel('time (s)');

%% Motor signal plot
figure; set(gcf,'numbertitle','off','name','Motor Signal');  
hold on;
plot(t,out1,'r');
plot(t,out2,'g');
plot(t,out3,'b');
plot(t,out4,'y');
legend('Motor1','Motor2','Motor3','Motor4'); 
ylabel('Motor inputs'); xlabel('time (s)'); ylim([1000 2000]); grid on;

%%%%%%%%% Your own coding work start from here %%%%%%%%%%%

%% Convert GPS raw measurements to local NED position values
% To convert GPS to Local NED, need to convert Geodetic to ECEF to NED

% Convert Latitude and Longitude to Radians
lat_rad = deg2rad(lat);
lon_rad = deg2rad(lon);

% Create a function to convert GPS to NED coordinate Frame
function [NED_Coordinates] = GPStoNED(lat_rad, lon_rad, alt)

    % Constants for Earth Ellipsoid to convert from Geodetic to ECEF
    a = 6378137;
    b = 6356752.3142;
    e2 = 1 - (b^2 / a^2);

    % Define initial UAV position as reference position
    lat_0 = lat_rad(1); % Reference Latitude
    lon_0 = lon_rad(1); % Reference Longitude
    alt_0 = alt(1); % Reference Altitude

    % Convert defined intial GPS reference position to ECEF coordinate frame
    [x_e0, y_e0, z_e0] = GPStoECEF(lat_0, lon_0, alt_0, a, e2);

    % Create arrays for NED Coordinates to be stored after iteration
    NED_Coordinates = zeros(length(lat_rad), 3);

    % Loop over all GPS raw measurements and convert each to NED coordinates
    for i = 1:length(lat_rad)
        % Convert GPS raw measurements to ECEF coordinate frame
        [x_e, y_e, z_e] = GPStoECEF(lat_rad(i), lon_rad(i), alt(i), a, e2);
        
        % Convert ECEF measurements to NED coordinate frame
        [x_n, y_n, z_n] = ECEFtoNED(x_e, y_e, z_e, x_e0, y_e0, z_e0, lat_rad(i), lon_rad(i));

        % Store NED coordinates in created arrays
        NED_Coordinates(i, :) = [x_n, y_n, z_n];
    end

end

% Create a function to convert GPS to ECEF coordinate frame
function [x_e, y_e, z_e] = GPStoECEF(lat, lon, alt, a, e2)
    % Define radius of curvature
    N = a ./ sqrt(1 - e2 .* sin(lat).^2); % Radius of curvature

    % Convert GPS raw measurements to ECEF coordinate frame
    x_e = (N + alt) .* cos(lat) .* cos(lon);
    y_e = (N + alt) .* cos(lat) .* sin(lon);
    z_e = ((1 - e2) .* N + alt) .* sin(lat);
end

% Create a function to convert ECEF to NED coordinate frame
function [x_n, y_n, z_n] = ECEFtoNED(x_e, y_e, z_e, x_e0, y_e0, z_e0, lat, lon)
    
    % Difference between ECEF current coordinate and ECEF reference point
    dx = x_e - x_e0;
    dy = y_e - y_e0;
    dz = z_e - z_e0;

    % Rotation matrix to go from ECEF to NED coordinate system
    R = [-sin(lat) * cos(lon), -sin(lon) , -cos(lat) * cos(lon) ;
         -sin(lat) * sin(lon), cos(lon)  , -cos(lat) * sin(lon) ;
         cos(lat)            ,     0     , -sin(lat)           ];
    
    % Calculate NED coordinates
    NED = R' * [dx; dy; dz];
    x_n = NED(1); y_n = NED(2); z_n = NED(3);
end

% Run the NED_Coordinates function to compute a GPS to NED Coordinate
% Transform and substitute into correct naming convention [x_n, y_n, z_n]
[NED_coords] = GPStoNED(lat_rad, lon_rad, alt);
x_n = NED_coords(:,1); y_n = NED_coords(:,2); z_n = NED_coords(:,3);

%% Implement EKF to estimate NED position and velocity
% Implementing EKF for time 2nd UAV take-off and landing where t_min is 
% 10 minutes before UAV taking off and t_max is 5 minutes after landing
% From Motor Signal, we can see that UAV took off at 1440 seconds,
% and landed at 1695 seconds

% Define time interval as provided in problem statement and define time
% index to synchronize data between GPS and IMU to align readings based on
% timestamp
start_t = 1443 - 600; % 10 min before taking-off a second time
start_i = find(t>= start_t,1); % Start time index
end_t = 1685 + 300; % 5 min after landing a second time
end_i = find(t>= end_t,1); % End time index
time_int = t(start_i:end_i); % Define time values from time dataset

% State Variables of interest [NED Position, NED Velocity, Accelerometer Bias]
% x = [x_n, y_n, z_n, v_x, v_y, v_z, b_x, b_y, b_z]
% Define initial state, covariance, and measurement noise covariance
x0 = [x_n(start_i); y_n(start_i); z_n(start_i); 0; 0; 0; 0; 0; 0]; % State
P0 = eye(9); % Covariance
R = diag([eph,eph,epv]) * 1e-3; % Measurement noise covariance - most reliable is GPS
x_hat = zeros(9, length(time_int)); % Create arrays for NED estimate coordinates

% System Matrices
for i = start_i:end_i
    
    dt = (time_int(end) - time_int(1))/(length(time_int)); % Assuming uniform sampling

    % Create Rotation Matrix to move from NED to Ground Frame
    R_NEDtoGround = [cos(psi(i)) * cos(tht(i)) , (cos(psi(i)) * sin(tht(i)) * sin(phi(i))) - (sin(psi(i)) * cos(phi(i))) , (cos(psi(i)) * sin(tht(i)) * cos(phi(i))) + (sin(psi(i)) * sin(phi(i)));
                     sin(psi(i)) * cos(tht(i)) , (sin(psi(i)) * sin(tht(i)) * sin(phi(i))) + (cos(psi(i)) * cos(phi(i))) , (sin(psi(i)) * sin(tht(i)) * cos(phi(i))) - (cos(psi(i)) * sin(phi(i)));
                     -sin(tht(i))              ,  cos(tht(i)) * sin(phi(i))                                              ,  cos(tht(i)) * cos(phi(i))                                            ];
    
    % Create the State Transition Matrix
    F_1 = [eye(3)     , diag([dt, dt, dt]) , diag([(-dt^2)/2, (-dt^2)/2, (-dt^2)/2]);
           zeros(3,3) , eye(3)             , diag([-dt, -dt, -dt])                 ];
    F_2 = [eye(6)     , zeros(6,3)    ;
           zeros(3,6) , R_NEDtoGround];
    F = [F_1 * F_2         ;  % Upper region of F
         zeros(3,6), eye(3)]; % Lower region of F

    % Define gravity for G matrix
    g = 9.81; % m/s^2

    % Create the Control Input Matrix
    G_1 = [diag([(dt^2)/2, (dt^2)/2 , (dt^2)/2]);
           diag([  dt    ,    dt    ,   dt    ]);
                          zeros(3,3)           ];
    G_2 = [R_NEDtoGround, [0 ; 0 ; g]];
    G = G_1 * G_2;

    % Create process noise covariance matrix
    Q = G * diag([0.7 , 0.4 , 1.5 , 0]) * G'; %qx, qy, qz

    % Prediction Step - using previous time step to produce estimation of
    % state at current time step
    x_pred = (F * x0) + (G * [acx(i) ; acy(i) ; acz(i) ; 1]);
    P_pred = (F * P0 * F') + Q;

    % Defining the Measurement Model of GPS
    y = [x_n(i+1) ; y_n(i+1) ; z_n(i+1)];

    % To account for difference in GPS update rate and IMU update rate,
    % need to input a condition such that GPS update rate is lower to
    % provide long-term position accuracy
        if mod(i, 2) == 0
            % Correction Step with GPS update (every 3rd iteration)
            H = [eye(3), zeros(3, 6)]; % Output Matrix
            S = (H * P_pred * H') + R; % Residual covariance
            W = P_pred * H' * inv(S);  % Optimal Kalman Gain

            % Update step using GPS
            x0 = x_pred + (W * (y - (H * x_pred)));
            P0 = (eye(9) - (W * H)) * P_pred;
        
        else
            % Update step without GPS (IMU update)
            x0 = x_pred;
            P0 = P_pred;
        end

    % Store estimates
    x_hat(:,i) = x0;
end   

% Extract estimated velocities
x_hat = x_hat(:,start_i:end_i);
EKF_pos_N = x_hat(1, :);
EKF_pos_E = x_hat(2, :);
EKF_pos_D = x_hat(3, :);
EKF_vel_N = x_hat(4, :);
EKF_vel_E = x_hat(5, :);
EKF_vel_D = x_hat(6, :);
EKF_bx = x_hat(7, :);
EKF_by = x_hat(8, :);
EKF_bz = x_hat(9, :);

% Calculate NED Velocites from GPS Measurement
dt_v = diff(time_int);
v_n = diff(x_n(start_i:end_i))./dt_v;
v_e = diff(y_n(start_i:end_i))./dt_v;
v_d = diff(z_n(start_i:end_i))./dt_v;

% Calculate NED Acceleration from GPS Measurement
dt_a = diff(dt_v);
a_n = diff(v_n)./dt_a;
a_e = diff(v_e)./dt_a;
a_d = diff(v_d)./dt_a;


%% Result plots

%% GPS Plot - Processed
figure; set(gcf,'numbertitle','off','name','GPS');  
subplot(3,2,1); plot(t, lon); title('Pre-Processing GPS Plot'); ylabel('Longitude'); grid on;
subplot(3,2,3); plot(t, lat); ylabel('Latitude'); grid on;
subplot(3,2,5); plot(t, alt); ylabel('Altitude'); grid on; xlabel('time (s)');
subplot(3,2,2); plot(t(125:end), lon(125:end)); title('Post-Processing GPS Plot'); ylabel('Longitude'); grid on;
subplot(3,2,4); plot(t(125:end), lat(125:end)); ylabel('Latitude'); grid on;
subplot(3,2,6); plot(t(125:end), alt(125:end)); ylabel('Altitude'); grid on; xlabel('time (s)');

%% GPS to NED Plot
figure; set(gcf,'numbertitle','off','name','NED Positions from GPS Raw Measurements');  
subplot(3,1,1); plot(t(125:end), x_n(125:end));title('NED Positions from GPS Raw Measurements'); ylabel('North (x_n, m)'); grid on;
subplot(3,1,2); plot(t(125:end), y_n(125:end)); ylabel('East (y_n, m)'); grid on;
subplot(3,1,3); plot(t(125:end), z_n(125:end)); ylabel('Down (z_n, m)'); grid on; xlabel('time (s)');

%% Plot of estimated NED positions based on EKF vs. time
figure; set(gcf,'numbertitle','off','name','EKF Estimated NED Positions');  
subplot(3,1,1); plot(time_int, EKF_pos_N, 'k', time_int, x_n(start_i:end_i), 'r--' ); legend('EKF Estimated', 'NED from GPS'); ylabel('North Position, x_n (m)'); grid on;
subplot(3,1,2); plot(time_int, EKF_pos_E, 'k', time_int, y_n(start_i:end_i), 'r--' ); legend('EKF Estimated', 'NED from GPS'); ylabel('East Position, y_n (m)'); grid on;
subplot(3,1,3); plot(time_int, EKF_pos_D, 'k', time_int, z_n(start_i:end_i), 'r--' ); legend('EKF Estimated', 'NED from GPS'); ylabel('Down Position, z_n (m)'); grid on;
xlabel('time (s)');

%% Estimated NED velocities based on EKF Implementation againt time
figure; set(gcf,'numbertitle','off','name','EKF Estimated NED Velocity');  
subplot(3,1,1); plot(time_int(2:end), v_n, 'r--', time_int, EKF_vel_N, 'k' ); legend('NED from GPS', 'EKF Estimated'); ylabel('North Velocity, v_xn (m/s)'); grid on;
subplot(3,1,2); plot(time_int(2:end), v_e, 'r--', time_int, EKF_vel_E, 'k' ); legend('NED from GPS', 'EKF Estimated'); ylabel('East Velocity, v_yn (m/s)'); grid on;
subplot(3,1,3); plot(time_int(2:end), v_d, 'r--', time_int, EKF_vel_D, 'k'); legend('NED from GPS', 'EKF Estimated'); ylabel('Down Velocity, v_zn (m/s)'); grid on;
xlabel('Time (s)');

%% Plot of Local NED Acceleration being converted from GPS vs. time
figure; set(gcf,'numbertitle','off','name','NED Acceleration from GPS');  
subplot(3,1,1); plot(time_int(3:end), a_n); ylabel('North Acceleration (a_n, m/s^2)'); grid on;
subplot(3,1,2); plot(time_int(3:end), a_e); ylabel('East Acceleration (a_e, m/s^2)'); grid on;
subplot(3,1,3); plot(time_int(3:end), a_d); ylabel('Down Acceleration (a_d, m/s^2)'); grid on;
xlabel('time (s)');

%% Plot of Accelerometer bias from EKF Implementation against time
figure; set(gcf,'numbertitle','off','name','EKF Estimated Accelerometer Bias');  
subplot(3,1,1); plot(time_int, EKF_bx); ylabel('EKF Estimated b_x (m/s^2)'); grid on;
subplot(3,1,2); plot(time_int, EKF_by); ylabel('EKF Estimated b_y (m/s^2)'); grid on;
subplot(3,1,3); plot(time_int, EKF_bz); ylabel('EKF Estimated b_z (m/s^2)'); grid on;
xlabel('time (s)');