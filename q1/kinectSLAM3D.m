function kinectSLAM3D()
%% Parameter Declarations
% Time Params
runTime = 22;  % End time (seconds)
dt = 0.1; % Time step (seconds)
T = 0:dt:runTime; % Time vector

% Vehicle Params
wheelbase = 0.5; 
kinectHeight = 1;
kinectDistFromFrontWheel = 0.25;

% Measurement Model Params (for 3D laser scanner)
maxRange = 6;
maxAzimuth = deg2rad(70)/2;
maxAltitude = deg2rad(45)/2;

% Measurement Noise
measNoiseCov = [0.08^2 0 0; 
                0 0.01^2 0;
                0 0 0.01^2];
measPerFeature = length(measNoiseCov);


% Vehicle Motion Disturbance
motionDistCov = [  0.05^2 0 0; 
                    0 0.05^2 0; 
                    0 0 0.01^2];
vehicleStateNum = length(motionDistCov);

% Control Signals
steerAngle = deg2rad(3)*ones(1, length(T)); % steering control signal
speed = 3*ones(1,length(T)); % commanded speed

%% Feature Generation
featureNum = 200; featureDistParam = 30; maxFeatureHeight = 2;
map = featureDistParam*rand(2,featureNum);
map(1,:) = map(1,:)-featureDistParam/2; 
map(2,:) = map(2,:)-featureDistParam/2; 
map(3,:) = maxFeatureHeight*rand(1,featureNum);
isNewFeature = ones(featureNum,1);

%% Simulation Initializations
initialState = [0 10 0]';
trueState = zeros(vehicleStateNum,length(T)); % State history 
trueState(:,1) = initialState;
fullStateNum = vehicleStateNum+measPerFeature*featureNum; % # of SLAM states
meas = zeros(measPerFeature*featureNum,length(T)); % Measurement history

% Prior over vehicle state
vehiclePrior = [0 0 0]';
vehiclePriorCov = 0.00000000001*eye(3);

% Prior over feature map
mapPrior = zeros(measPerFeature*featureNum,1);
mapPriorCov = 100*eye(measPerFeature*featureNum);

% Initial Full State Belief and Covariance
mu = [vehiclePrior; mapPrior];
S = [vehiclePriorCov zeros(vehicleStateNum,measPerFeature*featureNum);
    zeros(measPerFeature*featureNum,vehicleStateNum) mapPriorCov];

mu_S = zeros(fullStateNum,length(T)); % Belief history
mu_S(:,1) = mu; % Initial belief

%% Main Loop
for t=2:length(T)
    %% Simulation
    % Generate a disturbance
    motionDisturbance = mvnrnd([0 0 0], motionDistCov)';

    % Position update
    trueState(1,t) = trueState(1,t-1) + dt*speed(t)*cos(trueState(3,t-1));
    trueState(2,t) = trueState(2,t-1) + dt*speed(t)*sin(trueState(3,t-1));
    trueState(3,t) = trueState(3,t-1) - dt*speed(t)*(tan(steerAngle(t))/wheelbase);

    trueState(:,t) = trueState(:,t) + motionDisturbance;
    
    %% Estimation
    % Using Extended Kalman Filter SLAM
    
    %% Plotting
    figure(1);
    subplot(1,2,1); hold on; 
    if t == 2
        % only plot the trees on the first iteration of the main loop
        plot(map(1,:),map(2,:),'oc');
    end
    % add the newest part of the vehicle path to the plot
    plot(trueState(1,t-1:t),trueState(2,t-1:t), 'yx--');
    % show the true heading of the robot
    yaw = trueState(3,t); pl = 2; % pointer length
    plot([trueState(1,t) trueState(1,t)+pl*cos(yaw)],[trueState(2,t) trueState(2,t)+pl*sin(yaw)], 'r-')

    axis equal; axis([-15 15 -15 15]);
    title('3D SLAM with Range & Bearing Measurements','FontSize',14);
    
    subplot(1,2,2);
    image(S);
    colormap('gray');
    title('Covariance Matrix','FontSize',14);
end

% %% Figure Generation for Final Results
% % Trajectory figure
% figure(2);clf; hold on;
% % initial location marker
% plot(trueState(1,1),trueState(2,1),'gd','MarkerSize',10,'MarkerFaceColor','g');
% % plot the entire path
% plot(trueState(1,:),trueState(2,:),'r');
% plot(map(1,:),map(2,:),'oc');
% xlabel('X [m]','FontSize',14)
% ylabel('Y [m]','FontSize',14)
% title('Vehicle Trajectory','FontSize',14)
% axis equal
end

function inView = isInView(feature,vehicleState, maxRange, azimuthMax, altitudeMax)
% Returns 1 if a feature is within the FoV of the 3D scanner
dx = feature(1)-vehicleState(1);
dy = feature(2)-vehicleState(2);
dz = feature(3)-vehicleState(3);

range = sqrt(dx^2+dy^2+dz^2);
azimuthAngle = wrapToPi(atan2(dy,dx)-vehicleState(3));
altitudeAngle = wrapToPi(atan2(dz,dx));

if ((range<maxRange) && (abs(azimuthAngle)<azimuthMax) && (abs(altitudeAngle)<altitudeMax))
    inView = 1;
else
    inView = 0;
end
end


