function kinectSLAM3D()
%% Parameter Declarations
% Time Params
rng(528492);
runTime = 20;  % End time (seconds)
dt = 0.1; % Time step (seconds)
T = 0:dt:runTime; % Time vector
rtPlotFlag = 1;

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
robotStateNum = length(motionDistCov);

% EKF Estimates of Process and Measurement Noise
R = motionDistCov;
Q = measNoiseCov;

% Control Signals
steerAngle = deg2rad(3)*ones(1, length(T)); % steering control signal
speed = 3*ones(1,length(T)); % commanded speed
u = [speed; speed.*tan(steerAngle)]; clear steerAngle speed;

% Feature Generation
featureNum = 25; featureDistParam = 30; maxFeatureHeight = 2;
map = featureDistParam*rand(2,featureNum);
map(1,:) = map(1,:)-featureDistParam/2; 
map(2,:) = map(2,:)-featureDistParam/2; 
map(3,:) = maxFeatureHeight*rand(1,featureNum);
isNewFeature = ones(featureNum,1);

%% Simulation Initializations
initialState = [0 10 0]';
trueState = zeros(robotStateNum,length(T)); % State history 
trueState(:,1) = initialState;
fullStateNum = robotStateNum+measPerFeature*featureNum; % # of SLAM states
meas = zeros(measPerFeature*featureNum,length(T)); % Measurement history

% Prior over vehicle state
vehiclePrior = initialState;
vehiclePriorCov = 0.00000000001*eye(3);

% Prior over feature map
mapPrior = zeros(measPerFeature*featureNum,1);
mapPriorCov = 100*eye(measPerFeature*featureNum);

% Initial Full State Belief and Covariance
mu = [vehiclePrior; mapPrior];
S = [vehiclePriorCov zeros(robotStateNum,measPerFeature*featureNum);
    zeros(measPerFeature*featureNum,robotStateNum) mapPriorCov];

mu_S = zeros(fullStateNum,length(T)); % Belief history
mu_S(:,1) = mu; % Initial belief

ad = [];

%% Main Loop
for t=2:length(T)
    %% Simulation
    % Generate a disturbance
    motionDisturbance = mvnrnd([0 0 0], motionDistCov)';

    % Position update
    trueState(1,t) = trueState(1,t-1) + dt*cos(trueState(3,t-1))*u(1,t);
    trueState(2,t) = trueState(2,t-1) + dt*sin(trueState(3,t-1))*u(1,t);
    trueState(3,t) = trueState(3,t-1) - (dt/wheelbase)*u(2,t);
    trueState(:,t) = trueState(:,t) + motionDisturbance;
    trueState(3,t) = wrapTo2Pi(trueState(3,t));
    % Generate measurements
    isVisible = zeros(featureNum,1);
    for featureId=1:featureNum
        % If feature is visible
        if(isInView(map(:,featureId),[trueState(1:2,t)' kinectHeight trueState(3,t)],maxRange,maxAzimuth,maxAltitude))
            isVisible(featureId) = 1;
            % Generate a measurement disturbance
            measurementNoise = mvnrnd(zeros(1,measPerFeature), measNoiseCov)';
            % Determine measurement [range; azimuthAngle; altitudeAngle]
            trueDx = map(1,featureId)-trueState(1,t);
            trueDy = map(2,featureId)-trueState(2,t);
            trueDz = map(3,featureId)-kinectHeight;
            trueRange = sqrt(trueDx^2 + trueDy^2 + trueDz^2);
            trueRxy = sqrt(trueRange^2 - trueDz^2);
            trueAzimuthAngle = atan2(trueDy,trueDx)-trueState(3,t);
            trueAltitudeAngle = atan2(trueDz,trueRxy);
            featId = measPerFeature*(featureId-1)+1;
            meas(featId:featId+measPerFeature-1,t) = [trueRange;trueAzimuthAngle;trueAltitudeAngle] + measurementNoise;
            meas(featId+1:featId+measPerFeature-1,t) = wrapToPi(meas(featId+1:featId+measPerFeature-1,t));
        end
    end
    
    %% Estimation, Extended Kalman Filter SLAM 
    % Prediction Update
    mu(1:3) = [ mu(1)+u(1,t)*cos(mu(3))*dt;
                mu(2)+u(1,t)*sin(mu(3))*dt;
                mu(3)+u(2,t)*(-dt/wheelbase)];
    
    Gt = [ 1 0 -u(1,t)*sin(mu(3))*dt;
           0 1 u(1,t)*cos(mu(3))*dt;
           0 0 1];
    
    S(1:robotStateNum,1:robotStateNum) = Gt*S(1:robotStateNum,1:robotStateNum)*Gt' + R;
     
    % Measurement Update
    for fId=1:featureNum
        % iterate over each feature
        if (isVisible(fId))
            % only update if a feature is visible
            baseId = robotStateNum+measPerFeature*(fId-1)+1;
            featId = measPerFeature*(fId-1)+1;
            meas_range = meas(featId,t);
            meas_azi = meas(featId+1,t);
            meas_alt = meas(featId+2,t);
            bel_x = mu(1);
            bel_y = mu(2);
            bel_yaw = mu(3);
            if (isNewFeature(fId) == 1)
                % initialize new features (X,Y,Z positions)
                mu(baseId) =    bel_x+meas_range*cos(meas_alt)*cos(meas_azi+bel_yaw);
                mu(baseId+1) =  bel_y+meas_range*cos(meas_alt)*sin(meas_azi+bel_yaw);
                mu(baseId+2) =  kinectHeight + meas_alt*meas_range;
                isNewFeature(fId) = 0;
            end
            % Linearization about estimated state
            dx = mu(baseId)     - bel_x;
            dy = mu(baseId+1)   - bel_y;
            dz = mu(baseId+2)   - kinectHeight;
            rp = sqrt((dx)^2 + (dy)^2 + (dz)^2);
            rxy = sqrt(rp^2 - dz^2);
                
            % apply filter to use the correct feature
            featureFilter = zeros(robotStateNum+measPerFeature,fullStateNum);
            featureFilter(1:robotStateNum,1:robotStateNum) = eye(robotStateNum);
            featureFilter(robotStateNum+1:end,baseId:baseId+measPerFeature-1) = eye(measPerFeature);
            
            % linearized measurement matrix
            Ht = [ -dx/rp,-dy/rp,-dz/rp, dx/rp, dy/rp, dz/rp;
                dy/rxy^2,  -dx/rxy^2,  0, -dy/rxy^2, dx/rxy^2,0;
                dz/rxy^2,  0,  -dx/rxy^2, -dz/rxy^2, 0, dx/rxy^2]*featureFilter;
 
            % Actual measurement update
            kalmanGain = S*Ht'/(Ht*S*Ht'+Q); % calculate Kalman Gain
            
            % calculate innovation (diff between true meas and meas from estimated state)
            angleDiff = abs(wrapTo2Pi(meas(featId+1,t)) - wrapTo2Pi(atan2(dy,dx)-bel_yaw));
            angleDiff = min(angleDiff, 2*pi - angleDiff);
            ad(end+1) = angleDiff;
            innovation = [  meas(featId,t) - rp;
                            angleDiff;
                            meas(featId+2,t) - atan2(dz,rxy)]
            mu = mu + kalmanGain*innovation;  % update belief (kalman gain times the innovation)
            S = (eye(fullStateNum)-kalmanGain*Ht)*S; % covariance update
        end
    end
 
    % Store results
    mu_S(:,t) = mu;

    %% Plotting
    if rtPlotFlag
        figure(1);
        subplot(1,2,1); hold on; 
        if t == 2
            % only plot the features on the first iteration of the main loop
            plot(map(1,:),map(2,:),'om');
        end
        % add the newest part of the vehicle path to the plot
        plot(trueState(1,t-1:t),trueState(2,t-1:t), 'yx--');
        plot(mu_S(1,t-1:t),mu_S(2,t-1:t), 'go--');
        % show the true heading of the robot
        yaw = trueState(3,t); pl = 2; % pointer length
        plot([trueState(1,t) trueState(1,t)+pl*cos(yaw)],[trueState(2,t) trueState(2,t)+pl*sin(yaw)], 'r-')
        for fId=1:featureNum
              if (isVisible(fId))
                  % index offset to the features
                  fid = measPerFeature*(fId-1)+1;
                  xyRange = sqrt(abs(meas(fid,t)^2-kinectHeight^2));
                  % plot the rays from the laser scanner
%                   plot([trueState(1,t) trueState(1,t)+xyRange*cos(meas(fid+1,t)+yaw)],...
%                       [trueState(2,t) trueState(2,t)+xyRange*sin(meas(fid+1,t)+yaw)], '--c');
%                   plot(trueState(6+fid),trueState(6+fj), 'gx')
    %               mu_pos = [trueState(6+fid) trueState(6+fj)];
    %               S_pos = [S(6+fid,6+fid) S(6+fid,6+fj); S(6+fj,6+fid) S(6+fj,6+fj)];
    %               error_ellipse(S_pos,mu_pos,0.95);
              end
        end
        axis equal; axis([-15 15 -15 15]);
        title('3D SLAM with Range & Bearing Measurements','FontSize',14);

        subplot(1,2,2);
        image(10000*S);
        colormap('gray');
        title('Covariance Matrix','FontSize',14);
    end
end

%% Figure Generation for Final Results
% Trajectory figure
figure(2);clf; hold on;
% initial location marker
plot(trueState(1,1),trueState(2,1),'gd','MarkerSize',10,'MarkerFaceColor','g');
% plot the entire path
plot(trueState(1,:),trueState(2,:),'r');
plot(map(1,:),map(2,:),'oc');
xlabel('X [m]','FontSize',14)
ylabel('Y [m]','FontSize',14)
title('Vehicle Trajectory','FontSize',14)
axis equal
% State figure
figure(3);clf; hold on;
% initial location marker
% plot the entire path
subplot(3,1,1);
plot(T,trueState(1,:),'r');
subplot(3,1,2);
plot(T,trueState(2,:),'r');
subplot(3,1,3);
plot(T,trueState(3,:),'r');
end

function inView = isInView(feature,vehicleState, maxRange, azimuthMax, altitudeMax)
% Returns 1 if a feature is within the FoV of the 3D scanner
dx = feature(1)-vehicleState(1);
dy = feature(2)-vehicleState(2);
dz = feature(3)-vehicleState(3);
yaw = vehicleState(4);

range = sqrt(dx^2+dy^2+dz^2);
rxy = sqrt(range^2 - dz^2);
azimuthAngle = wrapToPi(atan2(dy,dx)-yaw);
altitudeAngle = wrapToPi(atan2(dz,rxy));

if ((range>1) &&(range<maxRange) && (abs(azimuthAngle)<azimuthMax) && (abs(altitudeAngle)<altitudeMax))
    inView = 1;
else
    inView = 0;
end
end


