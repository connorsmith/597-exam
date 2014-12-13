function kinectSLAM3D()
%% Parameter and Variable Declarations
% Simulation time
Tmax = 22;  % End time (seconds)
dt = 0.1; % Time step (seconds)
T = 0:dt:Tmax; % Time vector

% Fixed vehicle parameters
wheelbase = 0.5; % Bicycle length

% motion disturbance
disturbanceCov = [  0.05^2 0 0; 
                    0 0.05^2 0; 
                    0 0 0.01^2];

% allocating state and control arrays
state = zeros(3, length(T));    % bicycle state vector array
state(:,1) = [0 10 0]';          % initial state
steerAngle = deg2rad(3)*ones(1, length(T)); % steering control signal
speed = 3*ones(1,length(T)); % commanded speed

%% Feature Generation
featureNum = 200; featureDistParam = 30; maxFeatureHeight = 2;
map = featureDistParam*rand(2,featureNum);
map(1,:) = map(1,:)-featureDistParam/2; 
map(2,:) = map(2,:)-featureDistParam/2; 
map(3,:) = maxFeatureHeight*rand(1,featureNum);

%% Simulation Loop
for i=1:length(T)-1
   
    % Simulate a disturbance
    motionDisturbance = mvnrnd([0 0 0], disturbanceCov)';

    % Position update
    state(1,i+1) = state(1,i) + dt*speed(i)*cos(state(3,i));
    state(2,i+1) = state(2,i) + dt*speed(i)*sin(state(3,i));
    state(3,i+1) = state(3,i) - dt*speed(i)*(tan(steerAngle(i))/wheelbase);

    state(:,i+1) = state(:,i+1) + motionDisturbance;
end

%% Plotting
% Trajectory figure
figure(1);clf; hold on;
% initial location marker
plot(state(1,1),state(2,1),'gd','MarkerSize',10,'MarkerFaceColor','g');
% plot the entire path
plot(state(1,:),state(2,:),'r');
plot(map(1,:),map(2,:),'oc');
xlabel('X [m]','FontSize',14)
ylabel('Y [m]','FontSize',14)
title('Vehicle Trajectory','FontSize',14)
axis equal
end



