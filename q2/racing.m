% Aerial racing

I = imread('forest.png');
map = im2bw(I, 0.3); % Convert to 0-1 image
map = flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map); % Map size

% Robot start position
dxy = 0.1;
startPos = dxy*[25 350];

% Target locations
waypoints = dxy*[900 80; 450 700];

%% Simulation Initialization

% Time
simulationTime = 30;
dt = 0.1;
T = 0:dt:simulationTime;

% State [x y theta Vx Vy]
initialState = [startPos'; 0; 0; 0];
  
% Input [Ax Ay Omega]
B = zeros(5,3);
B(3,3) = dt;
B(4,1) = dt;
B(5,2) = dt;

% Motion Disturbance model
R = [0.05 0 0 0 0;
     0 0.05 0 0 0;
     0 0 0.02 0 0; 
     0 0 0 0.03 0;
     0 0 0 0 0.03];
R = R.^2;

% Control inputs (initialization)
u = zeros(3, length(T));
u(1,1:round(length(T)/2)) = .2;
u(1,round(length(T)/2):end) = -.2;

% Simulation Initializations
stateNum = length(R); % Number of vehicle states
trueState = zeros(stateNum,length(T)); % Vehicle states 
trueState(:,1) = initialState;

for t=2:length(T)
    % propagate state forwards
    Vx = trueState(4,t-1);
    Vy = trueState(5,t-1);
    theta = trueState(3,t-1);
    trueState(1,t) = trueState(1,t-1) + (Vx*cos(theta)-Vy*sin(theta))*dt;
    trueState(2,t) = trueState(2,t-1) + (Vx*sin(theta)+Vy*cos(theta))*dt;
    trueState(3:5,t) = trueState(3:5,t-1);
    
    % add control action and noise
    motionDisturbance = mvnrnd([0 0 0 0 0], R)';
    trueState(:,t) = trueState(:,t) + B*u(:,t) + motionDisturbance;
end

% Plotting
figure(1); clf; hold on;
colormap('gray');
imagesc(1-map');
plot(startPos(1)/dxy, startPos(2)/dxy, 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(waypoints(:,1)/dxy, waypoints(:,2)/dxy, 'gx', 'MarkerSize',10, 'LineWidth', 3 );
plot(trueState(1,:)/dxy,trueState(2,:)/dxy,'b--','LineWidth', 2);
labels = num2str((1:size(waypoints,1))','%d');    %'
text(waypoints(:,1)/dxy+20, waypoints(:,2)/dxy, labels, 'horizontal','left', 'vertical','bottom')
xlabel('North (decimeters)')
ylabel('East (decimeters)')
axis equal
