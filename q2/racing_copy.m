% Aerial racing
function racing_copy()
I = imread('forest.png');
map = im2bw(I, 0.3); % Convert to 0-1 image
map = flipud(map)'; % Convert to 0 free, 1 occupied and flip.
divFactor = 0.25;
map = imresize(map,divFactor);
[M,N]= size(map); % Map size

% Robot start position
dxy = 0.1/divFactor;
startPos = divFactor*dxy*[150 400];

% Target locations
waypoints = divFactor*dxy*[900 80; 450 700];

%% Simulation Initialization

% Time
simulationTime = 30;
dt = 0.1;
T = 0:dt:simulationTime;

% State [x y theta Vx Vy]
initialState = [startPos'; 1; 0; 0];
  
% Input [Ax Ay Omega]
B = zeros(5,3);
B(3,3) = dt;
B(4,1) = dt;
B(5,2) = dt;

% Motion Disturbance model
R = 0*[0.05 0 0 0 0;
     0 0.05 0 0 0;
     0 0 0.02 0 0; 
     0 0 0 0.03 0;
     0 0 0 0 0.03];
R = R.^2;

% Control inputs (initialization)
u = zeros(3, length(T));
u(3,:) = -1;

% Simulation Initializations
stateNum = length(R); % Number of vehicle states
trueState = zeros(stateNum,length(T)); % Vehicle states 
trueState(:,1) = initialState;

% Belief map
m = 0.5*ones(M,N);
L0 = log(m./(1-m));
L=L0;

% Sensor model parameters
sonarArray{1} = linspace(deg2rad(0),deg2rad(30),10);% Measurement headings
sonarArray{2} = linspace(deg2rad(90),deg2rad(120),10); % Measurement headings
sonarArray{3} = linspace(deg2rad(180),deg2rad(210),10); % Measurement headings
sonarArray{4} = linspace(deg2rad(270),deg2rad(300),10); % Measurement headings
sonarIndex = 1;
rmax = 15; % Max range
alpha = 1; % Width of an obstacle (Distance about measurement to fill in)
beta = 0.05; % Width of a beam (Angle beyond which to exclude) 

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
    
    meas_phi = sonarArray{sonarIndex};
    sonarIndex = sonarIndex+1;
    if(sonarIndex == 5)
        sonarIndex = 1;
    end
    
    % Generate a measurement data set
    meas_r = getranges(map,trueState(1:3,t),meas_phi,rmax,1/dxy);

    %% Map update
    % Get inverse measurement model
    invmod = inversescanner(M,N,trueState(1,t),trueState(2,t),trueState(3,t),meas_phi,meas_r,rmax,alpha,beta,1/dxy);
    
    % Calculate updated log odds
    L = L +log(invmod./(1-invmod))-L0;

    % Calculate probabilities
    m = exp(L)./(1+exp(L));

    %% Plot results
    
    % Map and vehicle path
    figure(1);clf; hold on;
    imagesc(1-map');
    colormap(gray);
    plot(trueState(1,1:t)/dxy,trueState(2,1:t)/dxy,'bx-')
    axis([0 M 0 N])

    % Inverse measurement model
    figure(2);clf; hold on;
    imagesc(100*(invmod'));
    colormap(gray);
    plot(trueState(1,t)/dxy,trueState(2,t)/dxy,'bx')
    for i=1:length(meas_r)
        plot((trueState(1,t)+meas_r(i)*cos(meas_phi(i)+ trueState(3,t)))/dxy ,...
            (trueState(2,t)+meas_r(i)*sin(meas_phi(i) + trueState(3,t)))/dxy,'ko')
    end
    axis([0 M 0 N])
    title('Measurements and inverse measurement model');


    % Belief map
    figure(3);clf; hold on;
    imagesc(100*(m'));
    colormap(gray);
    plot(trueState(1,max(1,t-10):t)/dxy,trueState(2,max(1,t-10):t)/dxy,'bx-')
    axis([0 M 0 N])
    title('Current occupancy grid map')

end

% Plotting
figure(3); clf; hold on;
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
end

function meas_r = getranges(map,X,meas_phi, rmax,scale)
% Generate range measurements for a laser scanner based on a map, vehicle
% position and sensor parameters.
% Rough implementation of ray tracing algorithm.

% Initialization
[M,N] = size(map);
x = X(1)*scale;
y = X(2)*scale;
th = X(3);
meas_r = scale*rmax*ones(size(meas_phi));

% For each measurement bearing
for i=1:length(meas_phi)
    % For each unit step along that bearing up to max range
   for r=1:rmax
       % Determine the coordinates of the cell
       xi = round(x+r*cos(th+meas_phi(i)));
       yi = round(y+r*sin(th+meas_phi(i)));
       % If not in the map, set measurement there and stop going further 
       if (xi<=1||xi>=M||yi<=1||yi>=N)
           meas_r(i) = r;
           break;
       % If in the map but hitting an obstacle, set measurement range and
       % stop going further
       elseif (map(xi,yi))
           meas_r(i) = r;
           break;
       end
   end
end
end

function [m] = inversescanner(M,N,x,y,theta,meas_phi,meas_r,rmax,alpha,beta,scale)
% Calculates the inverse measurement model for a laser scanner
% Identifies three regions, the first where no new information is
% available, the second where objects are likely to exist and the third
% where objects are unlikely to exist

x = x * scale;
y = y*scale;
rmax= rmax*scale;
m = 0.5*ones(M,N);
% Range finder inverse measurement model
for i = 1:M
    for j = 1:N
        % Find range and bearing to the current cell
        r = sqrt((i*alpha-x)^2+(j*alpha-y)^2);
        phi = mod(atan2(j*alpha-y,i*alpha-x)-theta,2*pi);
        
        % Find the applicable range measurement 
        [meas_cur,k] = min(abs(phi-meas_phi));
        phi_s(i,j) = phi;
        
        % If behind out of range measurement, or outside of field
        % of view, no new information is available
        if ((meas_r(k) == rmax) && (r - rmax >= -alpha/2)) || (abs(phi-meas_phi(k))>beta/2)
            m(i,j) = 0.5;
        % If the range measurement was before this cell, likely to be an object
        elseif ((r - meas_r(k) >= -alpha/2))
            m(i,j) = 0.6 - 0.1*(1-exp(-(r-meas_r(k))));
        % If the cell is in front of the range measurement, likely to be
        % empty
        else 
            m(i,j) = 0.2;
        end
        % Solid ground under robot
        %if (abs(phi+theta-3*pi/2) <= beta/2) && (r >= 1)
        %    m(i,j) = 0.9;
        %end
    end
end
end