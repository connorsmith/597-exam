% Aerial racing

I = imread('forest.png');
map = im2bw(I, 0.3); % Convert to 0-1 image
map = flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map); % Map size

% Robot start position
dxy = 0.1;
startpos = dxy*[25 350];

% Target locations
waypoints = dxy*[900 80; 450 700];

% Plotting
figure(1); clf; hold on;
colormap('gray');
imagesc(1-map');
plot(startpos(1)/dxy, startpos(2)/dxy, 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(waypoints(:,1)/dxy, waypoints(:,2)/dxy, 'gx', 'MarkerSize',10, 'LineWidth', 3 );
labels = num2str((1:size(waypoints,1))','%d');    %'
text(waypoints(:,1)/dxy+20, waypoints(:,2)/dxy, labels, 'horizontal','left', 'vertical','bottom')
xlabel('North (decimeters)')
ylabel('East (decimeters)')
axis equal
