% Cleaning floor map
clear;
map_boundary = [0 0; 12 0; 12 2; 14 2; 14 0; 40 0;40 2; 42 2; 42 0; 60 0;
       60 20; 
       55 20; 55 18; 43 18; 43 20; 32 20; 32 18; 15 18; 15 20; 10 20; 5 20; 5 10; 3 10; 3 20; 0 20; 
       0 0];
ob{1} = [ 59 2; 56 2; 56 3; 59 3; 59 2];
ob{2} = [ 54 2; 51 2; 51 3; 54 3; 54 2];
ob{3} = [49 2; 46 2; 46 3; 49 3; 49 2];
ob{4} = [34 2; 31 2; 31 3; 34 3; 34 2];
ob{5} = [ 29 2; 26 2; 26 3; 29 3; 29 2];
ob{6} = [24 2; 21 2; 21 3; 24 3; 24 2];
ob{7} = [41 12; 41 14; 44 14; 44 12; 41 12];
ob{8} = [14 6; 50 6;  50 7; 14 7;  14 6];
ob{9} = [3 5; 3 2;8 2; 8 8; 3 8; 3 7; 7 7; 7 3; 4 3; 4 5; 3 5];  
ob{10} = [8 10; 13 10; 13 17; 10 17; 10 14; 8 14; 8 10 ];
ob{11} = [21 10; 25 10; 25 16; 15 16; 15 10; 19 10; 19 11; 16 11; 16 15; 24 15; 24 11; 21 11; 21 10];
ob{12} = [39 16; 39 10; 28 10; 28 16; 35 16; 36 16; 36 15; 29 15; 29 11; 38 11; 38 16; 39 16;  ];
ob{13} = [57 16; 46 16; 46 10; 57 10; 57 13; 56 13; 56 11; 47 11; 47 15; 57 15; 57 16];

dock = [5 4];
spots = [17 12; 30 12; 42 19;48 12; 44 1]; %rearranged

figure(1);clf;hold on;
fill([0 60 60 0], [0 0 20 20], 'b');
fill(map_boundary(:,1),map_boundary(:,2),'y');
for i = 1:length(ob)
    fill(ob{i}(:,1),ob{i}(:,2),'b');
end
plot(dock(1), dock(2), 'ro')
plot(spots(:,1), spots(:,2), 'go')
axis equal
       
v_map = [map_boundary; NaN NaN];
for i=1:length(ob)
    v_map = [v_map; ob{i}; NaN NaN];
end
   
startPos = [5 4];
endPos = [17 12];

% Create visibility graph
tic;
numObsts = length(ob);

allGraphPts = [];
for i=1:numObsts
    allGraphPts = [allGraphPts;ob{i}(1:end,:)];
end

allGraphPts = [allGraphPts; map_boundary;startPos; spots];
n = length(allGraphPts(:,1));

% Set initial link possibilities
A = zeros(n,n);
D = A;

% Add the boundary of each obstacle
baseCounter = 0;
for obId=1:length(ob)
    for eId = 1:size(ob{obId},1)-1
        edgeId = eId + baseCounter;
        A(edgeId,edgeId+1) = 1;
        A(edgeId+1,edgeId) = 1;
        D(edgeId,edgeId+1) = norm(allGraphPts(edgeId,:)-allGraphPts(edgeId,:));
        D(edgeId+1,edgeId+1) = D(edgeId,edgeId+1);
    end
    
    baseCounter = baseCounter + size(ob{obId},1);
end

% Check for collisions among links
for i=1:n-1
    for j=i+1:n
        inColl = 0;
        % In collision if the link intersects the boundary
        inColl = inColl + EdgePolyIntersect(allGraphPts([i j],:),map_boundary);
        % In collision if the link midpoint is outside the boundary
        mid = (allGraphPts(i,:) + allGraphPts(j,:))/2;
        inColl = inColl + ~inpolygon(mid(1), mid(2), map_boundary(:,1), map_boundary(:,2));
        for k = 1:numObsts
            % In collision if the link intersects any ob
            inColl = inColl + EdgePolyIntersect([allGraphPts(i,:); allGraphPts(j,:)],ob{k});
            % In collision if the link midpoint is inside an ob
            inColl = inColl + inpolygon(mid(1), mid(2), ob{k}(:,1), ob{k}(:,2));
        end

        if (~inColl)
            A(i,j) = 1;
            A(j,i) = 1;
            D(i,j) = norm(allGraphPts(i,:)-allGraphPts(j,:));
            D(j,i) = D(i,j);
        end

    end
end
toc;

% Shortest path search
fullpath = [];
fulldist = 0;
oldnode = n-length(spots);
for wp = 1:length(spots)
    [spath,sdist] = shortestpath(allGraphPts, A, oldnode,oldnode+1);
    fullpath = [fullpath spath];
    fulldist = fulldist + sdist;
    oldnode = oldnode+1;
end
[spath,sdist] = shortestpath(allGraphPts, A, oldnode,n-length(spots));
fullpath = [fullpath spath];
fulldist = fulldist + sdist;

figure(2); clf; hold on;
plot(map_boundary(:,1),map_boundary(:,2));
for i=1:length(ob)
    patch(ob{i}(:,1),ob{i}(:,2), 'b');
end

for i=1:n
    for j=i+1:n
        if (A(i,j))
            figure(2); hold on;
            plot([allGraphPts(i,1) allGraphPts(j,1)],[allGraphPts(i,2) allGraphPts(j,2)],'m');

        end
    end
end
plot(map_boundary(:,1),map_boundary(:,2));
plot(startPos(1),startPos(2), 'co', 'MarkerSize', 10, 'LineWidth', 3)
plot(endPos(1),endPos(2), 'rx', 'MarkerSize', 10, 'LineWidth', 3)
plot(allGraphPts(fullpath,1),allGraphPts(fullpath,2),'g', 'LineWidth',2)
fulldist % output the full distance
       