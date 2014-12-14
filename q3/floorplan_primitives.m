% Cleaning floor map
function floorplan_primitives()
clear; rng(3883838);
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

v_map = [map_boundary; NaN NaN];
for i=1:length(ob)
    v_map = [v_map; ob{i}; NaN NaN];
end

sampleNum = 15;
r_positions = [ zeros(2,sampleNum);
                2*pi*rand(1,sampleNum);];
samples_generated = 1;

while(samples_generated <= sampleNum)
    pos = [60*rand(1,1);20*rand(1,1)];
    if(polygonsOverlap(pos',v_map))
        r_positions(1:2,samples_generated) = pos;
        samples_generated = samples_generated + 1;
    end
end
            
figure(1);clf;hold on;
fill([0 60 60 0], [0 0 20 20], 'b');
fill(map_boundary(:,1),map_boundary(:,2),'w');
for i = 1:length(ob)
    fill(ob{i}(:,1),ob{i}(:,2),'b');
end
plot(r_positions(1,:), r_positions(2,:), 'rx')

% vehicle itself
for rect=1:sampleNum
    b = getBoundingBox(0.25,0.125,r_positions(3,rect),0.5,0.625);
    b(1,:) = b(1,:) + r_positions(1,rect);
    b(2,:) = b(2,:) + r_positions(2,rect);
    plot(b(1,:),b(2,:),'m','LineWidth',2)
end

% % straight motion primitive
% for rect=1:sampleNum
%     b = getBoundingBox(0.25,0.125,r_positions(3,rect),0.5,2.625);
%     b(1,:) = b(1,:) + r_positions(1,rect);
%     b(2,:) = b(2,:) + r_positions(2,rect);
%     plot(b(1,:),b(2,:),'--c','LineWidth',2)
% end

% left turn motion primitive
for rect=1:sampleNum
    b = getBoundingBox(0.75,0.125,r_positions(3,rect),1.0154,0.8321);
    b(1,:) = b(1,:) + r_positions(1,rect);
    b(2,:) = b(2,:) + r_positions(2,rect);
    plot(b(1,:),b(2,:),'--c','LineWidth',2)
end

% left turn motion primitive
for rect=1:sampleNum
    b = getBoundingBox(0.25,0.125,r_positions(3,rect),1.0154,0.8321);
    b(1,:) = b(1,:) + r_positions(1,rect);
    b(2,:) = b(2,:) + r_positions(2,rect);
    plot(b(1,:),b(2,:),'--c','LineWidth',2)
end
axis equal
       
end

function b = getBoundingBox(x,y,h,xl,yl)
% function drawcar(x,y,h,xl,yl)
% This function plots a car at position x,y heading h with length xl and
% width yl.

% Draw the wheels

a = [0 0 xl xl 0; 0 yl yl 0 0]; % The coordinates of the head of the original vector
a = a - [x*ones(1,5);y*ones(1,5)];
b = [cos(h)  -sin(h) ; sin(h)  cos(h)] * a; % A matrix multiplication
end
       