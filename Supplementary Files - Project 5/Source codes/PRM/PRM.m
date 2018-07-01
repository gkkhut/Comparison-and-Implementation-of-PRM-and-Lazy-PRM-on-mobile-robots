
Cspace=im2bw(imread('map5.bmp')); % input map read from a bmp file. for new maps write the file name here

start=[10 490]; % source position in Y, X format
goal=[400 250]; % goal position in Y, X format
k=100; % number of points in the PRM
display=true; % display processing of nodes
% NewVid = VideoWriter('E:\Sem 1\ENPM 661\project5\codes\project5PRM');
% NewVid.FrameRate = 25;
% open(NewVid);

%%%%% parameters end here %%%%%

if ~checkpoint(start,Cspace), error('source lies on an obstacle or outside map'); end
if ~checkpoint(goal,Cspace), error('goal lies on an obstacle or outside map'); end

imshow(Cspace);
rectangle('position',[1 1 size(Cspace)-1],'edgecolor','k')
points=[start;goal]; % source and goal taken as additional vertices in the path planning to ease planning of the robot
if display, rectangle('Position',[points(1,2)-5,points(1,1)-5,10,10],'Curvature',[1,1],'FaceColor','r'); end
if display, rectangle('Position',[points(2,2)-5,points(2,1)-5,10,10],'Curvature',[1,1],'FaceColor','g'); end
tic;
while length(points)<k+2 % iteratively add vertices
    x=double(int32(rand(1,2) .* size(Cspace)));
      if checkpoint(x,Cspace), 
        points=[points;x]; 
        if display, rectangle('Position',[x(2)-5,x(1)-5,10,10],'Curvature',[1,1],'FaceColor','m'); end
      end
%      f = getframe(gca);
% im = frame2im(f);
% writeVideo(NewVid,im);
% pause(0.1)
end
% if display 
%     disp('click/press any key');
%     waitforbuttonpress; 
% end
edges=cell(k+2,1); % edges to be stored as an adjacency list
for i=1:k+2
    for j=i+1:k+2
          if collisioncheck(points(i,:),points(j,:),Cspace);
              if udistance(points(i,:),points(j,:))<=200
            edges{i}=[edges{i};j];edges{j}=[edges{j};i];
              
            if display, line([points(i,2);points(j,2)],[points(i,1);points(j,1)],'Color','y'); end
              end
          end
%           f = getframe(gca);
% im = frame2im(f);
% writeVideo(NewVid,im);
% pause(0.1)
    end
end
% if display 
%     disp('click/press any key');
%     waitforbuttonpress; 
% end
%% Applying A* algorithm to get the path   
%structure of a node is taken as index of node in vertex, udistance cost, heuristic cost, total cost, parent index in closed list (-1 for source) 
Q=[1 0 heuristic(points(1,:),goal) 0+heuristic(points(1,:),goal) -1]; % the processing queue of A* algorihtm, open list
closed=[]; % the closed list taken as a list
pathFound=false;
while size(Q,1)>0
     [A, I]=min(Q,[],1);
     n=Q(I(4),:); % smallest cost element to process
     Q=[Q(1:I(4)-1,:);Q(I(4)+1:end,:)]; % delete element under processing
     if n(1)==2 % goal test
         pathFound=true;break;
     end
     for mv=1:length(edges{n(1),1}) %iterate through all edges from the node
         newVertex=edges{n(1),1}(mv);
         if length(closed)==0 || length(find(closed(:,1)==newVertex))==0 % not already in closed
             udistanceCost=n(2)+udistance(points(n(1),:),points(newVertex,:));
             heuristicCost=heuristic(points(newVertex,:),goal);
             totalCost=udistanceCost+heuristicCost;
             add=true; % not already in queue with better cost
             if length(find(Q(:,1)==newVertex))>=1
                 I=find(Q(:,1)==newVertex);
                 if Q(I,4)<totalCost, add=false;
                 else Q=[Q(1:I-1,:);Q(I+1:end,:);];add=true;
                 end
             end
             if add
                 Q=[Q;newVertex udistanceCost heuristicCost totalCost size(closed,1)+1]; % add new nodes in queue
             end
         end           
     end
     closed=[closed;n]; % update closed lists
end
if ~pathFound
    error('no path found')
end

fprintf('processing time=%d \nPath Length=%d \n\n', toc,n(4)); 
path=[points(n(1),:)]; %retrieve path from parent information
prev=n(5);
while prev>0
    path=[points(closed(prev,1),:);path];
    prev=closed(prev,5);
end

imshow(Cspace);
rectangle('position',[1 1 size(Cspace)-1],'edgecolor','k')
line(path(:,2),path(:,1),'color','r');
% f = getframe(gca);
% im = frame2im(f);
% writeVideo(NewVid,im);
% close(NewVid);