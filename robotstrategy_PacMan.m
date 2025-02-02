function [move, mem] = robotstrategy_PacMan(env, mem)

%define constants
lmax = env.basic.lmax;
minFuelDiff = 100;
numOfMines = env.mines.nMine;
numOfFuels = env.fuels.nFuel;
start = env.info.myPos;
destination = start;
exist = env.fuels.fExist;
fpos = env.fuels.fPos;
mpos = env.mines.mPos;
mineExist = env.mines.mExist;
mineRad = env.basic.rMF;
xyLim = [0,10;0,10];
%know if currently in chase
if isempty(mem)
    mem = struct("chase",false);
end

%find closest available fuel tank
closestFtank = zeros(numOfFuels,2);
for i=1:numOfFuels
    closestFtank(i,1) = sqrt((fpos(i,1)-start(1))^2 + (fpos(i,2)-start(2))^2);
    closestFtank(i,2) = i;
end
[~, fuelIndex] = min(closestFtank(:,1));

%check if the robot should start a chase
%start a chase if fuel difference is at least 100
%start a chase if enemy is nearby and has at least 30 fuel points less
%stop chase if fuel level drops below the enemy level
if (env.info.fuel - env.info.fuel_op) >= minFuelDiff ||...
        (mem.chase == true && env.info.fuel > env.info.fuel_op+4) ||...
        ((env.info.fuel - env.info.fuel_op) > 40 && (GetDelta(env.info.myPos(1)-env.info.opPos(1),env.info.myPos(2)-env.info.opPos(2)) < 2))
    destination = env.info.opPos;
    mem.chase = true;
else %if there are available fuel tanks, find the closest active one
    mem.chase = false;
    if find(exist==1)
        while exist(fuelIndex) == 0
            closestFtank(fuelIndex,:) = [100,100];
            [~, fuelIndex] = min(closestFtank(:,1));
        end
        destination = fpos(fuelIndex,:);
    elseif (env.info.fuel >= env.info.fuel_op)
        destination = start;
    else
        destination = findNearestCorner(start,xyLim);
    end
end

%find the closest mine to me
closestMine = zeros(numOfMines,2);
for i=1:numOfMines
    closestMine(i,1) = sqrt((mpos(i,1)-start(1))^2 + (mpos(i,2)-start(2))^2);
    if mineExist(i) == 0
        closestMine(i,1) = 100;
    end
    closestMine(i,2) = i;
end
%%find index of the closest active mine
[~, mineIndex] = min(closestMine(:,1));

if start ~= destination %if robot has a destination avoid mines
%%find linear equation from me to my destination
    p = polyfit([start(1),destination(1)],[start(2),destination(2)],1);
%%find if the line intersects with the mine
    [collisionPointx,collisionPointy] = linecirc(p(1),p(2),mpos(mineIndex,1),mpos(mineIndex,2),mineRad); 
    collision = false;
    if (~isnan(collisionPointx)) %%see if the mine is in front or behind me
       if (hitMine(start,destination,[collisionPointx(1),collisionPointy(1)]) == true)
           collision  = true;
       end
    end
%%if a mine is in my way find the shortest way around it
    if (collision)
        middleX = (collisionPointx(1)+collisionPointx(2))/2;
        middleY = (collisionPointy(1)+collisionPointy(2))/2;
        middlePoint = [middleX, middleY];
        raviaVec = [middlePoint(1) - mpos(mineIndex,1), middlePoint(2) - mpos(mineIndex,2)];
        if (raviaVec(1)>=0 && raviaVec(2) >=0)
            destination = mpos(mineIndex,:) + (mineRad + 0.1);
        elseif (raviaVec(1)<0 && raviaVec(2) < 0)
            destination = mpos(mineIndex,:) - (mineRad + 0.1);
        elseif (raviaVec(1)>=0 && raviaVec(2) < 0)
            destination(1) = mpos(mineIndex,1) + (mineRad + 0.1);
            destination(2) = mpos(mineIndex,2) - (mineRad + 0.1);
        else
            destination(1) = mpos(mineIndex,1) - (mineRad + 0.1);
            destination(2) = mpos(mineIndex,2) + (mineRad + 0.1);
        end
    end
end
%set move vector to correct value

%if there are no more active fuel tanks and we have more fuel than enemy,
%congrats you are better than good
move = (destination - start);
move = move/sqrt(move*move')*lmax;

%%sub functions to help with collision detection
    %find the nearest corner to my current position
    function corner = findNearestCorner(myPos, xyLim)
        corners = [xyLim(1,1),xyLim(2,1);...
                   xyLim(1,1),xyLim(2,2);...
                   xyLim(1,2),xyLim(2,1);...
                   xyLim(1,2),xyLim(2,2)];
        cornersDist = zeros(1,4);
        cornersDist(1) = GetDelta(myPos(1)-xyLim(1,1),myPos(2)-xyLim(2,1));
        cornersDist(2) = GetDelta(myPos(1)-xyLim(1,1),myPos(2)-xyLim(2,2));
        cornersDist(3) = GetDelta(myPos(1)-xyLim(1,2),myPos(2)-xyLim(2,1));
        cornersDist(4) = GetDelta(myPos(1)-xyLim(1,2),myPos(2)-xyLim(2,2));
        [~,minCorner] = min(cornersDist);
        corner = corners(minCorner,:);
    end


    %checks if in collision course with mine and returns true or false
    function hit = hitMine(start,destination,collisionPoint)
        dist1 = GetDelta((collisionPoint(1) - start(1)),(collisionPoint(2) - start(2)));
        dist2 = GetDelta(collisionPoint(1) - destination(1),collisionPoint(2) - destination(2));
        dist3 = GetDelta(destination(1) - start(1),destination(2) - start(2));
        hit = false;
        if dist3 > dist1 && dist3 > dist2
            hit = true;
        end
    end
    %takes delta between 2 points and return the distance between them
    function distance = GetDelta(x,y)
    distance = sqrt(x.^2 + y.^2);
    end

end