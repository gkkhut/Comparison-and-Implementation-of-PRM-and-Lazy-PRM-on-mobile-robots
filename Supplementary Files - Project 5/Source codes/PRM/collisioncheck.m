
function feasible=collisioncheck(n,newPos,map)
feasible=true;
dir=atan2(newPos(1)-n(1),newPos(2)-n(2));
for r=0:0.5:sqrt(sum((n-newPos).^2))
    posCheck=n+r.*[sin(dir) cos(dir)];
    if ~(checkpoint(ceil(posCheck),map) && checkpoint(floor(posCheck),map) && ... 
            checkpoint([ceil(posCheck(1)) floor(posCheck(2))],map) && checkpoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;break;
    end
    if ~checkpoint(newPos,map), feasible=false; end
end