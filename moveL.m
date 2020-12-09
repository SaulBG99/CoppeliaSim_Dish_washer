function [regPos] = moveL(clientID, regPos, tarNum, target, pos, speed, obj, objPos)
vrep=remApi('remoteApi');
[r, p] = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking);
[r, o] = vrep.simxGetObjectOrientation(clientID, target, -1, vrep.simx_opmode_blocking);

for i=1:3
    if ((abs(pos(i+3)-o(i))>pi)&&(o(i)<0))
        o(i) = o(i) + 2*pi;
    elseif ((abs(pos(i+3)-o(i))>pi)&&(o(i)>0))
        o(i) = o(i) - 2*pi;
    end
end

old_pos = [p o];
delta_pos = pos-old_pos;
distance = norm(delta_pos(1:3));
samples_number = round(distance*50);

for i = 1:samples_number
    intermediate_pos = old_pos + (delta_pos/samples_number);
    tic;
    while (toc<(distance/(speed*samples_number)))
    end
    vrep.simxSetObjectPosition(clientID, target, -1, intermediate_pos, vrep.simx_opmode_oneshot);
    vrep.simxSetObjectOrientation(clientID, target, -1, intermediate_pos(4:6), vrep.simx_opmode_oneshot);
    if(obj~=-1)
        orient = intermediate_pos+objPos;
        vrep.simxSetObjectPosition(clientID, obj, -1, orient, vrep.simx_opmode_oneshot);
        vrep.simxSetObjectOrientation(clientID, obj, -1, orient(4:6), vrep.simx_opmode_oneshot);
    end
    
    if(tarNum == 1)
        [~, p1] = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_oneshot);
        t = clock;
        regPos(end+1, :) = [p1(1:3), intermediate_pos(1:3), regPos(end, 7:12), t(4)*3600+t(5)*60+t(6)];
    else
        [~, p2] = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_oneshot);
        t = clock;
        regPos(end+1, :) = [regPos(end, 1:6), p2(1:3), intermediate_pos(1:3), t(4)*3600+t(5)*60+t(6)];
    end
    
    old_pos = intermediate_pos;
end
