function [regPos] = moveL2(clientID, regPos, target1, target2, pos1, pos2, speed, plate, platePos, ida, sponge, spongePos)
vrep=remApi('remoteApi');
[r, p1] = vrep.simxGetObjectPosition(clientID, target1, -1, vrep.simx_opmode_blocking);
[r, o1] = vrep.simxGetObjectOrientation(clientID, target1, -1, vrep.simx_opmode_blocking);
[r, p2] = vrep.simxGetObjectPosition(clientID, target2, -1, vrep.simx_opmode_blocking);
[r, o2] = vrep.simxGetObjectOrientation(clientID, target2, -1, vrep.simx_opmode_blocking);

for i=1:3
    if ((abs(pos1(i+3)-o1(i))>pi)&&(o1(i)<0))
        o1(i) = o1(i) + 2*pi;
    elseif ((abs(pos1(i+3)-o1(i))>pi)&&(o1(i)>0))
        o1(i) = o1(i) - 2*pi;
    end
end
for i=1:3
    if ((abs(pos2(i+3)-o2(i))>pi)&&(o2(i)<0))
        o2(i) = o2(i) + 2*pi;
    elseif ((abs(pos2(i+3)-o2(i))>pi)&&(o2(i)>0))
        o2(i) = o2(i) - 2*pi;
    end
end


old_pos1 = [p1 o1];
delta_pos1 = pos1-old_pos1;
distance1 = norm(delta_pos1(1:3));
samples_number1 = round(distance1*50);

old_pos2 = [p2 o2];
delta_pos2 = pos2-old_pos2;
distance2 = norm(delta_pos2(1:3));
samples_number2 = round(distance2*50);


if samples_number1>samples_number2
    samples_number = samples_number1;
    distance = distance1;
else
    samples_number = samples_number2;
    distance = distance2;
end

for i = 1:samples_number
    intermediate_pos1 = old_pos1 + (delta_pos1/samples_number);
    intermediate_pos2 = old_pos2 + (delta_pos2/samples_number);
    
    %Wait
    tic;
    while (toc<(distance/(speed*samples_number)))
    end
    
    %Go to intermediate position
    vrep.simxSetObjectPosition(clientID, target1, -1, intermediate_pos1, vrep.simx_opmode_oneshot);
    vrep.simxSetObjectOrientation(clientID, target1, -1, intermediate_pos1(4:6), vrep.simx_opmode_oneshot);
    vrep.simxSetObjectPosition(clientID, target2, -1, intermediate_pos2, vrep.simx_opmode_oneshot);
    vrep.simxSetObjectOrientation(clientID, target2, -1, intermediate_pos2(4:6), vrep.simx_opmode_oneshot);
    if(plate~=-1)
        %orient = intermediate_pos1+platePos;%(i/samples_number)*[0.1 0 0 0 0 0];
        %alfa = orient(4); beta= orient(5)-pi; gama=orient(6);
        %transl = [0; 0; 0.09] - [cos(alfa), -sin(alfa), 0; sin(alfa), cos(alfa), 0; 0, 0, 1]*...
        %[cos(beta), 0, sin(beta); 0, 1, 0; -sin(beta), 0, cos(beta)]*...
        %[1, 0, 0; 0, cos(gama), -sin(gama); 0, sin(gama), cos(gama)]*[0; 0; 0.09];
        %orient = orient + [transl' 0 0 0];
        if(ida)
            idaI = i;
        else
            idaI = samples_number-i;
        end 
        orient = intermediate_pos1+platePos+(idaI/samples_number)*[0.06 -0.03 0.01 0 0 0];%0.1 -0.05
        vrep.simxSetObjectPosition(clientID, plate, -1, orient, vrep.simx_opmode_oneshot);
        vrep.simxSetObjectOrientation(clientID, plate, -1, orient(4:6), vrep.simx_opmode_oneshot);
    end
    if(sponge~=-1)
        orient = intermediate_pos2+spongePos;
        vrep.simxSetObjectPosition(clientID, sponge, -1, orient, vrep.simx_opmode_oneshot);
        vrep.simxSetObjectOrientation(clientID, sponge, -1, orient(4:6), vrep.simx_opmode_oneshot);
    end
    
    [~, p1] = vrep.simxGetObjectPosition(clientID, target1, -1, vrep.simx_opmode_oneshot);
    [~, p2] = vrep.simxGetObjectPosition(clientID, target2, -1, vrep.simx_opmode_oneshot);
    t = clock;
    regPos(end+1, :) = [p1(1:3), intermediate_pos1(1:3), p2(1:3), intermediate_pos2(1:3), t(4)*3600+t(5)*60+t(6)];
    
    old_pos1 = intermediate_pos1;
    old_pos2 = intermediate_pos2;
end
