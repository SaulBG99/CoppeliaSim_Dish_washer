clear all
close all
clc


vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
if(clientID >-1)
    disp('remoteApi');
    %object handles
    [~, j1] = vrep.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active1#0', vrep.simx_opmode_blocking);
    [~, j2] = vrep.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active2#0', vrep.simx_opmode_blocking);  
    [~, k1] = vrep.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active1', vrep.simx_opmode_blocking);
    [~, k2] = vrep.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active2', vrep.simx_opmode_blocking);  
    [~, target1] = vrep.simxGetObjectHandle(clientID, 'target1', vrep.simx_opmode_blocking);
    [~, target2] = vrep.simxGetObjectHandle(clientID, 'target2', vrep.simx_opmode_blocking);
    [~, sponge] = vrep.simxGetObjectHandle(clientID, 'sponge', vrep.simx_opmode_blocking);
    
    %[x, y, z, alpha, beta, gama]
    Fposition1 = [-0.65, 0.819, 1.52, 0, 0, 0; %Neutral
                    -1.005, 0.785, 1.21, 0 0 0;%pi*2/3, 0, pi;%Plate1
                    -0.29, 0.7, 1.4, 0, pi/4, pi/4; %Washing 
                    -1.005, 0.74, 1.26, 0 0 0; %Neutral 2
                    -0.36, 0.7, 1.4, 0, 0, 0];
    Fposition2 = [0.3165, 0.81938, 1.52, pi, 0, pi; %Neutral
                    0.080797, 0.728, 1.07, pi, 0, pi; %Sponge
                    -0.17, 0.7, 1.4, pi+pi/4, 0, pi-pi/4; %Washing Neutral
                    -0.26, 0.6, 1.35, pi+pi/4, 0, pi-pi/4;%Washing 1
                    -0.125, 0.72, 1.32, pi+pi/4, 0, pi-pi/4;%Washing 2
                    0.2, 0.6, 0.6, 0, 0, 0];
    
    %[ChangeInNext?(1:Stay, 0:Change), element, index/action]
    %instructions = [1, ]
    
    %gripper (clientiD, 0, j1, j2);pause(1.5); %open gripper
    %moveL(clientID, kuka_target, fposition3, 2);
    
    %while(end_test==0)
    %    moveL(clientID, kuka_target, fposition3, 2);
    %    gripper (clientiD, 0, j1, j2);pause(2);
    %   Gripper (clientID, 0, BHA0, BHA2, BHB1);pause(2);
    %end
    
    %Registro de posiciones
    regPos = [];
    format shortg
    speed = 1.5;
    plates = 4;
    
    %Ir a posiciones neutrales
    %moveL2(clientID, target1, target2, Fposition1(1, :), Fposition2(2, :), 2)
    %moveL(clientID, target1, Fposition1(1, :), 20)
    [regPos] = moveL2(clientID, regPos, target1, target2, Fposition1(1, :), Fposition2(1, :), speed, -1, -1,false, -1, -1);   
        
    for i=1:plates
        namePlate = convertStringsToChars('plate'+string(i));
        [res, plate] = vrep.simxGetObjectHandle(clientID, namePlate, vrep.simx_opmode_blocking);
        
        %Brazo 1: Ir a posicion neutral 2
        [regPos] = moveL(clientID, regPos, 1, target1, Fposition1(4, :), speed, -1);
        
        Gripper (clientID, 0, j1, j2);pause(2);
        %Gripper (clientID, 1, j1, j2);pause(2);
        %Gripper (clientID, 0, j1, j2);pause(2);
        
        %Ir por plato / Ir por esponja
        [regPos] = moveL2(clientID, regPos, target1, target2, Fposition1(2, :)-(i-1)*[0 0.03 0 0 0 0], Fposition2(2, :), speed, -1, -1, false, -1, -1);
        
        %Cancel difference in position
        [~, p1] = vrep.simxGetObjectPosition(clientID, target1, -1, vrep.simx_opmode_blocking);
        [~, p2] = vrep.simxGetObjectPosition(clientID, plate, -1, vrep.simx_opmode_blocking);
        [~, o1] = vrep.simxGetObjectOrientation(clientID, target1, -1, vrep.simx_opmode_blocking);
        [~, o2] = vrep.simxGetObjectOrientation(clientID, plate, -1, vrep.simx_opmode_blocking);
        platePos = [p2-p1 o2-o1];
        [~, p1] = vrep.simxGetObjectPosition(clientID, target2, -1, vrep.simx_opmode_blocking);
        [~, p2] = vrep.simxGetObjectPosition(clientID, sponge, -1, vrep.simx_opmode_blocking);
        [~, o1] = vrep.simxGetObjectOrientation(clientID, target2, -1, vrep.simx_opmode_blocking);
        [r, o2] = vrep.simxGetObjectOrientation(clientID, sponge, -1, vrep.simx_opmode_blocking);
        spongePos = [p2-p1 o2-o1];
        
        %Agarrar plato / Agarrar esponja        
        %Gripper (clientID, 0, j1, j2);pause(2);
        Gripper (clientID, 1, j1, j2);pause(2);

        %Traer plato a fregadero / Traer esponja a fregadero
        [regPos] = moveL2(clientID, regPos, target1, target2, Fposition1(3, :), Fposition2(3, :), speed, plate, platePos, true, sponge, spongePos);

        %Brazo 2: Lavar plato
        for j=1:5
            [regPos] = moveL(clientID, regPos, 2, target2, Fposition2(4, :), speed, sponge, spongePos);
            [regPos] = moveL(clientID, regPos, 2, target2, Fposition2(5, :), speed, sponge, spongePos);
        end
        
        %Dejar por plato / Dejar por esponja
        [regPos] = moveL2(clientID, regPos, target1, target2, Fposition1(2, :)-(i-1)*[0 0.03 0 0 0 0], Fposition2(2, :), speed, plate, platePos, false, sponge, spongePos);
        Gripper (clientID, 0, j1, j2);pause(2);
    end    
    %Ir a posiciones neutrales
    [regPos] = moveL2(clientID, regPos, target1, target2, Fposition1(1, :), Fposition2(1, :), speed, -1, -1, false, -1, -1);   
end

for i=1:13
    for j=2:size(regPos, 1)
        if(regPos(j, i)==0 && regPos(j-1, i)~=0)
            regPos(j, i)=regPos(j-1, i);
        end
    end
end
regPos(:, 13) = regPos(:, 13) - regPos(1, 13);
regPos(1:3, :)=[];
strAxis = ["x", "y", "z"];
for i=1:6
    subplot(3,2, i)
    if(mod(i, 2)==0)
        plot(regPos(:, 13),regPos(:, round((i)/2+6)), 'r', regPos(:, 13),regPos(:, round((i)/2+9)), 'g');
        title(strAxis(round(i/2))+" axis, with right hand");
        
    else
        plot(regPos(:, 13),regPos(:, round((i+1)/2)), 'r', regPos(:, 13),regPos(:, round((i+1)/2+3)), 'g');
        title(strAxis(round((i+1)/2))+" axis, with left hand");
    end
    ylabel("Position (meters)");
    xlabel("Time (seconds)");
    legend("Real", "Ideal");
    grid on
end



        