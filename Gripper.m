function Gripper (clientID, closing, j1, j2)
    vrep=remApi('remoteApi');
    [res, pos1] = vrep.simxGetJointPosition(clientID, j1, vrep.simx_opmode_blocking);%simx_opmode_blocking
    [res, pos2] = vrep.simxGetJointPosition(clientID, j2, vrep.simx_opmode_blocking);
    if (closing == 1)
        if (pos1 > (pos2 -0.008) )
            vrep.simxSetJointTargetVelocity (clientID, j1, -0.01, vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity (clientID, j2, -0.04, vrep.simx_opmode_blocking);
        else
            vrep.simxSetJointTargetVelocity (clientID, j1, -0.04, vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity (clientID, j2, -0.04, vrep.simx_opmode_blocking);
        end
    else
        if (pos1 > pos2)
            vrep.simxSetJointTargetVelocity (clientID, j1, 0.04, vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity (clientID, j2, 0.02, vrep.simx_opmode_blocking);
        else
            vrep.simxSetJointTargetVelocity (clientID, j1, 0.02, vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity (clientID, j2, 0.04, vrep.simx_opmode_blocking);
        end
    end

