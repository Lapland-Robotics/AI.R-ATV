// RobotDriveControl Source file

#include "RobotDriveControl.h"
#include <stdlib.h>

// Structure definition is private to this file
struct CommandVelocity {
    double linearX;
    double angularZ;
    double leftSpeed;
    double rightSpeed;
};

// Function to create a new CommandVelocity object
struct CommandVelocity* createCommandVelocity() {
    struct CommandVelocity *newRequest = (struct CommandVelocity*)malloc(sizeof(struct CommandVelocity));
    if (!newRequest) {
        return NULL; // Avoid using an uninitialized pointer
    }
    
    newRequest->linearX = 0.0;
    newRequest->angularZ = 0.0;
    newRequest->leftSpeed = 0.0;
    newRequest->rightSpeed = 0.0;
    return newRequest;
}

// Getter for angularZ
double getAngularZ(struct CommandVelocity *request) {
    return request->angularZ;
}

// Getter for linearX
double getLinearX(struct CommandVelocity *request) {
    return request->linearX;
}

// Getter for leftSpeed
double getLeftSpeed(struct CommandVelocity *request) {
    return request->leftSpeed;
}

// Getter for rightSpeed
double getRightSpeed(struct CommandVelocity *request) {
    return request->rightSpeed;
}

// Setter for LeftSpeed
void setLeftSpeed(struct CommandVelocity *request, double linearX, double angularZ) {
    if (!request) return;
    double leftSpeed = linearX + (angularZ * WheelBase / 2.0);
    request->leftSpeed = leftSpeed;
}

// Setter for rightSpeed
void setRightSpeed(struct CommandVelocity *request, double linearX, double angularZ) {
    if (!request) return;
    double rightSpeed = linearX - (angularZ * WheelBase / 2.0);
    request->rightSpeed = rightSpeed;
}

// Setter for angularZ
void setAngularZ(struct CommandVelocity *request, double value) {
    if (!request) return;
    if(value > Steering_Right_Limit) {
        value = Steering_Right_Limit;
    } else if (value < Steering_Left_Limit) {
        value = Steering_Left_Limit;
    }
    request->angularZ = value;
}

// Setter for linearX
void setLinearX(struct CommandVelocity *request, double value) {
    if (!request) return;
    if(value > Driving_Forward_Limit) {
        value = Driving_Forward_Limit;
    } else if (value < Driving_Reverse_Limit) {
        value = Driving_Reverse_Limit;
    }
    request->linearX = value;
}

void setCmdVelDiffDrive(struct CommandVelocity *request, double linearX, double angularZ) {
    if (!request) return;
    setAngularZ(request, angularZ);
    setLinearX(request, linearX);
    setLeftSpeed(request, linearX, angularZ);
    setRightSpeed(request, linearX, angularZ);
}

// Function to destroy a CommandVelocity object
void destroyCommandVelocity(struct CommandVelocity *request) {
    if (request) {
        free(request);
    }
}