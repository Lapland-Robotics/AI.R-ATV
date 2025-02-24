// ATV Source file

#include "ATV.h"
#include <stdlib.h>

// Structure definition is private to this file
struct CtrlRequest {
    float angularZ;
    float linearX;
    float linearY;
    float leftMotorSpeed;
    float rightMotorSpeed;
};

// Function to create a new CtrlRequest object
struct CtrlRequest* createCtrlRequest(int defaultSteering, int defaultSpeed) {
    struct CtrlRequest *newRequest = (struct CtrlRequest*)malloc(sizeof(struct CtrlRequest));
    if (newRequest) {
        newRequest->angularZ = defaultSteering;
        newRequest->linearX = defaultSpeed;
    }
    return newRequest;
}

// Function to destroy a CtrlRequest object
void destroyCtrlRequest(struct CtrlRequest *request) {
    free(request);
}

// Getter for angularZ
int getAngularZ(struct CtrlRequest *request) {
    return request->angularZ;
}

// Setter for angularZ
void setAngularZ(struct CtrlRequest *request, int value) {
    if(value > Steering_Right_Limit) {
        value = Steering_Right_Limit;
    } else if (value < Steering_Left_Limit) {
        value = Steering_Left_Limit;
    }
    request->angularZ = value;
}

// Getter for linearX
int getLinearX(struct CtrlRequest *request) {
    return request->linearX;
}

// Setter for linearX
void setLinearX(struct CtrlRequest *request, int value) {
    if(value > Driving_Forward_Limit) {
        value = Driving_Forward_Limit;
    } else if (value < Driving_Reverse_Limit) {
        value = Driving_Reverse_Limit;
    }
    request->linearX = value;
}