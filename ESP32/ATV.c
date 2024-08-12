// ATV Source file

#include "ATV.h"
#include <stdlib.h>

// Structure definition is private to this file
struct CtrlRequest {
    int steeringRequest;
    int speedRequest;
};

// Function to create a new CtrlRequest object
struct CtrlRequest* createCtrlRequest() {
    struct CtrlRequest *newRequest = (struct CtrlRequest*)malloc(sizeof(struct CtrlRequest));
    if (newRequest) {
        newRequest->steeringRequest = 50;
        newRequest->speedRequest = 50;
    }
    return newRequest;
}

// Function to destroy a CtrlRequest object
void destroyCtrlRequest(struct CtrlRequest *request) {
    free(request);
}

// Getter for steeringRequest
int getSteeringRequest(struct CtrlRequest *request) {
    return request->steeringRequest;
}

// Setter for steeringRequest
void setSteeringRequest(struct CtrlRequest *request, int value) {
    if(value > Steering_Right_Limit) {
        value = Steering_Right_Limit;
    } else if (value < Steering_Left_Limit) {
        value = Steering_Left_Limit;
    }
    request->steeringRequest = value;
}

// Getter for speedRequest
int getDrivingSpeedRequest(struct CtrlRequest *request) {
    return request->speedRequest;
}

// Setter for speedRequest
void setDrivingSpeedRequest(struct CtrlRequest *request, int value) {
    if(value > Driving_Forward_Limit) {
        value = Driving_Forward_Limit;
    } else if (value < Driving_Reverse_Limit) {
        value = Driving_Reverse_Limit;
    }
    request->speedRequest = value;
}