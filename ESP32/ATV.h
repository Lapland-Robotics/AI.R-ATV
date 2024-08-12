// ATV Header file

#ifndef ATV_CONTROL_H
#define ATV_CONTROL_H

#define RC_input_Count 2

struct CtrlRequest {
    int steeringRequest;           // Requested steering value 0-100, 0 = Full Left, 50 = Center and 100 = Full Right
    int speedRequest;      // Requested Driving Speed 0-100, 0 = Full Reverse, 50 = Stop and 100 = Full Forward
};

// Function prototypes
int getSteeringRequest(struct CtrlRequest *request);
void setSteeringRequest(struct CtrlRequest *request, int value);

int getDrivingSpeedRequest(struct CtrlRequest *request);
void setDrivingSpeedRequest(struct CtrlRequest *request, int value);

#endif // ATV_CONTROL_H