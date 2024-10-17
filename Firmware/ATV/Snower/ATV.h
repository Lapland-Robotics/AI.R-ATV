// ATV Header file

#ifndef ATV_H
#define ATV_H

#define RC_input_Count 2
#define Steering_Left_Limit -200    // Left direction limit value for Steering Pot
#define Steering_Right_Limit 200   // Right direction limit value for Steering Pot
#define Driving_Reverse_Limit -150  // Reverse Driving Speed limit (not actual speed m/s)
#define Driving_Forward_Limit 150  // Forward Driving Speed limit (not actual speed m/s)

struct CtrlRequest;

// Function prototypes
int getSteeringRequest(struct CtrlRequest *request);
void setSteeringRequest(struct CtrlRequest *request, int value);

int getDrivingSpeedRequest(struct CtrlRequest *request);
void setDrivingSpeedRequest(struct CtrlRequest *request, int value);

// Function to create and destroy CtrlRequest objects
struct CtrlRequest* createCtrlRequest(int defaultSteering, int defaultSpeed);
void destroyCtrlRequest(struct CtrlRequest *request);

#endif // ATV_CONTROL_H