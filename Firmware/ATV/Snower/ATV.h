// ATV Header file

#ifndef ATV_H
#define ATV_H

#define RC_input_Count 2
#define Steering_Left_Limit -255    // Left direction limit value for Steering Pot
#define Steering_Right_Limit 255   // Right direction limit value for Steering Pot
#define Driving_Reverse_Limit -255  // Reverse Driving Speed limit (not actual speed m/s)
#define Driving_Forward_Limit 255  // Forward Driving Speed limit (not actual speed m/s)

struct CtrlRequest;

// Function prototypes
int getAngularZ(struct CtrlRequest *request);
void setAngularZ(struct CtrlRequest *request, int value);

int getLinearX(struct CtrlRequest *request);
void setLinearX(struct CtrlRequest *request, int value);

int getLinearY(struct CtrlRequest *request);
void setLinearY(struct CtrlRequest *request, int value);

// Function to create and destroy CtrlRequest objects
struct CtrlRequest* createCtrlRequest(int defaultSteering, int defaultSpeed);
void destroyCtrlRequest(struct CtrlRequest *request);

#endif // ATV_CONTROL_H