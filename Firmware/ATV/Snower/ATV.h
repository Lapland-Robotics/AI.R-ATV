// ATV Header file

#ifndef ATV_H
#define ATV_H

#define RC_input_Count 2
#define Steering_Left_Limit -0.9    // Left direction limit value for Steering Pot
#define Steering_Right_Limit 0.9   // Right direction limit value for Steering Pot
#define Driving_Reverse_Limit -0.9  // Reverse Driving Speed limit (not actual speed m/s)
#define Driving_Forward_Limit 0.9  // Forward Driving Speed limit (not actual speed m/s)
#define WheelBase 58.5 // Distance between front wheels(center)

struct CommandVelocity;

// getter prototypes
double getAngularZ(struct CommandVelocity *request);
double getLinearX(struct CommandVelocity *request);
double getLeftSpeed(struct CommandVelocity *request);
double getRightSpeed(struct CommandVelocity *request);

// setter prototypes
void setCmdVel(struct CommandVelocity *request, double linearX, double angularZ);
void setAngularZ(struct CommandVelocity *request, double value);
void setLinearX(struct CommandVelocity *request, double value);

// Function to create and destroy CommandVelocity objects
struct CommandVelocity* createCommandVelocity(double linearX, double angularZ);
void destroyCommandVelocity(struct CommandVelocity *request);

#endif // ATV_CONTROL_H