// RobotDriveControl Header file

#ifndef RobotDriveControl_H
#define RobotDriveControl_H

#define RC_input_Count 2
#define Steering_Left_Limit -2.2    // Left direction limit value for Steering Pot
#define Steering_Right_Limit 2.2   // Right direction limit value for Steering Pot
#define Driving_Reverse_Limit -0.8  // Reverse Driving Speed limit (not actual speed m/s)
#define Driving_Forward_Limit 0.8  // Forward Driving Speed limit (not actual speed m/s)
#define WheelBase 0.68 // Distance between front wheels(center)

struct CommandVelocity;

// getter prototypes
double getAngularZ(struct CommandVelocity *request);
double getLinearX(struct CommandVelocity *request);
double getLeftSpeed(struct CommandVelocity *request);
double getRightSpeed(struct CommandVelocity *request);

// setter prototypes
void setCmdVelDiffDrive(struct CommandVelocity *request, double linearX, double angularZ);
void setAngularZ(struct CommandVelocity *request, double value);
void setLinearX(struct CommandVelocity *request, double value);

// Function to create and destroy CommandVelocity objects
struct CommandVelocity* createCommandVelocity();
void destroyCommandVelocity(struct CommandVelocity *request);

#endif // RobotDriveControl_H