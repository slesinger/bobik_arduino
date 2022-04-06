#ifndef bobik_h
#define bobik_h

#include "robot_config_types.h"
#include "robot_frame_config.h"
#include "caster.h"
#include "lidar.h"

class Bobik
{
public:
    Base_t cfg;

    static Caster *caster_fl;  //must be static because it is used in ISR
    static Caster *caster_fr;
    static Caster *caster_r;
    Lidar *lidar;

    Bobik();

    /**
     * @brief Set values from cmd_vel topic. Converts x,y,gamma to PID targets for rotation and drive (motor commands)
     * 
     * @param x - forward [m/s] -0.4; 0.4
     * @param y - strafe left [m/s] -0.4; 0.4
     * @param gamma - turn left [rad] ?;?
     */
    void setCmdVel(float x, float y, float gamma);

    /**
     * @brief Stops rotation and drive motors on all casters
     * 
     */
    static void stopAllCastersMotors();

    /**
     * @brief Call each frame to commit new values to hardware.
     * 
     */
    void execute();

//like private:
    /**
     * @brief Get internal intermediate calculations for unit tests and debugging. E.g. relative position of new base in new frame.
     * 
     * @param res - preallocated array of float. Size 12. 
     * [0,4,8] caster-x; 
     * [1,5,9] caster-y; 
     * [2,6,10] caster-gamma; 
     * [3,7,11] caster speed;
     */
    void getCmdVelDebug(float *res);

    float simplify_rad(float rad);
    int optimize_rotation(float current, float *target);

    /**
     * @brief Normally false. If true, it means casters are performing rotation and direction of new movement is far from current caster's direction. Hence casters are likely in unacceptable configuration. All drives are stopped during that time to avoid braking of casters.
     * 
     */
    bool driveStoppedDueToRotation;


private:
    RobotFrameConfig desired_frame_config;

    /**
     * @brief Helper function to get rotation angle from point. Center of rotation is [0;0].
     * 
     * @param dx point's X coordinate, typically center of rotation is subtracted. +X pointing forward
     * @param dy point's y coordinate, +y pointing right
     * @return rotation angle [rad]. 0 points forward, PI/2 left, -PI/2 right
     */
    // float point2rad(float dx, float dy);
    // float l2dist(float dx, float dy);
    /**
     * @brief If any wheel is requested to to go over max speed capability, calculate coefficient to reduce speed of all wheels to fix max speed.
     * 
     * @param max 
     * @param spd1 
     * @param spd2 
     * @param spd3 
     * @return float miltiplication coefficient to reduce speed
     */
    float speed_cap(float max, float spd1, float spd2, float spd3);
    void read_config() {
       #include "robot_config.h"
    }

};

#endif
