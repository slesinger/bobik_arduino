#ifndef caster_h
#define caster_h

#include <AS5048A.h> //absolute rotation encoder
#include "robot_config_types.h"
#include "component.h"

class Caster : public Component
{
public:
    Caster(Caster_t caster_cfg);

    /**
     * @brief Get the current rotation in units (-8191; 8191)
     *
     * @return int16_t
     */
    int16_t getRotation();

    /**
     * @brief Set the Rotation Target value
     *
     * @param rotation_units value from -8191 to 8191. Negative values rotating left. Ex. 4196 rotates 90deg right. 0 is center pointing forward.
     */
    void setRotationTarget(int16_t rotation_units);

    /**
     * @brief To show that rotation motor is alive, rotate 200ms right and back. Should be safe. for debugging purpose.
     *
     */
    void pingRotationMotor();

    /**
     * @brief To show that drive motor is alive, rotate 200ms right and back. Should be safe. for debugging purpose.
     *
     */
    void pingDriveMotor();

    /**
     * @brief Get number of drive ticks. It only counts when it is suppose to be driven by the motor at that moment.
     *
     */
    int16_t getDriveTicks();
    /**
     * @brief Run preemptive logic once a loop
     *
     * @return number of ticks. 1 tick ~ 2 deg
     */
    void execute();

private:
    Caster_t cfg;
    AS5048A *rotation_sensor;
    void clearDriveTicks();
    static void drive_sensor_interrupt_fl();
    static void drive_sensor_interrupt_fr();
    static void drive_sensor_interrupt_r();

    int16_t rotation_target;
    int16_t pid_prev_rotation;
    int16_t pid_i;
    static uint16_t drive_sensor_ticks_fl;
    static uint16_t drive_sensor_ticks_fr;
    static uint16_t drive_sensor_ticks_r;
};

#endif
