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
     * Gets number interrupts generated on IR sensor on edge for last frame. Codewheel has 120 holes => 240 edges. !Run preemptive logic once a loop
     * @return number of ticks. 1 tick ~ 2 deg
     */
    int16_t getDriveTicks();

    /**
     * @brief Add new requirement on number of ticks to current target (debt) Set Drive Effort for given frame before calling execute! PID included.
     *
     * @param drive_ticks positive number: forward, negative: backward
     */
    void setDriveTarget(int16_t drive_ticks);

    /**
     * @brief Runs all caster logic for current frame. !Read getDriveTicks() before calling execute()
     *
     */
    void execute();

    /**
     * @brief Stops rotation and drive motors on all casters
     * 
     */
    static void stopAllCastersMotors();

    /**
     * @brief Stop rotation and drive motors on this caster.
     * 
     */
    void stopMotors();

private:
    Caster_t cfg;
    AS5048A *rotation_sensor;
    static void drive_sensor_interrupt_fl();
    static Caster *caster_fl;
    static void drive_sensor_interrupt_fr();
    static Caster *caster_fr;
    static void drive_sensor_interrupt_r();
    static Caster *caster_r;
    void inc_drive_sensor_tick();

    int16_t rotation_target;
    int16_t pid_prev_rotation;
    int16_t pid_i_rotation;
    uint16_t drive_sensor_ticks;
    int8_t last_frame_ticks_dir;
    unsigned long drive_sensor_tick_last_update_ms; // to filter IR signal jitter on edges, interrupts are damn fast. Signal needs to be stable at least 2ms
    int16_t drive_target;
    int16_t pid_i_drive;

};

#endif