#ifndef caster_h
#define caster_h

#include <AS5048A.h> //absolute rotation encoder
#include "robot_config_types.h"
#include <CircularBuffer.h>

class Caster // : public Component
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
    int16_t getDriveTicksRealized();

    /**
     * @brief Tell how many ticks the caster has to perform until next frame. This calculates as debt = requested - realized.
     * 
     * @return int16_t ticks (2 edges per hole = 2 ticks)
     */
    float getDriveTicksDebt();

    /**
     * @brief Add new requirement on number of ticks to current target (debt) Set Drive Effort for given frame before calling execute! PID included.
     *
     * @param drive_ticks positive number: forward, negative: backward
     */
    void setDriveTarget(int16_t drive_ticks, bool stoppedFlag);

    /**
     * Do things at beginning of loop.
     */
    void loop_start();

    /**
     * @brief Runs all caster logic for current frame. !Read getDriveTicksRealized() before calling execute()
     *
     */
    void execute();

    /**
     * @brief Reset all values set by message handlers received from driver. This ensures that missing next frame message will not reuse old values.
     * 
     */
    void clean_afer_execute();

    /**
     * @brief Stop rotation and drive motors on this caster.
     * 
     */
    void stopMotors();

    /**
     * @brief This is called from HW Timer IRQ 1 from bobik.cpp because timer IRQ handling routine (ISR) can only be one.
     * 
     */
    void drive_sensor_tick();

    /**
     * @brief For smoothing PWM drive
     * 
     */
    int16_t pwm_drive_prev;  // dej zpatky do private


    int16_t debug_int;

private:
    /**
     * @brief Pointer to Bobik, the root object. This can be used for referencing.
     * 
     */
    Caster_t cfg;
    AS5048A *rotation_sensor;

    int16_t rotation_target;
    int16_t pid_prev_rotation;
    int16_t pid_i_rotation;
    bool driveStoppedDueToRotation;
    /**
     * Counter increased by interrupt.
     */
    uint16_t drive_sensor_ticks_current_counter;
    int16_t drive_sensor_ticks_last_frame;
    int last_drive_sensor_val;
    int8_t last_frame_ticks_dir;
    unsigned long drive_sensor_tick_last_update_ms; // to filter IR signal jitter on edges, interrupts are damn fast. Signal needs to be stable at least 2ms
    int16_t pid_i_drive;
    /**
     * Holds unrealized ticks. It is debt from last frame an cummulates to a buffer for running average
     */
    CircularBuffer<int16_t, 20> drive_debt_queue;
    int16_t drive_current_frame_required_ticks;
    /**
     * @brief Take memory of requested drive direction. This memory is later used to make drive ticks positive or negative (give drive sensor a direction).
     * @brief 1 = forward, 0 = stop, -1 = backward
     */
    int8_t drive_direction;

};

#endif
