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
     * @brief Run preemptive logic once a loop
     * 
     */
    void execute();

private:
    Caster_t cfg;
    AS5048A *rotation_sensor;

    int16_t rotation_target;
    int16_t pid_prev_rotation;
    int16_t pid_i = 0;
};

#endif
