#ifndef robot_utils_h
#define robot_utils_h


class RobotUtils
{
public:
    /**
     * @brief if X is negative return -1, else return 1
     * 
     * @param x 
     * @return int 
     */
    static int8_t sign(int x)
    {
        return (x < 0) ? -1 : 1;
    }

    /**
     * @brief Usefull for setting motor driver in1 pin
     * 
     * @param x signed pwm effort
     * @return int HIGH or LOW for pin value
     */
    static int sign1(int16_t x)
    {
        return (x > 0) ? HIGH : LOW;
    }

    /**
     * @brief Usefull for setting motor driver in2 pin
     * 
     * @param x signed pwm effort
     * @return int HIGH or LOW for pin value
     */
    static int sign2(int16_t x)
    {
        return (x <= 0) ? HIGH : LOW;
    }

    /**
     * @brief Same as arduino map function but it will not extrapolate value, just interpolate.
     * 
     * @param x Actual value
     * @param in_min 
     * @param in_max 
     * @param out_min 
     * @param out_max 
     * @return long Safe number between out_min and out_max
     */
    // static long map_cut(long x, long in_min, long in_max, long out_min, long out_max);
    static long map_cut(long x, long in_min, long in_max, long out_min, long out_max)
    {
        if (x <= in_min) return out_min;
        if (x >= in_max) return out_max;
        return (x - in_min) * (float)(out_max - out_min) / (float)(in_max - in_min) + out_min;
    }

private:

};

#endif