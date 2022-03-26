#ifndef robot_utils_h
#define robot_utils_h


class RobotUtils
{
public:
    RobotUtils();
    /**
     * @brief if X is negative return -1, else return 1
     * 
     * @param x 
     * @return int 
     */
    static int8_t sign(int x);

    /**
     * @brief Usefull for setting motor driver in1 pin
     * 
     * @param x signed pwm effort
     * @return int HIGH or LOW for pin value
     */
    static int sign1(int16_t x);

    /**
     * @brief Usefull for setting motor driver in2 pin
     * 
     * @param x signed pwm effort
     * @return int HIGH or LOW for pin value
     */
    static int sign2(int16_t x);

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
    static long map_cut(long x, long in_min, long in_max, long out_min, long out_max);

private:

};

#endif