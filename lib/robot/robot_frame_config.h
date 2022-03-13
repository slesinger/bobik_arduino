#ifndef robot_frame_config_h
#define robot_frame_config_h

struct CasterFrame_t {
    float x;
    float y;
    float gamma;
    float speed;
};

struct BaseFrame_t {
    CasterFrame_t caster_fl;
    CasterFrame_t caster_fr;
    CasterFrame_t caster_r;
};

class RobotFrameConfig
{
public:
    BaseFrame_t base;
    RobotFrameConfig();

private:
};

#endif
