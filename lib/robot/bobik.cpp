#include "bobik.h"
#include <math.h>
#include <unity.h>

Bobik::Bobik()
{
    read_config();

    // Casters
    caster_fl = new Caster(cfg.caster_fl);
    caster_fr = new Caster(cfg.caster_fr);
    caster_r  = new Caster(cfg.caster_r);
}

float Bobik::point2rad(float dx, float dy)
{
    float deg = NAN;
    if (dx == 0)  //90 or -90 deg
        if (dy > 0) deg = -M_PI / 2.0;
        else 
        if (dy < 0) deg = M_PI / 2.0;
        else deg = NAN;
    else
        deg = atan(-dy/dx);
    
    if (dx < 0)
    {
        if (dy > 0)
            deg -= M_PI;
        else
            deg += M_PI;
    }
    return deg;
}

float Bobik::l2dist(float dx, float dy)
{
    return sqrt(dx*dx + dy*dy);
}

float Bobik::speed_cap(float maxspd, float spd1, float spd2, float spd3)
{
    if ( (spd1 <= maxspd) && (spd2 <= maxspd) && (spd3 <= maxspd) )
        return 1.0;  //do not reduce speed if no wheel is asked to go over max speed
    else
        return max(max(maxspd/spd1, maxspd/spd2), maxspd/spd3);
}

float Bobik::simplify_rad(float rad)
{
    rad = fmod(rad, 2 * M_PI);
    if (rad < -M_PI) rad += 2 * M_PI;
    if (rad > M_PI) rad -= 2 * M_PI;
    return rad;
}

int Bobik::optimize_rotation(float current, float *target)
{
    // char buffer [128];
    float trg = *target;
    trg= simplify_rad(trg);
    int reverse_speed = 1;

    // calculate option As-Is
    float targetAsIs = trg;
    float travelAsIs = abs(targetAsIs - current);

    // calculate option using oposite direction
    float targetOposite = trg + M_PI;
    targetOposite = simplify_rad(targetOposite);
    float travelOposite = abs(targetOposite - current);

    if ( (abs(targetAsIs) + 0.8*travelAsIs) <= (abs(targetOposite) + 0.8*travelOposite) )
        trg = targetAsIs;
    else
    {
        trg = targetOposite;
        reverse_speed = -1;
    }
    *target = trg;
    return reverse_speed;
}

void Bobik::setCmdVel(float x, float y, float gamma)
{
    if ( (abs(x) < CASTER_DRIVE_MIN_SPEED) && (abs(y) < CASTER_DRIVE_MIN_SPEED) && (abs(gamma) < 0.0001) )
    {  // no move
        desired_frame_config.base.caster_fl.x = 0;
        desired_frame_config.base.caster_fl.y = 0;
        desired_frame_config.base.caster_fl.speed = 0;

        desired_frame_config.base.caster_fr.x = 0;
        desired_frame_config.base.caster_fr.y = 0;
        desired_frame_config.base.caster_fr.speed = 0;

        desired_frame_config.base.caster_r.x = 0;
        desired_frame_config.base.caster_r.y = 0;
        desired_frame_config.base.caster_r.speed = 0;

        caster_fl->setDriveTarget(0);
        caster_fr->setDriveTarget(0);
        caster_r->setDriveTarget(0);
        return;
    }

    // Rotate base
    float ax = LEN_SC * cos(DEG_A + gamma) + x;
    float ay = LEN_SC * -sin(DEG_A + gamma) + y;
    float bx = LEN_SC * cos(DEG_B + gamma) + x;
    float by = LEN_SC * -sin(DEG_B + gamma) + y;
    float cx = LEN_SC * cos(DEG_C + gamma) + x;
    float cy = LEN_SC * -sin(DEG_C + gamma) + y;

    // UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.001, 0, -day/dax, 28, "zlomek");
    float dega = point2rad(ax - POS_A_x, ay - POS_A_y);
    float degb = point2rad(bx - POS_B_x, by - POS_B_y);
    float degc = point2rad(cx - POS_C_x, cy - POS_C_y);

    // to be moved in 1 second
    float spda = l2dist(ax - POS_A_x, ay - POS_A_y);
    float spdb = l2dist(bx - POS_B_x, by - POS_B_y);
    float spdc = l2dist(cx - POS_C_x, cy - POS_C_y);

    float spd_coef = speed_cap(CASTER_DRIVE_MAX_SPEED, spda, spdb, spdc);
    spda *= spd_coef;
    spdb *= spd_coef;
    spdc *= spd_coef;

    desired_frame_config.base.caster_fl.x = ax;
    desired_frame_config.base.caster_fl.y = ay;
    desired_frame_config.base.caster_fl.gamma = dega;
    desired_frame_config.base.caster_fl.speed = spda;

    desired_frame_config.base.caster_fr.x = bx;
    desired_frame_config.base.caster_fr.y = by;
    desired_frame_config.base.caster_fr.gamma = degb;
    desired_frame_config.base.caster_fr.speed = spdb;

    desired_frame_config.base.caster_r.x = cx;
    desired_frame_config.base.caster_r.y = cy;
    desired_frame_config.base.caster_r.gamma = degc;
    desired_frame_config.base.caster_r.speed = spdc;

    // 1 go normal, -1 go reverse
    float readrot = 0.0;
    float reverse_speed_a = optimize_rotation(readrot, &dega);
    float reverse_speed_b = optimize_rotation(readrot, &degb);
    float reverse_speed_c = optimize_rotation(readrot, &degc);

    caster_fl->setRotationTarget(dega * CASTER_RAD2UNITS);
    caster_fr->setRotationTarget(degb * CASTER_RAD2UNITS);
    caster_r->setRotationTarget(degc * CASTER_RAD2UNITS);

    caster_fl->setDriveTarget(reverse_speed_a * spda * CASTER_METERS2TICKS / FPS);
    caster_fr->setDriveTarget(reverse_speed_b * spdb * CASTER_METERS2TICKS / FPS);
    caster_r->setDriveTarget(reverse_speed_c * spdc * CASTER_METERS2TICKS / FPS);

}

void Bobik::getCmdVelDebug(float *res)
{
    res[0] = desired_frame_config.base.caster_fl.x;
    res[1] = desired_frame_config.base.caster_fl.y;
    res[2] = desired_frame_config.base.caster_fl.gamma;
    res[3] = desired_frame_config.base.caster_fl.speed;

    res[4] = desired_frame_config.base.caster_fr.x;
    res[5] = desired_frame_config.base.caster_fr.y;
    res[6] = desired_frame_config.base.caster_fr.gamma;
    res[7] = desired_frame_config.base.caster_fr.speed;

    res[8] = desired_frame_config.base.caster_r.x;
    res[9] = desired_frame_config.base.caster_r.y;
    res[10] = desired_frame_config.base.caster_r.gamma;
    res[11] = desired_frame_config.base.caster_r.speed;
}

void Bobik::execute() {
    // rotation_actual = caster_fl->getRotation();
    // rotation_actual = caster_fr->getRotation();
    // rotation_actual = caster_r->getRotation();

    // ticks_actual_fl += caster_fl->getDriveTicks();
    // ticks_actual_fr += caster_fr->getDriveTicks();
    // ticks_actual_r  += caster_r->getDriveTicks();

    caster_fl->execute();
    caster_fr->execute();
    caster_r->execute();

}
