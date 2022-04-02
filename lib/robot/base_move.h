#ifndef base_move_h
#define base_move_h

#include "robot_utils.h"

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

    if ( (abs(targetAsIs) + 0.8*travelAsIs) <= (abs(targetOposite) + 0.8*travelOposite) +0.06 )  //0.06 is bias towards as-is to avoid random decisions based on rotation sensor noise
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

        caster_fl->setDriveTarget(0, false);
        caster_fr->setDriveTarget(0, false);
        caster_r->setDriveTarget(0, false);
        return;
    }

    // convert unit/second to unit/frame (20FPS)
    x /= FPS;
    y /= FPS;
    gamma /= FPS;
    // Rotate base
    float ax = LEN_SC * cos(DEG_A + gamma) + x;
    float ay = LEN_SC * -sin(DEG_A + gamma) + y;
    float bx = LEN_SC * cos(DEG_B + gamma) + x;
    float by = LEN_SC * -sin(DEG_B + gamma) + y;
    float cx = LEN_SC * cos(DEG_C + gamma) + x;
    float cy = LEN_SC * -sin(DEG_C + gamma) + y;

    // UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.001, 0, -day/dax, 28, "zlomek");
    float dega = RobotUtils::point2rad(ax - POS_A_x, ay - POS_A_y);
    float degb = RobotUtils::point2rad(bx - POS_B_x, by - POS_B_y);
    float degc = RobotUtils::point2rad(cx - POS_C_x, cy - POS_C_y);

    // to be moved in 1 second
    float spda = RobotUtils::l2dist(ax - POS_A_x, ay - POS_A_y);
    float spdb = RobotUtils::l2dist(bx - POS_B_x, by - POS_B_y);
    float spdc = RobotUtils::l2dist(cx - POS_C_x, cy - POS_C_y);

    float spd_coef = speed_cap(CASTER_DRIVE_MAX_SPEED / FPS, spda, spdb, spdc);
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
    float curr_rot_fl = caster_fl->getRotation() * CASTER_UNITS2RAD;
    float curr_rot_fr = caster_fr->getRotation() * CASTER_UNITS2RAD;
    float curr_rot_r = caster_r->getRotation() * CASTER_UNITS2RAD;
    float reverse_speed_a = optimize_rotation(curr_rot_fl, &dega);
    float reverse_speed_b = optimize_rotation(curr_rot_fr, &degb);
    float reverse_speed_c = optimize_rotation(curr_rot_r, &degc);

    caster_fl->setRotationTarget(dega * CASTER_RAD2UNITS);
    caster_fr->setRotationTarget(degb * CASTER_RAD2UNITS);
    caster_r->setRotationTarget(degc * CASTER_RAD2UNITS);

    if (
        (abs(dega - curr_rot_fl) > STOP_DUE_ROTATION_DIFF) ||
        (abs(degb - curr_rot_fr) > STOP_DUE_ROTATION_DIFF) ||
        (abs(degc - curr_rot_r) > STOP_DUE_ROTATION_DIFF)
    )
    {
        driveStoppedDueToRotation = true;
        caster_fl->setDriveTarget(0, driveStoppedDueToRotation);
        caster_fr->setDriveTarget(0, driveStoppedDueToRotation);
        caster_r->setDriveTarget(0, driveStoppedDueToRotation);
    }
    else
    {
        driveStoppedDueToRotation = false;
        caster_fl->setDriveTarget(reverse_speed_a * spda * CASTER_METERS2TICKS, driveStoppedDueToRotation);
        caster_fr->setDriveTarget(reverse_speed_b * spdb * CASTER_METERS2TICKS, driveStoppedDueToRotation);
        caster_r->setDriveTarget(reverse_speed_c * spdc * CASTER_METERS2TICKS, driveStoppedDueToRotation);
    }

}

void Bobik::getCmdVelDebug(float *res)
{
    res[0] = desired_frame_config.base.caster_fl.x;
    res[1] = desired_frame_config.base.caster_fl.y;
    // res[2] = desired_frame_config.base.caster_fl.gamma;
    res[2] = (float)(caster_fl->debug_int/1000.0);
    res[3] = desired_frame_config.base.caster_fl.speed;

    res[4] = desired_frame_config.base.caster_fr.x;
    res[5] = desired_frame_config.base.caster_fr.y;
    res[6] = desired_frame_config.base.caster_fr.gamma;
    // res[6] = (float)(caster_fr->debug_int/1000.0);
    res[7] = desired_frame_config.base.caster_fr.speed;

    res[8] = desired_frame_config.base.caster_r.x;
    res[9] = desired_frame_config.base.caster_r.y;
    res[10] = desired_frame_config.base.caster_r.gamma;
    // res[10] = (float)(caster_r->debug_int/1000.0);
    res[11] = desired_frame_config.base.caster_r.speed;
}

void Bobik::stopAllCastersMotors()
{
    caster_fl->stopMotors();
    caster_fr->stopMotors();
    caster_r->stopMotors();
}

ISR(TIMER1_COMPA_vect)
{
  Bobik::caster_fl->drive_sensor_tick();
  Bobik::caster_fr->drive_sensor_tick();
  Bobik::caster_r->drive_sensor_tick();
}


#endif