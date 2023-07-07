#include "mbed.h"
#include "MotorController.h"

MotorController::MotorController(PinName motor_p_, PinName motor_n_, float dt, mext::Ec &ec, CalPID &sc, CalPID &ac) : motor_p(motor_p_), motor_n(motor_n_)
{
    ec_ = &ec;
    sc_pid = &sc;
    ac_pid = &ac;
    motor_p.period_us(50);
    motor_n.period_us(50);
    setDeltaTime(dt);
    setAngleOffset(0);
    setAccelMax(200);
    setAngleOffset(0);
    setDutyLimit(0.98);
    pre_target_omega = 0;
}
float MotorController::limitValue(float value, float max, float min)
{
    float value_return = value;
    if (value_return > max)
        value_return = max;
    else if (value_return < min)
        value_return = min;
    return value_return;
}
//////////////////////////////////////////////////
void MotorController::period_ms(int ms)
{
    motor_p.period_ms(ms);
    motor_n.period_ms(ms);
}
void MotorController::period_us(int us)
{
    motor_p.period_us(us);
    motor_n.period_us(us);
}
void MotorController::period_ns(int ns)
{
    motor_p.period_ns(ns);
    motor_n.period_ns(ns);
}
void MotorController::setDutyLimit(float duty_limit)
{
    if (0 <= duty_limit && duty_limit <= 0.99f)
        duty_max = duty_limit;
    else
        duty_max = 0.99f;
}
void MotorController::setMaxScPID(float pid_max_sc)
{
    sc_pid->setMaxValue(pid_max_sc);
}
void MotorController::setMaxAcPD(float pid_max_ac)
{
    ac_pid->setMaxValue(pid_max_ac);
}
void MotorController::setPIDParamSc(float kp, float ki, float kd)
{
    sc_pid->setParameter(kp, ki, kd);
}
void MotorController::setPDParamAc(float kp, float kd)
{
    ac_pid->setParameter(kp, 0, kd);
}
void MotorController::setDeltaTime(float dt)
{
    delta_t = dt;
    sc_pid->setDELTA_T(dt);
    ac_pid->setDELTA_T(dt);
}
void MotorController::setEquation(float slope_f, float intercept_f, float slope_b, float intercept_b)
{
    slope_forward_rotation = slope_f;
    if (slope_b > 0)
        slope_backward_rotation = slope_b;
    else
        slope_backward_rotation = -slope_b;
    intercept_forward_rotation = intercept_f;
    intercept_backward_rotation = intercept_b;
}
void MotorController::setDutyOffset(float duty_off)
{
    duty_offset = duty_off;
}
void MotorController::setAccelMax(float a_max)
{
    acceleration_max = a_max * delta_t;
}
//////////////////////////////////////////////////////////////
float MotorController::getAngle()
{
    angle = ec_->getRad();
    angle += angle_offset;
    return angle;
}
void MotorController::setAngleOffset(float angle_calibration)
{
    angle_offset = angle_calibration;
    ec_->reset(); 
}
void MotorController::turn(float duty)
{
    if (duty < 1.0 && duty >= 0)
    {
        motor_p = duty;
        motor_n = 0;
    }
    else if (duty < 0 && duty > -1.0)
    {
        motor_p = 0;
        motor_n = -duty;
    }
}
void MotorController::Sc(float target_omega_input)
{
    float duty = calSc(target_omega_input);
    turn(duty);
}
float MotorController::calSc(float target_omega_input)
{
    ///////////////////////////////////////////////加速度制限
    float target_omega = target_omega_input;
    if ((target_omega_input - pre_target_omega) > acceleration_max)
    {
        target_omega = pre_target_omega + acceleration_max;
    }
    else if ((target_omega_input - pre_target_omega) < -acceleration_max)
    {
        target_omega = pre_target_omega - acceleration_max;
    }
    ///////////////////////////////////////////////PID計算
    ec_->calOmega(); 
    ec_->getAcceleration();
    float omega = ec_->getOmega();
    float devia = target_omega - omega;
    float val_feedback = sc_pid->calPI_D(devia, ec_->getAcceleration());
    ///////////////////////////////////////////////モーター特性を利用したFF計算
    float val_feedforward;
    if (target_omega > 0.0)
        val_feedforward = slope_forward_rotation * target_omega + intercept_forward_rotation;
    else if (target_omega < -0.0)
        val_feedforward = slope_backward_rotation * target_omega + intercept_backward_rotation;
    else
        val_feedforward = 0;
    //
    pre_target_omega = target_omega;
    //////////////////////////////////////////////
    float duty_ = val_feedforward + val_feedback;
    duty_ = limitValue(duty_, duty_max, -duty_max);
    return duty_;
}
void MotorController::Ac(float target_angle)
{
    getAngle();
    float devia = target_angle - angle;
    float omega = ac_pid->calP_D(devia, ec_->getOmega()); 
    float duty_offset_ = duty_offset * cos(angle);

    float duty = calSc(omega) + duty_offset_;
    // duty=calSc(omega);
    turn(duty);
}
void MotorController::stop()
{
    motor_p = 0;
    motor_n = 0;
}
void MotorController::reset()
{
    ec_->reset(); 
    sc_pid->resetIntegral();
}