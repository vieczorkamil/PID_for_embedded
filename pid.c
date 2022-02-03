#include "pid.h"

void PID_Set_Param(PIDController *pid, float proportionalGain, float integralTime, float derivativeTime, float samplingTime, float alfa)
{
    pid->Kp = proportionalGain;
    if (integralTime != 0)
    {
        pid->Ti = integralTime;
    }
    else
    {
        return;
    }
    pid->Td = derivativeTime;
    pid->h = samplingTime;

    pid->alfa = alfa;
}

void PID_Set_OutputLimits(PIDController *pid, float minimumLimit, float maximumLimit)
{
    pid->limMin = minimumLimit;
    pid->limMax = maximumLimit;
}

void PID_Init(PIDController *pid)
{
    /* Clear controller variables */
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevMeasurment = 0.0f;
    pid->out = 0.0f;
}

void PID_Reset(PIDController *pid)
{
    PID_Init(pid);
}

float PID_Update(PIDController *pid, float setpoint, float measurment)
{
    /* Error signal */
    float error = setpoint - measurment;

    /* Proportional term */
    float proportional = pid->Kp * error;

    /* Integral term */
    float integratorNewPart = (0.5f * pid->h / pid->Ti) * (error + pid->prevError);

    /* Aint-windup */
    if (pid->out >= pid->limMax || pid->out <= pid->limMin)
    {
        pid->integrator += 0;
    }
    else
    {
        pid->integrator += integratorNewPart;
    }

    /* Derivative + low pass filter */
    pid->differentiator = (2.0f * pid->Td * (error - pid->prevError) + (2.0f * pid->alfa * pid->Td - pid->h) * pid->differentiator) / (2.0f * pid->alfa * pid->Td + pid->h);

    /* Compute output and apply limits */
    pid->out = proportional + pid->integrator + pid->differentiator;

    /* Limits */
    if (pid->out > pid->limMax)
    {
        pid->out = pid->limMax;
    }
    else if (pid->out < pid->limMin)
    {
        pid->out = pid->limMin;
    }

    /* Store error and measurment for next use */
    pid->prevError = error;
    pid->prevMeasurment = measurment;

    /* Return controller output */
    return pid->out;
}