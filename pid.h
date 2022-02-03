#ifndef PID_H
#define PID_H

/*
    Real PID controller with real low pass filter in derivative part, anti windup and output limits

                                        1          Td*s
    Param      ->  PID(s) = Kp*( 1 + ------ + --------------- )
                                      Ti*s     alfa*Td*s + 1

                     2        1-z^-1
    Tustin  ->  s = --- * ( ---------- )
                     h        1+z^-1
*/

typedef struct
{
    /* Controller gains */
    float Kp;
    float Ti;
    float Td;

    /* Derivative low-pass filter time constant -> alfa << 1 for example 0.1 */
    float alfa;

    /* Output limits */
    float limMin;
    float limMax;

    /* Sample time [s] */
    float h;

    /* Controller static variale */
    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurment;

    /* Controller output */
    float out;
} PIDController;

void PID_Set_Param(PIDController *pid, float proportionalGain, float integralTime, float derivativeTime, float samplingTime, float alfa);
void PID_Set_OutputLimits(PIDController *pid, float minimumLimit, float maximumLimit);
void PID_Init(PIDController *pid);
void PID_Reset(PIDController *pid);
float PID_Update(PIDController *pid, float setpoint, float measurment);

#endif