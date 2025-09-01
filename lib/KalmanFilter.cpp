#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    q_ang = 0.001;
    q_gyro = 0.003;
    r = 0.03;
    x_post[0] = 0.0;
    x_post[1] = 0.0;
    x_prior[0] = 0.0;
    x_prior[1] = 0.0;
    p[0] = 0.0;
    p[1] = 0.0;
    p[2] = 0.0;
    p[3] = 0.0;
}

void KalmanFilter::process_posterior_state(float new_rate, float new_angle, float dt)
{
    // Predict the next state based on the gyro rate
    x_prior[0] += dt * (new_rate - x_post[1]);

    // Update the error covariance matrix P
    p[0] += dt * (dt * p[3] - p[1] - p[2] + q_ang);
    p[1] -= dt * p[3];
    p[2] -= dt * p[3];
    p[3] += q_gyro * dt;

    // Calculate the Kalman gain
    float s = p[0] + r;
    float k[2];
    k[0] = p[0] / s;
    k[1] = p[2] / s;

    // Calculate the innovation (measurement residual)
    float y = new_angle - x_prior[0];

    // Update the posterior state estimate
    x_post[0] = x_prior[0] + k[0] * y;
    x_post[1] = x_prior[1] + k[1] * y;

    // Update the covariance matrix P
    float p00_temp = p[0];
    float p01_temp = p[1];
    p[0] -= k[0] * p00_temp;
    p[1] -= k[0] * p01_temp;
    p[2] -= k[1] * p00_temp;
    p[3] -= k[1] * p01_temp;
}

float KalmanFilter::get_angle()
{
    return x_post[0];
}
