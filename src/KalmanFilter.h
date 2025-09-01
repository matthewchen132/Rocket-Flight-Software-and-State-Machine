#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter
{
private:
    // Posterior state estimate
    float x_post[2];
    // Prior state estimate
    float x_prior[2];
    // Covariance matrix P
    float p[4];
    // Process noise
    float q_ang;
    float q_gyro;
    // Measurement noise
    float r;

public:
    // Constructor
    KalmanFilter();

    // Process the posterior state estimate
    void process_posterior_state(float new_rate, float new_angle, float dt);

    // Get the angle estimate
    float get_angle();
};

#endif // KALMANFILTER_H
