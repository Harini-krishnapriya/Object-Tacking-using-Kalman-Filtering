# Object-Tacking-using-Kalman-Filtering
This repository contains various kalman filtering techniquies for tracking objects(Navigation).
## Table of Contents

- [Introduction](#introduction)
- [Friedlands Model for tracking](#friedlands_model_for_tracking)

## Introduction
  The Kalman filter is a mathematical algorithm that is widely used for estimating the state of a dynamic system by combining noisy measurements with a system model. It was developed by Rudolf Kalman in the 1960s and has since found applications in various fields, including control systems, robotics, computer vision, and signal processing.

The fundamental idea behind the Kalman filter is to estimate the current state of a system based on the previous state and the current measurement, while taking into account the uncertainties associated with both the system dynamics and the measurements. It is considered an optimal estimator in the sense that it minimizes the mean square error of the estimated state.

The Kalman filter operates in a recursive manner, meaning that it continuously updates the state estimate as new measurements become available. It consists of two main steps: the prediction step and the update step.

In the prediction step, the filter uses the system model to predict the next state based on the previous state estimate. This prediction incorporates the system dynamics and any control inputs. The prediction also includes an estimation of the uncertainty or covariance associated with the predicted state.

In the update step, the filter combines the predicted state with the actual measurement to obtain an improved estimate of the current state. This step involves calculating the Kalman gain, which determines the relative weight given to the prediction and the measurement. The Kalman gain is based on the covariance of the predicted state and the covariance of the measurement. The update step also updates the covariance of the state estimate based on the accuracy of the measurement.

By iteratively performing the prediction and update steps, the Kalman filter provides an optimal estimate of the system state. It effectively filters out noise from measurements and provides a smoothed estimate that is more accurate than either the raw measurements or the predictions alone.

The Kalman filter has been widely adopted due to its versatility and effectiveness in handling uncertain and noisy measurements. Its applications range from tracking objects in computer vision to stabilizing control systems and navigation systems. It is an essential tool for many real-time estimation and filtering tasks, providing reliable state estimation even in the presence of noise and uncertainty.

## Friedland Model for tracking
Friedland's model is a simplified version of the Kalman filter for 1-D tracking. It assumes linear, constant velocity motion and uses position and velocity as the two states. The filter predicts the next state based on the previous estimate and updates it using measurements, iteratively refining the tracking estimate. It is a straightforward approach for estimating position and velocity in 1-D tracking problems.
Friedland's model for tracking in 1-D can be mathematically described as follows:

1. State Variables:
   - Position: denoted by x
   - Velocity: denoted by v

2. System Dynamics:
   - Assuming constant velocity motion, the state update equation is given by:
     x(k) = x(k-1) + v(k-1) * Δt
     where x(k) is the position at time step k, v(k-1) is the velocity at time step k-1, and Δt is the time interval.

3. State Transition Matrix:
   - In this model, there is no explicit state transition matrix since the system assumes linear dynamics.

4. Measurement:
   - The measured position is denoted by z(k).

5. Measurement Model:
   - Assuming the measurement is corrupted by Gaussian noise, the measurement model can be described as:
     z(k) = x(k) + w(k)
     where z(k) is the measured position at time step k, x(k) is the estimated position at time step k, and w(k) is the measurement noise.

6. Kalman Gain:
   - The Kalman gain, denoted by K, determines the relative weight given to the prediction and the measurement. It is calculated as:
     K(k) = P(k-1) / (P(k-1) + R)
     where P(k-1) is the covariance matrix of the predicted state at time step k-1, and R is the covariance of the measurement noise.

7. State Update:
   - The updated state estimate is given by:
     x(k) = x(k-1) + K(k) * (z(k) - x(k-1))
     where x(k) is the updated position estimate at time step k, z(k) is the measured position at time step k, and x(k-1) is the predicted position at time step k-1.

8. Covariance Update:
   - The updated covariance matrix is given by:
     P(k) = (1 - K(k)) * P(k-1)
     where P(k) is the updated covariance matrix at time step k, and P(k-1) is the covariance matrix of the predicted state at time step k-1.

By iteratively applying these equations, the Friedland's model for tracking in 1-D provides an estimation of the position and velocity based on noisy measurements, incorporating the uncertainties associated with the system dynamics and measurements.
