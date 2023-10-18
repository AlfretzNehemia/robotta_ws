#!/usr/bin/env python
# encoding: utf-8
import time

# class simplePID:
#     def __init__(self, kp, ki, kd):
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd
#         self.targetpoint = 0
#         self.integral_ = 0
#         self.derivative = 0
#         self.prevError = 0

#     def compute(self, target, current):
#         error = target - current
#         self.integral_ += error
#         self.derivative = error - self.prevError
#         self.targetpoint = self.kp * error + self.ki * self.integral_ + self.kd * self.derivative
#         # self.integral_result = self.ki * self.integral_
#         self.prevError = error
#         # print("integral error: ", self.integral_result)
#         return self.targetpoint

#     def reset(self):
#         self.targetpoint = 0
#         self.integral_ = 0
#         self.derivative = 0
#         self.prevError = 0

class simplePID:
    def __init__(self, kp, ki, kd, limMax):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.targetpoint = 0
        self.integral_ = 0
        self.derivative = 0
        self.prevError = 0
        self.prevMeasurement = 0
        self.limMax = limMax
        self.limMin = -limMax
        self.time_sampling = 0.002
        self.tau = 0.001

    def compute(self, target, measurement):
        error = target - measurement
        self.proportional = self.kp * error

        if (self.limMax > self.proportional):
            limMaxInt = self.limMax - self.proportional
        else:
            limMaxInt = 0
        

        if (self.limMin < self.proportional):
            limMinInt = self.limMin - self.proportional
        else:
            limMinInt = 0
        
        self.integral_ = self.integral_ + 0.5 * self.ki * self.time_sampling * (error + self.prevError)

        # Anti-WIndup via integrator clamping
        if (self.integral_ > limMaxInt):
            self.integral_ = limMaxInt
        if (self.integral_ < limMinInt):
            self.integral_ = limMinInt
        
        # Derivative (band-limited differentiator)
        # self.derivative = -(2 * self.kd * (measurement - self.prevMeasurement) + (2 *self.tau - self.time_sampling) * self.derivative) / (2 * self.tau + self.time_sampling)
        # Derivative (without filter)
        self.derivative = (self.kd * (error - self.prevError)) / self.time_sampling
        # Output pid
        self.targetpoint = self.proportional + self.integral_ + self.derivative
        
        if (self.targetpoint > self.limMax):
            self.targetpoint = self.limMax
        if (self.targetpoint < self.limMin):
            self.targetpoint = self.limMin
        
        self.prevError = error
        self.prevMeasurement = measurement
        return self.targetpoint

    def reset(self):
        self.targetpoint = 0
        self.integral_ = 0
        self.derivative = 0
        self.prevError = 0
        self.prevMeasurement = 0