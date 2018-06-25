# Estimation Project #

## The Goal of this Project ##

In this project, you will be developing the estimation portion of the controller used in the CPP simulator. By the end of the project, your simulated quad will be flying with your estimator and your custom controller (from the previous project)!

## Writeup ##

### Step 1: Sensor Noise ###
#### For the controls project, the simulator was working with a perfect set of sensors, meaning none of the sensors had any noise. The first step to adding additional realism to the problem, and developing an estimator, is adding noise to the quad's sensors. The calculated standard deviation should correctly capture ~68% of the sensor measurements. ####

- The standard deviation of Quad.GPS.X and  Quad.IMU.AX from the config logs config/log/Graph1.txt and config/log/Graph1.txt is about 0.6 and 0.5.
- Set the values in config/6_Sensornoise.txt as below:
```
MeasuredStdDev_GPSPosXY = .6
MeasuredStdDev_AccelXY = .5
```
- The Result :
```
PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 68% of the time.
```
<p align="center">
<img src="images/FCNDEP01.png" width="400"/>
</p>

### Step 2: Attitude Estimation ###
####  In this step, you will be improving the complementary filter-type attitude filter with a better rate gyro attitude integration scheme. The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation.  ####

- use the Quaternion<float> class, which has a handy FromEuler123_RPY function for creating a quaternion from Euler Roll/PitchYaw.
```
    Quaternion<float> atd = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
    atd.IntegrateBodyRate(gyro, dtIMU); //this uses quaternions

```
- Normalize yaw to -pi .. pi
- Compute predictedPitch, predictedRoll non-linear with complimentary filter for attitude using quaternions.
<p align="center">
<img src="images/FCNDEP02.png" width="400"/>
</p>

- The Result :
```
PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
```

<p align="center">
<img src="images/FCNDEP03.png" width="400"/>
</p>

### Step 3: Prediction Step ###
####  Implementing the prediction step of the filter.  ####

- PredictState() function
```
1. There are 7 states should be predicted : x, y, z, x_dot, y_dot, z_dot and yaw
2. The yaw integral is already done in the IMU update.
3. References
```
<p align="center">
<img src="images/FCNDEP04.png" width="400"/>
</p>

- The correct calculation of the Rgb prime matrix
```
1. Return the partial derivative of the Rbg rotation matrix with respect to yaw.
3. References
```
