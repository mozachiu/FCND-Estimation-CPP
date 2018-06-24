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
<p align="center">
<img src="images/FCNDEP01.png" width="400"/>
</p>

### Step 2: Attitude Estimation ###
####  In this step, you will be improving the complementary filter-type attitude filter with a better rate gyro attitude integration scheme. The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation.  ####

- The equation below is used to calcuate body rate command in roll pitch controller.
<p align="center">
<img src="images/FCND02.png" width="400"/>
</p>

```
pqrCmd.x = (R21 * bc_dot.x - R11 * bc_dot.y) / R33;
pqrCmd.y = (R22 * bc_dot.x - R12 * bc_dot.y) / R33;

```
- Based on a desired global lateral acceleration and desired collective thrust.
