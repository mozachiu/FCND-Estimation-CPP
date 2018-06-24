# Estimation Project #

## The Goal of this Project ##

In this project, you will be developing the estimation portion of the controller used in the CPP simulator. By the end of the project, your simulated quad will be flying with your estimator and your custom controller (from the previous project)!

## Writeup ##

### Implemented body rate control in C++. ###
#### The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments. ####

- Translate the commanded roll, pitch and yaw into desired rotational acceleration with proportional controller(pqrCmd - pqr).
```
momentCmd = VI * kpPQR * (pqrCmd - pqr);
```


### Implement roll pitch control in C++. ###
#### The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles. ####

- The equation below is used to calcuate body rate command in roll pitch controller.
<p align="center">
<img src="images/FCND02.png" width="400"/>
</p>

```
pqrCmd.x = (R21 * bc_dot.x - R11 * bc_dot.y) / R33;
pqrCmd.y = (R22 * bc_dot.x - R12 * bc_dot.y) / R33;

```
- Based on a desired global lateral acceleration and desired collective thrust.
