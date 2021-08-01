# Overleap

<p align="center">
  <img src="Images/grass_still_landscape.jpeg">
</p>

## Overview

Overleap is an open-source low-cost 3D printed dynamic robotic leg designed be as accessible as possible. Overleap can be assembled with only a 3D printer and minimal tools, without CNC milling / waterjet cutting / laser cutting like other robotic legs.

Overleap's building process is tailored to be simple and seamless, from the assembly manual to the programming method, creating a unique assembly experience.

In the future, overleap is planned to be used in other legged projects, such as a low-cost quadruped robot.

## Capabilities

The overleap leg is capable of executing quick and dynamic movements. Overleap can perform a single jump

<p align="center">
  <img width="500" src="Images/grass_jump_slowmo.gif">
</p>

as well as continous hopping in a circle

<p align="center">
  <img src="Images/full_circle_fast_run.gif">
</p>

The height and stance time of each jump, as well as horizontal speed of the leg, can be varied as desired.

## Design

<p align="center">
  <img src="Drawings/Overleap Full Drawing.png">
</p>

### Quasi-Direct Drive Actuation

Overleap uses quasi-direct drive actuators to drive the leg. The motors are geared down 9:1, allowing for increased torque while having high backdrivability and bandwidth.

To keep the footprint small, the gear system for the hip joint is housed beside the hip, while the gear system for the knee joint is housed within the upper leg. The actuators are coaxial in the sense that the knee motor is on the same axis as the hip motor output shaft.

### Custom foot contact sensor

A custom foot contact sensor was developed to detect when overleap was on the ground. This foot sensor is cheap and requires no machining/moulding.

The foot sensor consists of a 3D printed frame and rubber outer padding for grip. Force detection is made possible through the use of a force sensitive resistor mounted between the outer foot pad and the frame.

The minimum actuation force of the foot sensor is around 10 grams, and can be increased in software depending on the desired conditions.

### 2 DOF rotating base

For testing overleap, the leg is attached to a rotating base with 2 degrees of freedom. This allows the leg to move in both the vertical and horizontal (circular) direction.

The base plate is made out of presswood, while the actual rotating mechanism consists of 3D printed parts. Square aluminum tubes are used as the stand and boom to attach the leg.

### Controlling leg movement

Overleap works by directly controlling the torque of both motors. The microcontroller sends ASCII commands through UART to the ODrive motor controller for the desired torque of each axis. The calculation for these values of torque depends on the current phase of the leg.

During the stance phase (when the foot is touching the ground), overleap calculates a sinusoidal force profile that provides the desired impulse to the leg, then applies this force profile throughout the duration of the stance phase. To calculate the necessary torque values that achieve the desired force at the foot, the transpose of the Jacobian is used.

During the aerial phase, overleap runs a PD control loop to hold a predefined position until the foot comes into contact with the ground. While inverse kinematics can be used to calculate the angles of the leg to achieve the desired foot position, overleap defines the foot position using the joint angles directly as it is easy to visualize where the foot would be.

### Force profile

The force profile in the vertical (y) direction is a sinusoidal curve, specifically of the form a sin^2(b pi t).  The parameters a and b are calculated based on the desired force impulse and stance duration, and t is the time (duration) of the force application. This allows for the regulation of jumping height as well as gradual application of force.

In the horizontal (x) direction, the force applied is a set proportion of the vertical force. Horizontal speed can then be modulated by defining the maximum horizontal force to be applied. On top of allowing for gradual force application, this also keeps the foot firmly in contact with the ground even on slippery surfaces by taking into consideration its coefficient of friction.

## Parts

The full parts list and STL models for the 3D printed parts are available at the [parts](Parts) folder.

## Assembly

### Mechanical assembly

Start by installing all the heat set inserts into the parts. After installing, they should sit flush against the plastic and feel secure when tightening screws into them.

Next, glue the magnets that came with the AS5047P encoder evaluation board onto the back of the motor. Super glue or epoxy can be used for this.

Line up the rubber pad against the inner sensing area of the force sensitive resistor and cut the rubber to the same size. This will be used as the foot pad later on.

At this point, we will need to jump ahead to one portion of the electrical assembly and solder the jumper wires to the encoder. Solder 5 cables to the A, B, I, 5V, and GND outputs of the encoder board. Take note which wire colors correspond to which outputs, as this will be needed later on.

Now, we can continue with the mechanical assembly. As they say, a picture is worth a thousand words, so this guide will be presented as a series of clean and simple drawings similar to Ikea manuals.

These illustrations are designed to be simplistic and intuitive such that the placement of screws, timing belts, and bearings can easily be inferred from them.

You can also view the [complete assembly guide](Drawings/Complete_Assembly.pdf) compiled as a single PDF.

<p align="center">
  <img src="Drawings/Step 1.png">
  <img src="Drawings/Step 2.png">
  <img src="Drawings/Step 3.png">
  <img src="Drawings/Step 4.png">
  <img src="Drawings/Step 5.png">
  <img src="Drawings/Step 6.png">
  <img src="Drawings/Step 7.png">
  <img src="Drawings/Step 8.png">
  <img src="Drawings/Step 9.png">
  <img src="Drawings/Step 10.png">
  <img src="Drawings/Step 11.png">
  <img src="Drawings/Step 12.png">
  <img src="Drawings/Step 13.png">
  <img src="Drawings/Step 14.png">
  <img src="Drawings/Step 15.png">
</p>

### Electrical assembly

As a side note, I wrote a [separate guide](https://github.com/aarondls/motor-position-control) on starting with the ODrive motor controller. This may be handy with getting familiar with ODrive.

The electronic schematic for overleap is as follows:

<p align="center">
  <img src="Drawings/Circuit drawing.png">
</p>

A power resistor can also be attached to the ODrive motor controller, if your power supply cannot dissipate excess current.
