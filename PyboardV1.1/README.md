# Introduction

pwmStepper is a micropython stepper motor driver for the Pyboard V1.0 and V1.1. 

It drives bipolar stepper motors throught hardware drivers like A4988, DRV8825, ...

A pwm signal with 5us length and adjustable period is generated for the step pin. 


A step counter keeps track of the motor position automatically.

Up to three motors can run simultaneoulsy and independantly.

Each motor speed can be changed "on the fly".

# The wiring

<p align="center">
    <img src="./images/wiring.png" width="500">
<p/>

# The driver

Copy pwmStepper.py in the Pyboard filesystem, then:

    >>> from pwmStepper import *
    >>> m0 = pwmStep(0)         # instantiate motor 0 with default config
    >>> m1 = pwmStep(1)         # instantiate motor 1 with default config
    >>> m0.do_steps(50)          # move motor 0 50 steps forward
    >>> m1.set_speed(-m1.speed) # change direction (warning: speed attribute is always >0)
    >>> m1.do_steps(50)         # move motor 1 50 steps backward

Warning:  __*do_steps*__  function enables the hardware driver (enable pin set to 0) and it stays enabled when the motor stops (in order to lock position). The driver must be disabled to stop current flowing through the coils.

## Configuration parameters

They must be set at object creation time:

    >>> m0 = pwmStep(..., param=param_value, ...)

with *param* being:

- id (integer) : driver id (see wiring), 0, 1 or 2
- step_size (float) : the stepper full step size, in user units (eg 1.8 for a Nema with 200 steps/rotation)
- step_unit (string) : the step unit (eg 'deg', 'mm', ...)
- step_res (int) : hardware step resolution (1=full step, 2=1/2 step, 4=1/4 step, ...). Default is 1.
- max_speed (float) : the maximum rotation speed, in step_unit/s
- min_speed (float) : the minimum rotation speed, in step_unit/s

## Driver functions

They can be called for each driver instantiation:

    >>> m0.func(args)

- speed:
  - set_speed(*speed*) : set speed to *speed* in step_unit/s
  - get_speed() : return current speed in step_units/s
- direction
  - set_dir(*dir*) : set direction to forward if *dir*=1 (int), backward otherwise
  - get_dir() : return current direction: 1=forward, -1=backward
- steps
  - do_steps(*nsteps*) : move *abs(nsteps)* (int) in current direction
  - stop() : stop the motor (abort run)
- position
  - reset_position() : set current position to zero (in step_unit)
  - get_position() : return motor position in step_unit
 

## how it works

Each driver instantiation uses two timers:
- a pulse timer to generate the pwm step signal. Its prescaler is set so that the period resolution is 1us. Therefore, the pulse timer period is the step duration in us. The pulse width is 5us which is large enough for the hardware driver. The pulse timer is master timer. 
- a count timer to count steps. It is configured as a slave timer with a clock triggered by the pulse timer. The count timer callback function is used to stop the pulse timer when the desired step number is reached.
