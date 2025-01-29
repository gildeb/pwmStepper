# Introduction

pwmStepper implements a bipolar stepper motor driver for the Raspberry Pi Pico (RP2). The board must be connected to a hardware driver which can be either DRV8834 or MPU6500 from Pololu.

The driver benefits of the RASP2 PIO feature to generate a pwm with acurate period to drive the step pin. The pulse width is set to 3 microseconds.

The step resolution and motor speed can be set and changed 'on the flight'.

Two motors can be driven simultaneoulsy.

# The wiring

<p align="center">
    <img src="./wiring.png" width=600">
<p/>

# The driver

Copy pwmStepper.py in the RASP2 filesystem, then:

    >>> from pwmStepper import pwmStep
    >>> m0 = pwmStep(0)      # instantiate motor 0 with default config
    >>> m1 = pwmStep(1)      # instantiate motor 1 with default config
    >>> m0.doSteps(100)      # move motor 0 100 steps forward
    >>> m2.doSteps(-50)      # move motor 1 50 steps backward

## Configuration parameters

They must be set at object creation time:

    >>> m0 = pwmStep(..., param=param_value, ...)

with *param* being:

- __dev__ (string) : the hardware motor driver. At present, only two devices are implemented: 'DRV8834' and 'MPU6500'. Default is 'DRV8834'.
- __step_size__ (float) : the stepper full step size, in user units (eg 1.8 for a Nema with 200 steps/rotation)
- __step_unit__ (string) : the step unit (eg 'deg', 'mm', ...)
- __stepRes__ (int) : initial step resolution (1=full step, 2=1/2 step, 4=1/4 step, ...). Default is 1.
- __max_speed__ (float) : the maximum rotation speed, in steep_unit/s
- __min_speed__ (float) : the minimum rotation speed, in steep_unit/s

## Driver functions

They can be called for each driver instantiation:

    >>> m0.func(args)

- step resolution:
  - __setRes__(*res*) : set step resolution to *res* (int). Resolution must be supported by the hardware device (see Pololu docs). Changing step resolution does not change motor speed. 
  - __getRes__() : return resolution (1=full step, 2=1/2 step, ...)
- speed:
  - __setSpeed__(*speed*) : set speed to *speed* in step_unit/s
  - __getSpeed__() : return current speed in step_units/s
- direction
  - __setDir__(*dir*) : set direction to forward if *dir*=1 (int), backward otherwise
  - __getDir__() : return current direction: 1=forward, -1=backward
- steps
  - __doSteps__(*nsteps*) : move *abs(nsteps)* (int) forward if *nsteps>0*, backward otherwise
  - __stop__() : stop the motor (abort current run)
- position
  - __resetPosition__() : set current position to zero (in step_unit)
  - __getPosition__() : return motor position in step_unit since last position reset
 
## Driver constants

- __STEP_MAX_TIME__ : maximum step duration in microseconds. Default is 0xffffffff
- __STEP_MAX_NB__ : maximum number of steps per run (__doSteps__). Default is 0xffffffff
- __PWM_PULSE_WIDTH__ : step pwm pulse width in 0.1 microsecond unit. Default is 30 -> pulse width = 3 microseconds


# Implementation

## pwm state machine

To achieve an acurate step pulse timing, a PIO state machine is used (one for each motor). The state machine frequency is set to 20MHz (corresponding to 1 instruction/100ns). The y scratch register keep the pulse width value. The pwm period is stored in isr register and loaded in x scratch register at the begining of each period, unless a new value can be pulled from the input FIFO. Afterward, x is decremented until it first reaches y, which triggers the step pin, then 0, which reset the step pin and start a new cycle.

## counter state machine

The second state machine is used as a step downcounter. x scratch register is loaded with the target *nsteps* number and is decremented each step. The synchronisation of the two state machine is achieved by an irq. When x reaches 0, the counter state machine triggers the __stepsEnd__() irq handler function and waits for another *nsteps* value in its input FIFO. The pwm state machine by irq.

Therefore, feeding the pwm FIFO during the run will instantaneously change the step period (motor speed) without modifying the total number of steps to run.

## position tracking

To determine position, the driver keep track of the total number of steps in each direction for each step resolution. The information is stored in the __resCountList__ (python dictionnary) attribute. 

As the dictionnary is updated after each run (__doSteps__), you should not change step resolution while the motor is rotating. Test the boleen attribute *running* to determine the moteur state.
