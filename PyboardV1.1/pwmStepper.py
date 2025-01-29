import os
import micropython
from machine import mem32
from pyb import Pin, Timer, freq, micros, elapsed_micros
from stm import TIM1, TIM2, TIM3, TIM4, TIM5, TIM8, \
                TIM_ARR, TIM_CR2, TIM_SMCR, TIM_CNT, TIM_PSC
#
micropython.alloc_emergency_exception_buf(300) # for debugging
#
STEP_MAX_TIME = 0xffff    # max value of ARR for pulse timers
PULSE_WIDTH   = 5         #  pwm pulse width in us
#
dir_pins         = ['X1', 'Y1', 'X9']
step_pins        = ['X2', 'Y2', 'X10']
enable_pins      = ['X3', 'Y3', 'X11']
#
if 'PYBv1.1' not in os.uname()[4] and 'PYBv1.0' not in os.uname()[4]:
    print('Warning:  This is not a pyboard V1.0 or V1.1')

pulse_timer      = [2, 8, 4]
pwm_channels     = [2, 2, 2]
pulse_timer_addr = [TIM2, TIM8, TIM4]
count_timer      = [1, 5, 3]
count_timer_addr = [TIM1, TIM5, TIM3]
timer_smcr       = [0b10111, 0b0110111, 0b0110111]
# set prescaler and period of pwm timer to get a 1 us period
period    = 999
prescaler = [freq()[2]*2//1000000 - 1, freq()[3]*2//1000000 - 1,
             freq()[2]*2//1000000 - 1]
print('##### pwmStepper module  #####')
print('     ', os.uname()[4])
print('      period:', period, '   prescalers:', prescaler[0],
      ',', prescaler[1], ',', prescaler[2])
#
################################################################################
#
#   Driver for bipolar stepper without acceleration/deceleration
#
#   Steps triggered by PWM
#
################################################################################

class pwmStep():

    def __init__(self, id=0, step_res=1, step_size=1.8, step_unit='deg',
                 max_speed=1800, min_speed=0):
        #
        if id not in (0,1,2):
            print('pwmStep.__init__ - Error : id ', id,
                  ' invalid (must be 0, or 1')
            return
        #
        self.id = id
        # Pin initialization
        self.dir_pin    = Pin(dir_pins[id],         Pin.OUT)
        self.step_pin   = Pin(step_pins[id],        Pin.OUT)
        self.enable_pin = Pin(enable_pins[id],      Pin.OUT)
        self.enable_pin.value(1)               # driver output disabled
        # motor attributes initialisation
        self.step_size  = step_size         # full step size
        self.step_unit  = step_unit
        self.step_count = 0
        self.step_res   = step_res         # step resolution
        self.min_speed  = max(min_speed,   # minimum speed
                              1000000*step_size/STEP_MAX_TIME/step_res)
        self.max_speed  = max_speed        # maximum speed
        self.speed      = max_speed
        self.step_time  = round(1000000*step_size/max_speed/step_res)
        self.dir        = 1                # rotation direction
        # Print setup
        print('*********** Stepper driver initialization ***********')
        print('     dir pin                 : ', self.dir_pin.names()[1])
        print('     step pin                : ', self.step_pin.names()[1])
        print('     enable pin              : ', self.enable_pin.names()[1])
        print('     full step size          : ', self.step_size, self.step_unit)
        print('     min/max speed           : ', self.min_speed, '/',
              self.max_speed, self.step_unit, '/s')
        # Init pulse timer (master)
        self.init_pulse_timer()
        # Init count timer (slave)
        self.init_count_timer()
        # Set initial resolution
        mem32[self.PTIM_ARR] = self.step_time
        # set initial speed
        self.set_speed(self.speed)
        self.nsteps = 0
        self.running = False

    def init_pulse_timer(self):
        # timer base address
        self.PTIM = pulse_timer_addr[self.id]
        # autoreload register address
        self.PTIM_ARR = self.PTIM + TIM_ARR
        # count register address
        self.PTIM_CNT = self.PTIM + TIM_CNT
        #
        self.pulse_timer = Timer(pulse_timer[self.id],
                                 prescaler=prescaler[self.id],
                                 period=self. step_time-1)
        self.pwm = self.pulse_timer.channel(pwm_channels[self.id],
                                            mode=Timer.PWM,
                                            pin=self.step_pin,
                                            pulse_width=0)
        #
        self.stop_pulse_timer()
        # configure timer as master
        mem32[self.PTIM+TIM_CR2] = 0b100000
        # set pwm pulse width
        self.pwm.pulse_width(PULSE_WIDTH)
        # draw step pin low
        mem32[self.PTIM_CNT] = PULSE_WIDTH

    def start_pulse_timer(self):
        mem32[self.PTIM] = 0b10000001

    def stop_pulse_timer(self):
        mem32[self.PTIM] = 0

    def init_count_timer(self):
        # timer base address
        self.CTIM = count_timer_addr[self.id]
        # autoreload register address
        self.CTIM_ARR = self.CTIM + TIM_ARR
        # count register address
        self.CTIM_CNT = self.CTIM + TIM_CNT
        #
        self.count_timer = Timer(count_timer[self.id],
                                 prescaler=0,
                                 period=999999)
        #
        self.stop_count_timer()
        mem32[self.CTIM_CNT] = 0
        # configure timer as slave
        mem32[self.CTIM+TIM_SMCR] = timer_smcr[self.id]
        #
        self.count_timer.callback(self.count_timer_callback)
        # slave timer must be started before master
        self.start_count_timer()

    def start_count_timer(self):
        mem32[self.CTIM] = 1

    def stop_count_timer(self):
        mem32[self.CTIM] = 0

    def count_timer_callback(self, e):
        # pulse count reached, stop pulse timer and counter timer
        mem32[self.PTIM] = 0
        self.running = False
        # update step count
        self.step_count += self.dir*mem32[self.CTIM_ARR]

    def enable(self):
        ''' enable hardware driver '''
        self.enable_pin.value(0)

    def disable(self):
        ''' disable hardware driver '''
        self.enable_pin.value(1)
        self.running = False

    def set_dir(self, dir):
        ''' set rotating direction to forward if dir=1,
            backward otherwise
        '''
        self.dir_pin.value(dir == 1)
        self.dir = 1 if self.dir_pin.value() else -1

    def get_dir(self):
        return self.dir

    def get_position(self):
        ''' return current position in step_unit '''
        pos = self.step_count*self.step_size/self.step_res
        pos += mem32[self.CTIM_CNT]*self.step_size*self.dir/self.step_res
        return pos

    def reset_position(self):
        ''' set current position to 0 step_unit '''
        self.step_count = 0

    def do_steps(self, nsteps):
        ''' move nsteps in current direction '''
        if nsteps == 0: return
        self.nsteps = abs(nsteps)
        # enable driver
        self.enable()
        self.running = True
        # minimum step nb
        mem32[self.CTIM_CNT] = 1
        # trigger pulse timer 1 us after start
        mem32[self.PTIM_CNT] = self.step_time-1
        # trigger count timer callback after nsteps
        mem32[self.CTIM_ARR] = self.nsteps
        # load pulse timer period
        mem32[self.PTIM_ARR] = self.step_time
        self.start_pulse_timer()

    def stop(self):
        ''' abort current run '''
        # stop pulse timer and count timer
        mem32[self.PTIM] = 0
        self.running = False
        # update step count
        self.step_count += self.dir*mem32[self.CTIM_CNT]

    def set_speed(self, speed):
        ''' set speed to abs(speed) in step_unit/s in forward direction
            if speed > 0, backward otherwise
        '''
        # check speed bounds
        if abs(speed) > self.max_speed:
            print(' speed value too high, must be <=', self.max_speed,
                  self.step_unit,'/s')
            return
        elif abs(speed) < self.min_speed:
            print(' speed value too low, must be >=', self.min_speed)
            return
        self.speed = abs(speed)
        self.set_dir(speed/self.speed)
        # compute step time (= pwm timer period)
        self.step_time = round(1000000*self.step_size/self.speed/self.step_res)
        if self.step_time > STEP_MAX_TIME:
            self.step_time = STEP_MAX_TIME
            self.speed = 1000000*self.step_size/self.step_time/self.step_res
            print('*** Warning *** maximum step time reached, speed set to ',
                  self.speed, self.step_unit, '/s')
        # load pulse timer period
        mem32[self.PTIM_ARR] = self.step_time

    def get_speed(self):
        ''' return current signed speed in step_unit/s '''
        return self.speed*self.dir