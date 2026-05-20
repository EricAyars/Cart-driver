'''
single_cart_servo_sweep.py
Eric Ayars
Modified from AutoResonance.py 5/18/26

Program to sweep through a resonance range for a single cart.
    * adjust frequency
    * wait for transient to die
    * take 1024 points on demand: time, drive for a single cart
    * analyze fft, save values
    * repeat ad nauseum

Use gdx.
Change 10/14/25: hopefully fixed the dropped-connection problem: getData() was missing a gdx.stop().

Arguments should be start f, stop f, f step, and filename.
'''


########################################
#
#   Libraries
#
########################################

# Basics
from sys import argv
from time import sleep, asctime
import numpy as np

# Vernier's communications API
from gdx import gdx
gdx = gdx.gdx()

Cart_ID = 'GDX-CART-Y 0D1017T7'

# CartDriver API
import pyvisa
from CartDriver import CartDriver

########################################
#
#   Constants
#
########################################

dT = 100            # sampling period, ms
startF = float(argv[1])    # start frequency sweep, Hz
stopF = float(argv[2])     # stop frequency sweep, Hz
fStep = float(argv[3])     # frequency sweep step size
amplitude = 2.0     # constant amplitude (may need to make lower)
N = 1024            # number of data points to take
deadTime = 120      # two minutes for transient decay?
filename = argv[4]

########################################
#
#   Variables
#
########################################

frequency = startF  # current frequency

########################################
#
#   Useful Functions
#
########################################

def connect():
    '''
    gdx.open(connection='ble')
    1: GDX-CART-Y 0D400518 BLE -78
    Only one cart, so adjust that.
    '''
    # adjust for cart!
    gdx.open(connection='ble', 
             device_to_open=Cart_ID)
    # hard-coded for the carts currently in my lab.

    # position is sensor 1 for each cart (only one.)
    gdx.select_sensors([[1]])


def getData():
    ''' 
    fills Servo, Green, and Yellow data-vectors
    '''
    Servo = np.zeros(N) # servo position
    cart = np.zeros(N)  # cart position

    gdx.start(dT)
    for j in range(N):
        '''
        # this is vestigial from the 2-cart software.
        try:
            Yellow[j], Green[j] = gdx.read()
        except:
            print('IndexError in gdx.read(), reconnecting...')
            gdx.close()
            connect()
            continue
        '''
        cart[j] = gdx.read()[0]
        Servo[j] = servo.getPosition()

    gdx.stop()
    return (Servo, cart)

def findPhases(data, drive_freq):
    '''
    Analyzes Servo and cart, courtesy of Nick's code that I don't 
    entirely understand yet.
    Returns array of amplitude and phase information.
    '''
    dt = dT

    servo_x = data[0]
    x_1 = data[1]
    #x_2 = data[2]

    times = np.arange(0, dt*len(servo_x), dt)
    spec_servo = np.fft.rfft(servo_x)
    spec_1 = np.fft.rfft(x_1)
    #spec_2 = np.fft.rfft(x_2)

    freqs = np.fft.rfftfreq(len(servo_x), dt)

    power_servo = np.abs(spec_servo)**2
    power_1 = np.abs(spec_1)**2
    #power_2 = np.abs(spec_2)**2

    #Don't include the DC signal
    peak_power_i1 = np.argmax(power_1[1:])+1
    #peak_power_i2 = np.argmax(power_2[1:])+1

    peak_i = peak_power_i1
    amp_servo = np.abs(spec_servo[peak_i])
    amp_1 = np.abs(spec_1[peak_i])

    phase_servo = np.arctan2(np.imag(spec_servo[peak_i]), np.real(spec_servo[peak_i]))
    phase_c1 = np.arctan2(np.imag(spec_1[peak_i]), np.real(spec_1[peak_i]))

    print('Servo Amp: ', np.round(amp_servo,3), ' Phase: ', np.round(phase_servo,3))
    print('Cart     : ', np.round(amp_1,3), ' Phase: ', np.round(phase_c1,3))


    # Adjust cart phases to be relative to drive phase, and within -pi..pi.
    phase_c1 -= phase_servo
    while phase_c1 < -np.pi:
        phase_c1 += 2.0*np.pi
    while phase_c1 > np.pi:
        phase_c1 -= 2.0*np.pi
    #while phase_c2 < -np.pi:
    #    phase_c2 += 2.0*np.pi
    #while phase_c2 > np.pi:
    #    phase_c2 -= 2.0*np.pi

    #return np.array([amp_1, phase_c1, amp_2, phase_c2])
    return np.array([amp_1, phase_c1])


def savePhases(data):
    '''
    Adds a line to the datafile showing what happened at this frequency.
    '''
    fh.write(f'{frequency:.2f}')
    for j in range(len(data)):
       fh.write(f'\t{data[j]:.5f}')
    fh.write('\n')


########################################
#
#   Communications with servo
#
########################################

rm = pyvisa.ResourceManager()
devices = rm.list_resources()
#usually this is the last thing attached...
servo = CartDriver(devices[-1])

# set up
servo.setAmplitude(amplitude)

# have it warm up while carts are connected
servo.setFrequency(startF)
servo.start()

########################################
#
#   Communications with carts
#
########################################

connect()

########################################
#
#   Write file header 
#
########################################

fh = open(filename, 'w')
fh.write('AutoResonance.py data\n')
fh.write(asctime() + '\n')
fh.write(f'amplitude {amplitude:.2f}\n')
fh.write(f'frequency range {startF:.2f} to {stopF:.2f} by {fStep:.2f} Hz\n')
fh.write(f'sampling period {dT} ms, {N} points per sample set taken {deadTime} seconds after frequency adjustment\n')
fh.write('\nFrequency\tC1 amp\tC1 phase\n')

########################################
#
#   Main data-collection loop
#
########################################

print('Frequency sweep begun at ' + asctime())

while frequency < stopF:

    servo.setFrequency(frequency)
    sleep(deadTime)    # wait for transient decay
    data = getData()
    phases = findPhases(data, frequency)
    print(phases)           # just so we know it's doing something!
    savePhases(phases)
    
    # increment frequency
    frequency += fStep

    # ... and do the loop again!

########################################
#
#   Clean up and quit
#
########################################

fh.close()

servo.stop()
servo.center()
servo.close()

gdx.stop()
gdx.close()

print('Frequency sweep ended at ' + asctime())
