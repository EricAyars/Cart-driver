'''
AutoResonance.py
Eric Ayars
10/8/25

Program to control this whole experiment.
    * sweep frequency
    * wait for transient to die
    * take 1024 points on demand: time, drive, yellow, green
    * analyze fft, save values
    * repeat ad nauseum

Use gdx.

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

# CartDriver API
from CartDriver import CartDriver

########################################
#
#   Constants
#
########################################

dT = 100            # sampling period, ms
startF = argv[1]    # start frequency sweep, Hz
stopF = argv[2]     # stop frequency sweep, Hz
fStep = argv[3]     # frequency sweep step size
amplitude = 1.5     # constant amplitude (may need to make lower)
N = 1024            # number of data points to take
deadTime = 120      # two minutes for transient decay?
filename = sys.argv[4]

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

def getData():
    ''' 
    fills Servo, Green, and Yellow data-vectors
    '''
    Servo = np.zeros(N) # servo position
    Yellow = np.zeros(N)# yellow cart position
    Green = np.zeros(N) # green cart position

    gdx.start(dT)
    for j in range(N):
        Yellow[j], Green[j] = gdx.read()
        Servo[j] = servo.getPosition()

    return (Servo, Yellow, Green)

def findPhases(data, drive_freq):
    '''
    Analyzes Servo, Green, and Yellow, courtesy of Nick's code that I don't 
    entirely understand yet.
    Returns array of amplitude and phase information.
    '''
    dt = dT

    servo_x = data[0]
    x_1 = data[1]
    x_2 = data[2]

    times = np.arange(0, dt*len(servo_x), dt)
    spec_servo = np.fft.rfft(servo_x)
    spec_1 = np.fft.rfft(x_1)
    spec_2 = np.fft.rfft(x_2)

    freqs = np.fft.rfftfreq(len(servo_x), dt)

    power_servo = np.abs(spec_servo)**2
    power_1 = np.abs(spec_1)**2
    power_2 = np.abs(spec_2)**2

    #Don't include the DC signal
    peak_power_i1 = np.argmax(power_1[1:])+1
    peak_power_i2 = np.argmax(power_2[1:])+1

    if peak_power_i1 == peak_power_i2:
        peak_i = peak_power_i2
        amp_servo = np.abs(spec_servo[peak_i])
        amp_1 = np.abs(spec_1[peak_i])
        amp_2 = np.abs(spec_2[peak_i])

        phase_servo = np.arctan2(np.imag(spec_servo[peak_i]), np.real(spec_servo[peak_i]))
        phase_c1 = np.arctan2(np.imag(spec_1[peak_i]), np.real(spec_1[peak_i]))
        phase_c2 = np.arctan2(np.imag(spec_2[peak_i]), np.real(spec_2[peak_i]))

        print('Servo Amp: ', np.round(amp_servo,3), ' Phase: ', np.round(phase_servo,3))
        print('Cart 1   : ', np.round(amp_1,3), ' Phase: ', np.round(phase_c1,3))
        print('Cart 2   : ', np.round(amp_2,3), ' Phase: ', np.round(phase_c2,3))
    else:
        '''
        I am worried about this 'else' here. Nick, do I need to worry? If not, can we 86 it?
        '''
        print('Peaks do not agree')
        print(freqs[peak_power_i1], peak_power_i1)
        print(freqs[peak_power_i2], peak_power_i2)

        print('Saving data to take a look later')

        filename = 'PositionData_'+str(np.round(drive_freq*1000))+'.npz'
        np.savez(filename, drive_frequency=drive_freq, times = times, servo_x = servo_x,
                 x_1 = x_1, x_2=x_2)
        
        # Put in some dummy values to return
        return np.array([-1, -10, -1, -10])


    # Adjust cart phases to be relative to drive phase, and within -pi..pi.
    phase_c1 -= phase_servo
    phase_c2 -= phase_servo
    while phase_c1 < -np.pi:
        phase_c1 += 2.0*np.pi
    while phase_c1 > np.pi:
        phase_c1 -= 2.0*np.pi
    while phase_c2 < -np.pi:
        phase_c2 += 2.0*np.pi
    while phase_c2 > np.pi:
        phase_c2 -= 2.0*np.pi

    return np.array([amp_1, phase_c1, amp_2, phase_c2])


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

'''
gdx.open(connection='ble')
1: GDX-CART-Y 0D400518 BLE -78
2: GDX-CART-G 0V4001X3 BLE -68
'''
gdx.open(connection='ble', 
         device_to_open="GDX-CART-Y 0D400518, GDX-CART-G 0V4001X3")
# hard-coded for the carts currently in my lab.

# position is sensor 1 for each cart
gdx.select_sensors([[1],[1]])

########################################
#
#   Write file header 
#
########################################

fh = open(filename, 'w')
fh.write('AutoResonance.py data\n')
fh.write(time.asctime + '\n')
fh.write(f'amplitude {amplitude:.2f}\n')
fh.write(f'frequency range {startF:.2f} to {stopF:.2f} by {fStep:.2f} Hz\n')
fh.write(f'sampling period {dT} ms, {N} points per sample set taken {deadTime} seconds after frequency adjustment\n')
fh.write('\nFrequency\tC1 amp\tC1 phase\tC2 Amp\tC2 phase\n')

########################################
#
#   Main data-collection loop
#
########################################

print('Frequency sweep begun at' + time.asctime())

while frequency < stopF:

    servo.setFrequency(frequency)
    time.sleep(deadTime)    # wait for transient decay
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

print('Frequency sweep ended at' + time.asctime())
