import numpy as np
import matplotlib.pyplot as plt
import os

def extract_amp_and_phase(filepath, delta_t, make_plots=False):

    data = np.genfromtxt(filepath, skip_header=1)
    
    dt = delta_t

    servo_x = data[:,0]
    x_1 = data[:,1]
    x_2 = data[:,2]

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

    

    if make_plots:


        plt.plot(times, servo_x, '-g', label='Driver')
        plt.plot(times, x_1, '-r', label="Cart 1")
        plt.plot(times, x_2, '-b', label='Cart 2')
        plt.title('Time Series')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.legend()

        plt.figure()
        plt.plot(freqs, power_servo, '-g', label='Driver')
        plt.plot(freqs, power_1, '-r', label='Cart 1')
        plt.plot(freqs, power_2, '-b', label='Cart 2')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Power')
        plt.yscale('log')
        plt.legend()

        phase_servo = np.arctan2(np.imag(spec_servo), np.real(spec_servo))
        phase_1 = np.arctan2(np.imag(spec_1), np.real(spec_1))
        phase_2 = np.arctan2(np.imag(spec_2), np.real(spec_2))

        plt.figure()
        plt.plot(freqs, phase_servo, '-g', label='Driver')
        plt.plot(freqs, phase_1, '-r', label='Cart 1')
        plt.plot(freqs, phase_2, '-b', label='Cart 2')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Phase Angle (rad)')
        plt.legend()

        plt.show()

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
        print('Peaks do not agree')
        print(freqs[peak_power_i1], peak_power_i1)
        print(freqs[peak_power_i2], peak_power_i2)

    amps = np.array([amp_servo, amp_1, amp_2])
    phases = np.array([phase_servo, phase_c1, phase_c2])

    return amps, phases


datadir = '/Users/njnelson/Research/CartDriver/Cart-driver/data/Nick/'

dt = 1/50.0

files = os.listdir('/Users/njnelson/Research/CartDriver/Cart-driver/data/Nick/')
data_files = []
freqs = []
for file in files:
    if file.endswith('.txt'):
        start, freq, end = file.split('.')
        freqs.append(float(freq)/100)
        data_files.append(file)


print(data_files)

n_files = len(freqs)
amps = np.zeros([n_files, 3])
phases = np.zeros([n_files, 3])


for i in range(n_files):
    amps[i,:], phases[i,:] = extract_amp_and_phase(datadir+data_files[i], dt, make_plots=False)

#Adjust to relative phases to driving
for i in range(n_files):
    phases[i,:] -= phases[i,0]
    for j in range(1,3):
        while phases[i,j] < -np.pi:
            phases[i,j] += 2.0*np.pi
        while phases[i,j] > np.pi:
            phases[i,j] -= 2.*np.pi


plt.figure()
plt.scatter(freqs, amps[:,0], color='g', label='Driver')
plt.scatter(freqs, amps[:,1], color='r', label='Cart 1')
plt.scatter(freqs, amps[:,2], color='b', label='Cart 2')
plt.legend()
plt.xlabel('Drive Freq. (Hz)')
plt.ylabel('Amplitude (m)')

plt.figure()
plt.scatter(freqs, phases[:,1], color='r', label='Cart 1')
plt.scatter(freqs, phases[:,2], color='b', label='Cart 2')
plt.legend()
plt.xlabel('Drive Freq. (Hz)')
plt.ylabel('Relative Phase to Driving (rad)')

plt.show()

