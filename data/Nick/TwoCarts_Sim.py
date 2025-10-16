import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as spi

def CoupledOsc_RHS(time, state, omega):
    x1 = state[0]
    v1 = state[1]
    x2 = state[2]
    v2 = state[3]

    rhs = np.zeros(4)

    rhs[0] = v1
    rhs[1] = - 2*omega0**2*x1 + omega0**2*x2 - 2*damping_beta*v1 + drive_amp*np.cos(omega*time)
    rhs[2] = v2
    rhs[3] = omega0*x1 - 2.0*omega0**2*x2 - 2*damping_beta*v2

    return rhs

omega0 = 2.0*np.pi*0.33
damping_beta = 2.0*np.pi*0.05
drive_amp = 0.1

n_times = 2000
t_end = 1000.0
times = np.linspace(t_end/2, t_end, n_times)
dt = times[1] - times[0]

ICs = [0.0, 0.0, 0.0, 0.0]

n_omegas = 100
drive_omegas = np.linspace(0.1*omega0, 3*omega0, n_omegas)
amps = np.zeros([n_omegas,2])
phases = np.zeros([n_omegas,2])

for i, omega in enumerate(drive_omegas):
    sol = spi.solve_ivp(CoupledOsc_RHS, [0.0, t_end], ICs, t_eval=times, args=(omega,))

    x_1 = sol.y[0]
    x_2 = sol.y[2]

    spec_1 = np.fft.rfft(x_1)
    spec_2 = np.fft.rfft(x_2)

    freqs = np.fft.rfftfreq(len(x_1), dt)

    power_1 = np.abs(spec_1)**2
    power_2 = np.abs(spec_2)**2

    #Don't include the DC signal
    peak_power_i1 = np.argmax(power_1[1:])+1
    peak_power_i2 = np.argmax(power_2[1:])+1

    if peak_power_i1 == peak_power_i2:
        peak_i = peak_power_i2
        amp_1 = np.abs(spec_1[peak_i])
        amp_2 = np.abs(spec_2[peak_i])

        phase_c1 = np.arctan2(np.imag(spec_1[peak_i]), np.real(spec_1[peak_i]))
        phase_c2 = np.arctan2(np.imag(spec_2[peak_i]), np.real(spec_2[peak_i]))

        #print('Cart 1   : ', np.round(amp_1,3), ' Phase: ', np.round(phase_c1,3))
        #print('Cart 2   : ', np.round(amp_2,3), ' Phase: ', np.round(phase_c2,3))
    else:
        print('Peaks do not agree')
        print(freqs[peak_power_i1], peak_power_i1)
        print(freqs[peak_power_i2], peak_power_i2)

    amps[i,:] = np.array([amp_1, amp_2])
    phases[i,:] = np.array([phase_c1, phase_c2])

plt.figure()
plt.plot(drive_omegas, amps[:,0], label='Cart 1')
plt.plot(drive_omegas, amps[:,1], label='Cart 2')
plt.legend()
plt.xlabel('Drive Frequency')
plt.ylabel('Amplitude')

plt.show()







