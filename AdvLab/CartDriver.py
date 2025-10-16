'''
CartDriver.py
Eric Ayars
10/7/25

API for PID-group Sin-Driver.

'''

# everything here requires pyvisa.
import pyvisa

class CartDriver:
    '''
    API for the CSU Chico PID Lab Sinusoidal Driver

    '''
    ''' 
    self.driving is a boolean that tracks whether the driver is wiggling or not.
    '''
    driving = False

    def __init__(self, 
                 resource_name=None, 
                 termination ='\n', 
                 timeout = 3000
                 ):
        """Initialize the driver using VISA."""
        self.rm = pyvisa.ResourceManager()
        if resource_name is None:
            # Open the first available resource if not specified
            resources = self.rm.list_resources()
            if not resources:
                raise ValueError("No VISA devices found.")
            else:
                print('Available resources:')
                print(resources)
                raise ValueError("Must specify address")
        self.inst = self.rm.open_resource(resource_name)
        self.inst.read_termination = termination
        self.inst.write_termination = termination
        self.inst.timeout = timeout
        print(f"Connected to: {self.inst.query('*IDN?') if hasattr(self.inst, 'query') else resource_name}")

    def identify(self):
        ''' *IDN? command '''
        return self.inst.query('*idn?')

    def start(self):
        """Send 'drive 1' command to the device, update driving status."""
        self.driving = True 
        self.inst.write('drive 1')

    def stop(self):
        """Send 'drive 0' command to the device, update driving status."""
        self.driving = False
        self.inst.write('drive 0')

    def getFrequency(self):
        '''
        Returns float value of current frequency setting.
        '''
        return float(self.inst.query('freq?'))

    def setFrequency(self, frequency):
        '''
        Sets frequency of device.
        '''
        self.inst.write(f'freq {frequency}')

    def getAmplitude(self):
        '''
        Returns float value of current amplitude setting.
        '''
        return float(self.inst.query('ampl?'))

    def setAmplitude(self, amplitude):
        '''
        Sets amplitude of device
        '''
        self.inst.write(f'ampl {amplitude}')

    def moveTo(self, angle):
        '''
        Moves servo to arbitrary angle.
        If drive was on, turns it off first.
        '''
        if self.driving:
            self.stop()
        self.inst.write(f'move {angle}')
    
    def center(self):
        '''
        moves servo to center and stops it.
        '''
        if self.driving:
            self.stop()
        self.inst.write('center')

    def getPosition(self):
        '''
        Returns (semi-) instantaneous position of driver.
        I am actuall unsure of the phase delay here, but it's probably 
        pretty short... I hope...
        '''
        return float(self.inst.query('posi?'))

    def write(self, message):
        '''
        write arbitrary SCPI command to device
        '''
        self.inst.write(message)

    def query(self, message):
        '''
        sends arbitrary SCPI query to device
        '''
        return self.inst.query(message)

    def close(self):
        """Close the VISA session."""
        self.inst.close()
        self.rm.close()
    

if __name__ == '__main__':
    '''
    Here's a bit of sample code that will hopefully help use this.
    '''
    from time import sleep

    rm = pyvisa.ResourceManager()
    devices = rm.list_resources()
    #usually this is the last thing attached...
    servo = CartDriver(devices[-1])

    servo.setFrequency(1.0)
    servo.setAmplitude(1.0)
    servo.start()

    sleep(5)

    servo.stop()
    servo.center()
    servo.close()
