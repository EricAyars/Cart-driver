#! /usr/bin/env python
'''
    Servo_controller.py
    Eric Ayars
    9/28/24

    A Python solution to controlling the sinusoidal servo driver

    The way this works:
        There are functions that define what to do when controls are pressed.
        There is a bit of code that draws the controls and puts it on the
            window in the right places.
        Then the program just sits there until something happens.

    9/28/24: V0.9
        starting from my Duffing Oscillator controller, trimming from there.

'''


########################################
#
#   The import list
#
########################################

import sys                      # useful system calls

# VISA stuff for communication with SCPI
import pyvisa

# GUI stuff
from PyQt5.QtWidgets import (
    QApplication, 
    QWidget, 
    QPushButton, 
    QVBoxLayout, 
    QHBoxLayout, 
    QFileDialog,
    QLabel,
    QComboBox,
    QLineEdit,
    QGridLayout
)
from PyQt5.QtGui import QDoubleValidator, QIntValidator

########################################
#
#   Variables
#
########################################

# global variables

ServoSelected = False         # Is the com port selected? (may be unnecessary)
driving = False                 # Is the drive on?
frequency = 0.0                 # current frequency of drive
amplitude = 0.0                 # current amplitude of drive
apparatus = 0                   # link to the apparatus

########################################
#
#   event-handlers
#
########################################

def setFrequency():
    # Sends frequency to the servo driver
    global frequency
    frequency = float(frequencySetting.text())
    apparatus.write(f'freq {frequency}')

def setAmplitude():
    # sends amplitude to the servo driver
    global amplitude
    amplitude = float(amplitudeSetting.text())
    apparatus.write(f'ampl {amplitude}')
    
def driveClicked():
    # toggles drive state
    global driving
    driving = not driving
    apparatus.write(f'drive {int(driving)}')
    if driving:
        driveButton.setText("Off")
    else:
        driveButton.setText("On")

def moveClicked():
    # tells servo to move to a position
    global position, driving
    # turn motor off
    driving = False
    # set to desired position
    position = int(servoSetting.text())
    apparatus.write(f'move {position}')
    driveButton.setText("On")

def interfaceChange():
    # change Duffing interface
    # needs to get current values of frequency and amplitude also.
    global apparatus, frequency, amplitude, DuffingSelected
    apparatus = rm.open_resource(interfaceMenu.currentText())
    apparatus.read_termination = '\n'
    apparatus.write_termination = '\n'
    apparatus.timeout = 3000  # 3 second timeout on communications.

    if apparatus:
        ServoSelected = True
        # Obtain current values
        frequency = float(apparatus.query("frequency?"))
        frequencySetting.setText("%0.3f" % frequency)   
        amplitude = float(apparatus.query("amplitude?"))
        amplitudeSetting.setText("%0.3f" % amplitude)   

    if ServoSelected:
        # make controls active
        frequencySetting.setDisabled(False)
        amplitudeSetting.setDisabled(False)
        driveButton.setDisabled(False)
        servoSetting.setDisabled(False)
        servoButton.setDisabled(False)
        #saveButton.setDisabled(False)


########################################
#
# Main program
#
########################################

app = QApplication(sys.argv)
widget = QWidget()

#
# Interface and parameter selections
#

# Check available VISA resources
rm = pyvisa.ResourceManager()
devices = rm.list_resources()

# interface selection drop-down
interfaceMenu = QComboBox(widget)
interfaceMenu.addItem('Interface')
interfaceMenu.addItems(devices)
interfaceMenu.currentIndexChanged.connect(interfaceChange)

# frequency control
frequencyLabel = QLabel(widget)
frequencyLabel.setText("Frequency")
frequencySetting = QLineEdit()
frequencySetting.setValidator(QDoubleValidator(0,2,3))
frequencySetting.setText("%0.3f" % frequency)   
frequencySetting.setDisabled(True)             
frequencySetting.editingFinished.connect(setFrequency)
frequencyControl = QHBoxLayout()
frequencyControl.addWidget(frequencyLabel)
frequencyControl.addWidget(frequencySetting)
frequencyControl.addStretch(1)

# amplitude control
amplitudeLabel = QLabel(widget)
amplitudeLabel.setText("Amplitude")
amplitudeSetting = QLineEdit()
amplitudeSetting.setValidator(QDoubleValidator(0,2,3))
amplitudeSetting.setText("%0.3f" % amplitude)   # 
amplitudeSetting.setDisabled(True)              # initially disabled.
amplitudeSetting.editingFinished.connect(setAmplitude)
amplitudeControl = QHBoxLayout()
amplitudeControl.addWidget(amplitudeLabel)
amplitudeControl.addWidget(amplitudeSetting)
amplitudeControl.addStretch(1)

# Drive toggle button, Collect button
driveLabel = QLabel(widget)
driveLabel.setText("Drive")
driveButton = QPushButton(widget)
driveButton.setText("On")
driveButton.setDisabled(True)
driveButton.clicked.connect(driveClicked)
driveControl = QHBoxLayout()
driveControl.addWidget(driveLabel)
driveControl.addWidget(driveButton)
driveControl.addStretch(1)

# servo position
servoLabel = QLabel(widget)
servoLabel.setText("Position Servo")
servoSetting = QLineEdit()
servoSetting.setValidator(QIntValidator(0,180))
servoSetting.setText("%d" % 90)
servoSetting.setDisabled(True)
servoButton = QPushButton(widget)
servoButton.setText("Go")
servoButton.setDisabled(True)
servoButton.clicked.connect(moveClicked)
servoControl = QHBoxLayout()
servoControl.addWidget(servoLabel)
servoControl.addWidget(servoSetting)
servoControl.addWidget(servoButton)
servoControl.addStretch(1)

########################################
# Window layout
########################################

# top line
interface = QHBoxLayout() 
interface.addWidget(interfaceMenu)
interface.addStretch(1)

# Control grid
controls = QGridLayout()
controls.addLayout(frequencyControl, 0,0)
controls.addLayout(amplitudeControl, 1,0)
controls.addLayout(driveControl, 2,0)
controls.addLayout(servoControl, 0,1)

layout = QVBoxLayout()
layout.addLayout(interface)
layout.addLayout(controls)

# Draw the screen
widget.setLayout(layout)
widget.setWindowTitle("Servo Driver Control")
widget.show()

sys.exit(app.exec_())
