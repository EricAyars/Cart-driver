/* drive.ino
 *
 * Eric Ayars
 * 9/28/24
 *
 * firmware for servo-based sinusoidal drive
 * 
 * The masterClock is the 256-event sin-drive counter. 
 * See https://www.pjrc.com/teensy/td_timing_IntervalTimer.html for details on this library.
 *
 * The servo is controlled using PWMServo.
 * See https://www.pjrc.com/teensy/td_libs_Servo.html
 *
 * V0.9 stealing basics of my duffing-drive firmware, trimming it down to do this job.
 * V1.0 Everything seems to work...
 * V1.1 Added SERVO_DIRECTION code, to be set once per hardware depending on servo orientation.
 * V1.2 Made all parameters EEPROM-configurable, added appropriate commands, added dump() for debug. Nothing works now.
 * V1.3 Added debug mode, got things working again. Subtle error involving geometric parameters set to zero so Amax was zero so amplitude couldn't be set to greater than zero... Anyway, it works now. 
 * V1.4 Added 'dumb' mode, which recalculates servo motion to just be sinusoidal and damn the harmonics. Dumb mode is not saved and is lost any time A is adjusted.
 * V1.5 Added commands to report output position, as opposed to servo position. Changed some SCPI accordingly
 *
 */

/****************************************
 * Debug flag
 ****************************************/

bool DEBUG = 0;

/****************************************
 * SCPI optimization
 ****************************************/

#define SCPI_ARRAY_SYZE 3
#define SCPI_MAX_TOKENS 18
#define SCPI_MAX_COMMANDS 25

/****************************************
 * Libraries
 ****************************************/

#include "Arduino.h"
#include <EEPROM.h>
#include <math.h>
#include "PWMServo.h"
#include "Vrekrer_scpi_parser.h"

/****************************************
 * Constants
 ****************************************/

// Identification, firmware version
const char* deviceID = "Sine-Drive V1.5, Chico State PID Lab";

// EEPROM Addresses
const uint8_t SAVE_ADDRESS = 0x00;	// whether to save values of f and A in EEPROM.
const uint8_t DIRECTION_ADDRESS = 0x01;	// servo polarity
const uint8_t L_ADDRESS = 0x10;			// horizontal distance, axis to outlet
const uint8_t R_ADDRESS = 0x20;			// servo arm length
const uint8_t AMPLITUDE_ADDRESS = 0x30;	// last amplitude setting
const uint8_t FREQUENCY_ADDRESS = 0x40;	// last frequency setting

// Microcontroller Pins
const uint8_t SERVO_PIN = 11;	// attachment pin for servo

// Hardware
PWMServo driver;				// create servo object
IntervalTimer masterClock;		// create master interrupt clock
SCPI_Parser Comms;

/****************************************
 * Variables
 ****************************************/

// Geometry
float R; 						// cm, load from EEPROM
float L;						// cm, load from EEPROM
float lo;						// hypoteneuse at center of servo travel
float Amax;						// Can't do more than servo arm will allow...
bool direction;					// Sets servo polarity.

// Servo and timing controll
volatile uint8_t phase = 0; // phase as a byte.
volatile uint8_t tickFlag=0;// do we update servo position?
uint8_t phaseConvert[256];	// look-up table for phase conversion to eliminate cosine error.
uint16_t servoPosition=90;	// Where is the servo now?
float amplitude;			// string (not servo) amplitude. In units of cm, max value Amax.
float frequency;			// driver frequency, units of Hz.
int16_t dt;					// time interval, microseconds

// Booleans
bool driving = 0;			// Whether servo is going (1) or not (0).
bool writeEEPROM = 1;		// whether to save (1) or not (0) F, A in EEPROM.

/****************************************
 * SCPI overhead 
 ****************************************/

void Identify(SCPI_C commands, SCPI_P parameters, Stream& interface) {
	// response to "*IDN?"
	interface.println(deviceID);
	interface.print("Max amplitude ");
	interface.println(Amax);
	if (DEBUG) interface.println("Debug mode on.");
}

void ReportLastError(SCPI_C commands, SCPI_P parameters, Stream& interface) {
	// response to error query
	switch(Comms.last_error){
		case Comms.ErrorCode::BufferOverflow: 
			interface.println(F("Buffer overflow error"));
			break;
		case Comms.ErrorCode::Timeout:
			interface.println(F("Communication timeout error"));
			break;
		case Comms.ErrorCode::UnknownCommand:
			interface.println(F("Unknown command received"));
			break;
		//case Comms.ErrorCode::InvalidParameter:
		//	interface.println(F("Invalid parameter received"));
		case Comms.ErrorCode::NoError:
			interface.println(F("No Error"));
			break;
	}
	Comms.last_error = Comms.ErrorCode::NoError;
}

void errorHandler(SCPI_C commands, SCPI_P parameters, Stream& interface) {
	//This function is called every time an error occurs

	/* The error type is stored in Comms.last_error
		Possible errors are:
		SCPI_Parser::ErrorCode::NoError
SCPI_Parser::ErrorCode::UnknownCommand
		SCPI_Parser::ErrorCode::Timeout
		SCPI_Parser::ErrorCode::BufferOverflow
	*/
	ReportLastError(commands, parameters, interface);

	/* For BufferOverflow errors, the rest of the messamge, still in the interface
	buffer or not yet received, will be processed later and probably 
	trigger another kind of error.
	Here we flush the incomming message*/
	if (Comms.last_error == SCPI_Parser::ErrorCode::BufferOverflow) {
		delay(2);
		while (interface.available()) {
			delay(2);
			interface.read();
		}

	/*
	For UnknownCommand errors, you can get the received unknown command and
	parameters from the commands and parameters variables.
	*/
	}
}

/****************************************
 * Oscillator functions
 ****************************************/

void buildWave(double A) {
	/*	Fills the array with actual angles so that the cosine-errored string moves sinusoidally.
		See calculations in "angle correction work.pdf", 5/2/24. (Eric Ayars lab notebooks)
		Call this each time A changes. (Frequency changes are handled by how fast the program
		cycles through these values.)
		
		Note: the time necessary for the Teensy 4.0 to completely recalculate this conversion
		array is under 240 microseconds. 
	*/

	//intermediate calculation values, done once for speed.
	double phi;
	double omegaT;
	double convert = 2*M_PI/256.0;
	double A2RL = A*A/(2*R*L);
	double loA = 2*lo/A;

	for (int j=0;j<256;j++) {
		omegaT = j*convert;
		double sOt = sin(omegaT);	// Rather than calculate sin(omegaT) three times per loop.
		phi = (asin(A2RL*(loA*sOt + sOt*sOt)) + M_PI/2.0)*180.0/M_PI;
		phaseConvert[j] = (uint8_t)(phi);
	}
	if (DEBUG) Serial.println("buildWave() complete");
}

void buildWaveDumb(double A) {
	/* 
	   Don't use this. It's here purely to generate bad data for my talk about this apparatus.

	   Fills the array with sinusoidal angles, so that the output reverts to the cosine-error
	   oscillation.
	*/

	double phi;
	double convert = 2*M_PI/256.0;

	for (int j=0;j<256;j++) {
		phi = (A/R*127*sin(j*convert)+M_PI/2.0)*180.0/M_PI;
		phaseConvert[j] = (uint8_t)(phi);
	}
	if (DEBUG) Serial.println("buildWaveDumb() complete");
}

void help(SCPI_C commands, SCPI_P parameters, Stream& interface) {
	// helpful information
	interface.println("SCPI commands:");
	interface.println("*IDN? - identify device and provide firmware version.");
	interface.println("AMPLitude {?|f} - query amplitude, or set to floating point value f. \nValue is constrained to Amax.");
	interface.println("FREQuency {?|f} - query frequency, or set to floating point value f. \nNo limit on f, other than reality.");
	interface.println("DRIVe {?|b} - query servo drive state, or set to boolean b.");
	interface.println("MOVE {?|i} - query servo position, or set to integer degree value i. (0<=i<180)");
	interface.println("POSItion? - reports position of output (not position of servo, which is 'MOVE?')" );
	interface.println("CENTer - move servo to center of range. Equivalent to 'MOVE 90'.");
	interface.println("CONFigure:RADIus {?|f} - Query or set servo arm length to float value");
	interface.println("CONFigure:LENGth {?|f} - Query or set servo offset length to float value");
	interface.println("CONFigure:DIREction {?|b} - Query or set servo polarity to boolean");
	interface.println("TUNE:EEPRom {?|b} - Query or set whether f & A values are saved in eeprom");
	interface.println("TUNE:DEBUg {b} - Turns on debug (verbose) mode");
	interface.println("TUNE:DUMP - Dumps LUT for analysis");
	interface.println("TUNE:DEEProm - Dumps EEPROM values for analysis");
	interface.println("TUNE:DUMB - Rebuilds phaseConvert() array to move the servo sinusoidally instead of corrected. Don't use this, it's here only to provide data for my talk on this apparatus.");
	interface.println("HELP - prints this information, but you know that already.");
	interface.println("Note that frequency and amplitude are saved in EEPROM to survive power cycling. (Along with other parameters!)  If you need to change these values more than 100,000 times, turn EEPROM off with 'TUNE:EEPRom 0'.");

}
	
void setAmplitude(SCPI_C commands, SCPI_P parameters, Stream& interface) {
	// sets amplitude. Constrains value to Amax.
	if (DEBUG) interface.println("setAmplitude() called");
	if (parameters.Size() > 0) {
		amplitude = String(parameters[0]).toFloat();
		if (amplitude > Amax) amplitude = Amax;
		if (DEBUG) {
			interface.print("amplitude set to ");
			interface.println(amplitude);
		}
		buildWave(amplitude);
		if (writeEEPROM) {
			EEPROM.put(AMPLITUDE_ADDRESS, amplitude);
		}
	}
}

void getAmplitude(SCPI_C commands, SCPI_P parameters, Stream& interface) {
	// let user know what the amplitude is.
	interface.println(amplitude, 6);
}

void setFrequency(SCPI_C commands, SCPI_P parameters, Stream& interface) {
	// sets drive frequency. No attempt is made to constrain this frequency to sane values.
	if (DEBUG) interface.println("setFrequency() called");
	if (parameters.Size() > 0) {
		frequency = String(parameters[0]).toFloat();
		if (DEBUG) {
			interface.print("frequency set to ");
			interface.println(frequency);
		}
		if (writeEEPROM) {
			EEPROM.put(FREQUENCY_ADDRESS, frequency);
		}
		if (driving) {
			// Drive is currently on, so change frequency.
			dt = (int16_t)(1000000.0/(256.0*frequency));
			masterClock.update(dt);
		}
	}
}

void getFrequency(SCPI_C commands, SCPI_P parameters, Stream& interface) {
	// lets user know what the frequency is.
	interface.println(frequency, 6);
}

void setDrive(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// turns servo on or off.
	if (parameters.Size() > 0) {
		uint16_t whatDrive = String(parameters[0]).toInt();
		if (whatDrive > 0) {
			driving = 1;
			// stepTime is in microseconds
			dt = (int16_t)(1000000.0/(256.0*frequency));
			// start master clock.
			masterClock.begin(clockTick, dt); 
		} else {
			driving = 0;
			masterClock.end();
		}
	}
}

void getDrive(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// Lets user know servo drive state, in case they're blind and deaf.
	interface.println(driving);
}

void centerServo(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// moves servo to center of range.
	if (driving) {
		driving = 0;
		masterClock.end();
	}
	servoPosition = 90;
	driver.write(servoPosition);		
	if (DEBUG) interface.println("Servo Centered");
}

void moveServo(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// Moves servo to a requested position
	if (DEBUG) interface.print("moveServo() ");
	if (parameters.Size() > 0) {
		servoPosition = String(parameters[0]).toInt();
		driver.write(servoPosition);
		if (DEBUG) interface.println(servoPosition);
	}
}

void getPosition(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// reports current position, calculated from amplitude and phase pointer. 
	// Probably does not give valid results if drive is not active.
	double pos = amplitude * sin((phase/256.0)*2.0*M_PI);
	interface.println(pos,6);
}

void whereServo(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// tells user where the servo is.
	interface.println(servoPosition);
}

/****************************************
 * Debug functions
 ****************************************/

void setDebugMode(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// turns on verbose mode to try figuring out WTF.
	if (DEBUG) interface.println("setDebugMode()");
	if (parameters.Size() > 0) {
		DEBUG = String(parameters[0]).toInt();
		if (DEBUG) {
			interface.print("Debug mode set to ");
			interface.println(DEBUG);
		}
	}
}

void dumpEEPROM(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// dumps first 0x50 bytes of eeprom to interface
	uint16_t address = 0;
	for (uint8_t j = 0;j<5;j++) {
		for (uint8_t k = 0;k<16;k++) {
			address = j*0x10 + k;
			interface.print(EEPROM.read(address));
			interface.print("\t");
		}
		interface.println("\n");
	}
}
void dumpLUT(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// dumps LUT to interface for debugging.
	uint16_t address = 0;
	for (uint8_t j=0;j<0x10;j++) {
		for (uint8_t k=0;k<0x10;k++) {
			address = j*0x10+k;
			interface.print(phaseConvert[address]);
			interface.print("\t");
		}
		interface.println("\n");
	}
}

void EEPROM_state(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// tells user whether EEPROM is being used to save A and f or not.
	if (DEBUG) interface.println("EEPROM_state() called.");
	interface.println(writeEEPROM);
}

void setEEPROM(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// sets (and saves) whether EEPROM is being used to save A and f.
	if (DEBUG) interface.println("setEEPROM() called");
	if (parameters.Size() > 0) {
		writeEEPROM = String(parameters[0]).toInt();
		EEPROM.write(SAVE_ADDRESS, writeEEPROM);
		if (DEBUG) {
			interface.print("EEPROM mode set to ");
			interface.println(writeEEPROM);	
		}
	}
}

void getDirection(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// tells user the servo polarity.
	if (DEBUG) interface.println("getDirection() called");
	interface.println(direction);
}

void setDirection(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// sets (and saves) servo polarity.
	if (DEBUG) interface.println("setDirection() called");
	if (parameters.Size() > 0) {
		direction = String(parameters[0]).toInt();
		EEPROM.write(DIRECTION_ADDRESS, direction);
		if (DEBUG) {
			interface.print("direction set to ");
			interface.println(direction);
		}	
	}
}

void getRadius(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// returns current value of R
	if (DEBUG) interface.println("getRadius() called");
	interface.println(R);
}

void getLength(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// returns current value of L
	if (DEBUG) interface.println("getLength() called");
	interface.println(L);
}

void setRadius(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// changes value of R, saves that value in EEPROM.
	if (DEBUG) interface.println("setRadius() called");
	if (parameters.Size() > 0) {
		R = String(parameters[0]).toFloat();
		// calculate new limits
		lo=sqrt(R*R+L*L);
		Amax = R+L-lo;
		// save value in EEPROM
		EEPROM.put(R_ADDRESS, R);
		if (DEBUG) {
			interface.print("Radius set to ");
			interface.println(R);
			interface.print("Amax = ");
			interface.println(Amax);
		}
	}
}

void setLength(SCPI_Commands, SCPI_P parameters, Stream& interface) {
	// changes value of L, saves that value in EEPROM.
	if (DEBUG) interface.println("setLength() called");
	if (parameters.Size() > 0) {
		L = String(parameters[0]).toFloat();
		// calculate new limits
		lo=sqrt(R*R+L*L);
		Amax = R+L-lo;
		// save value in EEPROM
		EEPROM.put(L_ADDRESS, L);
		if (DEBUG) {
			interface.print("Length set to ");
			interface.println(L);
			interface.print("Amax = ");
			interface.println(Amax);
		}
	}
}

/****************************************
 * Interrupt(s)
 ****************************************/

void clockTick() {
	// ISR for master clock. Updates tickFlag and phase
	tickFlag = 1;
	phase++;
}

/****************************************
 * Setup
 ****************************************/

void setup() {

	// Get geometry from EEPROM, calculate limit to A.
	EEPROM.get(L_ADDRESS, L);
	EEPROM.get(R_ADDRESS, R);
	lo=sqrt(R*R+L*L);
	Amax = R+L-lo;

	// get current amplitude and frequency settings from EEPROM
	EEPROM.get(AMPLITUDE_ADDRESS, amplitude);
	EEPROM.get(FREQUENCY_ADDRESS, frequency);
	
	// get whether or not to save to EEPROM from EEPROM.
	writeEEPROM = EEPROM.read(SAVE_ADDRESS);

	// get servo polarity from EEPROM
	direction = EEPROM.read(DIRECTION_ADDRESS);

	// Calculate look-up table for sine wave
	buildWave(amplitude);

	// Calculate dt
	dt = (int16_t)(1000000.0/(256.0*frequency));

	// start servo at center. 
	driver.attach(SERVO_PIN);
	driver.write(servoPosition);

	// configure SCPI commands
	Comms.RegisterCommand(F("*IDN?"), &Identify);
	Comms.RegisterCommand(F("AMPLitude"), &setAmplitude);
	Comms.RegisterCommand(F("AMPLitude?"), &getAmplitude);
	Comms.RegisterCommand(F("FREQuency"), &setFrequency);
	Comms.RegisterCommand(F("FREQuency?"), &getFrequency);
	Comms.RegisterCommand(F("DRIVe"), &setDrive);
	Comms.RegisterCommand(F("DRIVe?"), &getDrive);
	Comms.RegisterCommand(F("CENTer"), &centerServo);
	Comms.RegisterCommand(F("MOVE"), &moveServo);
	Comms.RegisterCommand(F("MOVE?"), &whereServo);
	Comms.RegisterCommand(F("POSItion?"), &getPosition);
	Comms.RegisterCommand(F("HELP"), &help);
	Comms.SetCommandTreeBase(F("CONFigure"));
		Comms.RegisterCommand(F(":RADIus?"), &getRadius);
		Comms.RegisterCommand(F(":RADIus"), &setRadius);
		Comms.RegisterCommand(F(":LENGth?"), &getLength);
		Comms.RegisterCommand(F(":LENGth"), &setLength);
		Comms.RegisterCommand(F(":DIREction?"), &getDirection);
		Comms.RegisterCommand(F(":DIREction"), &setDirection);
	Comms.SetCommandTreeBase(F("TUNE"));
		Comms.RegisterCommand(F(":DUMP"), &dumpLUT);
		Comms.RegisterCommand(F(":DEEProm"), &dumpEEPROM);
		Comms.RegisterCommand(F(":EEPRom?"), &EEPROM_state);
		Comms.RegisterCommand(F(":EEPRom"), &setEEPROM);
		Comms.RegisterCommand(F(":DEBUg"), &setDebugMode);
		Comms.RegisterCommand(F(":DUMB"), &buildWaveDumb);

	Comms.SetErrorHandler(&errorHandler);

	// start serial communications
	Serial.begin(9600);			// actual speed is set by USB bus, 9600 is vestigial.
	while (!Serial) delay(10);	// wait for Serial to start.
	
	if (DEBUG) Serial.println("Setup() complete");
}


/****************************************
 * Loop
 ****************************************/

void loop() {
	
	// Wait around until something happens. When it happens, do something.
	
	// Here's one thing that could happen: serial input. Send that to SCPI parser.
	Comms.ProcessInput(Serial, "\n");

	// Here's another thing that could happen: if driver is true, then every dt
	// there will be an interrupt, which sets tickFlag indicating it's time to
	// update the servo position

	if (tickFlag) {			// Next step!
		tickFlag = 0;		// clear the flag
		
		// update servo position using look-up table			
		servoPosition = phaseConvert[phase];
		if (direction) {
			driver.write(179-servoPosition);	
		} else {
			driver.write(servoPosition);	
		}
	}
}
