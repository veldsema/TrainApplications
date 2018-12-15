/*
 Name:    ServoDCC.ino
 Created: 12/6/2018 12:07:27 PM
 Author:  Jeroen.Veldsema
*/

#include <Wire.h>
#include "PCA9685.h"
#include <DCC_Decoder.h>
#include <Servo.h> 
#include <LiquidCrystal.h>
 

#define kDCC_INTERRUPT 0
 
#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 50

#define SELECT_RIGHT_KEY_CANCEL		0
#define SELECT_UP_KEY_SERVO_PLUS	1
#define SELECT_DOWN_KEY_SERVO_MIN	2

#define SELECT_UP_KEY_ANGLE_PLUS	1
#define SELECT_DOWN_KEY_ANGLE_MIN	2
#define SELECT_LEFT_KEY_MENU		3
#define SELECT_ENTER_KEY_SELECT		4

#define ANGLE_OUTPUT_INIT	0 
#define ANGLE_OUTPUT_ON		1 
#define ANGLE_OUTPUT_OFF	2 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fill in these 2 values ...
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const byte maxservos = 2; //The number of servos you have connected to this Arduino
const byte servotimer = 1000; //Servo angle change timer. Lower value -> higher speed
const byte maxAngleOutputConfig = 3; //pwmAngleOutputInit, pwmAngleOutputOff, pwmAngleOutputOn;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Items for configuration menu
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool bConfigMenuSet = false;
bool bMenuSelected = false;


int iConfigLoopServo = 0; 
int iConfigLoopServoAngle = 0; 


String menuItems[] = { "Head Menu", "Servo Menu", "Angle Menu", "Angle on Menu", "Angle off Menu" };
// Menu control variables
int iMenuPage = 0;
int iMaxMenuPages = round(((sizeof(menuItems) / sizeof(String)) / 2) + .5);
int iCursorPosition = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// initialize the library by associating any needed LCD keypath interface pin
// with the arduino pin number it is connected to
// LCD RS pin to digital pin 8 
// LCD Enable pin to digital pin 9
// LCD D4 pin to digital pin 4
// LCD D5 pin to digital pin 5
// LCD D6 pin to digital pin 6
// LCD D7 pin to digital pin 7
// LCD R/W pin to ground
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 

int analogPin = A0;
int adc_key_old;
int adc_key_in;
int NUM_KEYS = 5;
int key = -1;
int adc_key_val[5] = { 30, 150, 360, 535, 760 };

char msgs[5][17] = { "Right Key OK    ", "Up Key OK       ", "Down Key OK     ", "Left Key OK     ", "Select Key OK   " };
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned long timetoupdatesetpoint = millis() + servotimer;

PCA9685 servosDriver;
 
struct servoItem {
  int channel;
  bool stateServo;
  byte outputPin; // Arduino output pin for additional function (not where servo is attached to)
  uint16_t pwmAngleOutputInit;
  uint16_t pwmAngleOutputOff;
  uint16_t pwmAngleOutputOn;
  uint16_t setpointAngle;
};

struct DCCControl {
  int address; // DCC address to respond to Number of servo on multimaus
  int outputPin; // Arduino output pin
  byte output; // State of accessory: 1=on, 0=off (for internal use only) state where servo need to be set on
  servoItem servoItem; // output pin on servo board
  
};

DCCControl DccControl[maxservos];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fill in the address and pin for every accessory / function. Servos are 'coupled' to accessory[n] in line 72 and further.
// COPY - PASTE as many times as you have functions. The amount must be same as in line 22 above!
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ConfigureDecoderFunctions()
{
  for (int iServo = 0; iServo < maxservos; iServo++)
  {
    DccControl[iServo].address = (iServo + 1);
    DccControl[iServo].servoItem.channel = iServo;
    DccControl[iServo].servoItem.stateServo = false;
  }
} // END ConfigureDecoderFunctions

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Fill in the attributes for every servo
  // COPY - PASTE as many times as you have servo's. The amount must be same as maxservos in line 22 above!
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ConfigureDecoderServos()
{ 
  //ReadCsv();

  for (int iServo = 0; iServo < maxservos; iServo++)
  {
    DccControl[iServo].servoItem.pwmAngleOutputInit = 70; //initial angle of servo. Make this the same as offangle to avoid startup jitter.
    DccControl[iServo].servoItem.pwmAngleOutputOff  = 70; //minimum angle. Do not use value too close to 0, servo may stutter at the extremes.
    DccControl[iServo].servoItem.pwmAngleOutputOn   = 110; //maximum angle. Do not use value too close to 180, servo may stutter at the extremes.
  }

  /*	DCCControl
  accessory[0].address = 1; // DCC address for this accessory
  accessory[0].outputPin = 13; // Arduino pin where accessoryis connected to

  accessory[1].address = 2; // DCC address for this accessory
  accessory[1].outputPin = 13; // Arduino pin where accessory is connected to

  // Setup output pins for accessories
  for (int i = 0; i<maxservos; i++)
  {
  if (accessory[i].outputPin)
  {
  pinMode(accessory[i].outputPin, OUTPUT);
  digitalWrite(accessory[i].outputPin, LOW);
  driver.setChannelPWM(accessory[i].outputPin, pwmServo.pwmForAngle(-90));
  }
  }
  */

} // END ConfigureDecoderServos

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fill in the address and pin for lcd function.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ConfigureLcdFunctions()
{
  lcd.begin(16, 2);                         // set the lcd dimension
  lcd.clear();                              // LCD screen clear
  lcd.print(" Servo Shield ");                   // display the text
  lcd.setCursor(0, 1);                       // set lcd.setCursor (column,row) 
  lcd.print(" starting up! ");
  delay(3000);								// delay for 3000ms
  adc_key_old = analogRead(analogPin);      // store the unpress key value 
}// END ConfigureLcdFunctions

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DCC packet handler 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BasicAccDecoderPacket_Handler(int address, boolean activate, byte data)
{
  // Convert NMRA packet address format to human address
  address -= 1;
  address *= 4;
  address += 1;
  address += (data & 0x06) >> 1;

  boolean enable = (data & 0x01) ? 1 : 0;

  for (int iServo = 0; iServo<maxservos; iServo++)
  {
    if (address == DccControl[iServo].address)
    {
      if (enable) DccControl[iServo].output = 1;
      else DccControl[iServo].output = 0;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup (run once)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  Serial.println("Servo DCC starting up!");
  ConfigureLcdFunctions();
  lcd.print("Servo DCC starting up!");
  DCC.SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket_Handler, true);
  ConfigureDecoderFunctions();
  ConfigureDecoderServos();
  DCC.SetupDecoder(0x00, 0x00, kDCC_INTERRUPT);
  pinMode(2, INPUT_PULLUP); //Interrupt 0 with internal pull up resistor (can get rid of external 10k)
  pinMode(A5, INPUT_PULLUP); //If made LOW, all servos go to their min angle, to avoid jitter at starup.
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); //switch off Arduino led at startup
   
  Wire.begin();                 // Wire must be started first
  Wire.setClock(400000);        // Supported baud rates are 100kHz, 400kHz, and 1000kHz
  servosDriver.resetDevices();        // Software resets all PCA9685 devices on Wire line
  servosDriver.init(B000000);         // Address pins A5-A0 set to B000000
  servosDriver.setPWMFrequency(FREQUENCY);   // Set frequency to 50Hz
  lcd.print("Servo DCC finished starting up!");
  Serial.println("Servo DCC finished starting up!");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop (run continuous)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{	 
	if (bConfigMenuSet != true)
	{
		DccLoop();
		KeyPathLoop();
	}
	else
		configLoop();  
} //END MAIN LOOP

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Config loop (runs only when is set to config mode)
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void configLoop()
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Config Menu");
	drawInstructions();

	int activeButton = 0;
	while (activeButton == 0) {
		int button;
		adc_key_in = analogRead(analogPin);
		if (adc_key_in < 790) {
			delay(100);
			adc_key_in = analogRead(analogPin);
		}
		button = evaluateButton(adc_key_in);

		switch (button) {
		case 0: // When button returns as 0 there is no action taken
			break;
		case 1:  // This case will execute if the "Selected" button is pressed
			button = 0;
			menuItemAngle();
			break;
		case 2:  // This case will execute if the "Up servo" button is pressed
			lcd.clear();
			delay(500);
			iConfigLoopServo++;
			if (iConfigLoopServo >= maxservos - 1)
				iConfigLoopServo = maxservos - 1;
			PrintLcdServoConfig();
			break;
		case 3:  // This case will execute if the "Down servo" button is pressed
			lcd.clear();
			delay(500);
			iConfigLoopServo--;
			if (iConfigLoopServo < 0)
				iConfigLoopServo = 0;
			PrintLcdServoConfig();
			break;

		case 5:  // This case will execute if the "Cancel" button is pressed
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("Exit Config Menu");
			iConfigLoopServo = 0;
			delay(1000);
			bConfigMenuSet = false;
			activeButton = 1;
			break;
		}
	}
} //END CONFIG LOOP

void menuItemAngle() 
{	
	int activeButton = 0;	
	lcd.clear();
	lcd.setCursor(3, 0);
	lcd.print("Sub Menu Angle on/off init");

	while (activeButton == 0) {
		int button;
		adc_key_in = analogRead(analogPin);
		if (adc_key_in < 790) {
			delay(100);
			adc_key_in = analogRead(analogPin);
		}
		button = evaluateButton(adc_key_in);
		switch (button) {
			case 1:  // This case will execute if the "Selected" button is pressed
				button = 0;
				menuItemConfigAngle();
				break;
			case 2:  // This case will execute if the "Up Angle" button is pressed
				lcd.clear();
				delay(500);
				iConfigLoopServoAngle++;
				if (iConfigLoopServoAngle >= maxAngleOutputConfig - 1)
					iConfigLoopServoAngle = maxAngleOutputConfig - 1;
				PrintLcdAngleConfig();
				break;
			case 3:  // This case will execute if the "Down Angle" button is pressed
				iConfigLoopServoAngle--;
				if (iConfigLoopServoAngle < 0)
					iConfigLoopServoAngle = 0;
				PrintLcdAngleConfig();
				break;
			case 5:  // This case will execute if the "back" button is pressed
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Exit Angle Menu");
				iConfigLoopServoAngle = 0;
				delay(1000);
				button = 0;
				activeButton = 1;
				break;
		}
	}
}

void menuItemConfigAngle() 
{
	int setPointAngle = getAngleSetPoint();
	int activeButton = 0;
	lcd.clear();
	lcd.setCursor(3, 0);
	lcd.print("Sub Menu Angle on/off init");

	while (activeButton == 0) {
		int button;
		adc_key_in = analogRead(analogPin);
		if (adc_key_in < 790) {
			delay(100);
			adc_key_in = analogRead(analogPin);
		}
		button = evaluateButton(adc_key_in);
		switch (button) {
		case 1:  // This case will execute if the "Selected" button is pressed
			button = 0;
			//SAVE TO EPROM();
			break;
		case 2:  // This case will execute if the "Up Angle" button is pressed
			lcd.clear();
			delay(500);	
			setPointAngle++;
			setAngleSetPoint(setPointAngle);
			PrintLcdAngleConfig();
			break;
		case 3:  // This case will execute if the "Down Angle" button is pressed			
			setPointAngle--;
			setAngleSetPoint(setPointAngle);
			PrintLcdAngleConfig();
			break;
		case 5:  // This case will execute if the "back" button is pressed
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("Exit Angle set Menu");
			iConfigLoopServoAngle = 0;
			delay(1000);
			button = 0;
			activeButton = 1;
			break;
		}
	servosDriver.setChannelPWM(DccControl[iConfigLoopServo].servoItem.channel, pulseWidth(DccControl[iConfigLoopServo].servoItem.setpointAngle));

	}
}

int getAngleSetPoint()
{
	if (iConfigLoopServoAngle == ANGLE_OUTPUT_INIT)
	{
		return DccControl[iConfigLoopServo].servoItem.pwmAngleOutputInit;
	}
	else if (iConfigLoopServoAngle == ANGLE_OUTPUT_ON)
	{
		return DccControl[iConfigLoopServo].servoItem.pwmAngleOutputOn;
	}
	else if (iConfigLoopServoAngle == ANGLE_OUTPUT_OFF)
	{
		return DccControl[iConfigLoopServo].servoItem.pwmAngleOutputOff;
	}
	else
		return 0;
}

void setAngleSetPoint(int setpoint)
{
	DccControl[iConfigLoopServo].servoItem.setpointAngle = setpoint;
	if (iConfigLoopServoAngle == ANGLE_OUTPUT_INIT)
	{
		DccControl[iConfigLoopServo].servoItem.pwmAngleOutputInit = setpoint;
	}
	else if (iConfigLoopServoAngle == ANGLE_OUTPUT_ON)
	{
		DccControl[iConfigLoopServo].servoItem.pwmAngleOutputOn = setpoint;
	}
	else if (iConfigLoopServoAngle == ANGLE_OUTPUT_OFF)
	{
		 DccControl[iConfigLoopServo].servoItem.pwmAngleOutputOff = setpoint;
	}		
}

void PrintLcdServoConfig( )
{
	lcd.setCursor(0, 0);
	lcd.print("Servo adress:");
	lcd.print(DccControl[iConfigLoopServo].address);
	lcd.setCursor(0, 1);
	lcd.print("Angle:");
	lcd.print(DccControl[iConfigLoopServo].servoItem.pwmAngleOutputOn);
	lcd.print(" :");
	lcd.print(DccControl[iConfigLoopServo].servoItem.pwmAngleOutputOff);
}

void PrintLcdAngleConfig()
{
	lcd.setCursor(0, 0);
	lcd.print("Servo adress:");
	lcd.print(DccControl[iConfigLoopServo].address);
	lcd.setCursor(0, 1);
	if (iConfigLoopServoAngle == ANGLE_OUTPUT_INIT)
	{
		lcd.print("Angle Init:");
		lcd.print(DccControl[iConfigLoopServo].servoItem.pwmAngleOutputInit);
	}
	else if (iConfigLoopServoAngle == ANGLE_OUTPUT_ON)
	{
		lcd.print("Angle On:");
		lcd.print(DccControl[iConfigLoopServo].servoItem.pwmAngleOutputOn);
	}
	else if (iConfigLoopServoAngle == ANGLE_OUTPUT_OFF)
	{
		lcd.print("Angle Off:");
		lcd.print(DccControl[iConfigLoopServo].servoItem.pwmAngleOutputOff);
	}
} 

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DCC loop (run continuous)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DccLoop()
{
	static int address = 0;

	DCC.loop(); // DCC library

	if (++address >= maxservos) address = 0; // Next address to test

	Serial.print("Servo adress:");
	Serial.println(DccControl[address].servoItem.channel);	
	if (DccControl[address].output)
	{
		digitalWrite(DccControl[address].outputPin, HIGH);
		DccControl[address].servoItem.setpointAngle = pulseWidth(DccControl[address].servoItem.pwmAngleOutputOn);
		servosDriver.setChannelPWM(DccControl[address].servoItem.channel, pulseWidth(DccControl[address].servoItem.pwmAngleOutputOn));
	}
	else
	{
		digitalWrite(DccControl[address].outputPin, LOW);
		DccControl[address].servoItem.setpointAngle = pulseWidth(DccControl[address].servoItem.pwmAngleOutputOff);
		servosDriver.setChannelPWM(DccControl[address].servoItem.channel, pulseWidth(DccControl[address].servoItem.pwmAngleOutputOff));

	}
	if (millis() > timetoupdatesetpoint)
	{
		Serial.println("Servo DCC running!");
		timetoupdatesetpoint = millis() + servotimer;
		// printToLcd(address, false);
		printToSerial(address, false);
		Serial.println(servosDriver.getChannelPWM(DccControl[address].servoItem.channel));
		if (servosDriver.getChannelPWM(DccControl[address].servoItem.channel) == DccControl[address].servoItem.setpointAngle)
		{
			printToSerial(address, true);
		 printToLcd(address, true);     
			DccControl[address].output = DccControl[address].output ? 0 : 1;
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Move all servos to min position and set all function outputs to 0, to eliminate startup servo jerk current draw
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (digitalRead(A5) == LOW) { for (int n = 0; n<maxservos; n++) DccControl[n].output = 0; }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// KeyPath loop (run continuous)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void KeyPathLoop()
{
	adc_key_in = analogRead(analogPin);     // read ADC value
	adc_key_in = get_key(adc_key_in); 
	if (SELECT_LEFT_KEY_MENU == adc_key_in)
	{
		lcd.clear();
		lcd.print("Config Menu selected !!!");		
		bConfigMenuSet = true;
	}
	
}
void printToLcd(int servoAdress, bool onPos)
{
	//lcd.clear(); 
	if (onPos)
	{
		lcd.setCursor(0, 0);
		lcd.print("Servo:");
		lcd.print(servoAdress);
		lcd.setCursor(0, 1);
		lcd.print("on pos:");
		lcd.print(DccControl[servoAdress].servoItem.setpointAngle);
		lcd.println();  
		delay(1000);
	}
	else
	{
		lcd.setCursor(0, 0);
		lcd.print("Servo:");
		lcd.print(servoAdress);
		lcd.print((DccControl[servoAdress].output) ? " On " : " Off ");
		lcd.setCursor(0, 1);
		lcd.print("set angle:");
		lcd.print(DccControl[servoAdress].servoItem.setpointAngle);
	} 
}

void printToSerial(int servoAdress, bool onPos)
{  
	if (onPos)
	{
		Serial.print("Servo:");
		Serial.print(servoAdress);
		Serial.print(" is on position! ");
		Serial.println();  
	}
	else
	{ 
		Serial.print("Servo:");
		Serial.print(servoAdress);
		Serial.print((DccControl[servoAdress].output) ? " On" : " Off");      
		Serial.print(" set to angle:");
		Serial.println(DccControl[servoAdress].servoItem.setpointAngle);  
	}
}

int pulseWidth(int angle)
{
	int pulse_wide, analog_value;
	pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
	Serial.println(analog_value);
	return analog_value;
}

int get_key(unsigned int input)
{
  int k;

  for (k = 0; k < NUM_KEYS; k++)
  {
    if (input < adc_key_val[k])
    {
      return k;
    }
  }

  if (k >= NUM_KEYS)
    k = -1;     // No valid key pressed

  return k;
}


// This function is called whenever a button press is evaluated. 
//The LCD shield works by observing a voltage drop across the buttons all hooked up to A0.
int evaluateButton(int x) {
	int result = 0;
	if (x < 50) {
		result = 1; // right
	}
	else if (x < 195) {
		result = 2; // up
	}
	else if (x < 380) {
		result = 3; // down
	}
	else if (x < 555) {
		result = 4; // left
	}
	else if (x < 790) {
		result = 5; // select 
	}
	return result;
}
// If there are common usage instructions on more than 1 of your menu items you can call this function from the sub
// menus to make things a little more simplified. If you don't have common instructions or verbage on multiple menus
// I would just delete this void. You must also delete the drawInstructions()function calls from your sub menu functions.
void drawInstructions() {
	lcd.setCursor(0, 1); // Set cursor to the bottom line
	lcd.print("Use ");
	lcd.print(byte(1)); // Up arrow
	lcd.print("/");
	lcd.print(byte(2)); // Down arrow
	lcd.print(" buttons");
}

