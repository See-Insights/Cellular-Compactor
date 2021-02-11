/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/chipmc/Documents/Maker/Particle/Projects/Cellular-Compactor/src/Cellular-Compactor.ino"
/*
* Project Cellular-Compactor - converged software for Monitoring Control Systems
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland chip@seeinsights.com for Inductrial Power & Automation
* Date: Started on 6 December 2018 updated in Jan 2021
*/

/*  This is a variation of the control and monitoring code to sense the status of line voltage
    on two lines.  Based on these two sensors, alerts will be sent via Webhook to Ubidots.
    There are two items being sensed - an low oil warning light (input2) and a 
    compactor full warning light (input1)

    state value   input1      input2       Light behaviour
    0             no alert    no alert     off
    1             100% full   alert        input1 - Flashing  /  input2 - on solid
    2             75% full    NA           input1 - on solid  /  input2 - NA
    Once we get to alert status 1, we disable that interrupt and stop checking until the device is rebooted.

    Both input1 and input2 are inverted so the pin will go LOW when the light is ON!

    To discriminate between solid on and flashing, we will do the following:
      - Solid on will not go off in a 5 second period
      - Flashing will require 5 cycles of on and off (we do not know the period)

    Once we have disabled either interrupt - we will slow flash the Electron's Blue LED

    We will report every hour AND on every change of input1 or input2

    This device has no battery, is line powered, never sleeps, and is always connected
*/

// v0.2 - Moved input pins to B1 and B2.  Added a way to detect flashing for for B1 alerts
// v2.0 - Moving to a more modern construct deviceOS@2.0.1
// v4.0 - Added a debouncing state to the operation


// Particle Product definitions
void setup();
void loop();
bool takeMeasurements();
void sendEvent();
void UbidotsHandler(const char *event, const char *data);
void getSignalStrength();
int getTemperature();
void petWatchdog();
void watchdogISR();
void input1ISR();
void input2ISR();
bool connectToParticle();
bool disconnectFromParticle();
bool notConnected();
void flashLED();
int hardResetNow(String command);
int sendNow(String command);
int setVerboseMode(String command);
void dailyCleanup();
void loadSystemDefaults();
void checkSystemValues();
void checkCurrentValues();
int setTimeZone(String command);
void publishStateTransition(void);
void fullModemReset();
int setDSTOffset(String command);
bool isDSTusa();
#line 38 "/Users/chipmc/Documents/Maker/Particle/Projects/Cellular-Compactor/src/Cellular-Compactor.ino"
PRODUCT_ID(10747);                                   // Connected Counter Header
PRODUCT_VERSION(4);
#define DSTRULES isDSTusa
const char releaseNumber[4] = "4";                  // Displays the release on the menu 

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentStatusAddr     = 0x50                    // Where we store the current counts data structure
  };
};

const int FRAMversionNumber = 1;                    // Increment this number each time the memory map is changed

struct systemStatus_structure {                     // currently 14 bytes long
  uint8_t structuresVersion;                        // Version of the data structures (system and data)
  uint8_t placeholder;                              // available for future use
  uint8_t connectedStatus;
  uint8_t verboseMode;
  int resetCount;                                   // reset count of device (0-256)
  float timezone;                                   // Time zone value -12 to +12
  float dstOffset;                                  // How much does the DST value change?
  unsigned long lastHookResponse;                   // Last time we got a valid Webhook response
} sysStatus;

struct currentCounts_structure {                    // Current values on the device
  uint8_t input1;                                   // input1 is a bit more complicated - it is either off (no alert), on (75% full) or flashing (100% full)
  uint8_t input2;                                   // input2 - Low Oil is either off (no alert) or on solid (low oil alert)
  unsigned long lastEventTime;                      // When did we record our last event
  int temperature;                                  // Current Temperature
  int alertCount;                                   // What is the current alert count
  bool interruptDisconnected;                       // Flag we raise when either input1 or input2 interrupt has been disconnected
} current;


// Included Libraries
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "PublishQueueAsyncRK.h"                    // Async Particle Publish
#include "electrondoc.h"                                 // Documents pinout

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);    // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);         // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
retained uint8_t publishQueueRetainedBuffer[2048];
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, DEBOUNCE_OIL_STATE, DEBOUNCE_FULL_STATE, REPORTING_STATE, RESP_WAIT_STATE };
char stateNames[7][14] = {"Initialize", "Error", "Idle", "Debounce Oil", "Debounce Full", "Reporting", "Response Wait"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;


// Pin Constants
const int tmp36Pin =          A0;               // Simple Analog temperature sensor
const int tmp36Shutdwn =      B5;               // Can turn off the TMP-36 to save energy
const int input1         =    B1;               // Pin for Voltage Sensor interrupt 1
const int input2         =    B3;               // Pin for Voltage Sensor interrupt 2
const int wakeUpPin =         A7;               // This is the Particle Electron WKP pin
const int hardResetPin =      D4;               // Power Cycles the Electron and the Carrier Board
const int userSwitch =        D5;               // User switch with a pull-up resistor
const int donePin =           D6;               // Pin the Electron uses to "pet" the watchdog
const int blueLED =           D7;               // This LED is on the Electron itself


// Timing Variables
unsigned long webhookWait = 45000;              // How long we will wair for a webhook response
unsigned long resetWait = 30000;                // Honw long we will wait before resetting on an error
unsigned long webhookTimeStamp = 0;             // When did we send the webhook
unsigned long resetTimeStamp = 0;               // When did the error condition occur

// Program Variables
volatile bool watchdogFlag = false;
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
char currentOffsetStr[10];                          // What is our offset from UTC
int currentHourlyPeriod = 0;                        // Need to keep this separate from time so we know when to report
bool systemStatusWriteNeeded = false;
bool currentStatusWriteNeeded = false;

// FRAM and Unix time variables
time_t t;
bool dataInFlight = false;                                        // Tracks if we have sent data but not yet cleared it from counts until we get confirmation

// Functional Variables
volatile bool input1Flag = false;                                 // ISR will set this flag
volatile bool input2Flag = false;                                 // ISR will set this flag
char input1Str[16] = "No Alert";                                  // String to describe the state of the flag in the mobile app and console
char input2Str[16] = "No Alert";                                  // String to describe the state of the flag
bool input1ChangeFlag = false;                                    // Let us know if a change is detected

void setup()                                                            // Note: Disconnected Setup()
{
  pinMode(input1,INPUT_PULLUP);                                         // Voltage Sensor Interrupt pin
  pinMode(input2,INPUT_PULLUP);                                         // Voltage Sensor Interrupt pin
  pinMode(wakeUpPin,INPUT);                                             // This pin is active HIGH
  pinMode(userSwitch,INPUT);                                            // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                                             // declare the Blue LED Pin as an output
  pinMode(tmp36Shutdwn,OUTPUT);                                         // Supports shutting down the TMP-36 to save juice
  digitalWrite(tmp36Shutdwn, HIGH);                                     // Turns on the temp sensor
  pinMode(donePin,OUTPUT);                                              // Allows us to pet the watchdog
  pinMode(hardResetPin,OUTPUT);                                         // For a hard reset active HIGH

  digitalWrite(blueLED,HIGH);                                           // Signal we are in setup() 
  
  char responseTopic[125];
  String deviceID = System.deviceID();                                  // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);        // Subscribe to the integration response event

  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("Temperature", current.temperature);
  Particle.variable("Release",releaseNumber);
  Particle.variable("Input1", input1Str);                               // "Off", "Flashing", "Solid"
  Particle.variable("Input2", input2Str);                               // "Off" or "On"

  Particle.function("Hard-Reset",hardResetNow);
  Particle.function("Send-Now",sendNow);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-DSTOffset",setDSTOffset);

  connectToParticle();                                                  // This device is always connected

  // Load FRAM and reset variables to their correct values
  fram.begin();                                                         // Initialize the FRAM module

  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);
  if (tempVersion != FRAMversionNumber) {                               // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                       // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                     // Put the right value in
    loadSystemDefaults();                                               // Out of the box, we need the device to be awake and connected
  }
  else fram.get(FRAM::systemStatusAddr,sysStatus);                      // Loads the System Status array from FRAM

  checkSystemValues();                                                  // Make sure System values are all in valid range

  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
    systemStatusWriteNeeded = true;                                     // If so, store incremented number - watchdog must have done This
  }
  
  Time.setDSTOffset(sysStatus.dstOffset);                               // Set the value from FRAM if in limits
  DSTRULES() ? Time.beginDST() : Time.endDST();                         // Perform the DST calculation here
  Time.zone(sysStatus.timezone);                                        // Set the Time Zone for our device

  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);   // Load the offset string

  // Done with the System Stuff - now load the current counts
  fram.get(FRAM::currentStatusAddr, current);

  checkCurrentValues();                                                 // Make sure all is good

  currentHourlyPeriod = Time.hour();                                    // The local time hourly period for reporting purposes
  
  if (!digitalRead(userSwitch)) loadSystemDefaults();                   // Make sure the device wakes up and connects

  attachInterrupt(wakeUpPin, watchdogISR, RISING);                      // The watchdog timer will signal us and we have to respond

  // The device may have been reset without clearing the alerts.  Here is where we determine that at startup.  After this - tracked by interrupts
  if (digitalRead(input1))  {                                           // The input is not in an alert state
    strncpy(input1Str,"No Alert",sizeof(input1Str));
  }
  else {
    strncpy(input1Str,"75% Full",sizeof(input1Str));
    current.input1 = 2;                                               // Inidication is that we are now 75% full
  }
  attachInterrupt(input1, input1ISR, FALLING);                          // We need to watch for the input1 in both rising and falling states even if it was on at reset
  if (digitalRead(input2))  {                                           // The input is not in an alert state
    strncpy(input2Str,"No Alert",sizeof(input2Str));
    attachInterrupt(input2, input2ISR, FALLING);                          // On the input2 line, we just need to know when it goes LOW
  }
  else {
    strncpy(input2Str,"Low Oil",sizeof(input2Str));
    current.input2 = 1;                                               // Inidication is that we have a low oil alert
    current.interruptDisconnected = true;
  }

  takeMeasurements();

  if (state != ERROR_STATE) state = IDLE_STATE;                         // IDLE unless error from above code

  digitalWrite(blueLED,LOW);                                            // Signal done with startup
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if (state != oldState) publishStateTransition();
    if (input1Flag) state = DEBOUNCE_FULL_STATE;                        // Need to make sure lights are on for sure - harder since full can flash
    if (input2Flag) state = DEBOUNCE_OIL_STATE;                         // Need to make sure lights are on for sure
    if (Time.hour() != currentHourlyPeriod) state = REPORTING_STATE;    // We want to report on the hour
    break;

  case DEBOUNCE_FULL_STATE: {                                           // We got here because the input1Flag is high from a hardware interrupt - but is a noise or a signal? and is it flashing
    static unsigned long debounceTimeStamp = 0;

    if (state != oldState) {
      debounceTimeStamp = millis();                                     // Start the debounce timer
      publishStateTransition();
    }

    if ((millis() - debounceTimeStamp > 10000) && current.input1 == 0) {// Wait for 10 seconds unless we are looking for a flashing light if current.input1 = 0 then we are looking for solid on
      if (!digitalRead(input1)) {                                       // We are looking for the "full" light on and not yet at 75% which is on solid - hence the debounce
        takeMeasurements();
        state = REPORTING_STATE;
      }
      else {                                                            // False alarm input is lo longer low 
        input1Flag = false;
        state = IDLE_STATE;
      }
    }
    else if ((millis() - debounceTimeStamp > 200) && current.input1 ==2) {  // Less debounce since the light should be flashing
      debounceTimeStamp = millis();                                     // We need to reset as we will be coming back here often
      if (!digitalRead(input1)) {                                       // Remember we came to this state on a falling interrupt - is it still low?
        if (takeMeasurements()) state = REPORTING_STATE;                // The input1 is still low so off to takeMeasurements  - if returned true then the light was found to be flashing - if not more flashes needed
        else state = IDLE_STATE;                                        // Back to the IDLE_STATE until the next flash
      }
      else {
        input1Flag = false;                                             // False alarm must have been noise - not flashing
        state = IDLE_STATE;   
      }
    }
    } break;

  case DEBOUNCE_OIL_STATE: {                                            // We got here because the input2Flag is high from a hardware interrupt - but is a noise or a signal?
      static unsigned long debounceTimeStamp = 0;

      if (state != oldState) {
        debounceTimeStamp = millis();                                     // Start the debounce timer
        publishStateTransition();
      }

      if (millis() - debounceTimeStamp > 10000) {                         // Wait for 10 seconds unless we are looking for a flashing light
        if (!digitalRead(input2)) {                                       // Low oil light is on (assert low) and we have not yet set this alert
          takeMeasurements();                                             // This will take measurements and set the flags
          state = REPORTING_STATE;                                        // We need to report the change in alert level
        }
        else {
          input2Flag = false;                                             // False alarm the input is no longer low - remember there is no alert flag once low oil is on
          state = IDLE_STATE;
        }
      } 
    } break;

  case REPORTING_STATE: 
    if (state != oldState) publishStateTransition();
    if (!sysStatus.connectedStatus) connectToParticle();                // Only attempt to connect if not already New process to get connected
    if (Particle.connected()) {
      if (Time.hour() == 0) dailyCleanup();                             // Once a day, clean house
      takeMeasurements();                                               // Update Temp, Battery and Signal Strength values
      sendEvent();                                                      // Send data to Ubidots
      state = RESP_WAIT_STATE;                                          // Wait for Response
    }
    else {
      resetTimeStamp = millis();
      state = ERROR_STATE;
    }
    break;

  case RESP_WAIT_STATE:
    if (state != oldState) publishStateTransition();
    if (!dataInFlight)  state = IDLE_STATE;                             // Response Received
    else if (millis() - webhookTimeStamp > webhookWait) {               // If it takes too long - will need to reset
      resetTimeStamp = millis();
      publishQueue.publish("spark/device/session/end", "", PRIVATE);    // If the device times out on the Webhook response, it will ensure a new session is started on next connect
      state = ERROR_STATE;                                              // Response timed out
    }
    break;

  case ERROR_STATE:                                                     // To be enhanced - where we deal with errors
    if (state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait) {
      if (sysStatus.resetCount <= 3) {                                  // First try simple reset
        if (Particle.connected()) publishQueue.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else if (Time.now() - sysStatus.lastHookResponse > 7200L) { //It has been more than two hours since a sucessful hook response
        if (Particle.connected()) publishQueue.publish("State","Error State - Power Cycle", PRIVATE);  // Broadcast Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                  // Zero the ResetCount
        systemStatusWriteNeeded=true;
        digitalWrite(hardResetPin,HIGH);                              // This will cut all power to the Electron AND the carrier board
      }
      else {                                                          // If we have had 3 resets - time to do something more
        if (Particle.connected()) publishQueue.publish("State","Error State - Full Modem Reset", PRIVATE);            // Brodcase Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                     // Zero the ResetCount
        systemStatusWriteNeeded=true;
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }
  // Main loop housekeeping
  if (watchdogFlag) petWatchdog();

  if (current.interruptDisconnected) flashLED();                      // Signal that at least one interrupt is disconnected

  if (systemStatusWriteNeeded) {                                      // Batch write updates to FRAM
    fram.put(FRAM::systemStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }
  if (currentStatusWriteNeeded) {
    fram.put(FRAM::currentStatusAddr,current);
    currentStatusWriteNeeded = false;
  }
}

// Take measurements
bool takeMeasurements() {                                               // For clarity - 0 = no alert, 1 = alert (), 2 = 75% full and is flashing - Returns "True" if flashing
  const int cyclesRequired = 5;                                         // How many cycles will we need to count?
  static int cycleCount = 0;
  // input1 & input2 - are both inverted when the indicator light is off the input pin will go high
  // input1 - capacity light sensor - HIGH = 0 / no alert, FLASHING = 1 / 100% full, LOW = 2 / 75% full
  // input2 - oil light sensor - HIGH = 0 / no alert, LOW = 1 / alert
  // When either input gets to alert state 1, we disconnect the interrupt and stop monitoring
  // When either input is disconnected, we will start flashing the Electron Blue LED

  // Resolve input1 flags, interrupts and alerts
  if (input1Flag) {                                                     // input1 triggered an interrupt on FALLING
    if (!cycleCount) {                                                  // First interrupt and the Light is ON (input1 = LOW)
      current.input1 = 2;                                               // Inidication is that we are now 75% full
      strncpy(input1Str,"75% Full",sizeof(input1Str));
      cycleCount++;                                                     // Start counting cycles
    }
    else if (cycleCount < cyclesRequired) cycleCount++;                 // Increment the counter
    else if (cycleCount >= cyclesRequired) {                            // We have reached our cycle limit
      current.input1 = 1;                                               // We are at 100% full the light is flashing
      strncpy(input1Str,"100% Full",sizeof(input1Str));
      detachInterrupt(input1);                                          // Stop monitoring the input - wait for reset
      current.interruptDisconnected = true;                             // Set the interrupt disconnected flag which will trigger the blue flashing LED
    }
    input1Flag = false;
  }

  // Resolve input2 flags, interrupts and alerts
  if (input2Flag) {                                                     // input1 triggered an interrupt by going LOW this is an alert
    current.input2 = 1;                                                 // This is the alert value
    strncpy(input1Str,"Low Oil",sizeof(input1Str));
    detachInterrupt(input2);                                            // Stop checking until next reset
    current.interruptDisconnected = true;                               // Set the interrupt disconnected flag which will trigger the blue flashing LED
    input2Flag = false;
  }

  if (Cellular.ready()) getSignalStrength();                            // Test signal strength if the cellular modem is on and ready
  getTemperature();

  currentStatusWriteNeeded = true;
  
  if (current.input1 == 1) return true;                                 // We need to know if flashing was discovered - then we need to report
  else return false;  
}

void sendEvent() {                    
  char data[256];                                                       // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"input1\":%i, \"input2\":%i, \"temp\":%i, \"alerts\":%i, \"resets\":%i, \"timestamp\":%lu000}",current.input1, current.input2, current.temperature, current.alertCount, sysStatus.resetCount, Time.now());
  publishQueue.publish("HaulerCaller_Hook", data, PRIVATE);
  dataInFlight = true;                                                  // set the data inflight flag
  currentHourlyPeriod = Time.hour();                                    // Change the time period since we have reported for this one
  webhookTimeStamp = millis();
}

void UbidotsHandler(const char *event, const char *data)  // Looks at the response from Ubidots - Will reset Photon if no successful response
{
  // Response Template: "{{hourly.0.status_code}}"
  if (!data) {                                            // First check to see if there is any data
    publishQueue.publish("Ubidots Hook", "No Data",PRIVATE);
    return;
  }
  int responseCode = atoi(data);                          // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    if(sysStatus.verboseMode) publishQueue.publish("State","Response Received",PRIVATE);
    dataInFlight = false;                                 // Data has been received
    digitalWrite(blueLED, LOW);                           // Reset the LED and flags
  }
  else publishQueue.publish("Ubidots Hook", data,PRIVATE);             // Publish the response code
}

void getSignalStrength() {
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

int getTemperature() {
  int reading = analogRead(tmp36Pin);   //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;        // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                    // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));  //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  current.temperature = int((temperatureC * 9.0 / 5.0) + 32.0);  // now convert to Fahrenheit
  return current.temperature;
}

void petWatchdog() {
  digitalWrite(donePin, HIGH);                                        // Pet the watchdog
  digitalWrite(donePin, LOW);
  watchdogFlag = false;
}

// Here is were we will put the timer and other ISRs
void watchdogISR() {
  watchdogFlag = true;
}

void input1ISR() {
  input1Flag = true;
}

// Here is were we will put the timer and other ISRs
void input2ISR() {
  input2Flag = true;
}

bool connectToParticle() {
  Cellular.on();
  Particle.connect();
  // wait for *up to* 5 minutes
  for (int retry = 0; retry < 300 && !waitFor(Particle.connected,1000); retry++) {
    Particle.process();
  }
  if (Particle.connected()) {
    sysStatus.connectedStatus = true;
    systemStatusWriteNeeded = true;
    return 1;                               // Were able to connect successfully
  }
  else {
    return 0;                                                    // Failed to connect
  }
}

bool disconnectFromParticle() {
  Particle.disconnect();                                   // Disconnect from Particle in prep for sleep
  waitFor(notConnected,10000);
  Cellular.disconnect();                                   // Disconnect from the cellular network
  delay(3000);
  Cellular.off();                                           // Turn off the cellular modem
  return true;
}

bool notConnected() {
  return !Particle.connected();                             // This is a requirement to use waitFor
}


/* These are the particle functions that allow you to configure and run the device
 * They are intended to allow for customization and control during installations
 * and to allow for management.
*/

void flashLED() {
  static unsigned long lastTransition = 0;
  if (millis() - lastTransition > 1000) {
    digitalWrite(blueLED,!digitalRead(blueLED));
    lastTransition = millis();
  }
}

int hardResetNow(String command) {  // Will perform a hard reset on the Electron
  if (command == "1")
  {
    digitalWrite(hardResetPin,HIGH);          // This will cut all power to the Electron AND the carrir board
    return 1;                                 // Unfortunately, this will never be sent
  }
  else return 0;
}

int sendNow(String command) {// Function to force sending data in current hour
  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    publishQueue.publish("Mode","Set Verbose Mode",PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    publishQueue.publish("Mode","Cleared Verbose Mode",PRIVATE);
    return 1;
  }
  else return 0;
}

void dailyCleanup() {                                                 // Called from Reporting State ONLY - clean house at the end of the day
  publishQueue.publish("Daily Cleanup","Running", PRIVATE);               // Make sure this is being run
  sysStatus.verboseMode = false;
  Particle.syncTime();                                                // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                               // Wait for up to 30 seconds for the SyncTime to complete
  systemStatusWriteNeeded = true;
}

void loadSystemDefaults() {                                         // Default settings for the device - connected, not-low power and always on
  if (Particle.connected()) publishQueue.publish("Mode","Loading System Defaults", PRIVATE);
  sysStatus.structuresVersion = 1;
  sysStatus.verboseMode = false;
  sysStatus.timezone = -5;                                          // Default is East Coast Time
  sysStatus.dstOffset = 1;
  fram.put(FRAM::systemStatusAddr,sysStatus);                       // Write it now since this is a big deal and I don't want values over written
}

void checkSystemValues() {                                          // Checks to ensure that all system values are in reasonable range
  if (sysStatus.verboseMode < 0 || sysStatus.verboseMode > 1) sysStatus.verboseMode = false;
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.timezone < -12 || sysStatus.timezone > 12) sysStatus.timezone = -5;
  if (sysStatus.dstOffset < 0 || sysStatus.dstOffset > 2) sysStatus.dstOffset = 1;
  // None for lastHookResponse
  systemStatusWriteNeeded = true;
}

void checkCurrentValues() {                                             // Checks to ensure that all system values are in reasonable range
  current.input1 = 0;                                                   // Always reset at startup
  current.input2 = 0;
  current.interruptDisconnected = 0;                                    // Always false at startup
  if (current.alertCount < 0 || current.alertCount > 254) current.alertCount = 0;
  // None for lastHookResponse
  currentStatusWriteNeeded = true;
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  Particle.syncTime();                                                        // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                                       // Wait for up to 30 seconds for the SyncTime to complete
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;       // Make sure it falls in a valid range or send a "fail" result
  sysStatus.timezone = (float)tempTimeZoneOffset;
  Time.zone(sysStatus.timezone);
  systemStatusWriteNeeded = true;                                             // Need to store to FRAM back in the main loop
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);
  if (Particle.connected()) {
    snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
    publishQueue.publish("Time",data, PRIVATE);
    publishQueue.publish("Time",Time.timeStr(Time.now()), PRIVATE);
  }
  return 1;
}

void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if (sysStatus.verboseMode) publishQueue.publish("State Transition",stateTransitionString,PRIVATE);
}

void fullModemReset()   // Adapted form Rikkas7's https://github.com/rickkas7/electronsample
{
	Particle.disconnect(); 	                                         // Disconnect from the cloud
	unsigned long startTime = millis();  	                           // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=16\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}

int setDSTOffset(String command) {                                      // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2
  char * pEND;
  char data[256];
  time_t t = Time.now();
  int8_t tempDSTOffset = strtol(command,&pEND,10);                      // Looks for the first integer and interprets it
  if ((tempDSTOffset < 0) | (tempDSTOffset > 2)) return 0;              // Make sure it falls in a valid range or send a "fail" result
  Time.setDSTOffset((float)tempDSTOffset);                              // Set the DST Offset
  sysStatus.dstOffset = (float)tempDSTOffset;
  systemStatusWriteNeeded = true;
  snprintf(data, sizeof(data), "DST offset %2.1f",sysStatus.dstOffset);
  if (Time.isValid()) isDSTusa() ? Time.beginDST() : Time.endDST();     // Perform the DST calculation here
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);
  if (Particle.connected()) {
    publishQueue.publish("Time",data, PRIVATE);
    publishQueue.publish("Time",Time.timeStr(t), PRIVATE);
  }
  return 1;
}


bool isDSTusa() {
  // United States of America Summer Timer calculation (2am Local Time - 2nd Sunday in March/ 1st Sunday in November)
  // Adapted from @ScruffR's code posted here https://community.particle.io/t/daylight-savings-problem/38424/4
  // The code works in from months, days and hours in succession toward the two transitions
  int dayOfMonth = Time.day();
  int month = Time.month();
  int dayOfWeek = Time.weekday() - 1; // make Sunday 0 .. Saturday 6

  // By Month - inside or outside the DST window
  if (month >= 4 && month <= 10)
  { // April to October definetly DST
    return true;
  }
  else if (month < 3 || month > 11)
  { // before March or after October is definetly standard time
    return false;
  }

  boolean beforeFirstSunday = (dayOfMonth - dayOfWeek < 0);
  boolean secondSundayOrAfter = (dayOfMonth - dayOfWeek > 7);

  if (beforeFirstSunday && !secondSundayOrAfter) return (month == 11);
  else if (!beforeFirstSunday && !secondSundayOrAfter) return false;
  else if (!beforeFirstSunday && secondSundayOrAfter) return (month == 3);

  int secSinceMidnightLocal = Time.now() % 86400;
  boolean dayStartedAs = (month == 10); // DST in October, in March not
  // on switching Sunday we need to consider the time
  if (secSinceMidnightLocal >= 2*3600)
  { //  In the US, Daylight Time is based on local time
    return !dayStartedAs;
  }
  return dayStartedAs;
}