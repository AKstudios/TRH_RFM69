#include <RFM69.h>  //  https://github.com/LowPowerLab/RFM69
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <Adafruit_SHT31.h> //https://github.com/adafruit/Adafruit_SHT31

// define node parameters
char node[] = "201";
#define NODEID       201 // same sa above - must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define GATEWAYID     1
#define NETWORKID     101
#define FREQUENCY     RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY    "Tt-Mh=SQ#dn#JY3_" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define LED             9


// define objects
RFM69 radio;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// global variables
char dataPacket[50];

// ISR ****************************************************************
ISR(WDT_vect)  // Interrupt service routine for WatchDog Timer
{
  wdt_disable();  // disable watchdog
}


// setup ****************************************************************
void setup()
{
  pinMode(10, OUTPUT);
  Serial.begin(115200); // open serial at 115200 bps
  Serial.println("Setup");
  Wire.begin();  // initialize I2C using Wire.h library
  sht31.begin(0x44);
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  
  pinMode(LED, OUTPUT);  // pin 9 controls LED
  delay(1);
}


// sleep ****************************************************************
void sleep()
{
  Serial.flush(); // empty the send buffer, before continue with; going to sleep
  
  radio.sleep();
  
  cli();          // stop interrupts
  MCUSR = 0;
  WDTCSR  = (1<<WDCE | 1<<WDE);     // watchdog change enable
  WDTCSR  = 1<<WDIE | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0); // set  prescaler to 4 second
  sei();  // enable global interrupts

  byte _ADCSRA = ADCSRA;  // save ADC state
  ADCSRA = 0; // turn off ADC

  asm("wdr");
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down       
  //PORTD |= (1<<PORTD4); //Activate pullup on pin 4
  //PCICR |= (1<<PCIE2);
  //PCMSK2 |= (1<<PCINT20);
  cli();
  sleep_enable();  
  //sleep_bod_disable();  // turn off BOD
  sei();       
  sleep_cpu();    // goodnight!
    
  sleep_disable();   
  sei();  

  ADCSRA = _ADCSRA; // restore ADC state (enable ADC)
  delay(1);
}

// loop ****************************************************************
void loop()
{
  sleep();
  
  for(int i=0; i<5; i++)
  {
    analogRead(A0);  // first few readings from ADC may not be accurate, so they're cleared out here
    analogRead(A7);
  }
  delay(1);

  // get temperature
  float adc = averageADC(A0);
  float R = resistance(adc, 10000); // Replace 10,000 ohm with the actual resistance of the resistor measured using a multimeter (e.g. 9880 ohm)
  float temperature = steinhart(R);  // get temperature from thermistor using the custom Steinhart-hart equation by US sensors

  // get humidity
  float temp = sht31.readTemperature();
  float rh = sht31.readHumidity();

  // Battery voltage
  for(int i=0; i<5; i++)    // clear out first few readings from ADC after wake
    analogRead(A7);
  float b = (analogRead(A7)/1023.0 * 3.3) * 2.0;
  
  char _t[5];
  char _h[5];
  char _b[4];
  
  // convert all flaoting point and integer variables into character arrays
  //dtostrf(nodeID, 2, 0, _i);
  dtostrf(temperature, 3, 2, _t);  // this function converts float into char array. 3 is minimum width, 2 is decimal precision
  dtostrf(rh, 3, 2, _h);  // this function converts float into char array. 3 is minimum width, 2 is decimal precision
  dtostrf(b, 3, 2, _b);  // this function converts float into char array. 3 is minimum width, 2 is decimal precision
  delay(1);
  
  dataPacket[0] = 0;  // first value of dataPacket should be a 0
  
  // create datapacket by combining all character arrays into a large character array
  strcat(dataPacket, "i:");
  strcat(dataPacket, node);
  strcat(dataPacket, ",b:");
  strcat(dataPacket, _b);
  strcat(dataPacket, ",t:");
  strcat(dataPacket, _t);
  strcat(dataPacket, ",h:");
  strcat(dataPacket, _h);
  delay(1);
  
  Serial.println(dataPacket);
  Serial.println(strlen(dataPacket));
  delay(10);

  // send datapacket
  radio.sendWithRetry(GATEWAYID, dataPacket, strlen(dataPacket), 5, 100);  // send data, retry 5 times with delay of 100ms between each retry
  dataPacket[0] = (char)0; // clearing first byte of char array clears the array

  // blink LED to indicate wireless data is sent (not necessarily received)
  digitalWrite(LED, HIGH);
  delay(10);
  digitalWrite(LED, LOW);
}



// Averaging ADC values to counter noise in readings  *********************************************
float averageADC(int pin)
{
  float sum=0.0;
  for(int i=0;i<5;i++)
  {
     sum = sum + analogRead(pin);
  }
  float average = sum/5.0;
  return average;
}

// Get resistance ****************************************************************
float resistance(float adc, int true_R)
{
  float R = true_R/(1023.0/adc-1.0);
  return R;
}

// Get temperature from Steinhart equation (US sensors thermistor, 10K, B = 3892) *****************************************
float steinhart(float R)
{
  float A = 0.00113929600457259;
  float B = 0.000231949467390149;
  float C = 0.000000105992476218967;
  float D = -0.0000000000667898975192618;
  float E = log(R);
  
  float T = 1/(A + (B*E) + (C*(E*E*E)) + (D*(E*E*E*E*E)));
  delay(50);
  return T-273.15;
}
