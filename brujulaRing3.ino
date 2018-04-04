// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

//This code is intended for use with Arduino Leonardo and other ATmega32U4-based Arduinos

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <FastLED.h>
#define NUM_LEDS 24
#define DATA_PIN 3
// Define the array of leds
CRGB leds[NUM_LEDS];

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 8
//   Connect the GPS RX (receive) pin to Digital 7
// If using hardware serial:
//   Connect the GPS TX (transmit) pin to Arduino RX1 (Digital 0)
//   Connect the GPS RX (receive) pin to matching TX1 (Digital 1)

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

// If using hardware serial, comment
// out the above two lines and enable these two lines instead:

//Adafruit_GPS GPS(&Serial1);
//HardwareSerial mySerial = Serial1;


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

//---------------COMPASS
//#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L.h>

// Create a compass

HMC5883L compass;
//---------------declaraciones mias
float lat1;
float long1;
float lat2;
float long2;
float angR;

float deg2rad = PI / 180; // pi/180
float rad2deg = 180 / PI; //
float pi = PI;


void setup()
{

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  delay(5000);
  Serial.println("Adrranca la brujula!");

  //------- compass
    // Set measurement range
    compass.setRange(HMC5883L_RANGE_1_3GA);

    // Set measurement mode
    compass.setMeasurementMode(HMC5883L_CONTINOUS);

    // Set data rate
    compass.setDataRate(HMC5883L_DATARATE_30HZ);

    // Set number of samples averaged
    compass.setSamples(HMC5883L_SAMPLES_8);

    // Set calibration offset. See HMC5883L_calibration.ino
    compass.setOffset(-3, -138);

  //---------
  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

}

uint32_t timer = millis();



void loop()                     // run over and over again
{
  lat2 = 43.327757;
  long2 = -3.006068;
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    Serial.println("-----------.....-----...---...-- BUH!!");

    Serial.println("Conectando GPS");

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false

      return;  // we can fail to parse a sentence in which case we should just wait for another

  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 500) {
    timer = millis(); // reset the timer

  }


  if (GPS.fix) {
    Serial.print("......FIX: ");
    Serial.println(GPS.fix);
    delay(200);
    if (((int)GPS.satellites) > 0)
    {
      Serial.println("> > > > > >  M a s q c e r o < < < < < < <");
    }


    lat1 = GPS.latitudeDegrees;
    long1 = GPS.longitudeDegrees;
    Serial.println();
    Serial.print("Location1: ");
    Serial.print(lat1, 7); //Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.println(long1, 7);// Serial.println(GPS.lon);
    Serial.print("Location2: ");
    Serial.print(lat2, 7); //Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.println(long2, 7);// Serial.println(GPS.lon);


    //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    //Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

    Serial.println("----------------------");


    float Alat = deg2rad * (lat2 - lat1);
    float Along = deg2rad * (long2 - long1);

    float y = sin(Along) * cos(deg2rad * lat2);
    float x = cos(deg2rad * lat1) * sin(deg2rad * lat2) - sin(deg2rad * lat1) * cos(deg2rad * lat2) * cos(Along);

    Serial.print("x: ");
    Serial.println(x, 14);
    Serial.print("y: ");
    Serial.println(y, 14);
    angR = atan2(y, x);

    //float angR=atan(y/x);
    float angD = angR * rad2deg;
    if (angD < 0)
    {
      angD = 360 + angD;
    }
    Serial.print("rumbo entre puntos GSP: ");
    Serial.println(angD);

    //-----compass
    Serial.println("Initialize HMC5883L");
    while (!compass.begin())
    {
      Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
      delay(500);
    }



    Vector norm = compass.readNormalize();

    // Calculate heading
    float heading = atan2(norm.YAxis, norm.XAxis);

    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
    float declinationAngle = -(0 + (37 / 60.0)) / (180 / M_PI); //esto lo hw sacau de la page ya
    heading += declinationAngle;

    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0)
    {
      heading += 2 * PI;
    }

    if (heading > 2 * PI)
    {
      heading -= 2 * PI;
    }

    // Convert to degrees
    float headingDegrees = heading * 180 / M_PI;



    // Output

    Serial.print(" Degress Compass = ");
    Serial.print(headingDegrees);
    Serial.println();

    Serial.print("RUMBO:");

    if (headingDegrees > -90 && headingDegrees < 180)
    {
      Serial.println(angD - headingDegrees);
      led((angD - headingDegrees));
    }

    else if (headingDegrees < -90 && headingDegrees >= -180)
    {
      Serial.println(-(angD + ( 180 + (headingDegrees))));
      led(-(angD + ( 180 + (headingDegrees))));
    }
  }
}

void led(float angle)
{
  // NOS QUEDAMOS AQUI! en esta funcion el plan es leer el angulo respecto a direccion y encender el led pertinente
  if (angle >= -7.5 && angle < 7.5)
  {
    Serial.println("LED 0 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 0) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[0].setRGB(30, 30, 30);
    FastLED.show();
  }

  else if (angle >= 7.5 && angle < 22.5)
  {
    Serial.println("LED 1 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 1) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[1].setRGB(30, 30, 30);
    FastLED.show();

  }
  else if (angle >= 22.5 && angle < 37.5)
  {
    Serial.println("LED 2 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 2) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[2].setRGB(30, 30, 30);
    FastLED.show();

  }
  else if (angle >= 37.5 && angle < 52.5)
  {
    Serial.println("LED 3 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 3) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[3].setRGB(30, 30, 30);
    FastLED.show();

  }
  else if (angle >= 52.5 && angle < 67.5)
  {
    Serial.println("LED 4 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 4) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[4].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= 67.5 && angle < 82.5)
  {
    Serial.println("LED 5 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 5) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[5].setRGB(30, 30, 30);
    FastLED.show();

  }
  else if (angle >= 82.5 && angle < 97.5)
  {
    Serial.println("LED 6 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 6) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[6].setRGB(30, 30, 30);
    FastLED.show();

  }
  else if (angle >= 97.5 && angle < 112.5) //-165 a -135
  {
    Serial.println("LED 7 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 7) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[7].setRGB(30, 30, 30);
    FastLED.show();

  }
  else if (angle >= 112.5 && angle < 127.5)
  {

    Serial.println("LED 8 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 8) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[8].setRGB(30, 30, 30);
    FastLED.show();

  }
  else if (angle >= 127.5 && angle < 142.5)
  {
    Serial.println("LED 9 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 9) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[9].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= 142.5 && angle < 157.5)
  {
    Serial.println("LED 10 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 10) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[10].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= 157.5 && angle < 172.5)
  {
    Serial.println("LED 11 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 11) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[11].setRGB(30, 30, 30);
    FastLED.show();

  }
  else if ((angle >= 172.5 && angle < 187.5) || (angle > -187.5 && angle < -172.5))
  {
    Serial.println("LED 12 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 12) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[12].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= -172.5 && angle < -157.5)
  {
    Serial.println("LED 13 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 13) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[13].setRGB(30, 30, 30);
    FastLED.show();

  }
  else if (angle >= -157.5 && angle < -142.5)
  {
    Serial.println("LED 14 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 14) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[14].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= -142.5 && angle < -127.5)
  {
    Serial.println("LED 15 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 15) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[15].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= -127.5 && angle < -112.5)
  {
    Serial.println("LED 16 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 16) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[16].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= -112.5 && angle < -97.5)
  {
    Serial.println("LED 17 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 17) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[17].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= -97.5 && angle < -82.5)
  {
    Serial.println("LED 18 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 18) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[18].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= -82.5 && angle < -67.5)
  {
    Serial.println("LED 19 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 19) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[19].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= -67.5 && angle < -52.5)
  {
    Serial.println("LED 20 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 20) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[20].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= -52.5 && angle < -37.5)
  {
    Serial.println("LED 21 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 21) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[21].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= -37.5 && angle < -22.5)
  {
    Serial.println("LED 22 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 22) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[22].setRGB(30, 30, 30);
    FastLED.show();
  }
  else if (angle >= -22.5 && angle < -7.5)
  {
    Serial.println("LED 23 ON");
    for (int i = 0; i < 24; i++)
    {
      if (i == 23) {   // create jump in values
        continue;
      }
      leds[i].setRGB(0, 0, 0);
      FastLED.show();
    }
    leds[23].setRGB(30, 30, 30);
    FastLED.show();
  }

}

