/*!
 * A NEO6M GPS Module Data Parser - Nano R4 version using Interrupts.
 * 
 * Use this script to get specific raw data from the NEO-6M GPS module.
 * 
 * Extracting the time, date, lat and long fields from the GPRMC NMEA sentence.
 * Extracting the number of satellites field from the GPGGA NMEA sentence.
 * 
 * https://www.rfwireless-world.com/terminology/gps-nmea-sentences
 * https://www.best-microcontroller-projects.com/arduino-strtok.html
 * https://forum.arduino.cc/t/strcpy-and-strcat/880153/4
 * https://forum.arduino.cc/t/how-to-maintain-original-data-using-strchr/889710
 *
 * Latitude (North/South):
 *  Positive (+): Northern Hemisphere (North of the Equator).
 *  Negative (-): Southern Hemisphere (South of the Equator).
 * Longitude (East/West):
 *  Positive (+): Eastern Hemisphere (East of London/Prime Meridian).
 *  Negative (-): Western Hemisphere (West of London/Prime Meridian).
 */

#include "FspTimer.h"                     // Builtin R4 library, always available.
#include <Adafruit_GFX.h>                 // Installed via the library manager.
#include <Adafruit_SSD1306.h>             // Installed via the library manager.

//#define DEBUG

#define GPSBAUD     9600                  // NEO-6M GPS module baud rate.
#define SCRNWDTH    128                   // OLED display pixel width.
#define SCRNHGHT    64                    // OLED display pixel height.
#define OLEDRST     -1                    // Reset pin # (or -1 if sharing Arduino reset pin).
#define SCRNADDR    0x3C                  // OLED display module I2C address.
#define GPSBUFR1    512                   // Raw GPS data buffer size (bytes).
#define TZOFFSET    0                     // Change this for the local timezone (e.g., +1 or -5).
#define NMEAMAXLEN  79                    // Max length of 82 minus the start $ and the ending <CR> + <LF>.
#define HBCOUNTER   250                   // Toggle the HB LED in ms.
#define GPSLOST     2000                  // GPS data "No Data" timeout in ms.

// Circular Data Buffer (volatile because it's used in Interrupt).
volatile char gpsDataBuffer[GPSBUFR1];
volatile unsigned int bufferHead = 0;     // Updated by ISR. Read by loop().
volatile unsigned int bufferTail = 0;     // Updated by loop(). Read by ISR.
volatile bool bufferOverflow = false;     // Sticky buffer error flag.

// Other global variables.
char sentence[NMEAMAXLEN + 1];
bool gpsValid = false;                    // The GPS module is sending valid data.
byte gpsDateD, gpsDateM, gpsDateY;        // Date.
byte gpsTimeH, gpsTimeM, gpsTimeS;        // Time.
double gpsLatD, gpsLongD;                 // Location.
byte gpsSats;                             // Number of satellites being tracked.
unsigned int checksumErrors = 0;          // Count of sentences that failed the XOR check.
unsigned long lastFixMillis = 0;          // Timestamp of the last valid NMEA sentence.
bool signalWarning = false;               // Triggered if no data for > 2 seconds.

// My hardware interrupts.
FspTimer myTimer;

// The OLED display module.
Adafruit_SSD1306 myDisplay(SCRNWDTH, SCRNHGHT, &Wire, OLEDRST);

// Interrupt Service Routine (ISR) - 1000Hz Timer.
// Note: Ensure your timer setup calls this function every 1ms.
void gpsDataReadISR(timer_callback_args_t *isr_args) {
  (void)isr_args;                         // This line prevents a potiential "unused parameter" compiler warning.
  static byte hbCounter = 0;
  static bool ledStatus = LOW;
  unsigned int newBufferHead;
  // Toggle the Heartbeat LED.
  hbCounter = (hbCounter + 1) % HBCOUNTER;
  if (hbCounter == 0) {
    ledStatus = !ledStatus;
    digitalWrite(LED_BUILTIN, ledStatus);
  }
  while (Serial1.available() > 0) {
    newBufferHead = (bufferHead + 1) % GPSBUFR1;
    // Test for a buffer overflow, or read the incoming data into the GPS data buffer!
    if (newBufferHead == bufferTail) {
      bufferOverflow = true;              // The GPS data buffer is full! 
      (void)Serial1.read();               // Drop incoming byte to prevent corruption of read data.
    }
    else {
      gpsDataBuffer[bufferHead] = Serial1.read(); // Read the new gps data into the buffer.
      bufferHead = newBufferHead;                 // Point to where the next byte will be stored.
    }
  }
}

void setup() {
    // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize the USB port for the Serial Monitor.
  Serial.begin(115200);
  while(!Serial);
  // Initialize the hardware UART (pins D0/RX and D1/TX) connected to the GPS module.
  Serial1.begin(GPSBAUD);
  // Set up the OLED display.
  Serial.print("Starting SSD1306 display...");
  while(!myDisplay.begin(SSD1306_SWITCHCAPVCC, SCRNADDR)); 
  Serial.println(" OK!");
  // Set a faster I2C bus speed: 400000 (Fast Mode).
  Wire.setClock(400000);
  // Use the OLED display...
  myDisplay.display();                    // Allow the Adafruit logo to display.
  delay(1000); // Pause for 1 second.
  myDisplay.clearDisplay();
  myDisplay.setTextSize(1);               // Normal 1:1 pixel scale.
  myDisplay.setTextColor(SSD1306_WHITE);  // Draw white text.
  myDisplay.cp437(true);                  // Use full 256 char 'Code Page 437' font.
  myDisplay.setCursor(0,0);               // Start at top-left corner.
  myDisplay.println("Nano R4 Ready!");    // Show something on the OLED display.
  myDisplay.display();
  delay(2000); // Pause for 2 seconds to allow the Adafruit logo to be seen.
  // Clear the buffer
  myDisplay.clearDisplay();
  // Move on with the job...
  Serial.println("NEMA Sentence Data Parser...");
  Serial.println(" - Flushing GPS data stream noise!");
  while(Serial1.available()) Serial1.read(); // Flush startup noise.
  // Start the 1ms Timer (Standard AGT timer on Renesas).
  myTimer.begin(TIMER_MODE_PERIODIC, AGT_TIMER, 0, 1000.0f, 50.0f, gpsDataReadISR, nullptr);
  myTimer.setup_overflow_irq();
  myTimer.open();
  myTimer.start();
  Serial.println(" - Hardware interrupt armed and running!");
}

void loop() {
  bool bufferOverflowed = false;
  int h, d, m, y;                         // Signed integers for safe rollover math.
  byte maxDays;                           // Used for month-end logic.
  bool isLeap;                            // Leap year flag.
  // Lookup table for days in each month.
  const byte daysInMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  // 1. Check for the buffer overflow warning.
  noInterrupts();
  if (bufferOverflow) {
    bufferOverflowed = true;
    bufferOverflow = false; 
  }
  interrupts();
  if (bufferOverflowed) {
    Serial.println("SYSTEM ERROR: GPS Buffer Overflow - loop() is too busy!");   
  }
  // 2. If we have extracted a NMEA sentence from the GPS data buffer...
  if (getSentence()) {
    parseSentence();                      // Parse it.
    // ROBUST TIMEZONE CALCULATION - Do signed maths to allow for negative results.
    h = gpsTimeH + TZOFFSET;
    d = gpsDateD;
    m = gpsDateM;
    y = gpsDateY;
    // Handle Hour Rollover
    if (h >= 24) {
      h -= 24;
      d++; // Move to next day
    } else if (h < 0) {
      h += 24;
      d--; // Move to previous day
    }
    // Determine if it's a leap year (GPS provides 2-digit year).
    isLeap = (y % 4 == 0);                // Valid for 2000-2099 only!
    // Handle Day/Month Rollover.
    //  - if d was incremented or decremented, we check against month boundaries.
    //  - if y was incremented or decremented, we check against the century boundaries.
    if (d > 0) {
      // Check for moving into the next month.
      maxDays = (m == 2 && isLeap) ? 29 : daysInMonth[m];
      if (d > maxDays) {
        d = 1;
        m++;
        if (m > 12) {
          m = 1;
          y++;
          if (y > 99) y = 0;              // Wrap year 2099 back to year 2000.
        }
      }
    } else {
      // Handle d < 1 (moved to previous month).
      m--;
      if (m < 1) {
        m = 12;
        y--;
        if (y < 0) y = 99;                // Wrap year 2000 back to year 2099.
      }
      d = (m == 2 && isLeap) ? 29 : daysInMonth[m];
    }
    // Update the globals with the adjusted local time (casting back to byte safely).
    gpsTimeH = (byte)h;
    gpsDateD = (byte)d;
    gpsDateM = (byte)m;
    gpsDateY = (byte)y;
    // Output the GPS data.
    showGPSDateTime();    // Send data to the serial monitor.
    displayGPSDateTime(); // Send data to the OLED display.
  }
  // 3. Checking to see if we have lost the GPS data stream.
  if (millis() - lastFixMillis > GPSLOST) {
    if (!signalWarning) {
      // Update the OLED display.
      myDisplay.clearDisplay();           // Clear the OLED display.
      myDisplay.drawRect(6, 20, 116, 24, SSD1306_WHITE);
      myDisplay.fillRect(6, 20, 116, 24, SSD1306_WHITE);
      myDisplay.setTextSize(2);
      myDisplay.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Black text on white.
      myDisplay.setCursor(22, 24);        // Centering the "NO DATA" message.
      myDisplay.println("NO DATA");
      myDisplay.display();
      // Send a message to the serial monitor.
      Serial.print("No GPS data!");
    }
    signalWarning = true;                 // No valid data for over 2 seconds.
  }
  else {
    signalWarning = false;
  }
}

// Extract and validate a NMEA sentence from the GPS data buffer.
bool getSentence() {
  static bool dataStart = false, dataStop = false;
  static byte dataCounter = 0;
  unsigned int bufferHeadCopy;
  bool dataValid = false;
  char character;
  byte ckSumCalc = 0, ckSumComp = 0;
  char *nmeaData, *nmeaCkSum;
  //if we have unread data in the GPS data buffer.
  noInterrupts();
  bufferHeadCopy = bufferHead; // Snapshot 'bufferHead' once to ensure a stable loop limit.
  interrupts();
  while (bufferTail != bufferHeadCopy) {
    character = gpsDataBuffer[bufferTail];
    bufferTail = (bufferTail + 1) % GPSBUFR1;
    if (character == '$') {
      dataStart = true;
      dataStop = false;
      dataCounter = 0;
    }
    else {
      // GPS buffer data cannot be accepted if we have not seen the start of a NMEA sentence.
      if (dataStart) {
        if (character == '\r' || character == '\n' || dataCounter == NMEAMAXLEN) {
          dataStop = true;
          sentence[dataCounter] = '\0'; // Ensure the read NMEA sentence is terminated.
          break; // <--- EXIT the while loop immediately!
        }
        else {
          // Add the GPS buffer data to the NMEA sentence.
          sentence[dataCounter] = character;
          dataCounter += 1;
        }
      }
    }
  }
  // If we have a complete NMEA sentence we can validate it.
  if (dataStart && dataStop) {
    nmeaData = strtok(sentence, "*");
    if (nmeaData != NULL) {
      nmeaCkSum = strtok(NULL, "*");
      // And another check just in case we have a lonley "*".
      if (nmeaCkSum != NULL && strlen(nmeaCkSum) >= 2) {
        // Calculate the NMEA data checksum.
        for (dataCounter = 0; nmeaData[dataCounter] != '\0'; dataCounter++) {
          ckSumCalc ^= byte(nmeaData[dataCounter]);
        }
        // Get the data checksum sent in the NMEA data.
        ckSumComp = (htoi(nmeaCkSum[0]) << 4) + htoi(nmeaCkSum[1]);
        // Compare the computed and sent NMEA data checksums.
        if (ckSumCalc == ckSumComp) {
          dataValid = true;
          lastFixMillis = millis();       // Reset the "Age" timer.
          #ifdef DEBUG
            Serial.print("Accepted: $");
            Serial.print(nmeaData);       // The data part.
            Serial.print("*");
            Serial.println(nmeaCkSum);    // The checksum part.
          #endif
        }
        else {
          checksumErrors++;               // We got data, but it was garbled!
          #ifdef DEBUG
            Serial.print("Rejected: $");
            Serial.print(nmeaData);       // The data part.
            Serial.print("*");
            Serial.print(nmeaCkSum);      // The checksum part.
            Serial.print("(Calc:");
            Serial.print(ckSumCalc, HEX);
            Serial.println(")");
          #endif
          Serial.print("CRC Error Count: ");
          Serial.println(checksumErrors);
        }
      }
    }
    // Clear the NMEA data sentence markers for next time.
    dataStart = false;
    dataStop = false;
    dataCounter = 0;
  }
  return dataValid;
}

// Parse a NMEA sentence looking for a specific NMEA sentence.
void parseSentence() {
  byte fieldCounter = 0;
  char *nmeaPart, *pointer;
  char nmeaType1[] = "GPRMC"; // The first NMEA sentence type to look for.
  char nmeaType2[] = "GPGGA"; // The second NMEA sentence type to look for.
  // Isolate the NMEA sentence type.
  nmeaPart = strchr(sentence, ',');
  if (nmeaPart != NULL) {
    *nmeaPart = '\0';
    // --- CASE 1: GPRMC (Time, Date, Coordinates) ---
    if (strcmp(sentence, nmeaType1) == 0) {
    // if (strComp(sentence, nmeaType1)) {  // Replaced with standard library function strcmp().
      #ifdef DEBUG
        Serial.print("RMC record: ");
        Serial.println(nmeaPart + 1);
      #endif
      pointer = nmeaPart + 1;
      do {
        fieldCounter += 1;
        nmeaPart = strchr(pointer, ',');
        if (nmeaPart != NULL) {
          *nmeaPart = '\0';
        }
        // Check that the field isn't empty (,,) before parsing.
        if (strlen(pointer) > 0) {
          if (fieldCounter == 1) {  // Time (UTC).
            gpsTimeH = dtoi(pointer[0]) * 10 + dtoi(pointer[1]);
            gpsTimeM = dtoi(pointer[2]) * 10 + dtoi(pointer[3]);
            gpsTimeS = dtoi(pointer[4]) * 10 + dtoi(pointer[5]);
          }
          if (fieldCounter == 2) {  // Valid flag.
            gpsValid = (strcmp(pointer, "A") == 0);
            // gpsValid = strComp(pointer, "A");  // Replaced with standard library function strcmp().
          }
          if (fieldCounter == 3) {  // Latitude.
            gpsLatD = dtoi(pointer[0]) * 10 + dtoi(pointer[1]) + (atof(pointer + 2) / 60);
          }
          if (fieldCounter == 4) {  // N/S.
            if(strcmp(pointer, "S") == 0) {
            // if(strComp(pointer, "S")) {  // Replaced with standard library function strcmp().
              gpsLatD = -gpsLatD;
            }
          }
          if (fieldCounter == 5) {  // Longitude.
            gpsLongD = dtoi(pointer[0]) * 100 + dtoi(pointer[1]) * 10 + dtoi(pointer[2]) + (atof(pointer + 3) / 60);
          }
          if (fieldCounter == 6) {  // E/W.
            if(strcmp(pointer, "W") == 0) {
            // if(strComp(pointer, "W")) {  // Replaced with standard library functionstrcmp().
              gpsLongD = -gpsLongD;
            }
          }
          if (fieldCounter == 9) {  // Date.
            gpsDateD = dtoi(pointer[0]) * 10 + dtoi(pointer[1]);
            gpsDateM = dtoi(pointer[2]) * 10 + dtoi(pointer[3]);
            gpsDateY = dtoi(pointer[4]) * 10 + dtoi(pointer[5]);
          }
          if (nmeaPart != NULL) pointer = nmeaPart + 1; // Move pointer EVERY valid loop.
        }
      } while (nmeaPart != NULL);
    }
    // --- CASE 2: GPGGA (Satellites) ---
    else if (strcmp(sentence, nmeaType2) == 0) {
    // else if (strComp(sentence, nmeaType2)) { // Replaced with standard library function strcmp().
      #ifdef DEBUG
        Serial.print("GGA record: ");
        Serial.println(nmeaPart + 1);
      #endif
      pointer = nmeaPart + 1;
      do {
        fieldCounter += 1;
        nmeaPart = strchr(pointer, ',');
        if (nmeaPart != NULL) {
          *nmeaPart = '\0';
        }
        // If we have the interesting field and isn't empty (,,) before parsing.
        if (fieldCounter == 7 && strlen(pointer) > 0) {
          gpsSats = gpsValid ? atoi(pointer) : 0; // Safely converts "7", "08", or "12" to an integer, or return 0.
          //gpsSats = dtoi(pointer[0]) * 10 + dtoi(pointer[1]);
        }
        if (nmeaPart != NULL) pointer = nmeaPart + 1; // Move pointer EVERY valid loop.
      } while (nmeaPart != NULL);
    }
  }
  #ifdef DEBUG
    else {
      Serial.println("Delimiter not found!");
    }
  #endif
}

// Print the GPS date, time and location to the serial monitor.
void showGPSDateTime(void) {
  static bool noSignal = false;
  static byte pastTimeS = 255;
  static byte pastSats = 0;
  if (gpsValid) {
    noSignal = false;
    if (gpsTimeS != pastTimeS) {
      Serial.print("Sats: ");
      Serial.print(gpsSats);
      if (pastSats != gpsSats) {
        Serial.print("  (Was: ");
        Serial.print(pastSats);
        Serial.println(")");
      }
      else {
        Serial.println();
      }
      Serial.print("Date: ");
      serPrint2d(gpsDateD);
      Serial.print("/");
      serPrint2d(gpsDateM);
      Serial.print("/20");
      serPrint2d(gpsDateY);
      Serial.print("  Time: ");
      serPrint2d(gpsTimeH);
      Serial.print(":");
      serPrint2d(gpsTimeM);
      Serial.print(":");
      serPrint2dln(gpsTimeS);
      Serial.print(" Lat: ");
      Serial.print(gpsLatD, 5);
      Serial.print("°  Long: ");
      Serial.print(gpsLongD, 5);
      Serial.println("°");
      pastTimeS = gpsTimeS;
      pastSats = gpsSats;
    }
  }
  else {
    if (!noSignal) {
      noSignal = true;
      Serial.println("No signal!");
      pastTimeS = 255;
      pastSats = 0;
    }
  }
}

// Print the GPS date, time and location to the OLED display.
void displayGPSDateTime(void) {
  static bool noSignal = false;
  static byte pastTimeS = 255;
  // Some locals to stress test the display spacing.
  if (gpsValid) {
    noSignal = false;
    if (gpsTimeS != pastTimeS) {              // Ensure we only update the display every second.
      myDisplay.clearDisplay();               // Clear the OLED display.
      // --- HEARTBEAT TRIANGLES ---
      // Toggle based on even/odd seconds.
      if (gpsTimeS % 2 == 0) {
        // Draw triangle in top-left corner.
        myDisplay.fillTriangle(0, 0, 5, 0, 0, 5, SSD1306_WHITE); 
      } else {
        // Draw triangle in top-right corner.
        myDisplay.fillTriangle(127, 0, 122, 0, 127, 5, SSD1306_WHITE);
      }      
      myDisplay.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // White text, black background.
      // --- DRAW CENTERED TITLE ---
      myDisplay.setTextSize(2);               // Double size text (where each character is 12 pixels wide including the gap).
      myDisplay.setCursor(16, 0);             // Top of screen, (128 - (titleLen * 12)) / 2 = 16
      myDisplay.println("GPS DATA");
      // --- TOP OF DATA LINE --- 
      myDisplay.drawFastHLine(0, 18, 128, SSD1306_WHITE); // Starts at X=0, Y=18, width=128 pixels.
      // --- DRAW DATA ---
      myDisplay.setTextSize(1);              // Reset to normal size text.
      myDisplay.setCursor(0, 22);            // Start data below title and line (title is ~16px high).
      myDisplay.print("Date: ");
      oledPrint2d(gpsDateD);
      myDisplay.print("/");
      oledPrint2d(gpsDateM);
      myDisplay.print("/20");
      oledPrint2dln(gpsDateY);
      myDisplay.print("Time:  ");
      oledPrint2d(gpsTimeH);
      myDisplay.print(":");
      oledPrint2d(gpsTimeM);
      myDisplay.print(":");
      oledPrint2dln(gpsTimeS);
      myDisplay.println();                    // Blank line spacer.
      myDisplay.print(" Lat: ");
      myDisplay.print(gpsLatD, 5);
      myDisplay.write(248);                   // Degree symbol for CP437.
      myDisplay.println();
      myDisplay.print("Long: ");
      myDisplay.print(gpsLongD, 5);
      myDisplay.write(248);                   // Degree symbol for CP437.
      myDisplay.println();
      // --- SATELLITE LABEL AND COUNT IN THE MARGIN ---
      myDisplay.setTextSize(1);
      myDisplay.setCursor(102, 30);           // Positioned slightly above the gap.
      myDisplay.print("SATS");                // Calculated position: Right side (~110px), centered on the blank line (~38px).
      myDisplay.setTextSize(2);
      // 0 - 9 satellites.
      if (gpsSats >= 0 && gpsSats < 10) {
        myDisplay.setCursor(108, 40); 
        myDisplay.print(gpsSats);
      // 10 - 99 satellites.
      } else if (gpsSats >= 10 && gpsSats < 100) {
        // If sats >= 10, we need to move the count to the left to fit two digits.
        myDisplay.setCursor(102, 40);
        myDisplay.print(gpsSats);
      // Not ever expecting to have 100 or more sats, but just in case...
      } else {
        myDisplay.setCursor(108, 40); 
        myDisplay.print("+");
      }
      // --- BOTTOM OF DATA LINE ---
      // We draw this at Y=63 (the last row of pixels).
      myDisplay.drawFastHLine(0, 63, 128, SSD1306_WHITE);
      // Display everything on the OLED display now.
      myDisplay.display();
      pastTimeS = gpsTimeS;
    }
  }
  else {
    if (!noSignal) {
      noSignal = true;                        // Ensure we only display this once.
      myDisplay.clearDisplay();               // Clear the OLED display.
      // --- HEARTBEAT TRIANGLES ---
      // Toggle based on even/odd seconds.
      if (gpsTimeS % 2 == 0) {
        // Draw triangle in top-left corner.
        myDisplay.fillTriangle(0, 0, 5, 0, 0, 5, SSD1306_WHITE); 
      } else {
        // Draw triangle in top-right corner.
        myDisplay.fillTriangle(127, 0, 122, 0, 127, 5, SSD1306_WHITE);
      }      
      // Draw Box: x, y, width, height, color.
      myDisplay.drawRect(6, 20, 116, 24, SSD1306_WHITE); // msgX - 4, msgY - 4, 108 + 8, 16 + 8
      myDisplay.fillRect(6, 20, 116, 24, SSD1306_WHITE);
      myDisplay.setTextSize(2);               // Make "No Signal" big too!
      myDisplay.setCursor(10, 24);            // Somewhere in the middle of the screen, msgX: (128 - 108) / 2, msgY: 24
      myDisplay.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Black text, white background.
      myDisplay.println("NO SIGNAL");         // 9 chars * 12px = 108px wide.
      // Display everything on the OLED display now.
      myDisplay.display();
    }
  }
}

// Simple function to compare 2 strings (char arays).
// Replaced with standard library function strcmp().
// bool strComp(char* lookIn, char* lookFor) {
//   byte pointer;
//   byte numChars = strlen(lookFor);
//   // If the strings are not the same length they cannot be equal.
//   if (strlen(lookIn) != numChars) {
//     return false;
//   }
//   else {
//     // Compare the strings character by character.
//     for (pointer = 0; pointer < numChars; pointer++) {
//       if (lookIn[pointer] != lookFor[pointer]) {
//         return false;  // STOP immediately if a character doesn't match.
//       }
//     }
//   }
//   return true;
// }

// Convert a hex digit to an integer.
byte htoi(char digit) {
  digit = toupper(digit);               // Only upper case considered.
  if(digit >= '0' && digit <= '9')      // Digit is between 0 and 9.
    return(digit - '0');                // Subtract ASCII '0' to get the hex value.
  else if(digit >= 'A' && digit <= 'F') // Digit is between A and F.
    return(digit - 'A' + 10);           // Subtract ASCII 'A' to get the hex value.
  else
    return(0);                          // Must not be a hex character, so return zero.
}

// Convert a decimal digit to an integer.
byte dtoi(char digit) {
  if(digit >= '0' && digit <= '9')      // Digit is between 0 and 9.
    return(digit - '0');                // Subtract ASCII '0' to get the decimal value.
  else
    return(0);                          // Must not be a decimal character, so return zero.
}

// Print a 2 digit integer with a leading zero and a newline.
void serPrint2dln(byte number) {
  serPrint2d(number);
  Serial.println();
 }

// Print a 2 digit integer with a leading zero.
void serPrint2d(byte number) {
  if (number < 10) {
    Serial.print("0");
  }
  Serial.print(number);
}

// Display a 2 digit integer with a leading zero and a newline.
void oledPrint2dln(byte number) {
  oledPrint2d(number);
  myDisplay.println();
 }

// Display a 2 digit integer with a leading zero.
void oledPrint2d(byte number) {
  if (number < 10) {
    myDisplay.print("0");
  }
  myDisplay.print(number);
}

// EOF
