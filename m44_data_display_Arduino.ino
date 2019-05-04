
/********************************************************************************************************
   Motronic 4.4 Volvospeed 608 bin data output display and boost gauge by Dangerous Dave.
   This program runs a serial read pick up data from the stream of bytes from the motronic ecu.
   The bytes are stored in an array and split into the various engine data values.
   One display shows 16 values selected from approx 46 total.
   This code is written for use on an ESP8266 processor running an ILI9341 320x240 LCD and will need
   adapting if intended for use with Arduino

*******************************************************************************************************/

#include <SPI.h>
#include <TFT_eSPI.h>

#define TFT_GREY 0x5AEB // Dark grey 16 bit colour

//setup data read variables
const byte numBytes = 46;
byte receivedBytes[numBytes];
byte numReceived = 0;
byte ndx = 0;
byte discase = 3; //setup for display switching
byte pin1 = D1;
static int16_t oldPos, pos;
int tmot;    //engine temp
int pVmess;
int pTmot;
int startMarker = 0x5AA5;
int endMarker = 0xAA55; //not needed any more
float boost;//boost
float tl;     //load
float zwout;  //actual ignition angle
byte mlh;    //air mass high byte
byte mll;    //air mass low byte
byte tih;    //injection time high byte
byte til;    //injection time low byte
float wideband;
int xtra;   //ltft idle
float xfra;   //ltft part load
float xfr;    //stft
float xwdkbl; //throttle angle
byte hfkorr; //fuel correction high byte
byte lfkorr; //fuel correction low byte
float vmess;  //vehicle speed
float tvmbeg; //tcv duty cycle
float maf; //mass air flow
float ti; //injection time
float fkorr; //fuel correction
float idc; //injector duty cycle (calculated)
float vac; //intake vacuum
float kpa; //kpa value from sensor
float barboost; //boost bargraph
float peak1; //peak boost 1
float pTl;
float pXwdkbl;
float pMaf;
float pTi;
float pTvmbeg;
float pBoost;
float pIdc;
float mpg;
float afr;                      //AFR value
unsigned int nmot; //rpm
unsigned long previousMillis1;  //setup timer 1
unsigned long previousMillis2;  //setup timer 2
unsigned long previousMillis3;  //setup timer 3
unsigned long previousTextTimer;//setup text timer
unsigned long currentTextTimer; //setup text timer


//booleans
boolean dataSetup = false;
boolean gaugeSetup = false;
boolean maximumSetup = false;
boolean page1Setup = false;
boolean debugSetup = false;
boolean newData = false;
boolean recvInProgress = false;

TFT_eSPI tft = TFT_eSPI(); //initiate ILI9341 library

// #######################################################################################################
// # MAIN SETUP                                                                                          #
// #######################################################################################################

void setup() {

  Serial.begin(125000);
  pinMode(D1, INPUT_PULLUP);

  //setup TFT display items
  tft.init();
  tft.setRotation(3); //set display orientation
  tft.fillScreen(TFT_BLACK);
}

// #######################################################################################################
// # MAIN LOOP                                                                                           #
// #######################################################################################################

void loop() {


  readData();   // run serial read routine

  calcData();   // run calculate data routine

  switches();  // run rotary encoder routine

  switch (discase) {
    case 0:
      displayData(); // switch to data display
      break;

    case 1:
      displayMaximums(); // switch to max data display
      break;

    case 2:
      displayBoostGauge();  // switch to boost display
      break;

  }

} // MAIN LOOP END

// #######################################################################################################
// # Serial read and split data to string                                                                #
// #######################################################################################################

void readData() {
  byte rb;
  byte msb;
  byte lsb;
  int smTest;

  //testing new serial read which should properly check for the start marker being 16bit
  if (Serial.available() > 0) {
    rb = Serial.read();
    if (recvInProgress == true) {
      receivedBytes[ndx] = rb;
      ndx++;

      if (ndx == 46) {          // if ndx reaches the max stream of 45 values
        recvInProgress = false;
        ndx = 0;
        newData = true;

      }
    }
    else if (rb == 0x5A) {
      lsb = Serial.read();
      smTest = rb * 256 + lsb;
      if (smTest == 0x5AA5) {
        recvInProgress = true;
      }
    }
  }
}

// #######################################################################################################
// # Calculate Data                                                                                      #
// #######################################################################################################

void calcData() {

  //calculate data from string
  if (newData = true) {
    nmot = receivedBytes[0] * 30;
    tl = receivedBytes[1] * 0.048;
    vmess = receivedBytes[31] * 0.621371;
    xwdkbl = receivedBytes[28] * 0.4166667;
    zwout = (receivedBytes[3] * -0.75) + 78;
    mlh = receivedBytes[19];
    mll = receivedBytes[20];
    maf = ((mlh << 8) + mll) * 0.125;    //calculated air mass from 2 bytes
    tih = receivedBytes[21];
    til = receivedBytes[22];
    ti = (((tih << 8) + til) * 2.0) / 1000.0;   //calculated injection time from 2 bytes
    xtra = (receivedBytes[24] * 4) - 512;
    xfra = ((receivedBytes[25] * 0.0078125) - 1) * 100;
    xfr = ((receivedBytes[26] * 0.0078125) - 1) * 100;
    hfkorr = receivedBytes[29];
    lfkorr = receivedBytes[30];
    fkorr = ((hfkorr << 8) + lfkorr) / 32768.0;
    tmot = receivedBytes[32] - 80;
    tvmbeg = (receivedBytes[38] / 256.0) * 100.0;
    wideband = ((receivedBytes[42] - 10) * 0.039215686) + 10;
    idc = (nmot * ti) / 1200.0;
    newData = false;
  }

  kpa = (((analogRead(0) / 1023.0) + 0.04) / 0.004); //calculate kpa value for mpx4250ap sensor
  boost = (kpa - 100) * 0.145; // Calculate boost/vac levels in psi
  vac = boost * -2.036025; // Used 'minus' 2.036025 to display as positive
  barboost = boost * (15.4);

  // peak values update
  // Column 1
  if (tl > pTl) {
    pTl = tl;
  }
  if (vmess > pVmess) {
    pVmess = vmess;
  }
  if (xwdkbl > pXwdkbl) {
    pXwdkbl = xwdkbl;
  }
  if (maf > pMaf) {
    pMaf = maf;
  }
  if (ti > pTi) {
    pTi = ti;
  }

  //Column 2
  if (tmot > pTmot) {
    pTmot = tmot;
  }
  if (tvmbeg > pTvmbeg) {
    pTvmbeg = tvmbeg;
  }
  if (boost > pBoost) {
    pBoost = boost;
  }
  if (idc > pIdc) {
    pIdc = idc;
  }
}

// #######################################################################################################
// # Switch Stuff                                                                                        #
// #######################################################################################################

void switches() {

  pin1 = digitalRead(D1);
  if (pin1 == 1) {
    discase = (discase + 1) % 3;
  }

}
// #######################################################################################################
// multiple data display and setup
// #######################################################################################################

void displayData() {
  tft.setTextSize(1);

  if (!dataSetup) {
    displayDataSetup();
    // reset setup flags
    dataSetup = true;
    gaugeSetup = false;
    page1Setup = false;
    
    }

  unsigned long currentMillis3 = millis();
  if ((currentMillis3 - previousMillis3) >= 200UL) { //set time delay to 200ms to prevent to slow down refresh rate
    previousMillis3 = currentMillis3;

    //Column 1 - print all white text first
    tft.setTextPadding(78);  //set text padding to remove characters once DP moves.  Padding starts from 1st character.
    tft.setTextColor(TFT_WHITE, TFT_BLACK); //Reset colour back to white
    tft.drawFloat(tl, 1, 83, 30, 4);
    tft.drawFloat(wideband, 2, 83, 60, 4);
    tft.drawFloat(xwdkbl, 1, 83, 90, 4);
    tft.drawFloat(zwout, 2, 83, 120, 4);
    tft.drawFloat(maf, 1, 83, 150, 4);
    tft.drawFloat(ti, 1, 83, 180, 4);

    //RPM Colours
    if (nmot >= 0 && nmot < 5500) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString("RPM:", 20, 0, 4);
      tft.drawNumber(nmot, 83, 0, 4);
    }
    if (nmot >= 5500 && nmot < 6000) {
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.drawString("RPM:", 20, 0, 4);
      tft.drawNumber(nmot, 83, 0, 4);
    }
    if (nmot >= 6000) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("RPM:", 20, 0, 4);
      tft.drawNumber(nmot, 83, 0, 4);
    }


    //Column 2 - Print all white text first///////////////////////
    tft.setTextPadding(85);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);   //Reset colour back to white
    tft.drawFloat(xtra, 1, 235, 0, 4);
    tft.drawFloat(xfra, 1, 235, 30, 4);
    tft.drawFloat(xfr, 1, 235, 60, 4);
    tft.drawFloat(fkorr, 2, 235, 90, 4);
    //tft.drawFloat(tvmbeg, 1, 235, 120, 4);
    //tft.drawString("Temp:", 9, 210, 4);
    //tft.drawNumber(tmot, 83, 210, 4);
    //Engine Temp Colours
    if (tmot < 87) {
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
      tft.drawString("Temp:", 161, 120, 4);
      tft.drawNumber(tmot, 235, 120, 4);
    }
    if (tmot >= 87 && tmot < 103) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString("Temp:", 161, 120, 4);
      tft.drawNumber(tmot, 235, 120, 4);
    }
    if (tmot >= 103 && tmot <= 105) {
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.drawString("Temp:", 161, 120, 4);
      tft.drawNumber(tmot, 235, 120, 4);
    }
    if (tmot > 105) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("Temp:", 161, 120, 4);
      tft.drawNumber(tmot, 235, 120, 4);
    }

    // Injector Duty Cycle Colours/////////////////////////////////////////////////////////////////////////
    if (idc < 80) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString("InjDC:", 163, 180, 4);
      tft.drawFloat(idc, 1, 235, 180, 4);
    }
    if (idc >= 80 && idc < 85) {
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.drawString("InjDC:", 163, 180, 4);
      tft.drawFloat(idc, 1, 235, 180, 4);
    }
    if (idc >= 85) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("InjDC:", 163, 180, 4);
      tft.drawFloat(idc, 1, 235, 180, 4);
    }

    // Boost and Vacuum colours ///////////////////////////////////////////////////////////////////////////
    if (boost < 0) {
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
      tft.drawString("     Vac:", 159, 150, 4);
      tft.drawFloat(vac, 1, 235, 150, 4);
    }
    if (boost >= 0 && boost <= 10) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString("Boost:", 160, 150, 4);
      tft.drawFloat(boost, 1, 235, 150, 4);
    }
    if (boost > 10 && boost <= 15) {
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.drawString("Boost:", 160, 150, 4);
      tft.drawFloat(boost, 1, 235, 150, 4);
    }
    if (boost > 15) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("Boost:", 160, 150, 4);
      tft.drawFloat(boost, 1, 235, 150, 4);
    }
    tft.setTextPadding(100);

  }
}

void displayDataSetup() {

  //setup text
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(40);

  //Column 1
  tft.drawString("Load:", 17, 30, 4);
  tft.drawString("AFR:", 22, 60, 4);
  tft.drawString("TPS:", 28, 90, 4);
  tft.drawString("Ign:", 38, 120, 4);
  tft.drawString("MAF:", 22, 150, 4);
  tft.drawString("TI:", 54, 180, 4);
  
  //Column 2
  tft.drawString("Ltft-I:", 177, 0, 4);
  tft.drawString("LtftPL:", 162, 30, 4);
  tft.drawString("Stft:", 188, 60, 4);
  tft.drawString("Fkorr:", 168, 90, 4);
  //tft.drawString("TCV:", 179, 120, 4);
  //tft.drawString("AFR:", 178, 210, 4);
  //tft.drawString("kpa:", 178, 210, 4);
  //tft.drawString("EFlap:", 160, 210, 4);
  
}

// ##################################################################################################
// Maximum Data Display and Setup
// ##################################################################################################

void displayMaximums() {  // check static display items are set up
  if (!maximumSetup) {
    maximumDisplaySetup();
    // reset setup flags
    maximumSetup = true;
    dataSetup = false;
    page1Setup = false;
    gaugeSetup = false;
    
    //Column 1
    tft.drawFloat(pTl, 1, 83, 60, 4);
    tft.drawNumber(pVmess, 83, 90, 4);
    tft.drawFloat(pXwdkbl, 1, 83, 120, 4);
    tft.drawFloat(pMaf, 1, 83, 150, 4);
    tft.drawFloat(pTi, 1, 83, 180, 4);

    //Column 2
    tft.drawNumber(pTmot, 235, 60, 4);
    tft.drawFloat(pTvmbeg, 1, 235, 90, 4);
    tft.drawFloat(pBoost, 1, 235, 120, 4);
    tft.drawFloat(pIdc, 1, 235, 150, 4);
    //tft.drawString("AFR:", 235, 180, 4);
  }
}

void maximumDisplaySetup() {

  //setup text
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextPadding(40);
  tft.drawString("Maximum Values", 65, 0, 4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  //Column 1
  tft.drawString("Load:", 17, 60, 4);
  tft.drawString("Speed:", 0, 90, 4);
  tft.drawString("TPS:", 28, 120, 4);
  tft.drawString("MAF:", 22, 150, 4);
  tft.drawString("TI:", 54, 180, 4);

  //Column 2
  tft.drawString("Temp:", 161, 60, 4);
  tft.drawString("TCV:", 179, 90, 4);
  tft.drawString("Boost:", 160, 120, 4);
  tft.drawString("InjDC:", 163, 150, 4);
  tft.drawString("AFR:", 178, 180, 4);

}

// ################################################################################################
// Boost gauge display and setup
// ###############################################################################################

void displayBoostGauge() {

  if (!gaugeSetup) {
    displayGaugeSetup();
    // reset setup flags
    gaugeSetup = true;
    page1Setup = false;
    dataSetup = false;
    maximumSetup = false;
    
  }
  unsigned long currentMillis2 = millis();

  if ((currentMillis2 - previousMillis2) >= 100UL) {
    previousMillis2 = currentMillis2;
    //display data------------------------------------------------
    unsigned long currentMillis1 = millis();
    tft.setTextPadding(78);
    if (boost > 0) {
      tft.fillRect(6, 165, barboost, 60, TFT_GREEN); //fill bargraph with green bar
      tft.fillRect(6 + barboost, 165, 308 - barboost, 60, TFT_BLACK); //fill end of bar with black to refresh

      // Different colour text for higher pressure values
      if (boost > 0) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawFloat(boost, 1, 144, 5, 4);
        tft.drawString("psi", 233, 5, 4);
      }

      if (boost > 15) {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.drawFloat(boost, 1, 144, 5, 4);
        tft.drawString("psi", 233, 5, 4);
      }

      if (boost > 20) {
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawFloat(boost, 1, 144, 5, 4);
        tft.drawString("psi", 233, 5, 4);
      }
    }
    //display vacuum and inHg
    else {
      tft.fillRect(6, 165, 308, 60, TFT_BLACK); //clear bargraph once below 0 boost
      vac = boost * -2.036025; // Used 'minus' 2.036025 to display as positive

      tft.setTextColor(TFT_CYAN, TFT_BLACK);
      tft.drawFloat(vac, 1, 144, 5, 4);
      tft.drawString("inHg", 233, 5, 4); //display units
    }

    //Display Peak1 Value
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawFloat(peak1, 1, 144, 44, 4); //display temporary peak value

    //Display Peak Boost Value
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawFloat(pBoost, 1, 144, 83, 4); //display overall peak value

    // Store highest boost pressure for limited time
    if (boost > peak1) { // If current boost is higher than peak1, store new value
      peak1 = boost;  // Store new peak1 value in peak memory
      previousMillis1 = currentMillis1; //reset timer when new peak value stored
    }

    //Peak1 value reset after x milliseconds, uncomment to activate
    if (peak1 > 0) { //Change value for higher reset limit
      if ((unsigned long)(currentMillis1 - previousMillis1) >= 5000UL) { //timer for x milliseconds
        peak1 = 0; //resets the peak1 value to 0
        previousMillis1 = currentMillis1; //reset timer
      }
    }
  }
}

void displayGaugeSetup() {

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  //tft.setTextSize(1);
  //tft.drawString("Boost -", 0, 10, 4);
  //tft.drawString("Peak -", 17, 65, 4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Boost:", 0, 5, 4);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawString("Peak1:", 0, 44, 4);
  tft.drawString("psi", 233, 44, 4); //display units
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("Peak2:", 0, 83, 4);
  tft.drawString("psi", 233, 83, 4); //display units
  tft.drawRect(0, 159, 320, 72, TFT_WHITE); //Draw bargraph outline
  //Draw bargraph figures
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("0", 3, 125, 2);
  tft.drawString("5", 80, 125, 2);
  tft.drawString("10", 152, 125, 2);
  tft.drawString("15", 229, 125, 2);
  tft.drawString("20", 303, 125, 2);

  //Draw lines for accurate bargraph
  tft.drawLine(6, 144, 6, 159, TFT_WHITE);
  tft.drawLine(83, 144, 83, 159, TFT_WHITE);
  tft.drawLine(160, 144, 160, 159, TFT_WHITE);
  tft.drawLine(237, 144, 237, 159, TFT_WHITE);
  tft.drawLine(314, 144, 314, 159, TFT_WHITE);
  //Draw smaller lines for intermediates
  tft.drawLine(22, 150, 22, 159, TFT_WHITE);
  tft.drawLine(37, 150, 37, 159, TFT_WHITE);
  tft.drawLine(52, 150, 52, 159, TFT_WHITE);
  tft.drawLine(67, 150, 67, 159, TFT_WHITE);

  tft.drawLine(99, 150, 99, 159, TFT_WHITE);
  tft.drawLine(114, 150, 114, 159, TFT_WHITE);
  tft.drawLine(129, 150, 129, 159, TFT_WHITE);
  tft.drawLine(144, 150, 144, 159, TFT_WHITE);

  tft.drawLine(176, 150, 176, 159, TFT_WHITE);
  tft.drawLine(191, 150, 191, 159, TFT_WHITE);
  tft.drawLine(206, 150, 206, 159, TFT_WHITE);
  tft.drawLine(221, 150, 221, 159, TFT_WHITE);

  tft.drawLine(253, 150, 253, 159, TFT_WHITE);
  tft.drawLine(268, 150, 268, 159, TFT_WHITE);
  tft.drawLine(283, 150, 283, 159, TFT_WHITE);
  tft.drawLine(298, 150, 298, 159, TFT_WHITE);
}
