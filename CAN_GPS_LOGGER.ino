/************************************************************************************************* 
  ESP32 CAN and GPS logger
  NARDINI CLAIZONI QUEIROZ @ JAN27, 2020  
  
  The code has the purpose to generate a log with CAN frames atached to the geoposition at the 
  moment that they were broadcast into CAN network. 
  The SD card shield is conected to the standard HSPI pins and the MCP2515 is conected to VSPI
  standard pins.
  
  Its also possible to ask for information 
  through PID codes as shown bellow.
  
  Query
  send id: 0x7df
      dta: 0x02, 0x01, PID_CODE, 0, 0, 0, 0, 0
  Response
  From id: 0x7E9 or 0x7EA or 0x7EB
      dta: len, 0x41, PID_CODE, byte0, byte1(option), byte2(option), byte3(option), byte4(option)
      
  https://en.wikipedia.org/wiki/OBD-II_PIDs
  
  Input a PID, then you will get reponse from vehicle, the input should be end with '\n'

  Modified by: NARDINI CLAIZONI QUEIROZ
***************************************************************************************************/
//Libraries for SPI comunication and CAN controler
#include <SPI.h>
#include "mcp_can.h"
//Libraries to comunicate with GPS receiver.
#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
// Libraries for SD card
#include "FS.h"
#include "SD.h"
/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
  #define SERIAL2 Serial2
#endif

//SoftwareSerial serial_connection(27,26); //RX=pin 10(cs), TX=pin 11(mosi)
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 15;//9
const int SD_CS = 5;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

#define PID_ENGIN_PRM           0x0C
#define PID_VEHICLE_SPEED       0x0D
#define PID_COOLANT_TEMP        0x05
#define PID_THROTTLE_POSITION   0x11
#define PID_OIL_TEMP            0x5c
#define PID_RUN_TIME            0x1f

#define CAN_ID_PID          0x7DF

unsigned char PID_INPUT;
unsigned char getPid    = 0;

String latitude;
String longitude;
String message;
static const int spiClk = 1000000; // 1 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

void set_mask_filt() {
    /*
     * set mask, set both the mask to 0x3ff
     */
    CAN.init_Mask(0, 0, 0x7FC);
    CAN.init_Mask(1, 0, 0x7FC);

    /*
     * set filter, we can receive id from 0x04 ~ 0x09
     */
    CAN.init_Filt(0, 0, 0x7E8);                 
    CAN.init_Filt(1, 0, 0x7E8);

    CAN.init_Filt(2, 0, 0x7E8);
    CAN.init_Filt(3, 0, 0x7E8);
    CAN.init_Filt(4, 0, 0x7E8); 
    CAN.init_Filt(5, 0, 0x7E8);
}

void sendPid(unsigned char __pid) {

    unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};
    SERIAL.print("SEND PID: 0x");
    SERIAL.println(__pid, HEX);
    CAN.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);
}



void setup() {
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  
  //clock miso mosi ss

  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin();
  //alternatively route through GPIO pins of your choice
  //hspi->begin(0, 2, 4, 33); //SCLK, MISO, MOSI, SS
  
  //initialise hspi with default pins
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi->begin(); 
  //alternatively route through GPIO pins
  //hspi->begin(25, 26, 27, 32); //SCLK, MISO, MOSI, SS

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(5, OUTPUT); //VSPI SS
  pinMode(15, OUTPUT); //HSPI SS
    
  SERIAL.begin(115200);
  SERIAL2.begin(9600);//This opens up communications to the GPS
  Serial.println("GPS Start");
    
    // Initialize SD card
  SD.begin(SD_CS);  
  while(!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    return;    // init failed
  }

  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/data.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/can_log.txt", "Reading ID, Date, Hour, Temperature \r\n");
  }
  else {
    Serial.println("File already exists"); 
    Serial.println("Overwriting file...");
    writeFile(SD, "/can_log.txt", "Reading ID, DATA, Latitude, Longitude \r\n"); 
  }
  file.close();

   CAN.setSPI(hspi);//informs the can object which SPI comunication it will use
    
   while (CAN_OK != CAN.begin(CAN_500KBPS)){    // init can bus : baudrate = 500k
    
        SERIAL.println("CAN BUS Shield init fail");
        SERIAL.println(" Init CAN BUS Shield again");
        delay(100);
    }
    SERIAL.println("CAN BUS Shield init ok!");
    set_mask_filt();
}


void loop() {
    
    GetMessage();
    taskDbg();

    if(getPid){          // GET A PID
    
        getPid = 0;
        sendPid(PID_INPUT);
        PID_INPUT = 0;
    }
}

void convertInfo(long id, char len, unsigned char buf[]){

    int pid = buf[2]; 
    byte A = buf[3];
    byte B = buf[4];
    byte C = buf[5];
    byte D = buf[6];

    int rpm, oil, coolant, pos, runtime;

    if(buf[1] == 0x41) {
        switch(pid){
            case PID_ENGIN_PRM: 
                        
                        Serial.print("RPM: ");
                        rpm = (256*A + B)/4;
                        Serial.println(rpm);
                        break;

            case PID_VEHICLE_SPEED: 

                        Serial.print("Speed ");
                        Serial.print(A);                    
                        Serial.println(" km/h");
                        break;

            case PID_COOLANT_TEMP:
                        
                        Serial.print("Coolant Temp"); 
                        coolant = A - 40;
                        Serial.print(coolant);
                        Serial.println(" °C");
                        break;

            case PID_THROTTLE_POSITION: 
                        
                        Serial.print("Throttle: ");
                        pos = (100*A)/255;
                        Serial.print(pos);
                        Serial.println(" %");
                        break;

            case PID_OIL_TEMP: 
                        
                        Serial.print("Oil ");
                        oil = A - 40;
                        Serial.print(oil);
                        Serial.println(" °C");
                        break;

            case PID_RUN_TIME:

                        Serial.print("Run Time since Engine start ");
                        runtime = 256*A + B;
                        Serial.print(runtime);
                        Serial.println(" seconds");
                        break;
        }
    }

}

String GetMessage() {
    
    unsigned char len = 0;
    unsigned char buf[8];
    String frame;
    if(CAN_MSGAVAIL == CAN.checkReceive()) {                   // check if get data
    
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        /*SERIAL.println("\r\n------------------------------------------------------------------");
        SERIAL.print("Get Data From id: 0x");
        SERIAL.println(CAN.getCanId(), HEX);
        */
        frame=String(CAN.getCanId(),HEX); //Salva o ID no frame
        
        for(int i = 0; i<len; i++){    // print the data

            frame=frame+"\t"+String(buf[i], HEX);              
            
            /*SERIAL.print("0x");
            SERIAL.print(buf[i], HEX);
            SERIAL.print("\t");*/
        }
       
         frame=frame+"\t"+"Latitude:"+String(gps.location.lat(),6)+"\t"+"Longitude:"+String(gps.location.lng(),6)+"\r\n";;
                //return frame;
        //SERIAL.println(gps.location.lat());
        appendFile(SD, "/can_log.txt", frame.c_str());
        SERIAL.println(frame);
        
        //SERIAL.println();

        //String canID = decToHex(CAN.getcanId(), 2);

        //convertInfo(CAN.getCanId(), len, buf);
    }
}

void GPS_Cordinates(){
  while(SERIAL2.available()){
          gps.encode(SERIAL2.read());
          latitude=gps.location.lat();
          longitude=gps.location.lng();
  }
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void taskDbg() {

    while(SERIAL.available()) {
    
        char c = SERIAL.read();
        
        if(c>='0' && c<='9')
        {
            PID_INPUT *= 0x10;
            PID_INPUT += c-'0';
            
        }
        else if(c>='A' && c<='F')
        {
            PID_INPUT *= 0x10;
            PID_INPUT += 10+c-'A';
        }
        else if(c>='a' && c<='f')
        {
            PID_INPUT *= 0x10;
            PID_INPUT += 10+c-'a';
        }
        else if(c == '\n')      // END
        {
            getPid = 1;
        }
    }
}
