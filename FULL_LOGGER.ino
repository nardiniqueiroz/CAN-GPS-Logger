/************************************************************************************************* 
  ESP32 DTCs AND FRAMES LOGGER CODE
   
  Nardini Claizoni Queiroz @ APR30, 2020
  
  The code has the purpose to generate a log with CAN frames and DTCs atached to the geoposition at the 
  moment that they happened. 
  The SD card shield is conected to the standard HSPI pins and the MCP2515 is conected to VSPI
  standard pins.
  
 
  
  
***************************************************************************************************/
//Libraries for SPI comunication and CAN controler
#include <SPI.h>
#include "mcp_can.h"
//Libraries to comunicate with GPS receiver.
#include <TinyGPS++.h>
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

TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data

const int SPI_CS_PIN = 15;//9
const int SPI_CS_PIN1 = 4;
const int SPI_CS_PIN2 = 21;
const int SD_CS = 5;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin
MCP_CAN CAN1(SPI_CS_PIN1);
MCP_CAN CAN2(SPI_CS_PIN2);

#define CAN_ID_PID          0x18DA10F1 //Diagnostic Services ID for ECM
#define DTC_SIZE 0x03
#define DTC_SID 0x19
#define DTC_MSB_DID 0x02
#define DTC_LSB_DID 0x08
#define CF_SIZE 0x30
#define CF_MSB 0x00
#define CF_LSB 0x00

int received=1;
int FrameNumber=0;
int CntDTC=0;
int CntCharDTC=0;
int CntFrame=0;

unsigned long t1;
unsigned long t2;

unsigned char ECM_DTC_PAST[60];
unsigned char ECM_DTC[60];
unsigned char PID_INPUT;
unsigned char getPid    = 0;

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

void set_mask_filt()
{/*
    /*
      set mask, set both the mask to 0x3ff
     */
 
    CAN.init_Mask(0, 0, 0x18DAF110);
    CAN.init_Mask(1, 1, 0x18DAF110);
    
    
     /*set filter, we can receive id from 0x04 ~ 0x09
     */
    
    CAN.init_Filt(0, 0,0x18DAF110 );               
    CAN.init_Filt(1, 1,0x18DAF110 ); 
    CAN.init_Filt(2, 1, 0x18DAF110); 
    CAN.init_Filt(3, 1, 0x18DAF110); 
    CAN.init_Filt(4, 1, 0x18DAF110); 
    CAN.init_Filt(5, 1, 0x18DAF110); 
}

void set_mask_filt1()
{
  
}

void set_mask_filt2(){
  
}

void sendSID(unsigned char sid,unsigned char did_msb,unsigned char did_lsb) {

    unsigned char tmp[8] = {0x03,sid, did_msb, did_lsb, 0, 0, 0, 0};
    
    CAN.sendMsgBuf(CAN_ID_PID, 1, 8, tmp); // INT8U id, INT8U ext, INT8U len, data_buf


}
void sendCF() {

    unsigned char tmp[8] = {0x30,0x00, 0x00, 0, 0, 0, 0, 0};
    
    CAN.sendMsgBuf(CAN_ID_PID, 1, 8, tmp); // INT8U id, INT8U ext, INT8U len, data_buf
     
     for(int i = 0; i<8; i++){            // print the data

            //SERIAL.print("0x");
            SERIAL.print(tmp[i], HEX);
            SERIAL.print("\t");
        }
        SERIAL.println();

}

void taskCanRecv();
void writeFile();
void appendFile();


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

  CAN.setSPI(hspi);//informs the can object which SPI comunication it will use
  CAN1.setSPI(hspi);
  CAN2.setSPI(hspi);
    
   while (CAN_OK != CAN.begin(CAN_500KBPS,MCP_8MHz)){    // init can bus : baudrate = 500k
    
        SERIAL.println("CAN BUS Shield init fail");
        SERIAL.println(" Init CAN BUS Shield again");
        delay(100);
    }
    SERIAL.println("CAN BUS Shield init ok!");
    set_mask_filt();

  while (CAN_OK != CAN1.begin(CAN_500KBPS,MCP_8MHz)){    // init can bus : baudrate = 500k
      
          SERIAL.println("CAN BUS 1 Shield init fail");
          SERIAL.println(" Init CAN BUS 1 Shield again");
          delay(100);
      }
      SERIAL.println("CAN BUS Shield init ok!");
      set_mask_filt1();
  while (CAN_OK != CAN2.begin(CAN_250KBPS,MCP_8MHz)){    // init can bus : baudrate = 500k
    
        SERIAL.println("CAN BUS 2 Shield init fail");
        SERIAL.println(" Init CAN BUS 2 Shield again");
        delay(100);
    }
    SERIAL.println("CAN BUS Shield init ok!");    
    set_mask_filt2();
    // Initialize SD card
  SD.begin(SD_CS);  
  if(!SD.begin(SD_CS)) {
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

  // If the dtc_log.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/dtc_log.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/dtc_log.txt", "Reading ID, Date, Hour, Temperature \r\n");
  }
  else {
    Serial.println("File already exists"); 
    Serial.println("Overwriting file...");
    writeFile(SD, "/dtc_log.txt", "Reading ID, DATA, Latitude, Longitude \r\n"); 
  }
  file.close();

    file = SD.open("/can_log.txt");
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

 
}

void loop() {   
   
    t2=millis ();
   
    if(((t2-t1)>500)||(received==1)){//Asks for the DTCs every 0,5s  
        t1=t2;
        sendSID(DTC_SID,DTC_MSB_DID,DTC_LSB_DID);   
        received=0;     
    }
    
    GPS_Cordinates();    
    taskCanRecv();
    
}

int getFrameNumber(int bsize){

  int nframes=0;
  
  if(((bsize-6)%7)==0){
    nframes=(bsize-6)/7;
  }else{
    nframes=1+((bsize-6)/7);
  }
 return nframes;
 }

void HandleDTC(long id, char len, unsigned char buf[]){
    int bytesize=0;
    if((buf[0]==0x10)&&(buf[2]==0x59)){       //process the begin of multiframe response to DTC request
      bytesize=buf[1];
      FrameNumber=getFrameNumber(bytesize);
      
      for(int i=0;i<3;i++){
        ECM_DTC[i]=buf[i+5];
        //CntCharDTC=i;
      }
      CntCharDTC=3;
      sendCF();
      
   }if(buf[1]==0x59){        //process single frame response to DTC request
     bytesize=buf[0];
     for(int i=0;i<4;i++){  
      ECM_DTC[i]=buf[i+4];
   }  
    received=1;
    }
    
   if(FrameNumber>CntFrame){//process the consecutive frames of the response to the DTC request
    Serial.println("FrameNumber:"+String(FrameNumber)+" CntFrame:"+String(CntFrame));
    if((buf[0]>0x20)&&(buf[0]<=0x2F)){
      
        for(int i=0;i<(len-1);i++){
         //Serial.println();
         // Serial.println("CntDTC:"+String(CntDTC)+" CntCharDTC:"+String(CntCharDTC)+" i:"+ String(i));
          ECM_DTC[CntCharDTC]=buf[i+1];
          
          CntCharDTC++;

      }
    CntFrame++;  
    }
   }if(FrameNumber==CntFrame)
   {
   Serial.println();
   received=1;
   CntFrame=0;
   CntCharDTC=0;
   CntDTC=0;
   FrameNumber=0;
   CmpDTC();
   } 
   for(int i=0;i<60;++i){//Debuging DTC vector print
    Serial.print(ECM_DTC[i],HEX);
    Serial.print("\t");
   } 
}
   
void taskCanRecv() {
    
    unsigned char len = 0;
    unsigned char buf[8];
    String frame;
    String frame1;
    String frame2; 
     if(CAN_MSGAVAIL == CAN2.checkReceive()) {                   // check if get data
    
        CAN2.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        
        frame2=String(CAN2.getCanId(),HEX); //logs ID in frame string
  
        for(int i = 0; i<len; i++){    // print the data
            
            frame2=frame2+"\t"+String(buf[i], HEX);
             
        }
        
        appendFile(SD, "/can_log.txt", frame2.c_str());
    }
  if(CAN_MSGAVAIL == CAN1.checkReceive()) {                   // check if get data
    
        CAN1.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        
        frame1=String(CAN2.getCanId(),HEX); //logs ID in frame string

        for(int i = 0; i<len; i++){    // print the data
            
            frame1=frame1+"\t"+String(buf[i], HEX);

        }

        appendFile(SD, "/can_log.txt", frame1.c_str());
    }
    
    if(CAN_MSGAVAIL == CAN.checkReceive()) {                   // check if get data
    
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        
        frame=String(CAN.getCanId(),HEX); //logs ID in frame string


        for(int i = 0; i<len; i++){    // print the data
            
            frame=frame+"\t"+String(buf[i], HEX);

        }

        appendFile(SD, "/can_log.txt", frame.c_str());
        

        HandleDTC(CAN.getCanId(), len, buf);
    }
   
}

void CmpDTC(){//function to compare the new DTCs to the old ones

  String DTCGPS;
  int CharCmpCnt;
  int DTCFound;
  for(int k=0;k<59;k+=4){
    for(int j=0;j<59;j+=4){
      for(int i=0;i<3;i++){
       if(ECM_DTC[k+i]==ECM_DTC_PAST[j+k]){
          CharCmpCnt++;
        }
      }
       if((CharCmpCnt==3)){//&&NullCnt<4
         //Serial.println("Line "+String(k)+" from EMC_DTC2 is equals to line "+String(j)+" from ECM_DTC_PAST");
         DTCFound=1;
         CharCmpCnt=0;
       }else{
         CharCmpCnt=0;
          
        } 
    }
   if(!DTCFound){
    Serial.println("We got a new DTC:");
    for(int i=0;i<4;i++){
      DTCGPS= String(ECM_DTC[k+i],HEX)+"\t";
      
      } 
      DTCGPS=DTCGPS+"\t"+"Latitude:"+String(gps.location.lat(),6)+"\t"+"Longitude:"+String(gps.location.lng(),6)+"\r\n";
      appendFile(SD, "/dtc_log.txt", DTCGPS.c_str());
      Serial.println();
      DTCFound=0;
    }else{
      DTCFound=0;
      }
   } 
}

void GPS_Cordinates(){
  if(SERIAL2.available()){
          gps.encode(SERIAL2.read());
         
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
