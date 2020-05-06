/*
  The program was designed to run on Heltec ESP32 LoRa V2
  1 - listen to the serial port, capture input from device (PC/Nvidia Jetson Nano/TX2/Raspberry Pi)
  2 - construct the data packet 
  3 - send it out via LoRa
  4 - (optional) resend a missing packet (maybe store a few packet in the RAM)
  Created 04/13/2020
  by Yang Zhong
*/
#include "heltec.h"
#include "WiFi.h"
#define BAND 915E6            // set band according to the region, e.g. 868E6(EU),915E6(US),433E6(China)
#define SERIAL_BAUD 115200    // set the serial baud to match the baud at device (Jetson) 

bool isInit = false;          // the flag shows if the OLED screen has been initalized
int defaultDelay = 1000;      // a default time for delay the string display on screen to disappear
int currentLine = 0;          // the current line indicator 
const int maxLines = 6;       // the maximum number of lines the OLED can display - default is 6
const int maxMsgLength = 25;  // the maximum string chars the OLED can display with a single line 
const int lineHeight = 10;    // the height for each line

const String START_RECEIVING_SERIAL = "##STRT#";
const String END_RECEIVING_SERIAL = "##END##";
const int LORA_DELAY = 500;           // the default interval for LoRa sending signal (Note: in millisecond, and will be execute twice, 1500 means the interval between lora sending is 1.5 second *2 times = 3 seconds ) 
const int DEFAULT_PCK_SIZE = 255;     // the default reading size from the packet
int idxCurrentPkt = 0;                // the current packet index
bool isStartReceiving = false;        // the flag to notify start receiving a datapacket queue from serial port

const byte ENDING_LORA_BYTES[3] = {0x45, 0x4E, 0x44};     // the ending signal, the 3 bytes for 'END'
const int ENDING_SIGNAL_TIMES = 2;                        // number of times the ending signal will be sent
const int NUM_PCK_BUFF = 400;                             // the maxium packet can be set is around 400 (more will be out of RAM)
const byte REQUEST_HEADER_CHK[2] = {0x34, 0x33};          // the request starting header, the 2 bytes for 'RQ'
const byte REQUEST_END_CHK[2] = {0x45, 0x4E};             // the request starting header, the 2 bytes for 'EN'
const byte REQUEST_DONE[4] = {0x44,0x4F,0x4E,0x45};       // the request to end a file sending, the 4 bytes for 'DONE', following to the header

byte dataPacketsBuff[NUM_PCK_BUFF][DEFAULT_PCK_SIZE];     // the buff
long missingIndexesRequest[10];                           // an array for storing processed request timestamp id 
const int MAX_MISS_INDEXES = 123;                         // check size 255 - 2 - 4 - 2 = 247/2 = 123, meaning the max number of missing index is 123 
const int MISS_INDEX_PCK_SIZE = 255; 
int missingPacketIndex[MAX_MISS_INDEXES];
int missingPacketCount = 0;
   
bool isSendingMissingPacket = false;
bool isCleaning = false;
long latestCompletedFileId = 0;

void setup() {
  // initialize ESP32 and Serial port
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Serial.begin(SERIAL_BAUD);
  intLoRa();
}


void loop() {

  if(!isInit){
    Heltec.display->clear();
    isInit = true;
  }

  if (Serial.available() > 0) {                 //When receive a datapacket from serial port, 
    byte dataPacket[DEFAULT_PCK_SIZE];
    int pSize = Serial.readBytes(&dataPacket[0], DEFAULT_PCK_SIZE);

    if(String((char *)dataPacket).startsWith(START_RECEIVING_SERIAL)){
      idxCurrentPkt = 0;
      isStartReceiving = true;
      clearDataBuff();
      return;
    } else if (String((char *)dataPacket).startsWith(END_RECEIVING_SERIAL)){
      isStartReceiving = false;
      writeToScreenByLine("Total " + String(idxCurrentPkt) + " packets to send!",0);
      startSendingLoRaQueue(idxCurrentPkt);           //start sending buff
    }
     
    if(isStartReceiving){                       //process the datapacket and save into buff
      processDataPacket(&dataPacket[0], pSize);
      idxCurrentPkt = idxCurrentPkt + 1;
    }   
  }

  // handle missing packet
  if(isSendingMissingPacket){
    startSendingMissingQueue(missingPacketCount);
  }

  if(isCleaning){
    cleaningResources();
    writeToScreenByLine("Ready to send next file!", 0);
    isCleaning = false;
  }

}

/*
 * Reset everything and release the resources
 */
void cleaningResources(){
    clearDataBuff();
    clearMissingIndex();
    clearMissingRequestID();
    idxCurrentPkt = 0;
    missingPacketCount = 0;
}

/*
 * This function is to save the reading packet into buff
 */
void processDataPacket(byte *dataPacket, int dataSize){

  // save the data into buff
  for(int k=0;k<dataSize;k++){
    dataPacketsBuff[idxCurrentPkt][k] = dataPacket[k];
  }

  int idx = (int)dataPacketsBuff[idxCurrentPkt][2] * 256 + (int)dataPacketsBuff[idxCurrentPkt][3];
  int total = (int)dataPacketsBuff[idxCurrentPkt][4] * 256 + (int)dataPacketsBuff[idxCurrentPkt][5];
  String msg = "#" + String(idxCurrentPkt) + " id:"+ String(idx) + " t:" + String(total) + " size:" + String(dataSize);
  writeToScreenByLine(msg,0);

}


/*
 * This function is to clear all the value in dataPacketsBuff
 */
void clearDataBuff(){
  memset(dataPacketsBuff, 0, sizeof(dataPacketsBuff[0][0]) * NUM_PCK_BUFF * DEFAULT_PCK_SIZE);
}

void clearMissingIndex(){
  memset(missingPacketIndex, 0, sizeof(missingPacketIndex[0] * MAX_MISS_INDEXES));
}

void clearMissingRequestID(){
  memset(missingIndexesRequest, 0, sizeof(missingIndexesRequest[0] * 10));
}

/*
 * This function is to intialize the LoRa sender 
 * It will be called in setup
 * make sure the same setting have been applied on both end (sender/receiver)
 */
void intLoRa(){
  LoRa.setFrequency(BAND);                      //e.g. 868E6(EU),915E6(US),433E6(China)
  LoRa.setSignalBandwidth(125E3);               //Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, and 250E3
  LoRa.setCodingRate4(5);                       //Supported values are between 5 and 8, these correspond to coding rates of 4/5 and 4/8. The coding rate numerator is fixed at 4
  LoRa.setPreambleLength(8);                    //Supported values are between 6 and 65535
  LoRa.setSyncWord(0xF3);                       //defaults to 0x34 changed to 0xF3 to avoid receiving from other LoRa
  LoRa.setSpreadingFactor(7);                   //Supported values are between 6 and 12, defaults to 7
  LoRa.onReceive(onReceive);
  LoRa.receive();
}

void onReceive(int packetSize)
{
  if (packetSize == 0) return;
  
  //otherwise
  Serial.println("received a request - size:" + String(packetSize) + " rssi:" + String(LoRa.packetRssi()));
  byte contentBuff[MISS_INDEX_PCK_SIZE];
  for (int i = 0; i < packetSize; i++){
    byte b = (byte)LoRa.read();    
    contentBuff[i] = b;
  }
  
  if(contentBuff[0]==REQUEST_HEADER_CHK[0] && contentBuff[1]==REQUEST_HEADER_CHK[1]){     // only process if it matches the request header

    if(contentBuff[2] == REQUEST_DONE[0] && contentBuff[3] == REQUEST_DONE[1] && contentBuff[4] == REQUEST_DONE[2] && contentBuff[5] == REQUEST_DONE[3]){ // matches the ending signal
      //check the timestamp see if it's already handled
      long doneTimestamp = 0;
      doneTimestamp += contentBuff[6];
      doneTimestamp += (contentBuff[7] << 8);
      doneTimestamp += (contentBuff[8] << 16);
      doneTimestamp += (contentBuff[9] << 24);
      if(latestCompletedFileId!=doneTimestamp){
        latestCompletedFileId = doneTimestamp;
        isCleaning = true;
      }
      return; 
    }
    
    long reqTimestamp = 0;
    reqTimestamp += contentBuff[2];
    reqTimestamp += (contentBuff[3] << 8);
    reqTimestamp += (contentBuff[4] << 16);
    reqTimestamp += (contentBuff[5] << 24);

    Serial.println("processing request id:" + String(reqTimestamp));  //mostly for debug

    // if the request is not processed, send the missing index packets
    if(!isRequestProcessed(reqTimestamp)){
      //clear missingIndex array
      clearMissingIndex();
      missingPacketCount = 0;
      //int missingIndexSize = (packetSize - 2 - 4 - 2)/2; // (total size - 2 bytes header - 4 bytes timestamp - 2 bytes ending bytes)/2 bytes for each index
      for (int j=0; j< MAX_MISS_INDEXES; j++){
        if(contentBuff[6+j*2]==REQUEST_END_CHK[0] && contentBuff[6+j*2+1] == REQUEST_END_CHK[1]){
          break;
        }
        if(j>0 && contentBuff[6+j*2]==0x00 && contentBuff[6+j*2+1] == 0x00){  // check zero for safety
          break;
        }
      
        int mIndex = 0;
        mIndex += contentBuff[6+j*2] << 8;
        mIndex += contentBuff[6+j*2 + 1];
        
        missingPacketIndex[j] = mIndex;
        missingPacketCount ++;  
      }
      if(missingPacketCount > 0){
        isSendingMissingPacket = true;
      } 
    }
  }
}


/*
 * This function is to check the timestamp to see if it's already executed
 * if not, it will add the timestamp into the array
 */
bool isRequestProcessed(long r){
  for(int i=0; i<sizeof(missingIndexesRequest); i++){
    if(missingIndexesRequest[i]==0){
      missingIndexesRequest[i] = r;
      return false;
    } else if (missingIndexesRequest[i]==r) {
      return true;
    }
  }
  return false;
}


/*
 * This function is to start sending a datapacket queue ny LoRa
 */
void startSendingMissingQueue(int numPackets){

  //sending an inital request to activate the other side
  sendWakupSignal(DEFAULT_PCK_SIZE);
  delay(LORA_DELAY);
  
  for (int i=0; i<numPackets; i++){
    int mIdx = missingPacketIndex[i];
    Serial.println("Sending missing packet idx:" + String(mIdx));  //mosly for debug    
    int suc = sendLoRaDataPacket(&dataPacketsBuff[mIdx][0], DEFAULT_PCK_SIZE);
    delay(LORA_DELAY);
    String statusTxt = "OK";
    if(suc!=1){
      statusTxt = "ERR";
    }
    String msg = "p#" + String(i+1) + "/"+ String(numPackets) +" resent " + statusTxt;
    writeToScreenByLine(msg,0);
  }
  sendEndingSignal(ENDING_SIGNAL_TIMES, DEFAULT_PCK_SIZE);
  isSendingMissingPacket = false;
}


/*
 * This function is to start sending a datapacket queue ny LoRa
 * The data packets of the queue are all stored at local ESP32 RAM
 * The maxium RAM in theory can hold up to 100KB 
 * Meaning the max size of an transferable image is 100KB 
 */
void startSendingLoRaQueue(int numPackets){
  for (int i=0; i<numPackets; i++){
    int suc =  sendLoRaDataPacket(&dataPacketsBuff[i][0], DEFAULT_PCK_SIZE);
    String statusTxt = "OK";
    if(suc!=1){
      statusTxt = "ERR";
    }
    String msg = "p#" + String(i+1) + "/"+ String(numPackets) +" sent " + statusTxt;
    writeToScreenByLine(msg,0);
  }
  sendEndingSignal(ENDING_SIGNAL_TIMES, DEFAULT_PCK_SIZE);
  
}

/*
 * This function is to send a ending signal,
 * the times can be set to avoid signal loss
 */
int sendEndingSignal(int times, int packetSize){
  byte ending[packetSize];
  ending[0] = ENDING_LORA_BYTES[0];
  ending[1] = ENDING_LORA_BYTES[1];
  ending[2] = ENDING_LORA_BYTES[2];
  long timestamp = millis();
  ending[3] = (byte) timestamp;
  ending[4] = (byte) (timestamp >> 8);
  ending[5] = (byte) (timestamp >> 16);
  ending[6] = (byte) (timestamp >> 24);

  Serial.println("ending request sent - id:" + String(timestamp)); //mostly for debug

  for (int t=0; t< times; t++){
    sendLoRaDataPacket(ending, packetSize);
  }
  LoRa.receive();                         // set LoRa back to receive mode 
}

/*
 * This function is to send a data packet by LoRa
 * Note: the max LoRa data packet by LoRa is 255 bytes
 */
int sendLoRaDataPacket(byte *dataPacket, int packetSize) {
  int success;
  LoRa.beginPacket();
  LoRa.setTxPower(12, RF_PACONFIG_PASELECT_PABOOST);
  LoRa.write(dataPacket, packetSize);
  success = LoRa.endPacket();  
  return success;
}

byte blankArray[255];

int sendWakupSignal(int packetSize){
  int success;
  LoRa.beginPacket();
  LoRa.setTxPower(12, RF_PACONFIG_PASELECT_PABOOST);
  LoRa.write(blankArray, packetSize);
  success = LoRa.endPacket();  
  return success;
}


/*
 *  This funtion is to display text to the ESP32 OLED screen
 *  Note: it will display on the first line align left (x = 0, y = 0)
 */
void displayToScreen(String str, int delaytime) {
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, str);
  Heltec.display->display();
  delay(delaytime);
}

/*
 * This function is to write a line (with line break at the end) 
 * to the OLED screen
 * Note:
 * The max string length can be display is 21
 * The max lines the screen can display is 6 (can be set in the global variables)
 */
void writeToScreenByLine(String str, int delaytime) {
  if(currentLine >= maxLines){
    currentLine = 0; //reset current line
    Heltec.display->clear();
  }
  int startPoint_y = currentLine * lineHeight;

  String strDisplay = "";
  if (str.length() >=maxMsgLength){ 
    strDisplay = str.substring(0,maxMsgLength);
  } else {
    strDisplay = str;
  }
  Heltec.display->drawString(0, startPoint_y, strDisplay);
  Heltec.display->display();
  currentLine = currentLine + 1;
  delay(delaytime);
}
