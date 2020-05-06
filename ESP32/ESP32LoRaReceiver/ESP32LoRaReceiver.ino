/*
  The program was designed to run on Heltec ESP32 LoRa V2
  1 - listen to the LoRa, capture data packet
  2 - send the data packet to serial port 
  4 - (optional) request a missing packet (Not implemented yet)
  Created 04/13/2020
  by Yang Zhong
*/
#include "heltec.h"
#include "WiFi.h"
#define BAND 915E6                // set band according to the region, e.g. 868E6(EU),915E6(US),433E6(China)
#define SERIAL_BAUD 115200        // set the serial baud to match the baud at device (Jetson) 

// the index request size - 255 is safe, can try further, (tried 90 is bad), 
// this gives us the ability of 255 - 2 (header) - 4 (timestamp) - 2 (ending) = 72 bytes which is 36 missing index to send 
const int MISS_INDEX_PCK_SIZE = 255;   
int MAX_MISSING_IDX  = (MISS_INDEX_PCK_SIZE - 2 - 2 - 4)/2;
const int PKT_SIZE = 255;         // the packet size, this need to be set to the same value in all programs, default is 255
const int maxLines = 6;           // the maximum number of lines the OLED can display - default is 6
const int maxMsgLength = 25;      // the maximum string chars the OLED can display with a single line 
const int lineHeight = 10;        // the height for each line
int defaultDelay = 1000;          // the default time for delay the string display on screen to disappear
int currentLine = 1;              // the current line indicator 

const int NUM_PCK_BUFF = 400;                   // the maxium packet can be set is around 400 (more will be out of RAM)
byte dataPacketsBuff[NUM_PCK_BUFF][PKT_SIZE];   // the buff
const byte HEADER_CHK_BYTE[2] = {0x07, 0xE4};   // the header check

// request header - the request LoRa will start with two bytes, then followed with 4 bytes of timestamp, then from the 5th byte, each one will be a index, the ending byte 0x
const byte ENDING_LORA_BYTES[3] = {0x45, 0x4E, 0x44};     // the ending signal, the 3 bytes for 'END'
const byte REQUEST_HEADER_CHK[2] = {0x34, 0x33};          // the request starting header, the 2 bytes for 'RQ'
const byte REQUEST_END_CHK[2] = {0x45, 0x4E};             // the request starting header, the 2 bytes for 'EN'
const byte REQUEST_DONE[4] = {0x44,0x4F,0x4E,0x45};       // the request to end a file sending, the 4 bytes for 'DONE', following to the header
const int REQUEST_SENT_TIME = 2;                          // the missing index request send time
const int LORA_DELAY = 800;                               // the default lora delay time, in ms
                                                     
long endRequestTimestamps[10];                            // an array for storing processed request timestamp         
int missingIndexPayload[120];                             // support up to 120 missing packets per request 
byte requestPacket[MISS_INDEX_PCK_SIZE];
int currentMissingCount = 0;
bool isSendingMissingIndxRequest = false;
int totalNumPackets = 0;
int numReceivedPkt = 0; 
int testPacketLoss = 0;                                   // a number (0-100) represents % if you want to test missing packet, e.g. testPacketLoss = 40 means 40% packet will be ignore (lost) then request again

bool debugMode = false;                                    // turn the serial debug output on/off
bool isPushingToPC = false;
bool isReceiving = false;
int lastRssi = 0;

void setup() {                // initialize ESP32 and Serial port
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Serial.begin(SERIAL_BAUD);
  LoRa.onReceive(onReceive);
  LoRa.receive();             // put the radio into receive mode
  intLoRa();
}

void loop(){

  if(isSendingMissingIndxRequest){
    sendMissingIndexRequest(currentMissingCount, REQUEST_SENT_TIME);
    isSendingMissingIndxRequest = false;
  }

  if(isPushingToPC){
    writePacketsToSerial();
    sendFinishSignal(REQUEST_SENT_TIME);
    cleaningResources();
    isPushingToPC = false; 
  }

}

/*
 * Reset everything and release the resources
 */
void cleaningResources(){
  memset(dataPacketsBuff, 0, sizeof(dataPacketsBuff[0][0]) * NUM_PCK_BUFF * PKT_SIZE);
  memset(requestPacket, 0, sizeof(requestPacket[0]) * MISS_INDEX_PCK_SIZE);  
  memset(endRequestTimestamps, 0, sizeof(endRequestTimestamps[0]) * 10);
  memset(missingIndexPayload, 0, sizeof(missingIndexPayload[0]) * 120); 
  numReceivedPkt = 0;
  totalNumPackets = 0;
  currentMissingCount = 0;
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
  LoRa.setSyncWord(0xF3);                       //defaults to 0x34, change to 0xF3 to avoid receive from other LoRa
  LoRa.setSpreadingFactor(7);                   //Supported values are between 6 and 12, defaults to 7
}

/*
 * receive reading by each byte
 */
void onReceive(int packetSize)
{
  byte contentBuff[PKT_SIZE];
  for (int i = 0; i < packetSize; i++)
  {
    byte b = (byte)LoRa.read();
    contentBuff[i] = b;
  }
  int rssi = LoRa.packetRssi();
  lastRssi = rssi;
  LoRa.flush();
  
  if(contentBuff[0]==HEADER_CHK_BYTE[0] && contentBuff[1]==HEADER_CHK_BYTE[1]){     // only process if it matches the header two bytes
    if(totalNumPackets == 0){                                                       // initialize the total if hasn't
      totalNumPackets = (int)contentBuff[4]*256 + (int)contentBuff[5];              // bytes to int
    }
    int indexOfPacket = (int)contentBuff[2]*256 + (int)contentBuff[3];    
    int ignore = random(1, 100);                                                    //NOTE: for testing or debuging, ignore a few packet for testing, can set the value to global 'testPacketLoss'
    if(indexOfPacket < totalNumPackets && ignore > testPacketLoss){  
      for(int k=0;k<PKT_SIZE;k++){
        dataPacketsBuff[indexOfPacket][k] = contentBuff[k];     // save the data to the buff to the lot
      }
      numReceivedPkt = numReceivedPkt + 1;    
      String msg = "new pkt:" + String(numReceivedPkt) + "/" + String(totalNumPackets) + " idx:" + String(indexOfPacket) + " rssi:" + String(rssi);
      serialOutput(msg);   // for debug
    }
  }
  // if the end signal byte received, the first two bytes equal to ending byte
  if(contentBuff[0]==ENDING_LORA_BYTES[0] && contentBuff[1]==ENDING_LORA_BYTES[1] && contentBuff[2]==ENDING_LORA_BYTES[2]){
    long endTimestamp = 0;
    endTimestamp += contentBuff[3];
    endTimestamp += (contentBuff[4] << 8);
    endTimestamp += (contentBuff[5] << 16);
    endTimestamp += (contentBuff[6] << 24);    
    if(!isEndingRequestProcessed(endTimestamp)){
      serialOutput("processing ending request id:" + String(endTimestamp));   // for debug
      processEndingRequest();
    }
  }
}

/*
 * This function is to check the timestamp to see if it's already executed
 * if not, it will add the timestamp into the array
 */
bool isEndingRequestProcessed(long r){
  for(int i=0; i<sizeof(endRequestTimestamps); i++){
    if(endRequestTimestamps[i]==0){
      endRequestTimestamps[i] = r;
      return false;
    } else if (endRequestTimestamps[i]==r) {
      return true;
    }
  }
  return false;
}

/*
 * This function is to make sure all the packets are received
 * if yes, push it to pc via serial port
 * if not, send a LoRa to request the missing indexes
 */
void processEndingRequest(){
  //check see if all the index can be found
  String missingIdxMsg = "";
  int countMissing = 0;
  for(int i=0; i<totalNumPackets; i++){
    if(dataPacketsBuff[i][0]==0){
      missingIdxMsg = missingIdxMsg + " " + i;
      missingIndexPayload[countMissing] = i;
      countMissing = countMissing + 1;
    }
  }
  // if there is no missing index, send it to serial port
  if(countMissing == 0){
    currentMissingCount = 0;
    serialOutput("all packets have been received"); 
    //send to PC, also clear all variables
    isPushingToPC = true;
  } else {
    //send LoRa request back
    serialOutput("missing indexes are:" + missingIdxMsg);          // for debug
    currentMissingCount = countMissing;
    isSendingMissingIndxRequest = true;
  }
}

/*
 * this is to push to packet into serial
 * Need to be called in the main loop instead of onreceive
 */
void writePacketsToSerial(){
  serialOutput("total packet:" + String(totalNumPackets));
  for (int i=0; i<totalNumPackets; i++){
    serialOutput("Content for index:" + String(i) + " size:" + String(PKT_SIZE));
    Serial.write(&dataPacketsBuff[i][0], PKT_SIZE); 
  } 
}

/*
 * This method is to send a request for the missing packets
 */
void sendMissingIndexRequest(int count, int times){
  // if greater than the MAX_MISSING_IDX, 
  // request the first MAX_MISSING_IDX of the missing index
  if(count > MAX_MISSING_IDX){
    count = MAX_MISSING_IDX;
  }
  
  int sizeOfPacket = 2 + 4 + (count *2) + 2;    // 2 (request hearder) + 4 (timestamp) + 2 * n (each two bytes for a missing index) + 2 (ending bytes)
  serialOutput("Total # of missing pkts:" + String(count));  //debug
  if(sizeOfPacket <= MISS_INDEX_PCK_SIZE){                      
    requestPacket[0] = REQUEST_HEADER_CHK[0];
    requestPacket[1] = REQUEST_HEADER_CHK[1];
    // append the timestamp
    long timestamp = millis();
    serialOutput("missing idx request id:" + String(timestamp));
    requestPacket[2] = (byte) timestamp;
    requestPacket[3] = (byte) (timestamp >> 8);
    requestPacket[4] = (byte) (timestamp >> 16);
    requestPacket[5] = (byte) (timestamp >> 24);
    // append the missing index, 2 bytes each
    for (int i=0; i< count; i++){
      int missingIdx = missingIndexPayload[i];
      requestPacket[5+i*2+1] = (byte) missingIdx >> 8;
      requestPacket[5+i*2+2] = (byte) missingIdx;
    }
    requestPacket[5+count*2+1] = REQUEST_END_CHK[0];
    requestPacket[5+count*2+2] = REQUEST_END_CHK[1];
    // send the request mutiple times to avoid packet loss
    for (int t=0; t< times; t++){
      int success = sendLoRaDataPacket(&requestPacket[0], MISS_INDEX_PCK_SIZE);
      serialOutput("missing index request sent via LoRa - status:" + String(success)); //debug
    }
    LoRa.receive();   // set LoRa back to receive mode  - very important!
    displayResendingInfo();
  }
}

/*
 * This is for displying signal info on the screen
 */
void displayResendingInfo(){
    String msg = "processing.."; 
    writeToScreenByLine(msg,0);
    String msg2 = "lastest rssi:" + String(lastRssi);
    writeToScreenByLine(msg2,0);
    String msg3 = "lost pkt request sent";
    writeToScreenByLine(msg3,0);
}

/*
 * notify the other lora it can release the resoucres
 */
void sendFinishSignal(int times){
  requestPacket[0] = REQUEST_HEADER_CHK[0];
  requestPacket[1] = REQUEST_HEADER_CHK[1];
  requestPacket[2] = REQUEST_DONE[0];
  requestPacket[3] = REQUEST_DONE[1];
  requestPacket[4] = REQUEST_DONE[2];
  requestPacket[5] = REQUEST_DONE[3];
  //append request timestamp as id
  long timestamp = millis();
  requestPacket[6] = (byte) timestamp;
  requestPacket[7] = (byte) (timestamp >> 8);
  requestPacket[8] = (byte) (timestamp >> 16);
  requestPacket[9] = (byte) (timestamp >> 24);  
  for (int t=0; t< times; t++){
      sendLoRaDataPacket(&requestPacket[0], MISS_INDEX_PCK_SIZE);
      serialOutput("send the done request times:" + t);
  }
  String msg = "new file received!";
  writeToScreenByLine(msg,0);
  String msg2 = "lastest rssi:" + String(lastRssi);
  writeToScreenByLine(msg2,0);
  LoRa.receive();
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
  
  delay(LORA_DELAY);   
  return success;
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

/*
 * This function is to reset the OLED screen
 * The new message will be starting from the top left
 */
void resetScreen(){
  currentLine = 0;
  Heltec.display->clear();
}

/*
 * This function is to make output to monitor, mainly for debug
 * if debug mode turn off, then in production mode
 */
void serialOutput(String msg){
  if(debugMode){
    Serial.println(msg);
  }
}
