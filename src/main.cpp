/*****************************************************************
 * Sharp PC-G850V(S) ESP32 Interface
 * By: ChrisHerman
 * Date: December 5th, 2021
 *
 * Receives files from Sharp G850 and sends to PC via USB-Serial (Nano)
 * Requires G850's RTS and CTS lines to be connected witch each other 
 * and pulled-up to +5V. otherwise SHarp will wait forever.
 * 
 *
 * Hardware Hookup:
 * * 
 * Sharp G850 11 pin port            ESP32
 *=========================================
 *  GND (pin 3) __________________ GND
 * 
 * Rx (pin 6)  ___________________ GPIO4 (Tx)
 * 
 * Tx (pin 7)  ___________________ GPIO5 (Rx)
 * 
 * RTS (pin 4) __________
 *                       |
 * CTS (pin 9) __________+
 *                       |
 * +5V         ---[10K]--+
 * 
 *
 * This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 *****************************************************************/



#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <Ticker.h>  
#include "config.h"
#include "ATScanner.h"


//below to ensure Strings from platformio.ini are handled as intended by pre-compiler
#define ST(A) #A
#define STR(A) ST(A)

#define RX_PIN 5
#define TX_PIN 4 

// CTS signal pin
#define CTS_PIN 14

// mode switch
#define MODE_PIN 13

//#define LED_PIN 2
#define LED_PIN 10
#define LEDON 0x1
#define LEDOFF 0x0
const int BlinkDelay= 10; // blink period in ms

// second button on the PCB should be PRG-pin
#define PRG_PIN 0

#ifndef SLEEPTIMER_DIV
  #define SLEEPTIMER_DIV 10
#endif



//SoftSerial related objects
#define BUFFER_SIZE 1024    
byte buff[BUFFER_SIZE];
SoftwareSerial SoftSerial(RX_PIN, TX_PIN, true); // RX, TX, inverse_logic = true
void GoTheFuckToSleep();
ATScanner SoftATscanner(SoftSerial, GoTheFuckToSleep);


#define READ_BUFFER_SIZE 254    
byte read_buff[READ_BUFFER_SIZE+2];

// global objects
WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;
WiFiServer server(RAW_TCP_PORT);
WiFiClient  client;

// serial baudrate (grobal)
int Baud;

void checkFlash(){
#ifdef DEBUG
  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  Serial.printf("\n");
  Serial.printf("Flash real id:   %08X\n", ESP.getFlashChipId());
  Serial.printf("Flash real size: %u bytes\n", realSize);
  Serial.printf("Flash ide  size: %u bytes\n", ideSize);
  Serial.printf("Flash ide speed: %u Hz\n", ESP.getFlashChipSpeed());
  Serial.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));

  if (ideSize != realSize) 
    Serial.println("Flash Chip configuration wrong!\n");
  else 
    Serial.println("Flash Chip configuration ok.\n");
#endif

}


void OTASetup(){

  ArduinoOTA.setPassword(GlobalConfig.otapw);
  ArduinoOTA.onStart([]() {
    #ifdef DEBUG
      Serial.println("Start");
    #endif
  });
  ArduinoOTA.onEnd([]() {
    #ifdef DEBUG
      Serial.println("\nEnd");
    #endif
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    #ifdef DEBUG
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
  });

  ArduinoOTA.onError([](ota_error_t error) {
    #ifdef DEBUG
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    #endif
  });

  ArduinoOTA.begin();
}

void listAllFilesInDir(String dir_path)
{
	Dir dir = LittleFS.openDir(dir_path);
	while(dir.next()) {
		if (dir.isFile()) {
			// print file names
			Serial.print("File: ");
			Serial.println(dir_path + dir.fileName());
		}
		if (dir.isDirectory()) {
			// print directory names
			Serial.print("Dir: ");
			Serial.println(dir_path + dir.fileName() + "/");
			// recursive file listing inside new directory
			listAllFilesInDir(dir_path + dir.fileName() + "/");
		}
	}
}



int GetBaudrate(int i){

  #ifdef DEBUG
    Serial.println( String( "GetBaudrate=") + i);
  #endif

  switch (i){

    case 0: return SOFTBAUDRATE0;
    break;

    case 1: return SOFTBAUDRATE1;
    break;
    
    case 2: return SOFTBAUDRATE2;
    break;
    
    case 3: return SOFTBAUDRATE3;
    break;
    
    case 4: return SOFTBAUDRATE4;
    break;

    default: return SOFTBAUDRATE4;
    break;
  }
}






/////////////////////////////
//what to do with our LED: 
enum blinkstates {Off, Single, Double, Tripple, On} ;
blinkstates blinkstate= Off;

static void blinker(){


  
  #ifdef DEBUG
    //Serial.print("*");
  #endif


  static int cycle= 0;
  cycle++;
  switch (blinkstate){
    case Off: 
      digitalWrite(LED_PIN, LEDOFF); 
      cycle=0;
    break;

    case Single: 
        switch(cycle){
          case 1: digitalWrite(LED_PIN, LEDON); break; 
          case 150: cycle=0; break;
          default: digitalWrite(LED_PIN, LEDOFF); break;
        }
    break;
        
    case Double: 
        switch(cycle){
          case 1: digitalWrite(LED_PIN, LEDON); break; 
          case 20: digitalWrite(LED_PIN, LEDON); break;
          case 170: cycle=0; break;
          default: digitalWrite(LED_PIN, LEDOFF); break;
        }
    break;
    
    case Tripple: 
        switch(cycle){
          case 1: digitalWrite(LED_PIN, LEDON); break; 
          case 21: digitalWrite(LED_PIN, LEDON); break;
          case 41: digitalWrite(LED_PIN, LEDON); break;
          case 190: cycle=0; break;
          default: digitalWrite(LED_PIN, LEDOFF); break;
        }
    break;
    case On: 
        digitalWrite(LED_PIN, LEDON); 
        cycle=0;
    break;

    default: 
      digitalWrite(LED_PIN, LEDOFF);
    break;
  }
}

void SetBlinker(blinkstates bs){
  //set pin as output
  static bool firstcall= true;
  if(firstcall){
      pinMode(LED_PIN, OUTPUT);
      firstcall=false;
  }

  blinkstate= bs;
}

Ticker BlinkTimer(blinker, BlinkDelay, 0, MILLIS);


void GoTheFuckToSleep(){
  digitalWrite(LED_PIN, LEDOFF);
  delay(500);
  while(true){
    ESP.deepSleep(0);   
    delay(100);
  }
}

//////////////////////////////////
// sleep-timout related stuff:
int sleepCountdown=0;
static void SleepCheck(){
  #ifdef DEBUG
    Serial.println("Going to sleep");
  #endif

  sleepCountdown++;

  if(sleepCountdown==(SLEEPTIMER_DIV-1)){
    SetBlinker(Double);
  } else  if(sleepCountdown==SLEEPTIMER_DIV){
    SetBlinker(Tripple);
  } else if(sleepCountdown>SLEEPTIMER_DIV){
    GoTheFuckToSleep();
  } else
    SetBlinker(Single);

}

Ticker SleepTimer(SleepCheck, 30000/SLEEPTIMER_DIV, 0, MILLIS);  //we need to create on the heap otherwise the sleeptimeout is not known


void SleepTimerRestart(){
  sleepCountdown=0;
  SetBlinker(Single);
  SleepTimer.start(); //reset timer
}

// Reset the sleep timer upon button release
void CheckPrgButton(){
  const unsigned long BUTTONPRESSTIME= 5000;        // duration of keypress


  //set PRG pin as input when called first time
  static bool firstcall= true;
  if(firstcall){
    pinMode(PRG_PIN, INPUT);
    firstcall=false;
  }


  static bool wasPressed= false;
  static unsigned long lastpress= 0;


  if(digitalRead(PRG_PIN)==LOW){ //button pressed
    if(wasPressed==false){      // first time that happpened
    wasPressed= true;           // let's remember that
    lastpress=millis();         // and when that happened
    } else {                    // wasPressed flag was already set
      //not really doing anything here...
    }

    ////////////////////////////////////////////////
    // Press 'Prog button' to inform the local IP.
    ////////////////////////////////////////////////
    int cnt = 0;
    while ( digitalRead(PRG_PIN)==LOW) {
        //LED ON
        digitalWrite(LED_PIN, LEDON);
        delay(10); // Delay for Button ON
        // 1 sec pressed ?
        if ( cnt++ > 100 ){
            char sbuff[64];
            uint32_t v4 = WiFi.localIP().v4();
            char * pv4 = (char *)&v4;
            sprintf(sbuff,"1 WiFi connect\r\n2 IP:%d.%d.%d.%d\r\n",pv4[0],pv4[1],pv4[2],pv4[3]);
            //change 1920bps -> 9600bps
            if ( Baud==SOFTBAUDRATE5 ) {
                SoftSerial.end();
                SoftSerial.begin(SOFTBAUDRATE4);
            }
            //IP address send to G850V
            SoftSerial.write(sbuff);
            SoftSerial.flush();
            // restart SoftwareSerial driver
            if ( Baud==SOFTBAUDRATE5 ) {
                SoftSerial.end();
                SoftSerial.begin(SOFTBAUDRATE5);
            }
            //finished
            digitalWrite(LED_PIN, LEDOFF);
            for ( cnt=0  ;cnt < 5 ; cnt++){
                digitalWrite(LED_PIN, LEDON);
                delay(100);
                digitalWrite(LED_PIN, LEDOFF);
                delay(100);
            }
            break;
        }
    }

  } else{ //button not pressed
    if(wasPressed==true){   // but it was pressed just a moment ago
      wasPressed= false;    // let's reset that memory

      if((lastpress+BUTTONPRESSTIME)<millis()){   // button was pressed longer than the timeout time
        #ifdef DEBUG
          Serial.println("PRG button: restoring failsafe config");
        #endif      
        loadFailSafeConfiguration(GlobalConfig);
        saveConfiguration(GlobalConfig);
        #ifdef DEBUG
          Serial.println("restarting with failsafe config");
        #endif  
        delay(1000);
        ESP.reset();
      } else{                                       // was only pressed briefly
        #ifdef DEBUG
          Serial.println("PRG button: timer reset");
        #endif
        SleepTimerRestart();  //payload
      }

    }
  }
}


/***************************************************************/
/*   GetLine : Get Line                                        */
/*   処理    : ソースバッファから１行分の文字列を取得する      */
/*   出力    : バイト数                                        */
/***************************************************************/
int GetLine( byte *dst_buff ,byte *src_buff ,int size )
{
int	sts;
int	i;
int	cnt=0;
	/* 読出しバッファ上限まで */
	size = (size >= READ_BUFFER_SIZE ) ? READ_BUFFER_SIZE : size;
	/* 1行終了まで取り出す */
	for( i = 0 ; i < size ; i++ ){
		/* 1文字取り出し */
		sts = *src_buff++;

		switch ( sts ){
		case '\n':
		case 0x1a:
			*dst_buff++ = (char)sts;
			cnt++;
			/* ファイル終了 */
			return cnt;
		default:
			*dst_buff++ = (char)sts;
			cnt++;
			break;
		}
	}
	return cnt;
}




void setup() {

  wifi_set_sleep_type(LIGHT_SLEEP_T);

  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  
  if (LittleFS.begin()){
    #ifdef DEBUG
      Serial.println("File system mounted");
    #endif
  }
  else {
    #ifdef DEBUG
      Serial.println("File system error");
    #endif
  }

  loadConfiguration(GlobalConfig);
  #ifdef DEBUG
    PrintConfig(Serial);
    checkFlash();
    listAllFilesInDir("/");
  #endif

  SleepTimer.interval((GlobalConfig.sleeptimeout*1000)/SLEEPTIMER_DIV);
  SleepTimer.start();
  SetBlinker(On);
  BlinkTimer.start();  

  pinMode(CTS_PIN, INPUT);	//CTS signal pin
  pinMode(MODE_PIN, INPUT);	//mode switch

  //Check Mode switch
  if (digitalRead(MODE_PIN)==LOW){
    // 19200Bps
    SoftSerial.begin(SOFTBAUDRATE5);
    Baud = SOFTBAUDRATE5;
  }
  else{
    // user setting
    SoftSerial.begin(GlobalConfig.softbaudrate);
    //現在のボーレートを保存する
    Baud = GlobalConfig.softbaudrate;
  }

  //WiFi stuff:
  WiFi.begin(GlobalConfig.wifissid, GlobalConfig.wifipassword);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event)
  {
    WiFi.setHostname(GlobalConfig.hostname);
    SetBlinker(Single);
    #ifdef DEBUG
      Serial.print("Station connected, IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("Hostname:");
      Serial.println(WiFi.getHostname());
    #endif
  });
  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event)
  {
    SetBlinker(On);
    #ifdef DEBUG
      Serial.println("Station disconnected");
    #endif
  });

  delay(200);
  OTASetup();


  server.begin(GlobalConfig.rawport);
  #ifdef DEBUG
    Serial.println("servers started");
  #endif
  SleepTimer.start();
}


void loop() {
  //static unsigned long timer=millis();
  int size = 0;

  client = server.available();              //wait for client connection 

  if (client){

    //clear any bytes received
    while(SoftSerial.available()>0)  {  
      SoftSerial.read();  
    }

    while (client.connected()) {
      ATScanner netscanner(client, GoTheFuckToSleep);
    
      // read data from wifi client and send to serial
      while ((size = client.available())) {
            size = (size >= BUFFER_SIZE ? BUFFER_SIZE : size);
            client.read(buff, size);
            netscanner.scan(buff, size);

            int index=0;
            while (size > 0) {
                /* 1行分のデータを取り出す */
                int cnt = GetLine( read_buff , &buff[index] , size );
                /* 1行分のデータを送付してWAITする */
                SoftSerial.write(read_buff, cnt);
                SoftSerial.flush();
                while(digitalRead(CTS_PIN)==LOW){
                  //19200Bps時はCTS制御しない
                  if (digitalRead(MODE_PIN)==LOW){
                     break;
                  }
                  delay(1); // Wait for CTS=HIGH.
                }
                /* 次回サイズ更新 */
                size -= cnt;
                index += cnt;
            }
            SleepTimerRestart();
            BlinkTimer.update();
      }
    
      // read data from serial and send to wifi client
      while ((size = SoftSerial.available())) {
            size = (size >= BUFFER_SIZE ? BUFFER_SIZE : size);
            SoftSerial.readBytes(buff, size);
            SoftATscanner.scan(buff, size);
            client.write(buff, size);
            client.flush();
            SleepTimerRestart();
            BlinkTimer.update();
      }
      
      SleepTimer.update();
      BlinkTimer.update();
      ArduinoOTA.handle();
      CheckPrgButton();

      //Mode switch = LOW
      int chg_baud = 0;
      if (digitalRead(MODE_PIN)==LOW){
      	if(Baud != SOFTBAUDRATE5){
      		SoftSerial.end();
      		// Set to 19200Bps
      		Baud = SOFTBAUDRATE5;
      		SoftSerial.begin(Baud);
      		chg_baud = 1;
      	}
      }
      //Mode switch = Hi
      else{
      	if(Baud != GlobalConfig.softbaudrate){
      		SoftSerial.end();
      		// Set to default
      		Baud = GlobalConfig.softbaudrate;
      		SoftSerial.begin(Baud);
      		chg_baud = 1;
      	}
      }
      //baudrate is changed?
      if(chg_baud){
      	digitalWrite(LED_PIN, LEDOFF);
      	for ( int cnt=0  ;cnt < 5 ; cnt++){
      		digitalWrite(LED_PIN, LEDON);
      		delay(100);
      		digitalWrite(LED_PIN, LEDOFF);
      		delay(100);
      	}
      }

    }

    client.stop();    
  }


  //no longer connected
  //keep watching serial port for commands
  while ((size = SoftSerial.available())) {
        size = (size >= BUFFER_SIZE ? BUFFER_SIZE : size);
        SoftSerial.readBytes(buff, size);
        SoftATscanner.scan(buff, size);
        SleepTimerRestart();
        BlinkTimer.update();
        CheckPrgButton();
  }


  SleepTimer.update();
  BlinkTimer.update();
  ArduinoOTA.handle();
  CheckPrgButton();
  delay(5); // w/o this delay., OTA gives you trouble
}
