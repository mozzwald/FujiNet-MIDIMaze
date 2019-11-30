/*
 * FujiNet MIDIMaze Testing
 * 
 * This passes MIDI-OUT data from the Atari to another FujiNet
 * and passes MIDI-IN data from another FujiNet to the Atari
 * 
 * Original example sketch by RoboRemo (www.roboremo.com)
 * https://github.com/roboremo/ESP8266-WiFi-UART-Bridge
 */

#include <ESP8266WiFi.h>

// Uncomment for Debug on 2nd UART (GPIO 2)
#define DEBUG_S

#define PIN_LED         2
#define PIN_INT         5
#define PIN_PROC        4
#define PIN_MTR        16
#define PIN_CMD        12
#define PIN_CKO        14

#define DELAY_T5          1500
#define READ_CMD_TIMEOUT  12
#define CMD_TIMEOUT       50

bool startbit = false;

#define UART_BAUD 31250 // MIDI Baud Rate
#define packTimeout 5 // ms (if nothing more on UART, then send packet)
#define bufferSize 8192

//#define MODE_AP // connect directly to ESP
#define MODE_STA // ESP connects to WiFi router

//#define PROTOCOL_TCP
//#define TCP_SERVER // uncomment for server side
//#define TCP_CLIENT // uncomment for client side
//IPAddress serverIP(192, 168, 1, 111); // uncomment for client side

#define PROTOCOL_UDP
IPAddress sendIP(192,168,1,119); // IP of the other FujiNet

#ifdef MODE_AP
// For AP mode:
const char *ssid = "myssid";  // connect to this Access Point
const char *pw = "qwerty123"; // and this is the password
IPAddress ip(192, 168, 0, 1); // connect to this IP
IPAddress netmask(255, 255, 255, 0);
const int port = 9876; // and this port
#endif

#ifdef MODE_STA
// For STATION mode:
const char *ssid = "myssid";  // Your ROUTER SSID
const char *pw = "qwerty123"; // and WiFi PASSWORD
const int port = 9876;
#endif

#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer server(port);
WiFiClient client;
#endif

#ifdef PROTOCOL_UDP
#include <WiFiUdp.h>
WiFiUDP udp;
IPAddress remoteIp;
#endif

uint8_t buf1[bufferSize];
uint8_t i1=0;

uint8_t buf2[bufferSize];
uint8_t i2=0;

void setup() {
  // Set up Pins
  pinMode(PIN_INT, OUTPUT);
  pinMode(PIN_PROC, OUTPUT);
  pinMode(PIN_MTR, INPUT);
  pinMode(PIN_CMD, INPUT);
  pinMode(PIN_CKO, OUTPUT);
  //analogWriteFreq(31250); // Set PWM Clock for MIDI
  analogWriteFreq(33125); // Set PWM Clock for MIDI, closer to actual frequency
  analogWrite(PIN_CKO, 511); // Turn on PWM @ 50% duty cycle

#ifdef DEBUG_S
  Serial1.begin(19200);
  Serial1.println();
  Serial1.println("FujiNet MIDIMaze 2 Player Test");
#endif

  Serial.begin(UART_BAUD);
  //Serial.begin(19200);
  Serial.swap(); // Swap serial port pins

  #ifdef MODE_AP 
  //AP mode (connect directly to ESP, no router)
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP 
  WiFi.softAP(ssid, pw); // configure ssid and password for softAP
  #endif

  
  #ifdef MODE_STA
  // STATION mode (ESP connects to router and gets an IP)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  #endif

  #ifdef PROTOCOL_TCP
  #ifdef TCP_SERVER
#ifdef DEBUG_S
  Serial1.println("Starting TCP Server");
  Serial1.print("IP Address: ");
  Serial1.println(WiFi.localIP());
#endif
  server.begin(); // start TCP server 
  #endif
  #endif

  #ifdef PROTOCOL_UDP
#ifdef DEBUG_S
  Serial1.println("Starting UDP Server");
#endif
  udp.begin(port); // start UDP server 
  #endif
}


void loop() {

  #ifdef PROTOCOL_TCP
  #ifdef TCP_SERVER
  if(!client.connected()) { // if client not connected
    client = server.available(); // wait for it to connect
    return;
  }
  #endif
  #ifdef TCP_CLIENT
  if(!client.connected()) { // if client not connected
    if (client.connect(serverIP, port)) // connect to it
    {
      #ifdef DEBUG_S
      Serial1.println("Connected to server");
      #endif
      //client.write("Connected!", 10);
    }
    else
      return;
  }
  #endif

  if(client.available()) {
    byte newbuf;
/*  while(client.available()) {
      buf1[i1] = (uint8_t)client.read(); // read char from client
      if(i1<bufferSize-1) i1++;
    }*/
    newbuf = (uint8_t)client.read();
    // now send to UART:
    Serial.write(newbuf);
    Serial.flush();
#ifdef DEBUG_S
    Serial1.print("MIDI-IN: ");
    Serial1.println(newbuf, HEX);
#endif
    i1 = 0;
  }

  if(Serial.available())
  {
    if (digitalRead(PIN_MTR) == LOW || digitalRead(PIN_CMD) == LOW)
    {
      Serial.read(); // Toss the data if motor or command is asserted
    }
    else
    {
      buf2[i2] = (char)Serial.read(); // read char from UART
      // now send to WiFi:
      client.write((char*)buf2, 1);
#ifdef DEBUG_S
      Serial1.print("MIDI-OUT: ");
      Serial1.println(buf2[i2], HEX);
#endif
    }
  }

/*
  if(Serial.available()) {

    // read the data until pause:
    
    while(1) {
      if(Serial.available()) {
        if (digitalRead(PIN_MTR) == LOW || digitalRead(PIN_CMD) == LOW)
        {
          Serial.read(); // Toss the data if motor or command is asserted
        }
        else
        {
          buf2[i2] = (char)Serial.read(); // read char from UART
          if(i2<bufferSize-1) i2++;
#ifdef DEBUG_S
          Serial1.print("MIDI-OUT: ");
          Serial1.println(buf2[i2]);
#endif
        }
      } else {
        //delayMicroseconds(packTimeoutMicros);
        delay(packTimeout);
        if(!Serial.available()) {
          break;
        }
      }
    }
    // now send to WiFi:
    client.write((char*)buf2, i2);
    i2 = 0;
  }*/
  #endif

  #ifdef PROTOCOL_UDP
  // if there’s data available, read a packet
  int packetSize = udp.parsePacket();
  if(packetSize>0) 
  {
    remoteIp = udp.remoteIP(); // store the ip of the remote device
    udp.read(buf1, bufferSize);
    // now send to UART:
    Serial.write(buf1, packetSize);
#ifdef DEBUG_S
    Serial1.print("MIDI-IN: ");
    Serial1.println((char*)buf1);
#endif
    Serial.flush();
  }

  if(Serial.available()) {
    // read the data until pause:
    if (digitalRead(PIN_MTR) == LOW || digitalRead(PIN_CMD) == LOW)
    {
      Serial.read(); // Toss the data if motor or command is asserted
    }
    else
    {
      while(1)
      {
        if(Serial.available())
        {
            buf2[i2] = (char)Serial.read(); // read char from UART
  #ifdef DEBUG_S
            Serial1.print("MIDI-OUT: ");
            Serial1.println(buf2[i2]);
  #endif
            if(i2<bufferSize-1) i2++;
        }
        else
        {
          //delayMicroseconds(packTimeoutMicros);
          delay(packTimeout);
          if(!Serial.available())
            break;
        }
      }

      // now send to WiFi:
      udp.beginPacket(sendIP, port); // remote IP and port
      udp.write(buf2, i2);
      udp.endPacket();
      i2 = 0;
    }
  }
  #endif
  
  
}
