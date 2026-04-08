// ESP8266main.c
// testmain for ESP8266 module to act as a WiFi client or server
// Currently restricted to one incoming or outgoing connection at a time
//
// Steven Prickett (steven.prickett@gmail.com)
// Modified version by Dung Nguyen, Wally Guzman
// Modified by Jonathan Valvano, March 28, 2017
// Consolidated by Andreas Gerstlauer, April 6, 2020 
// Converted to MSPM0G3507 UART2 by Jonathan Valvano, Jan 18, 2026
/* 
  THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
  OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
  VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
  OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*/

// NOTE: see ESP8266 files in datasheets folder

/* Hardware connections
 Vcc is a separate regulated 3.3V supply with at least 215mA
 /------------------------------\
 |              chip      1   8 |
 | Ant                    2   7 |
 | enna       processor   3   6 |
 |                        4   5 |
 \------------------------------/ 
 Connects MSPM0 
    UART1 on (PA17/PA18) or UART2 on (PB17/PB18)
    Reset on PA25
    Ok to not access PB19 because of the internal pullup in ESP8266
 ESP8266    MSPM0     Motor board version 7
  1 URxD    PA17      UART1 out of MSPM0, into ESP8266 115200 baud
  2 GPIO0             +3.3V for normal operation (ground to flash)
  3 GPIO2   PB19      GPIO, high/float on startup, has internal pullup, can be used for I/O
  4 GND     Gnd       GND (70mA)
  5 UTxD    PA18      UART out of ESP8266, UART1 into MSPM0 115200 baud
  6 Ch_PD             chip select, 10k resistor to 3.3V
  7 Reset   PA25      MSPM0 GPIO output, can issue output low to cause hardware reset
  8 Vcc               regulated 3.3V supply with at least 70mA

 ESP8266    MSPM0     Motor board version 7.1
  1 URxD    PB17      UART2 out of MSPM0, into ESP8266 115200 baud
  2 GPIO0             +3.3V for normal operation (ground to flash)
  3 GPIO2   PB19      GPIO, high/float on startup, has internal pullup, can be used for I/O
  4 GND     Gnd       GND (70mA)
  5 UTxD    PB18      UART out of ESP8266, UART2 into MSPM0 115200 baud
  6 Ch_PD             chip select, 10k resistor to 3.3V
  7 Reset   PA25      MSPM0 GPIO output, can issue output low to cause hardware reset
  8 Vcc               regulated 3.3V supply with at least 70mA
 */
 /* Derived from i2c_controller_rw_multibyte_fifo_poll_LP_MSPM0G3507_nortos_ticlang
  VCC to +3.3V
  I2C SCL: PB2 is the SSD1306 SCL, with 1.5k pullup to 3.3V
  I2C SDA: PB3 is the SSD1306 SDA
  GND to GND
*/

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/SSD1306.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../RTOS_Labs_common/esp8266.h"
#include "../inc/UART.h"



// Client or Server ESP8266 Initialization
// 0 means client, != 0 means server at specified port
//#define SERVER       80   // port 80 is for http

// Transparently forwarding debug mode
// #define TRANSPARENT  1


#if (! TRANSPARENT) && (! SERVER)
// client mode
const char Fetch[] = "GET /data/2.5/weather?q=Austin%20Texas&APPID=e8dc75caa5babe657b849dd5e75a19a9 HTTP/1.1\r\nHost:api.openweathermap.org\r\n\r\n";
// 1) go to http://openweathermap.org/appid#use 
// 2) Register on the Sign up page
// 3) get an API key (APPID) replace the 1234567890abcdef1234567890abcdef with your APPID
char LOGDATA[] ="GET /php/json/write.php?name=Robot13&bump=19&steering=123&right=456&left=678&systick=44&addthread=75&jitter=75 HTTP/1.0\r\nHOST: embedded.ece.utexas.edu\r\n\r\n";

char Response[512];
char Status[16];
uint16_t StartTime,EndTime,ElaspedTime;
const char OpenWeathermap[] ="api.openweathermap.org";
const char Embedded_ece[] ="embedded.ece.utexas.edu";

int main(void){ // main1
uint32_t bump; char *s;
  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();                  // set system clock to 80 MHz
  UART_Init();       // UART0 only used for debugging
  TimerG8_Init(8,250); // 50us   3.2 s
  __enable_irq();

  UART_OutString("\r\nESP8266 ECE445M Test\r\n");
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_DrawString(0, 0,"ESP8266 ECE445M Test",SSD1306_WHITE); SSD1306_OutBuffer();
  if(!ESP8266_Init(true,false)) {  // initialize with rx echo
    SSD1306_DrawString(0,16,"No Wifi adapter",SSD1306_WHITE); SSD1306_OutBuffer();
    UART_OutString("\r\n---No ESP detected\r\n");
    while(1) {}
  }
  UART_OutString("\r\n-----------System starting...\r\n");
  ESP8266_GetVersionNumber();
  if(!ESP8266_Connect(true)) {  // connect to access point
    SSD1306_DrawString(0,16,"No Wifi network",SSD1306_WHITE); SSD1306_OutBuffer();
    UART_OutString("\r\n---Failure connecting to access point\r\n");
    while(1) {}
  }
  SSD1306_DrawString(0,16,"Wifi connected",SSD1306_WHITE);  SSD1306_OutBuffer();
  GPIOB->DOUTSET31_0 = BLUE; 

  // Lab 6 test
  ESP8266_GetStatus(); 
  while(1){
    UART_OutString(LOGDATA);
    bump = 10*(LOGDATA[42]-'0')+(LOGDATA[43]-'0');
    SSD1306_DrawString(0,32,"bump: ",SSD1306_WHITE); 
    SSD1306_DrawUDec(56,32,bump,SSD1306_WHITE); SSD1306_OutBuffer();  
    
    if(ESP8266_MakeTCPConnection((char *)Embedded_ece, 80, 0, false)){ 
      // open socket to web server on port 80
      ESP8266_StartReceiveSearch("status=");
      StartTime = TIMG8->COUNTERREGS.CTR;
      if(ESP8266_Send(LOGDATA)){
 
     //   if(ESP8266_Receive(Response, 512)){  // receive response
//          UART_OutString(Response);
        uint32_t TimeOut=10000000;
        do{ s = ESP8266_GetReceiveBuffer();  // get status
          TimeOut--;
        }while((s==0)&&TimeOut);
        EndTime = TIMG8->COUNTERREGS.CTR;
        ElaspedTime = StartTime-EndTime;
        if(s){
          int i=0;
          while(((*s)!=' ')&&(i<15)){
            Status[i] = *s;
            s++; i++;
          }
          Status[i] = 0;
          SSD1306_DrawString(0,44,"              ",SSD1306_WHITE); 
          SSD1306_DrawString(0,44,Status,SSD1306_WHITE); 
          SSD1306_DrawString(0,56,"Time(ms)      ",SSD1306_WHITE); 
          SSD1306_DrawUDec(56,56,ElaspedTime/20,SSD1306_WHITE);   
          SSD1306_OutBuffer();
        }  
      }
    }
       
    while(LaunchPad_InS2()==0){// wait for S2 touch
    }
    if(LOGDATA[43] == '9'){
      LOGDATA[42] = LOGDATA[42]+1;
      LOGDATA[43] = '0';
    }else{
     LOGDATA[43] = LOGDATA[43]+1;
    }
    GPIOB->DOUTCLR31_0 = GREEN;
    GPIOB->DOUTSET31_0 = BLUE; 
    GPIOB->DOUTTGL31_0 = RED;
  }
}

int main2(void){ uint32_t len; char *s; char *e; int32_t data; 
  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();                  // set system clock to 80 MHz
  UART_Init();       // UART0 only used for debugging
  __enable_irq();

  UART_OutString("\r\nESP8266 GetWeather Test\r\n");
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_DrawString(0, 0,"ESP8266 GetWeather",SSD1306_WHITE); SSD1306_OutBuffer();
  if(!ESP8266_Init(true,false)) {  // initialize with rx echo
    SSD1306_DrawString(0,16,"No Wifi adapter",SSD1306_WHITE); SSD1306_OutBuffer();
    UART_OutString("\r\n---No ESP detected\r\n");
    while(1) {}
  }
  UART_OutString("\r\n-----------System starting...\r\n");
  ESP8266_GetVersionNumber();
  if(!ESP8266_Connect(true)) {  // connect to access point
    SSD1306_DrawString(0,16,"No Wifi network",SSD1306_WHITE); SSD1306_OutBuffer();
    UART_OutString("\r\n---Failure connecting to access point\r\n");
    while(1) {}
  }
  SSD1306_DrawString(0,16,"Wifi connected",SSD1306_WHITE);  SSD1306_OutBuffer();
  GPIOB->DOUTSET31_0 = BLUE; 

  ESP8266_GetStatus(); 
  while(1){
    ESP8266_GetStatus();
    if(ESP8266_MakeTCPConnection((char *)OpenWeathermap, 80, 0, false)){ // open socket to web server on port 80
      if(ESP8266_Send(Fetch)){  // send request 
        SSD1306_DrawString(0,16,"                    ",SSD1306_WHITE); // 20 characters
        SSD1306_DrawString(0,32,"                    ",SSD1306_WHITE); // 20 characters
        SSD1306_DrawString(0,40,"                    ",SSD1306_WHITE); // 20 characters
        SSD1306_OutBuffer();
        if(ESP8266_Receive(Response, 512)){  // receive response
          if(strncmp(Response, "HTTP", 4) == 0) { // received HTTP response?
            SSD1306_DrawString(0,16,"Weather fetched",SSD1306_WHITE); SSD1306_OutBuffer();
            GPIOB->DOUTCLR31_0 = BLUE; 
            GPIOB->DOUTSET31_0 = GREEN; 
            len = 0;    
            while(strlen(Response)) {  // parse HTTP headers until empty line
              if(!ESP8266_Receive(Response, 512)){
                len = 0;
                break;
              }
              if(strncmp(Response, "Content-Length: ", 16) == 0) { 
                len = atol(Response+16); // get HTML body size
              }
            }
            if(len) {   // Get HTML body and parse for weather info
              ESP8266_Receive(Response, (len < 512)? (len+1) : 512);
              s = strstr(Response, "\"temp\":");  // get temperature
              if(s){
                data = atol(s+7);
                SSD1306_DrawString(0,32,"Temp [C]: ",SSD1306_WHITE); 
                SSD1306_DrawUDec(56,32,data-273,SSD1306_WHITE); SSD1306_OutBuffer();       
              }
              s = strstr(Response, "\"description\":"); // get description    
              if(s){
                e = strchr(s+15, '"'); // find end of substring
                if(e){  
                  *e = 0;  // temporarily terminate with zero
                  SSD1306_DrawString(0,40,s+15,SSD1306_WHITE); SSD1306_OutBuffer();
                }
              }
            } else {
              SSD1306_DrawString(0,16,"Empty response",SSD1306_WHITE); SSD1306_OutBuffer();
            }    
          } else {
            SSD1306_DrawString(0,16,"Invalid response",SSD1306_WHITE); SSD1306_OutBuffer();
          }
        } else {
          SSD1306_DrawString(0,16,"No response",SSD1306_WHITE); SSD1306_OutBuffer();
        }
      } else {
        SSD1306_DrawString(0,16,"Send failed",SSD1306_WHITE); SSD1306_OutBuffer();
      }      
      ESP8266_CloseTCPConnection();  // close connection   
    } else { 
      SSD1306_DrawString(0,16,"Connection failed",SSD1306_WHITE); SSD1306_OutBuffer();
    }          
    while(LaunchPad_InS2()==0){// wait for S2 touch
    }
    GPIOB->DOUTCLR31_0 = GREEN;
    GPIOB->DOUTSET31_0 = BLUE; 
    GPIOB->DOUTTGL31_0 = RED;
  }
}
#elif SERVER

/*
======================================================================================================================
==========                                     Simple HTTP SERVER                                           ==========
======================================================================================================================
*/

const char formBody[] = 
  "<!DOCTYPE html><html><body><center> \
<h1>Enter a message to send to your microcontroller:</h1> \
  <form> \
  <input type=\"text\" name=\"message\" value=\"Hello ESP8266!\"> \
  <br><input type=\"submit\" value=\"Go!\"> \
  </form></center></body></html>";

const char statusBody[] = 
  "<!DOCTYPE html><html><body><center> \
  <h1>Message sent successfully!</h1> \
  </body></html>";

/*
===================================================================================================
  HTTP :: HTTP_ServePage  
   - constructs and sends a web page via the ESP8266 server
   - NOTE: this seems to work for sending pages to Firefox (and maybe other PC-based browsers),
           but does not seem to load properly on iPhone based Safari. May need to add some more
           data to the header.
===================================================================================================
*/
void itoa(uint32_t n, char message[8]);
int HTTP_ServePage(const char* body){
  char header[] = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\nContent-Length: ";
    
  char contentLength[16];
  itoa(strlen(body),contentLength);
 // sprintf(contentLength, "%d\r\n\r\n", strlen(body));
  strcat(contentLength, "\r\n\r\n");

  if(!ESP8266_SendBuffered(header)) return 0;
  if(!ESP8266_SendBuffered(contentLength)) return 0;
  if(!ESP8266_SendBuffered(body)) return 0;    
  
  if(!ESP8266_SendBufferedStatus()) return 0;
  
  return 1;
}

char HTTP_Request[64];

int main(void){  
    __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();                  // set system clock to 80 MHz
  UART_Init();       // UART0 only used for debugging
  __enable_irq();
  UART_OutString("\r\nESP8266 Server Test\r\n");
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_DrawString(0, 0,"ESP8266 Server Test",SSD1306_WHITE); SSD1306_OutBuffer();
  if(!ESP8266_Init(true,false)){  // initialize with rx echo
    SSD1306_DrawString(0,16,"No Wifi adapter",SSD1306_WHITE); SSD1306_OutBuffer();
    UART_OutString("\r\n---No ESP detected\r\n");
    while(1) {}
  }
  UART_OutString("\r\n-----------System starting...\r\n");
  ESP8266_GetVersionNumber();
  if(!ESP8266_Connect(true)){  // connect to access point
    SSD1306_DrawString(0,16,"No Wifi network",SSD1306_WHITE); SSD1306_OutBuffer();
    UART_OutString("\r\n---Failure connecting to access point\r\n");
    while(1) {}
  }
  SSD1306_DrawString(0,16,"Wifi connected",SSD1306_WHITE);  SSD1306_OutBuffer();
 

  if(!ESP8266_StartServer(SERVER,600)){  // 5min timeout
    SSD1306_DrawString(0,16,"Server failure",SSD1306_WHITE); SSD1306_OutBuffer();
    UART_OutString("\r\n---Failure starting server\r\n");
    while(1) {}
  }  
  SSD1306_DrawString(0,24,"Server started",SSD1306_WHITE);  SSD1306_OutBuffer();

  GPIOB->DOUTSET31_0 = BLUE; 

  while(1) {
    // Wait for connection
    ESP8266_WaitForConnection();
    SSD1306_DrawString(0,32,"Server started",SSD1306_WHITE);  SSD1306_OutBuffer();
    
    // Receive request
    if(!ESP8266_Receive(HTTP_Request, 64)){
      SSD1306_DrawString(0,40,"No request",SSD1306_WHITE);  SSD1306_OutBuffer();
      ESP8266_CloseTCPConnection();
      continue;
    }
    
    // check for HTTP GET
    if(strncmp(HTTP_Request, "GET", 3) == 0) {
      char* messagePtr = strstr(HTTP_Request, "?message=");
      if(messagePtr) {
        // Clear any previous message
        SSD1306_DrawString(0,40,"                   ",SSD1306_WHITE);  SSD1306_OutBuffer();
        // Process form reply
        if(HTTP_ServePage(statusBody)) {
          SSD1306_DrawString(0,40,"Served status",SSD1306_WHITE);  SSD1306_OutBuffer();
          GPIOB->DOUTSET31_0 = BLUE;
          GPIOB->DOUTCLR31_0 = GREEN;
        } else {
          SSD1306_DrawString(0,40,"Error serving status",SSD1306_WHITE);  SSD1306_OutBuffer();
        }
        // Terminate message at first separating space
        char* messageEnd = strchr(messagePtr, ' ');
        if(messageEnd) *messageEnd = 0;  // terminate with null character
        // Print message on terminal
        SSD1306_DrawString(0,48,messagePtr + 9,SSD1306_WHITE);  SSD1306_OutBuffer();
        UART_OutString("\r\n---Message from the Internet: ");
        UART_OutString(messagePtr + 9);
        UART_OutString("\n\r");
      } else {
        // Serve web page
        if(HTTP_ServePage(formBody)) {
          SSD1306_DrawString(0,40,"Served form",SSD1306_WHITE);  SSD1306_OutBuffer();
          GPIOB->DOUTCLR31_0 = BLUE;
          GPIOB->DOUTSET31_0 = GREEN;
        } else {
          SSD1306_DrawString(0,40,"Error serving form",SSD1306_WHITE);  SSD1306_OutBuffer(); 
        }         
      }        
    } else {
      // handle data that may be sent via means other than HTTP GET
      SSD1306_DrawString(0,40,"Not a GET request",SSD1306_WHITE);  SSD1306_OutBuffer();
    }
    GPIOB->DOUTTGL31_0 = RED;
    ESP8266_CloseTCPConnection();
  }
}
// works for 0 to 999
void itoa(uint32_t n, char message[8]){
  if(n>999)n=999;
  if(n>=100){  // 100 to 999
    message[0] = (n/100+'0'); /* hundreds digit */
    n = n%100; //the rest
    message[1] = (n/10+'0'); /* tens digit */
    n = n%10; //the rest
    message[2] = (n+'0'); /* tenths digit */
    message[3] = 0;
  }else { // 0 to 99
    if(n>=10){ // 10 to 99
      message[0] = (n/10+'0'); /* tens digit */
      n = n%10; //the rest
      message[1] = (n+'0'); /* tenths digit */
      message[2] = 0;
    }else{ // 0 to 9
      message[0] = (n+'0'); /* tenths digit */
      message[1] = 0;
    }
  }
}
#else  // TRANSPARENT

// transparent mode for testing
void ESP8266_SendCommand(char *);
void ESP8266_OutChar(char);
int main(void){  char data;
  DisableInterrupts();
  PLL_Init(Bus80MHz);
  LED_Init();  
  Output_Init();       // UART0 as a terminal
  EnableInterrupts();
  if(!ESP8266_Init(true,false)) {  // initialize with rx echo
    printf("\r\n---No ESP detected\r\n");
    while(1) {}
  }
  printf("\r\n-----------System starting...\r\n");
  ESP8266_Reset();
//  ESP8266_SendCommand("AT+UART=115200,8,1,0,3\r\n");
//  data = UART_InChar();
  
  while(1){
// echo data back and forth
    data = UART_InCharNonBlock();
    if(data){
      ESP8266_OutChar(data);
    }
  }
}

#endif
