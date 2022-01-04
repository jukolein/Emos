#ifdef ESP32  //only the ESP32 has enough power for the NMEA2K-computation, so no need to define it otherwiese 
  #define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5, must be at the very top to work, don't move down
  #define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4, must be at the very top to work, don't move down
#endif
//--------------------------------------------------------
// general and NMEA0183 includes
//--------------------------------------------------------
#include <ArduinoOTA.h>
#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager
#include <Wire.h>
#include <AS5600.h>   //https://github.com/Seeed-Studio/Seeed_Arduino_AS5600

#ifdef ESP32  //in some cases, the ESP32 needs other libraries than the ESP8266
  #include <WiFi.h>
  #include <ESPmDNS.h>
  #include <WebServer.h>
  #include "SPIFFS.h"
  #include <WiFi.h>
    //--------------------------------------------------------
    // NMEA2000 includes
    //--------------------------------------------------------
    #include <Preferences.h>
    #include <NMEA2000_CAN.h>  //This will automatically choose the right CAN library and create a suitable NMEA2000 object
    #include <N2kMessages.h>  //includes all the code needed to create the NMEA2000-messages
#else
  #include <ESP8266WiFi.h> // ESP8266 library, http://arduino.esp8266.com/stable/pa...com_index.json
  #include <ESP8266mDNS.h>
  #include <ESP8266WebServer.h>
  #include <FS.h>
#endif


//******************************************************************************************************************************************************
// general and NMEA0813 variables in global scope
//******************************************************************************************************************************************************

#define DELAY_MS 1000 //MS to wait in the loop function
#define DELAY_COUNT 1 //Counter of DELAY_MS to wait, 3600 is 1 hour
#define TCP_PORT 8080 //Port to connect for to receive the NMEA sentences via TCP
#define HTTP_PORT 80  //Port to reach the HTTP webserver 
#define MAX_CLIENTS 3 //maximal number of simultaneousy connected clients

#define LED 2  //at ESP32_dev_kit, LED is at pin 2, also at the ESP8266

AMS_5600 ams5600;  //select correct chip

String sensor = "";  //string for storing the type of the sensor, mainly needed for the callibration of the wind speed

double WS = 0;  //stores the value of the wind speed
int Z = 0;  //stores the angle of the wind direction in degrees
double rate_of_rotation = 0;  //stores how often the weel rotates in one second

String numReadings_angle;  //number of values used to calculate the moving average of the angle
int readings_angle[500];  //array for storing the angle-values in. As it is not at all trivial to change the size if an array during runtime, it is fixed at 500
int readIndex_angle = 0;  //stores the position in the array
long total_angle = 0;  //sum of the entries 0 to numReadings
   
String numReadings_speed;  //number of values used to calculate the moving average of the wind speed
double readings_speed[500];  //array for storing the wind speed-values in. As it is not at all trivial to change the size if an array during runtime, it is fixed at 500
int readIndex_speed = 0;  //stores the position in the array
double total_speed = 0;  //sum of the entries 0 to numReadings

char CSSentence[30] = "" ; //used for handing the MWV-sentence over to the checksum-function
char MWVSentence[30] = "" ; //used for MWV-sentence assembly (Wind Speed and Angle)
char e[12] = "0";  //wind angle in sentence assembly
char c[12] = "0";  //wind speed in sentece assembly
char g[80] = "0";  //the calculated checksum of the MWV-sentence
char result[60] = "0";  //used for the final NMEA-assembly of the MWV-sentence (dollar symbol, MWV, CS)
volatile long times = millis();  //used to calculate the time between two status changes, neede for the wind speed
long oldtimes = millis();   //used to calculate the time between two status changes, needed for the wind speed

String offset;  //user defined offset of the wind angle
String factor;  //user defined liear correction factor of the wind speed
String blIPblink;  //stores if the IP is to be blinked or not. "geblinkt" equals "true", "nicht geblinkt" equals "false"
String ws_unit;  //stores the user deifned unit of the wind speed, like knots or meters per second. Only affects the graphical display
String language;  //stores the user defined display language
String ap;  //stores if an acces point should be created

#ifdef ESP32

  //--------------------------------------------------------
  // NMEA2000 variables in global scope
  //--------------------------------------------------------

  String nmea2k;  //stores if NMEA2000 should be processed and send or not
  int NodeAddress;  //stores last Node Address
  Preferences preferences;  //stores LastDeviceAddress

  const unsigned long TransmitMessages[] PROGMEM = {13030L,  //Set the information for other bus devices, which messages we support, here: wind (13030). see: https://www.nmea.org/Assets/20151026%20nmea%202000%20pgn_website_description_list.pdf
                                                    0
                                                    };


  //--------------------------------------------------------
  // variables for EMOS temperature and humidity sensors in 
  // global scope
  //--------------------------------------------------------
    String hum_temp; //stores if humidity and temperature should be measuread and processed

    TaskHandle_t Task1;  //needed to handle the calculation-task that gets sheduled to te other core 
    #define  DRIVE1  32  //first pin for the generation of the AC voltage
    #define  DRIVE2  33  //second pin for the generation of the AC voltage
    #define  HSENSE  34  //analog input of the hygrometer
    #define  TSENSE  35  //analog input of the thermistor

    #define  PERIOD_US 1000  //for 1kHz frequence of the AC 
    #define  HALF_PERIOD_US (PERIOD_US/2)  //half the value of PERIOD_US

    unsigned long prev_time;  //Used to generate the pure AC square wave for the hygrometer 
    byte phase = 0;  //like prev_time, used for the generation of AC squar wave 
      
    const double R1 = 4700000;  //4,7M Ohm resistor as voltage divider for the hygrometer
    const double R2 = 10000;  //10K Ohm resistor as voltage divider for the thermistor 
    const double VCC = 3.3;  // ESP pin voltage is 3.3V
    
    const double adc_resolution = 4095; // ESP allows for an ADC-resolution of 12-bit (0-4095)
    double adc_temp_value;  //double for storing the ADC-read of the thermistor
    
    const double A = 0.001129148;  //thermistor equation parameter
    const double B = 0.000234125;  //thermistor equation parameter
    const double C = 0.0000000876741; //thermistor equation parameter
    double Vout_temp;  //voltage output of the thermistor
    double R_temp;  //calculated resistance of the thermistor
    double temperature;  //final temperature value
    double public_temperature;  //hands over the temerature value from one core to the other. Is only updated after all the calculation is done to avoid dirty reads
    float prevVoltage = 0.0;  //last voltage value, to create a moving average 
    int printMe = 0;  //couter to limit logging of values 
    double hum;  //humidity value
    double public_hum;  ///hands over the temerature value from one core to the other. Is only updated after all the calculation is done to avoid dirty reads
    
    char XDRSentence[300] = "" ; //string for XDR-sentence assembly (temperature and humidity)
    char temp_char[50] = "0";   //temperature for the sentence assembly
    char hum_char[120] = "0";  //humidity for the sentence assembly
    char hum_temp_check[800] = "0";  //calculated checksum of the XDR-sentence
    char hum_temp_result[600] = "0";  //string for NMEA-assembly of the XDR-sentence (dollar symbol, XDR, CS)

    String numReadings_temp;  //number of values used to calculate the moving average of the temperature
    double readings_temp[500];  //array for storing the temperature-values in. As it is not at all trivial to change the size if an array during runtime, it is fixed at 500
    int readIndex_temp = 0;  //stores the position in the array
    double total_temp = 0;  //sum of the entries 0 to numReadings
    
    String numReadings_humidity;  //number of values used to calculate the moving average of the humidity
    double readings_humidity[500];  //array for storing the humidity-values in. As it is not at all trivial to change the size if an array during runtime, it is fixed at 500
    int readIndex_humidity = 0;  //stores the position in the array
    double total_humidity = 0;  //sum of the entries 0 to numReadings
    
    String correction_factor_temp;  //user defined correction factor to the temperature
    String offset_temp;  //user defined offset to the temperature
    String correction_factor_humidity;  //user defined correction factor to the humidity
    String offset_humidity;  //user defined offset to the humidity
    
#endif
                                                  
//--------------------------------------------------------
// ESP8266 WiFi and webserver variables
//--------------------------------------------------------

WiFiServer server(TCP_PORT); //define the server port
WiFiClient clients[MAX_CLIENTS]; //Array of clients

#ifdef ESP32
  WebServer webserver(HTTP_PORT); //define the HTTP-webserver port
#else
  ESP8266WebServer webserver(HTTP_PORT); //define the HTTP-webserver port
#endif

//******************************************************************************************************************************************************
// HTML PAGES
//******************************************************************************************************************************************************

//--------------------------------------------------------
// correction_page - the HTML code for the page where
// the correction values are set
// located in PROGMEM, as it is going to be static
//--------------------------------------------------------

const char correction_page_de[] PROGMEM = R"=====(
<!DOCTYPE html>
<head>
  <meta content="text/html; charset=UTF-8" http-equiv="content-type">
<style>
  body{ 
  background-image:url("data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSI1IiBoZWlnaHQ9IjUiPgo8cmVjdCB3aWR0aD0iNSIgaGVpZ2h0PSI1IiBmaWxsPSIjZmZmIj48L3JlY3Q+CjxyZWN0IHdpZHRoPSIxIiBoZWlnaHQ9IjEiIGZpbGw9IiNjY2MiPjwvcmVjdD4KPC9zdmc+");">
}
</style>
</head>
<html>
<body>
<h1 style="text-align: center;">sensor_placeholder<h1>
<h3>Einstellungen</h3>
<br>  <br>
Aktuell ausgewählter Windsensor-Typ: sensor_placeholder 
   <input type="button" onclick="location.href='/sensor_jukolein';" value="Jukolein" />
   <input type="button" onclick="location.href='/sensor_ventus';" value="Ventus" />
   <input type="button" onclick="location.href='/sensor_emos';" value="Emos" />
<br>  <br>
<form action="/action_page">
  Vorzeichenbehafteter Offset der Windrichtung in Grad: <input type="text" name="offset_html" value="offset_placeholder">
  <br>  <br>
  linearer Korrekturfaktor der Windgeschwindigkeit: <input type="text" name="factor_html" value="factor_placeholder">
  <br>  <br>
  Mittlung der Windrichtung über <input type="text" name="numReadings_angle_html" value="numReadings_angle_placeholder"> Werte
  <br>  <br>
  Mittlung der Windgeschwindigkeit über <input type="text" name="numReadings_speed_html" value="numReadings_speed_placeholder"> Werte
  <br>  <br>
  <input type="submit" value="Anwenden">
</form> 
<br>  <br>
Aktuelle Einheit der Windgeschwindigkeit: ws_unit_placeholder 
   <input type="button" onclick="location.href='/meters';" value="Meter pro Sekunde" />
   <input type="button" onclick="location.href='/kilometers';" value="Kilometer pro Stunde" />
   <input type="button" onclick="location.href='/knots';" value="Knoten" />
<br>  <br>
   Aktuell wird die IP beim Start ipblink_placeholder <input type="button" onclick="location.href='/ipblink';" value="Ändern" />
<br>  <br>
   Beim nächsten Start ap_placeholder <input type="button" onclick="location.href='/ap';" value="Ändern" />
<br>  <br>
   nmea2k_placeholder
   
   Sprache / Language
   <input type="button" onclick="location.href='/language_en';" value="English" />
   <input type="button" onclick="location.href='/language_de';" value="Deutsch" />
<br>  <br>
    EMOS_placeholder
<h3>  <a href='/'> Zur Windanzeige </a> </h3>

</body>
</html>
)=====";

const char correction_page_en[] PROGMEM = R"=====(
<!DOCTYPE html>
<head>
  <meta content="text/html; charset=UTF-8" http-equiv="content-type">
<style>
  body{ 
  background-image:url("data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSI1IiBoZWlnaHQ9IjUiPgo8cmVjdCB3aWR0aD0iNSIgaGVpZ2h0PSI1IiBmaWxsPSIjZmZmIj48L3JlY3Q+CjxyZWN0IHdpZHRoPSIxIiBoZWlnaHQ9IjEiIGZpbGw9IiNjY2MiPjwvcmVjdD4KPC9zdmc+");">
}
</style>
</head>
<html>
<body>
<h1 style="text-align: center;">sensor_placeholder<h1>
<h3>Settings</h3>
Currently selected sensor type: sensor_placeholder 
   <input type="button" onclick="location.href='/sensor_jukolein';" value="Jukolein" />
   <input type="button" onclick="location.href='/sensor_ventus';" value="Ventus" />
   <input type="button" onclick="location.href='/sensor_emos';" value="Emos" />
<br>  <br>
<form action="/action_page">
  signed offset of the wind direction in degrees: <input type="text" name="offset_html" value="offset_placeholder">
  <br>  <br>
  linear correction factor: <input type="text" name="factor_html" value="factor_placeholder">
  <br>  <br>
  The wind direction will be the moving average of <input type="text" name="numReadings_angle_html" value="numReadings_angle_placeholder"> values
  <br>  <br>
  The wind speed will be the moving average of <input type="text" name="numReadings_speed_html" value="numReadings_speed_placeholder"> values
  <br>  <br>
  <input type="submit" value="Apply">
</form>
<br>  <br>
current wind speed unit: ws_unit_placeholder 
   <input type="button" onclick="location.href='/meters';" value="meters per second" />
   <input type="button" onclick="location.href='/kilometers';" value="kilometres per hour" />
   <input type="button" onclick="location.href='/knots';" value="knots" />
<br>  <br>
At the next boot, the IP of the sensor will ipblink_placeholder <input type="button" onclick="location.href='/ipblink';" value="Change" />
<br>  <br>
At the next boot ap_placeholder <input type="button" onclick="location.href='/ap';" value="Change" />
<br>  <br>
   nmea2k_placeholder
   
 Language / Sprache
   <input type="button" onclick="location.href='/language_en';" value="English" />
   <input type="button" onclick="location.href='/language_de';" value="Deutsch" />
<br>  <br>
    EMOS_placeholder
<br>  <br>
 <h3>  <a href='/'> To the graphic display </a> </h3>
</body>
</html>

)=====";


//--------------------------------------------------------
// graphic - the HTML code for the page where 
// the wind data is displayed graphically
// located in PROGMEM, as it is going to be static
//--------------------------------------------------------

// MANY THANKS TO https://ziegenhagel.com FOR WRITING THE CODE

const char graphic[] PROGMEM = R"=====(
<div class="pos_upper_right">
    <a href="/config">Einstellungen</a>
</div>
<div id="panels">
    <div id="instrument" class="layer">
        <div id="wind_direction" class="layer">--- deg</div>
        <div id="wind_speed" class="layer">--- m/s</div>
        <div id="nesw" style="margin-top:-5px">
            <div class="layer" style="transform:rotate(0deg)">0</div>
            <div class="layer" style="transform:rotate(90deg)">90</div>
            <div class="layer" style="transform:rotate(180deg)">
                <div style="transform:rotate(180deg);">180</div>
            </div>
            <div class="layer" style="transform:rotate(270deg)">270</div>
        </div>
        <div id="degs"> </div>
        <div id="pointer" class="layer">
            <div id="needle"></div>
        </div>
        <div id="dot_on_needle" class="layer"></div>
        <div id="reflection"
            style="background: linear-gradient(#2223, #2220); height: 140%; width: 200%; margin-left: -50%; margin-top: 41%; opacity: 0.5; "
            class="layer"></div>
    
        <div id="sideboxes">
            <div class="box-panel">
                <div class="box-heading">Temperatur</div>
                <div id="temperature" class="box">21 &deg;C</div>
            </div>
            <div class="box-panel">
                <div class="box-heading">Luftfeuchtigkeit</div>
                <div id="humidity" class="box">90%</div>
            </div>
        </div>
    </div>
</div>
<script>
    // version 0.4
    // config, change to your needs
    const API_URL = "/data"
    const API_ENABLED = true
    const SIDEBOXES_ENABLED = true
    const API_REFRESH_MS = 1000

    // code, don't touch
    let el_pointer = document.getElementById("pointer")
    let el_direction = document.getElementById("wind_direction")
    let el_speed = document.getElementById("wind_speed")
    let el_temperature = document.getElementById("temperature")
    let el_humidity = document.getElementById("humidity")
    let el_sideboxes = document.getElementById("sideboxes")
    let degs = document.getElementById("degs")
    var el, rot;
    var hum, temp;

    if(!SIDEBOXES_ENABLED) el_sideboxes.style.display="none"

    function refresh_gui(data) {
        rotateThis(el_pointer, data[1])
        el_direction.innerHTML = data[1]+"&deg;"
        el_temperature.innerHTML = parseFloat(data[7]).toFixed(1)+" &deg;C"
        el_humidity.innerHTML = data[11]+"%"
        el_speed.innerHTML = (parseFloat(data[3] * unit_manipulation
    }

    function rotateThis(element, nR) {
        var aR;
        rot = rot || 0; // if rot undefined or 0, make 0, else rot
        aR = rot % 360;
        if (aR < 0) { aR += 360; }
        if (aR < 180 && (nR > (aR + 180))) { rot -= 360; }
        if (aR >= 180 && (nR <= (aR - 180))) { rot += 360; }
        //  rot += (nR - aR);
        rot += (nR - aR);
        element.style.transform = ("rotate( " + rot + "deg )");
    }
    
    function build_gui() {
        let color
        for(let i=0;i<360; i+=10) {
            if(i%90 != 0) {
                if(i > 0 && i < 60) color="green"
                else if(i < 360 && i > 300) color="red"
                else color="gray"
                degs.innerHTML += '<div class="layer" style="transform:rotate('+i+'deg);"><span style="background-color:'+color+';"></span></div>' 
            }
        }
    }
    function refresh_data() {
        fetch(API_URL)
        .then((data) => data.text())
        .then((data) => {
            console.log("API response:",data)
            refresh_gui(data.split(","))
        })
        .catch((error) => {
            console.error("Error:", error)
        })
    }
    build_gui()
    if(API_ENABLED)
        setInterval(refresh_data,API_REFRESH_MS)
    else
        refresh_gui("$WIMWV,153,R,0.0,M,A*39$WIXDR,C,19.49,C,EMOS_TEMP,H,52,P,EMOS_HUM,*14".split(","))
</script>
<style>
    body {
        display:flex;
        justify-content:center;
        align-items:center;
        background:#222;
    }
    #degs {
        margin-top:5px;
    }
    #degs > div {
        padding-top:0px;
        color:#fff2;
        font-weight:bold;
    }
    #degs > div > span {
        height:21px;
        width:10px;
        display:inline-block;
    }
    #nesw > div {
        padding-top:11px;
        color:#ccc;
    }
    #sideboxes {
        margin-left:500px;
        position: relative;
    }
    #instrument {
        position:relative;
        background:linear-gradient(#222,#555);
        border:6px solid gray;
    }
    .layer {
        font-size:30px;
        text-align:center;
        width:400px;
        height:400px;
        position:absolute;
        border-radius:50%;
    }
    #pointer {
        transition:1s;
    }
    #needle {
        background:linear-gradient(#d00,#a00);
        width:9px;
        height:32%;
        border-radius:4px;
        margin:auto;
        margin-top:-6%;
        box-shadow:5px 5px 8px #333;
    }
    #dot_on_needle {
        border-radius:50%;
        width:40px;height:40px;
        margin:-20px;
        left:50%;
        top:50%;
        background:#555;
        box-shadow:2px 2px 3px #1113, inset -3px -3px 10px #0005, inset 3px 3px 10px #fff3;
    }
    #wind_speed, #wind_direction, .box {
        font-family:arial;
        background: #B1BAA9;
        padding:10px;
        height:auto;
        width:110px;
        left:50%;
        margin-left:-65px;
        top:23%;
        box-shadow: inset 2px 2px 3px #0007, inset -2px -2px 3px #fff3;
        border-radius:2px;
    }
    .box-panel {  margin-top:40px}
    .box-heading { margin-left:0; margin-bottom:10px; color: #ccc;font-family:arial}
    .box { margin-left:0; margin-bottom:80px;width:140px}
    #wind_speed {
        top:63%;
        margin-top:10px;
    } .pos_upper_right {
        padding:10px;
        position:absolute;
        right:0;
        top:0;
    } .pos_upper_right a {
        color:gray;
        font-family:arial;
        text-decoration:none;
    } .pos_upper_right a:hover {
        color:white;
        text-decoration:underline;
    }
   </style>
)=====";


//--------------------------------------------------------
// EMOS - if needed, adds the EMOS-related content to the
// HTML-page. Two versions, english and german.
// located in PROGMEM, as it is going to be static
//--------------------------------------------------------

const char EMOS_de[] PROGMEM = R"=====(
Temperatur und Luftfeuchtigkeit werden hum_temp_placeholder <input type="button" onclick="location.href='/hum_temp';" value="Ändern" />
<br>  <br>
<form action="/action_page"> 
  Mittelwertbildung der Temperatur über <input type="text" name="numReadings_temp_html" value="numReadings_temp_placeholder"> Werte
<br>  <br> 
  Mittelwertbildung der Luftfeutigkeit über <input type="text" name="numReadings_humidity_html" value="numReadings_humidity_placeholder"> Werte  
<br>  <br>
  Korrekturfaktor für die Temperatur: <input type="text" name="correction_factor_temp_html" value="correction_factor_temp_placeholder">
<br>  <br> 
  Offset für die Temperatur <input type="text" name="offset_temp_html" value="offset_temp_placeholder"> 
<br>  <br>
  Korrekturfaktor für die Luftfeuchtigkeit: <input type="text" name="correction_factor_humidity_html" value="correction_factor_humidity_placeholder">
<br>  <br> 
  Offset für die Luftfeuchtigkeit <input type="text" name="offset_humidity_html" value="offset_humidity_placeholder"> 
<br>  <br>
<input type="submit" value="Anwenden"> </form> <br>  <br>
)=====";

const char EMOS_en[] PROGMEM = R"=====(
Temperature and humidity are hum_temp_placeholder <input type="button" onclick="location.href='/hum_temp';" value="Change" />
<br>  <br>
<form action="/action_page"> 
  The temperature will be the moving average of <input type="text" name="numReadings_temp_html" value="numReadings_temp_placeholder"> values
<br>  <br> 
  The humidity will be the moving average of <input type="text" name="numReadings_humidity_html" value="numReadings_humidity_placeholder"> values  
<br>  <br>
  Correction factor for the temperature: <input type="text" name="correction_factor_temp_html" value="correction_factor_temp_placeholder">
<br>  <br> 
  Offset for the temperature <input type="text" name="offset_temp_html" value="offset_temp_placeholder"> 
<br>  <br>
  Correction factor for the humidity: <input type="text" name="correction_factor_humidity_html" value="correction_factor_humidity_placeholder">
<br>  <br> 
  Offset for the humidity: <input type="text" name="offset_humidity_html" value="offset_humidity_placeholder"> 
<br>  <br>
<input type="submit" value="Apply"> </form> <br>  <br>
)=====";


//******************************************************************************************************************************************************
// Support functions for the core code and calculation of data and NMEA0813
//******************************************************************************************************************************************************

//--------------------------------------------------------
// ip_blink - Blinks the IP address of the ESP with the build
// in LED, so you can connect to it without having to scan
// the network or connecting via serial.
// Syntax: It blinks ever digit seperatly, and, in order to
// display the 0, it blinks n+1 times. The dot is indicated
// by three rapid flashes.
// Example: "192." will be displayed as:
// 2 FLASHES, BREAK, 10 FLASHES, BREAK, 3 FLASHES, 3 RAPID FLASHES
//--------------------------------------------------------

void ip_blink() {
  String ip = WiFi.localIP().toString();  //sore current IP in string
  for (int i = 0; i <= ip.length(); i++) {  //for the length of the IP
    if (ip.charAt(i) == '.') {  //check for dots and indicate them by three rapid flashes
      for (int i = 0; i <= 2; i++) {
        digitalWrite(LED, LOW);   //turn the LED on (HIGH is the voltage level)
        delay(100);                       //wait for a second
        digitalWrite(LED, HIGH);   //turn the LED on (HIGH is the voltage level)
        delay(100);                       //wait for a second
      }
    }
    else {
      int times_blink = ip.charAt(i) - '0';  //store the digit at the current position as int
      //    Serial.print(times);
      for (int j = 0; j <= times_blink; j++) {
        digitalWrite(LED, LOW);   //turn the LED on (HIGH is the voltage level)
        delay(200);                       //wait for a second
        digitalWrite(LED, HIGH);    //turn the LED off by making the voltage LOW
        delay(200);
      }
    }
    delay(1000);  //long wait between the digits
  }
}


//--------------------------------------------------------
// nmea_checksum - create NMEA 0183checksum
// Expecting NMEA string that begins with '$' and ends with '*'
// like $WIMWV,152,R,5.4,M,A*
//--------------------------------------------------------

String checksum(char* CSSentence) {
  int cs = 0; //stores the generated dezimal-checksum
  char f[30] = "0";  //stores the final hex-checksum
  for (int n = 0; n < strlen(CSSentence); n++) {
    cs ^= CSSentence[n]; //calculates the checksum
  }
  sprintf(f, "%02X", cs);  //convert the checksum into hex
  return f;
}


//--------------------------------------------------------
// WinSensInterupt - measure the time between the inerrupts
// As this function runs whenever an interrupt is triggered,
// it needs to have the "ICACHE_RAM_ATTR" at its beginning,
// so it can run in RAM
//--------------------------------------------------------

ICACHE_RAM_ATTR void WinSensInterupt() {
  digitalWrite(LED, HIGH);
  if ((millis() - oldtimes) > 50) {  //debounce the signal
    times = millis() - oldtimes;  //get time delta
    oldtimes = millis();  //set oldtimes to current time
  }
  digitalWrite(LED, LOW);
}


//--------------------------------------------------------
// Function: convertRawAngleToDegrees
// In: angle data from AMS_5600::getRawAngle
// Out: human readable degrees as float
// Description: takes the raw angle and calculates
// float value in degrees.
//--------------------------------------------------------

float convertRawAngleToDegrees(word newAngle) {
  float retVal = newAngle * 0.087;  //Raw data reports 0 - 4095 segments, which is 0.087 of a degree
  int ang = 360-retVal;
  return ang;
}


//******************************************************************************************************************************************************
// Support functions for the HTTP-webserver and the correction-values
//******************************************************************************************************************************************************

//--------------------------------------------------------
// readFile - return a string with the information stored 
// in the txt-file that was handed over to the function
//--------------------------------------------------------

String readFile(fs::FS &fs, const char * path){
  #ifdef ESP32
    File file = SPIFFS.open(path);  //open the named file with reading-rights
  #else
    File file = fs.open(path, "r");  //open the named file with reading-rights
  #endif
  if(!file || file.isDirectory()){  //check if there is a file to read from
    return String();
  }
  String fileContent;  //string to store the content of the file in
  while(file.available()){
    fileContent+=String((char)file.read());  //itterate through the file and store the findings in the string
  }
  file.close();
  return fileContent;
}


//--------------------------------------------------------
// writeFile - persistently store a given correction-value
// in a given txt-file
//--------------------------------------------------------

void writeFile(fs::FS &fs, const char * path, const char * message){
  #ifdef ESP32
    File file = SPIFFS.open(path, FILE_WRITE);  //open the named file with writing-rights
  #else
    File file = fs.open(path, "w");  //open the named file with writing-rights
  #endif
  if(!file){  //check if there is a file to write to
    return;
  }
  if(file.print(message)){
  file.close();
  }
    //after writing the new value, the global variable with the correction-value
    //needs to be updated, so the changes take effect immediadly and don't
    //require a reboot or a permanent check in the loop()-function.
    //for simplicity, all values are updated. As the change of the values
    //should only rarely happen, that's not a big deal
    offset = readFile(SPIFFS, "/offset.txt");  //update the offset-value
    factor = readFile(SPIFFS, "/factor.txt");  //upate the factor-value
    blIPblink = readFile(SPIFFS, "/blipblink.txt");  //update the blIPblink state
    ws_unit = readFile(SPIFFS, "/ws_unit.txt");  //update the blIPblink state
    ap = readFile(SPIFFS, "/ap.txt");  //load the persistent data as variable
    sensor = readFile(SPIFFS, "/sensor.txt"); //update the sensor type
    numReadings_angle = readFile(SPIFFS, "/numReadings_angle.txt"); //update the number of values for the wind angle average 
    numReadings_speed = readFile(SPIFFS, "/numReadings_speed.txt");  //update the number of values for the wind speed average 
    #ifdef ESP32
      nmea2k = readFile(SPIFFS, "/nmea2k.txt"); //update the nmea2k-state
      hum_temp = readFile(SPIFFS, "/hum_temp.txt"); //update the hum_temp state
      numReadings_temp = readFile(SPIFFS, "/numReadings_temp.txt"); //update the number of values for the temp average 
      numReadings_humidity = readFile(SPIFFS, "/numReadings_humidity.txt"); //update the number of values for the humidity average 
      correction_factor_temp = readFile(SPIFFS, "/correction_factor_temp.txt"); //update the correction factor for the temperature
      offset_temp = readFile(SPIFFS, "/offset_temp.txt"); //update the offset for the humidity
      correction_factor_humidity = readFile(SPIFFS, "/correction_factor_humidity.txt"); //update the correction factor for the humidity
      offset_humidity = readFile(SPIFFS, "/offset_humidity.txt"); //update the offset for the humidity
    #endif
   }


//--------------------------------------------------------
// handleCorrection - send the HTML-code for the  
// correction page to the client and display the current
// values
//--------------------------------------------------------

void handleCorrection() {
  
  if(language.equals("de")){
    String s = correction_page_de; //choose the german version, if langauge is set to "de"
    s.replace("offset_placeholder", offset);  //display current offset in the form
    s.replace("factor_placeholder", factor);  //display current factor in the form
    s.replace("ipblink_placeholder", blIPblink);  //display in the form if IP is going to be blinked at next boot
    s.replace("ws_unit_placeholder", ws_unit);  //display the unit of the windspeed
    s.replace("ap_placeholder", ap); //display if windsesnor will crate AP or connect to WiFi
    if(sensor.equals("Emos")){  //if sensor type is set to "Emos", display the german emos-related HTML-code
      s.replace("EMOS_placeholder",EMOS_de);
    }
    else{
      s.replace("EMOS_placeholder","");  //if not, leave empty
    }
    s.replace("numReadings_angle_placeholder", numReadings_angle);  //display current number of values used for the moving average of the wind angle
    s.replace("numReadings_speed_placeholder", numReadings_speed);  //display current number of values used for the moving average of the wind speed
    
    #ifdef ESP32
      s.replace("hum_temp_placeholder", hum_temp); //display if windsesnor will measure humidity and temperature or not
      s.replace("numReadings_temp_placeholder", numReadings_temp);  //display current number of values used for the moving average of the temperature
      s.replace("numReadings_humidity_placeholder", numReadings_humidity);  //display current number of values used for the moving average of the humidity
      s.replace("correction_factor_temp_placeholder", correction_factor_temp);  //display current correction factor of the temperature
      s.replace("offset_temp_placeholder", offset_temp);  //display current offset of the temperature
      s.replace("correction_factor_humidity_placeholder", correction_factor_humidity);  //display current correction factor of the humidity
      s.replace("offset_humidity_placeholder", offset_humidity);  //display current offset of the humidity
      s.replace("sensor_placeholder", sensor);  //display currently selected sensor type
      s.replace("nmea2k_placeholder", nmea2k); //display if NMEA2000 should be processed and send
    #else
      s.replace("nmea2k_placeholder", "");  //display nothing, if not ESP32, as ESP8266 can not process NMEA2000
    #endif
    
    webserver.send(200, "text/html", s);  //send modified HTML to the client
    }
  else {  
    String s = correction_page_en; //else, choose the english version
    s.replace("offset_placeholder", offset);  //display current offset in the form
    s.replace("factor_placeholder", factor);  //display current factor in the form
  
    if (blIPblink.equals("nicht geblinkt")) {
      s.replace("ipblink_placeholder", "not be blinked");  //display in the form if IP is going to be blinked at next boot
    }
    else {
      s.replace("ipblink_placeholder", "be blinked");  //display in the form if IP is going to be blinked at next boot
    }
  
    if(ws_unit.equals("Kilometer pro Stunde")) {
    s.replace("ws_unit_placeholder", "kilometres per hour");  //display the unit of the windspeed
    }
    else if (ws_unit.equals("Meter pro Sekunde")) {
      s.replace("ws_unit_placeholder", "metres per second");  //display the unit of the windspeed
    }
    else if(ws_unit.equals("Knoten")) {
      s.replace("ws_unit_placeholder", "knots");  //display the unit of the windspeed
    }
  
    if(ap.equals("befindet sich der Windsensor im Acces-Point-Modus")) {
      s.replace("ap_placeholder", "the wind sensor will create an access point");  //display that AP will be created
    }
    else {
        s.replace("ap_placeholder", "the wind sensor will connect to an existing network"); //display that AP will not be created
   }
   
   s.replace("numReadings_angle_placeholder", numReadings_angle);  //display current number of values used for the moving average of the wind angle
   s.replace("numReadings_speed_placeholder", numReadings_speed);  //display current number of values used for the moving average of the wind speed
   s.replace("sensor_placeholder", sensor);  //display currently selected sensor type
   
   if(sensor.equals("Emos")){
      s.replace("EMOS_placeholder", EMOS_en);  //display the english emos-related HTML-code
    }
    else{
      s.replace("EMOS_placeholder","");  //don't display the english emos-related HTML-code
    }
   
   #ifdef ESP32
      if(hum_temp.equals("gemessen")) {
        s.replace("hum_temp_placeholder", "measured");  //display that temperature and humidity will be measured
      }
      else {
          s.replace("hum_temp_placeholder", "not beeing measured"); //display that temperature and humidity will not be measured
      }
      s.replace("numReadings_temp_placeholder", numReadings_temp);  //display current number of values used for the moving average of the temperature
      s.replace("numReadings_humidity_placeholder", numReadings_humidity);  //display current number of values used for the moving average of the humidity
      s.replace("correction_factor_temp_placeholder", correction_factor_temp);  //display current correction factor of the temperature
      s.replace("offset_temp_placeholder", offset_temp);  //display current offset of the temperature
      s.replace("correction_factor_humidity_placeholder", correction_factor_humidity);  //display current correction factor of the humidity
      s.replace("offset_humidity_placeholder", offset_humidity);  //display current offset of the humidity

     if(nmea2k.equals("NMEA2000-Daten werden erstellt und ausgegeben <input type=\"button\" onclick=\"location.href='/nmea2k';\" value=\"Ändern\" /> <br>  <br>")) {  
       s.replace("nmea2k_placeholder", "NMEA2000-datagrams will be created and send <input type=\"button\" onclick=\"location.href='/nmea2k';\" value=\"Change\" /> <br>  <br>");  //display, that NMEA2000 will be calculated and send
     }
     else {
      s.replace("nmea2k_placeholder","NMEA2000-datagrams will not be created or send <input type=\"button\" onclick=\"location.href='/nmea2k';\" value=\"Change\" /> <br>  <br>");  //display, that NMEA2000 will not be calculated and send
     }
   #else
     s.replace("nmea2k_placeholder", "");  //if not on an ESP32, don't display the above NMEA2000-section
   #endif
   webserver.send(200, "text/html", s);  //send out the altered HTML page
  }
}

//--------------------------------------------------------
// handleForm - store the correction-values handed over by 
// the webpage into the permanent SPIFFS-storage of the ESP
//--------------------------------------------------------

void handleForm() {

  if(not((webserver.arg("offset_html")).equals(""))){  //only update if the field was not empty 
    String offset_html = webserver.arg("offset_html"); //read the value after "offset_html" of the HTTP-GET-request send by the client
    writeFile(SPIFFS, "/offset.txt", offset_html.c_str());  //persistently store it in a text file in the SPIFFS-section
  }
 
  if(not((webserver.arg("factor_html")).equals(""))){  //only update if the field was not empty 
    String factor_html = webserver.arg("factor_html"); //read the value after "factor_html" of the HTTP-GET-request send by the client
    writeFile(SPIFFS, "/factor.txt", factor_html.c_str());  //persistently store it in a text file in the SPIFFS-section
  }

  if(not((webserver.arg("numReadings_angle_html")).equals(""))){
    String numReadings_angle_html = webserver.arg("numReadings_angle_html"); //read the value after "numReadings_angle_html" of the HTTP-GET-request send by the client
    writeFile(SPIFFS, "/numReadings_angle.txt", numReadings_angle_html.c_str());  //persistently store it in a text file in the SPIFFS-section
    for(int i = 0; i<500;i++){  //to avoid errors in the calculation of the moving average, overwrite everything with 0
      readings_angle[i]=0;
    }
    total_angle = 0;  //reset the array index
    readIndex_angle = 0;  //reset the sum
  }
  
  if(not((webserver.arg("numReadings_speed_html")).equals(""))){
    String numReadings_speed_html = webserver.arg("numReadings_speed_html"); //read the value after "numReadings_speed_html" of the HTTP-GET-request send by the client
    writeFile(SPIFFS, "/numReadings_speed.txt", numReadings_speed_html.c_str());  //persistently store it in a text file in the SPIFFS-section
    for(int i = 0; i<500;i++){
      readings_speed[i] = 0;  //to avoid errors in the calculation of the moving average, overwrite everything with 0
    }
    total_speed = 0;  //reset the array index
    readIndex_speed = 0;  //reset the sum
  }
  
  #ifdef ESP32
    if(not((webserver.arg("numReadings_temp_html")).equals("") or (webserver.arg("numReadings_temp_html")).equals("0"))){  //only update if the field was not empty or 0
      String numReadings_temp_html = webserver.arg("numReadings_temp_html"); //read the value after "numReadings_temp_html" of the HTTP-GET-request send by the client
      writeFile(SPIFFS, "/numReadings_temp.txt", numReadings_temp_html.c_str());  //persistently store it in a text file in the SPIFFS-section
      for(int i = 0; i<500;i++){  //to avoid errors in the calculation of the moving average, overwrite everything with 0
        readings_temp[i]=0;
      }
      total_temp = 0;  //reset the array index
      readIndex_temp = 0;  //reset the sum
    }
    if(not((webserver.arg("numReadings_humidity_html")).equals("") or webserver.arg("numReadings_humidity_html").equals("0"))){  //only update if the field was not empty or 0
      String numReadings_humidity_html = webserver.arg("numReadings_humidity_html"); //read the value after " numReadings_humidity_html" of the HTTP-GET-request send by the client
      writeFile(SPIFFS, "/numReadings_humidity.txt", numReadings_humidity_html.c_str());  //persistently store it in a text file in the SPIFFS-section
      for(int i = 0; i<500;i++){  //to avoid errors in the calculation of the moving average, overwrite everything with 0
        readings_humidity[i] = 0;
      }
      total_humidity = 0;  //reset the array index
      readIndex_humidity = 0;  //reset the sum
    }

    if(not((webserver.arg("correction_factor_temp_html")).equals(""))){  //only update if the field was not empty 
       String correction_factor_temp_html = webserver.arg("correction_factor_temp_html"); //read the value after "correction_factor_temp_html" of the HTTP-GET-request send by the client
       writeFile(SPIFFS, "/correction_factor_temp.txt", correction_factor_temp_html.c_str());  //persistently store it in a text file in the SPIFFS-section
     }
    if(not((webserver.arg("offset_temp_html")).equals(""))){  //only update if the field was not empty 
       String offset_temp_html = webserver.arg("offset_temp_html"); //read the value after "offset_temp_html" of the HTTP-GET-request send by the client
       writeFile(SPIFFS, "/offset_temp.txt", offset_temp_html.c_str());  //persistently store it in a text file in the SPIFFS-section
     }
    if(not((webserver.arg("correction_factor_humidity_html")).equals(""))){  //only update if the field was not empty 
       String correction_factor_humidity_html = webserver.arg("correction_factor_humidity_html"); //read the value after "correction_factor_humidity_html" of the HTTP-GET-request send by the client
       writeFile(SPIFFS, "/correction_factor_humidity.txt", correction_factor_humidity_html.c_str());  //persistently store it in a text file in the SPIFFS-section
     }
    if(not((webserver.arg("offset_humidity_html")).equals(""))){  //only update if the field was not empty 
       String offset_humidity_html = webserver.arg("offset_humidity_html"); //read the value after "offset_humidity_html" of the HTTP-GET-request send by the client
       writeFile(SPIFFS, "/offset_humidity.txt", offset_humidity_html.c_str());  //persistently store it in a text file in the SPIFFS-section
     }
   #endif
webserver.sendHeader("Location", String("/config"), true);  //send a header that leads back to the config page
webserver.send ( 302, "text/plain", "");
}


//--------------------------------------------------------
// gui - send the HTML-code (or rather mainly JS) that  
// displays the wind data on a web page, and change it to
// the desired unit
//--------------------------------------------------------

void gui() {

  String s = graphic; //string to store the HTML page
  
  if(language.equals("en")){
    s.replace("Einstellungen", "Settings");  //if language is set to english, replace the german word for "settings" with the english one
    s.replace("Temperatur", "Temperature");  //if language is set to english, replace the german word for "settings" with the english one
    s.replace("Luftfeuchtigkeit", "Humidity");  //if language is set to english, replace the german word for "settings" with the english one
  }
  if(ws_unit.equals("Kilometer pro Stunde")){
    s.replace("unit_manipulation", "3.6)).toFixed(1) + \"km/h\"");  //multiply the wind speed by 3.6, for m/s to km/h, and display only one dezimal
  } else if(ws_unit.equals("Knoten")){
    s.replace("unit_manipulation", "1.944)).toFixed(1) + \"kn\"");  //multiply the wind speed by 1.994, for m/s to kn, and display only one dezimal
  } else {
    s.replace("unit_manipulation", "1)).toFixed(1) + \"m/s\"");  //multiply the wind speed by 1, as it already is in m/s, and display only one dezimal
  }
  #ifdef ESP32
    if(hum_temp.equals("gemessen")){
      s.replace("const SIDEBOXES_ENABLED = false", "const SIDEBOXES_ENABLED = true");  //only display the fields for temperature and humidity if they are measured
    } else {
      s.replace("const SIDEBOXES_ENABLED = true", "const SIDEBOXES_ENABLED = false");
      s.replace("data[7]", "0");  //if temperature and humidity are not measured, set the vlaues to 0, so it does not look for a value that is not present
      s.replace("data[11]", "0");  //if temperature and humidity are not measured, set the vlaues to 0, so it does not look for a value that is not present
    }
  #endif
  webserver.send(200, "text/html", s); //Send web page
}


//--------------------------------------------------------
// data - called by the graphic-JS-script
// sends the HTML-code that displays the wind data as 
// plain NMEA0813 text. That is then fetched by the 
// graphics-API
//--------------------------------------------------------

void data() {
  char both[300] = "";  //combine the results of the wind angle/speed and the humidity/temperature to one string, that will be evaluated by the GUI
  strcat(both, result);
  #ifdef ESP32
    strcat(both, hum_temp_result);  //only add temperature and humidity if actually measured
  #endif
  String s = both; //string to store the final combination of both NMEA0813-sentence

  webserver.send(200, "text/html", s); //Send the NMEA0813-sentence
}


//--------------------------------------------------------
// sensor - if called, this function changes the sensor
// type. Mainly effects preset values for the wind speed
//--------------------------------------------------------
void sensor_jukolein() {
    sensor = "Jukolein";  //set sensor to Jukolein
    writeFile(SPIFFS, "/sensor.txt", sensor.c_str());  //persistently store it in a text file in the SPIFFS-section
webserver.sendHeader("Location", String("/config"), true);
webserver.send ( 302, "text/plain", "");
}
void sensor_ventus() {
    sensor = "Ventus";  //set sensor to Ventus
    writeFile(SPIFFS, "/sensor.txt", sensor.c_str());  //persistently store it in a text file in the SPIFFS-section
webserver.sendHeader("Location", String("/config"), true);
webserver.send ( 302, "text/plain", "");
}
void sensor_emos() {
    sensor = "Emos";  //set sensor to Emos
    writeFile(SPIFFS, "/sensor.txt", sensor.c_str());  //persistently store it in a text file in the SPIFFS-section
webserver.sendHeader("Location", String("/config"), true);
webserver.send ( 302, "text/plain", "");
}


//--------------------------------------------------------
// ipBlinkToggle - if called, this function toggles the 
// value that defines if the IP is going to blinked at boot
//--------------------------------------------------------

void ipBlinkToggle() {
   String blIPblink_html;  //string for storing the value that will be written
   if (blIPblink.equals("nicht geblinkt")) {  //check current state, and, if "false" (that means "nicht geblinkt" in this case), change it to "true" ("geblinkt")...
    blIPblink_html = "geblinkt";}
   else {
    blIPblink_html = "nicht geblinkt";  //... and if it is "true", make it "false"
    }
    writeFile(SPIFFS, "/blipblink.txt", blIPblink_html.c_str());  //persistently store it in a text file in the SPIFFS-section
webserver.sendHeader("Location", String("/config"), true);
webserver.send ( 302, "text/plain", "");
}


//--------------------------------------------------------
// AP_Toggle - if called, this function toggles the 
// value that defines if the IP is going to blinked at boot
//--------------------------------------------------------

void AP_Toggle() {
   String ap_html;  //string for storing the value that will be written
   if (ap.equals("befindet sich der Windsensor im Acces-Point-Modus")) {  //check current state, and, if "false", change it to "true" ...
    ap_html = "verbindet sich der Windsensor mit einem bestehendem Netzwerk";}
   else {
    ap_html = "befindet sich der Windsensor im Acces-Point-Modus";  //... and if it is "true", make it "false"
    }
    writeFile(SPIFFS, "/ap.txt", ap_html.c_str());  //persistently store it in a text file in the SPIFFS-section
webserver.sendHeader("Location", String("/config"), true);
webserver.send ( 302, "text/plain", "");
}

//--------------------------------------------------------
// ws_unitChange - if called, this function changes the 
// value of the unit to be used for the graphical display
//--------------------------------------------------------

void ws_unitChange_meters() {
    ws_unit = "Meter pro Sekunde";  //set ws_unit to meters per second
    writeFile(SPIFFS, "/ws_unit.txt", ws_unit.c_str());  //persistently store it in a text file in the SPIFFS-section
webserver.sendHeader("Location", String("/config"), true);
webserver.send ( 302, "text/plain", "");

}

void ws_unitChange_kilometers() {
    ws_unit = "Kilometer pro Stunde";  //set ws_unit to kilometers per second
    writeFile(SPIFFS, "/ws_unit.txt", ws_unit.c_str());  //persistently store it in a text file in the SPIFFS-section
webserver.sendHeader("Location", String("/config"), true);
webserver.send ( 302, "text/plain", "");

}

void ws_unitChange_knots() {
    ws_unit = "Knoten";  //set ws_unit to knots
    writeFile(SPIFFS, "/ws_unit.txt", ws_unit.c_str());  //persistently store it in a text file in the SPIFFS-section
webserver.sendHeader("Location", String("/config"), true);
webserver.send ( 302, "text/plain", "");
}


//--------------------------------------------------------
// language - if called, this function changes the display
// language
//--------------------------------------------------------
void language_de() {
    language = "de";  //set ws_unit to kilometers per second
    writeFile(SPIFFS, "/language.txt", language.c_str());  //persistently store it in a text file in the SPIFFS-section
webserver.sendHeader("Location", String("/config"), true);
webserver.send ( 302, "text/plain", "");
}
void language_en() {
    language = "en";  //set ws_unit to kilometers per second
    writeFile(SPIFFS, "/language.txt", language.c_str());  //persistently store it in a text file in the SPIFFS-section
webserver.sendHeader("Location", String("/config"), true);
webserver.send ( 302, "text/plain", "");
}


//--------------------------------------------------------
// smooth - if called, this function toggles the 
// value that defines if humidity and temperature will be
// measured and processed or not
//--------------------------------------------------------

double smooth(double value, String type) {
  double average;
  
  if(type.equals("speed")){
    total_speed = total_speed - readings_speed[readIndex_speed];  //subtract the last reading
    readings_speed[readIndex_speed] = value;  //add the current value
    total_speed = total_speed + readings_speed[readIndex_speed];  //add value to total
    readIndex_speed = (readIndex_speed + 1) % numReadings_speed.toInt();  //handle index
    average = total_speed / numReadings_speed.toInt();  //calculate the average:
  }
  if(type.equals("angle")){
    total_angle = total_angle - readings_angle[readIndex_angle];  //subtract the last reading
    readings_angle[readIndex_angle] = value;  //add the current value
    total_angle = total_angle + readings_angle[readIndex_angle];  //add value to total
    readIndex_angle = (readIndex_angle + 1) % numReadings_angle.toInt();  //handle index
    average = total_angle / numReadings_angle.toInt();  //calculate the average:
  }

  #ifdef ESP32
    if(type.equals("temperature")){
      total_temp = total_temp - readings_temp[readIndex_temp];  //subtract the last reading
      readings_temp[readIndex_temp] = value;  //add the current value
      total_temp = total_temp + readings_temp[readIndex_temp];  //add value to total
      readIndex_temp = (readIndex_temp + 1) % numReadings_temp.toInt();  //handle index
      average = total_temp / numReadings_temp.toInt();  //calculate the average:
    }
    if(type.equals("humidity")){
      total_humidity = total_humidity - readings_humidity[readIndex_humidity];  //subtract the last reading
      readings_humidity[readIndex_humidity] = value;  //add the current value
      total_humidity = total_humidity + readings_humidity[readIndex_humidity];  //add value to total
      readIndex_humidity = (readIndex_humidity + 1) % numReadings_humidity.toInt();  //handle index
      average = total_humidity / numReadings_humidity.toInt();  //calculate the average:
    }
  #endif
  
  return average;
}

#ifdef ESP32

  //******************************************************************************************************************************************************
  // Functions for NMEA2000
  //******************************************************************************************************************************************************

  //--------------------------------------------------------
  // nmea2kToggle - if called, this function toggles the
  // value that defines if NMEA2k is processed and send
  //--------------------------------------------------------
  
  void nmea2kToggle() {
     String nmea2k_html;  //string for storing the value that will be written
     if (nmea2k.equals("NMEA2000-Daten werden nicht erstellt oder ausgegeben <input type=\"button\" onclick=\"location.href='/nmea2k';\" value=\"Ändern\" /> <br>  <br>")) {  //check current state, and, if "false" change it to "true"...
      nmea2k_html = "NMEA2000-Daten werden erstellt und ausgegeben <input type=\"button\" onclick=\"location.href='/nmea2k';\" value=\"Ändern\" /> <br>  <br>";}
     else {
      nmea2k_html = "NMEA2000-Daten werden nicht erstellt oder ausgegeben <input type=\"button\" onclick=\"location.href='/nmea2k';\" value=\"Ändern\" /> <br>  <br>";  //... and if it is "true", make it "false"
      }
      writeFile(SPIFFS, "/nmea2k.txt", nmea2k_html.c_str());  //persistently store it in a text file in the SPIFFS-section
  webserver.sendHeader("Location", String("/config"), true);
  webserver.send ( 302, "text/plain", "");
  }


  //--------------------------------------------------------
  // CheckSourceAddressChange - check for canges in the
  // source address, and, if found, update
  //--------------------------------------------------------

  void CheckSourceAddressChange() {
    int SourceAddress = NMEA2000.GetN2kSource();  //int for storing the source address

    if (SourceAddress != NodeAddress) { //check if source address equals node address
      NodeAddress = SourceAddress;  //if not, overwrite node address with source address
      preferences.begin("nvs", false);  //and update the preferences
      preferences.putInt("LastNodeAddress", SourceAddress);
      preferences.end();
      Serial.printf("Address Change: New Address=%d\n", SourceAddress);
    }
  }
    
    //******************************************************************************************************************************************************
    // Functions for the temperature and humidity calculation
    //******************************************************************************************************************************************************

    //--------------------------------------------------------
    // hum_tempToggle - if called, this function toggles the 
    // value that defines if humidity and temperature will be
    // measured and processed or not
    //--------------------------------------------------------
    
    void hum_tempToggle() {
       String hum_temp_html;  //string for storing the value that will be written
       if (hum_temp.equals("nicht gemessen")) {  //check current state, and, if "false" (that means "nicht gemessen" in this case), change it to "true" ("gemessen")...
        hum_temp_html = "gemessen";}
       else {
        hum_temp_html = "nicht gemessen";  //... and if it is "true", make it "false"
        }
        writeFile(SPIFFS, "/hum_temp.txt", hum_temp_html.c_str());  //persistently store it in a text file in the SPIFFS-section
    webserver.sendHeader("Location", String("/config"), true);
    webserver.send ( 302, "text/plain", "");
    }

    //--------------------------------------------------------
    // hum_and_temp - makes use of the humidity and temperature 
    // sensors of the EMOS weather station. The humidity sensor 
    // needs AC to work. That is created by taking two output 
    // pins (DRIVE1 and DRIVE2 and altering there HIGH/LOW 
    // states, shifted by half a phase. To keep things simple,
    // this is done by busy waiting. As this would interfere
    // with the measurement of wind speed and direction, the
    // entire job is done by the second core of the ESP32.
    //--------------------------------------------------------

    void hum_and_temp(void * pvParameters) {
      while(true){  //keeps this loop running forever
        // Generate 1kHz square wave to drive sensor. Busy wait, so blocks here.
        while (micros () - prev_time < HALF_PERIOD_US) {
          }  //wait for next 0.5ms time segment. As it completely blocks the loop while waiting, it is calles busy wait
        prev_time += HALF_PERIOD_US;  //prepare for the next loop
        digitalWrite (DRIVE1, phase == 0);  //AC is required, so DRIVE1 and DRIVE2 get inverted
        digitalWrite (DRIVE2, phase != 0);  //AC is required, so DRIVE1 and DRIVE2 get inverted


        //--------------------------------------------------------
        // ADC reading of the HSENSE-pin to get the humidity.
        // The first reading is ignored to get better results
        // two readings are performed, one on each side of the wave 
        //--------------------------------------------------------
        int val = analogRead (HSENSE);  //firs read, is going to be ignored
        val = analogRead (HSENSE);  //second read allows much longer for high impedance reading to settle
        if (phase == 0){
          val = adc_resolution - val;  //reverse sense on alternate half cycles
        }
        float v = VCC * val / adc_resolution;  //convert the reading to volts
        float voltage = (v + prevVoltage) / 2; //smooths the reading with a moving average
        prevVoltage = voltage;  //make the current value the last value
        phase = 1 - phase;  //invert phase

        
        //--------------------------------------------------------
        // ADC reading of the TSENSE-pin and calculation of the 
        // resluting temperature in °C
        //--------------------------------------------------------
        digitalWrite(DRIVE2,HIGH);  //the temperature sensor needs DC, so DRIVE2 is set to high to provide it
        adc_temp_value = analogRead(TSENSE);  //read of the temperature value
        adc_temp_value = analogRead(TSENSE);  //read of the temperature value
        Vout_temp = (adc_temp_value * VCC) / 1023;  //converts the reading to volts
        R_temp = (VCC * R2 / Vout_temp) - R2;  //with the equation of a voltage divier, the resistance in Ohm is calculated


        //--------------------------------------------------------
        // Steinhart-Hart Thermistor Equation:
        // Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)
        // where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8
        //--------------------------------------------------------
        temperature = (1 / (A + (B * log(R_temp)) + (C * pow((log(R_temp)),3))));  //temperature in kelvin
        temperature = temperature - 273.15;  //temperature in degree celsius
        

        //--------------------------------------------------------
        // voltToHumidity - gets the humidity sensor readings in
        // Volts and the current temperature. Calculates the 
        // relative humidity with the help of some strange formula
        // and returns that value. 
        //--------------------------------------------------------
        float relativeHumidity = 0.0f;  //variable to store the final result in
        float imp = voltage/(3.3*100000/R1);  //with the equation of a voltage divier, the resistance in Ohm is calculated. Due to the wiring, a correction polynom is needed to get the correct values
        float i = 10 * log10f( imp );  //strange things going on
        float hum0 = (-5.897e-4 * powf(i,3) + 6.716e-2 * powf(i,2) - 4.019 * i + 1.187e2);  //I wish I knew what appes here, however I haven't got the slightest idea. However, it seems to work.
        hum = hum0 - public_temperature / 2.15;  //calculation of the relative humidity out of hum0 and the temperature
        
        printMe = (printMe+1) % 200;  //needed to limit the amount of prints
        if(printMe==1) {  //every 200 times
          public_temperature = (double)(smooth((int)(temperature*100*correction_factor_temp.toInt()), "temperature"))/100+offset_temp.toInt();
          public_hum = (double)(smooth((int)(hum*100*correction_factor_humidity.toInt()), "humidity"))/100+offset_humidity.toInt();
        }
         
        vTaskDelay(10);  //needed to feed the watchdog, very important!!
      }
    }
        
#endif


//******************************************************************************************************************************************************
// Setup
//******************************************************************************************************************************************************

void setup() {

  Serial.begin(115200);  //Start the Serial communication to send messages to the computer
  Wire.begin();  //Initiate the Wire library and join the I2C bus
  #ifdef ESP32
    if (!SPIFFS.begin(true)) { //mount the SPIFFS storage
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  #else
    SPIFFS.begin();  //mount the SPIFFS storage
  #endif
  
  #ifdef ESP32
    pinMode(18, INPUT_PULLUP);  //Initialize Pin 18 as Input and PullUp, to avoid floating states
    pinMode(LED, OUTPUT);  //Initialize the bulid in LED as an output
      pinMode (DRIVE1, OUTPUT);  //prepare DRIVE1 as output, needed to create the AC 
      pinMode (DRIVE2, OUTPUT);  //prepare DRIVE2 as output, needed to create the AC 
      prev_time = micros ();  //initialize prev_time with the current time
  #else
    pinMode(14, INPUT_PULLUP);  //Initialize Pin 14 as Input and PullUp, to avoid floating states
    pinMode(LED_BUILTIN, OUTPUT);  //Initialize the bulid in LED as an output
  #endif
  
  Serial.print("begin Setup");
#ifdef ESP32
  //--------------------------------------------------------
  // NMEA2000 setup
  //--------------------------------------------------------

    uint8_t chipid[6];  //variable for storing the chip id
    uint32_t id = 0;  //variable for storing the ID

  //--------------------------------------------------------
  // Reserve enough buffer for sending all messages.
  //--------------------------------------------------------
    NMEA2000.SetN2kCANMsgBufSize(8);
    NMEA2000.SetN2kCANReceiveFrameBufSize(150);
    NMEA2000.SetN2kCANSendFrameBufSize(150);


  //--------------------------------------------------------
  // Reserve enough buffer for sending all messages.
  //--------------------------------------------------------

    esp_efuse_read_mac(chipid);
    for (int i = 0; i < 6; i++){
    id += (chipid[i] << (7 * i));
    }

  //--------------------------------------------------------
  // set product information
  //--------------------------------------------------------

    NMEA2000.SetProductInformation("1", //Manufacturer's Model serial code
                                   100, //Manufacturer's product code
                                   "My Sensor Module",  //Manufacturer's Model ID
                                   "1.0.2.25 (2019-07-07)",  //Manufacturer's Software version code
                                   "1.0.2.0 (2019-07-07)" //Manufacturer's Model version
                                   );

                                
  //--------------------------------------------------------
  // set device information
  //--------------------------------------------------------

    NMEA2000.SetDeviceInformation(id, //Unique number, generated out of MAC of the ESP32
                                  130, //Device function=Atmospheric. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  85, //Device class=Device class=External Environment. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  2046 //Just choosen freely from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                 );

    preferences.begin("nvs", false);                          //Open nonvolatile storage (nvs)
    NodeAddress = preferences.getInt("LastNodeAddress", 34);  //Read last NodeAddress stored there, default set to 34
    preferences.end();
    Serial.printf("NodeAddress=%d\n", NodeAddress);

    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, NodeAddress);  //If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
    NMEA2000.ExtendTransmitMessages(TransmitMessages);

    NMEA2000.Open();  //start NMEA2000 handler

    delay(200);  //giv it some time to settle
#endif
//--------------------------------------------------------
// end of NMEA2000 setup, start of gerneral setup
//--------------------------------------------------------

  //--------------------------------------------------------
  // WiFiManager - takes care of the WiFi
  // If the ESP can not connect to a known WiFi-Network,
  // it opens an AP to connect to and set up a connection
  // therefore, no physical access is requiered to change
  // networks. Furthermore, it offers OTA FW-Updates.
  //--------------------------------------------------------

  ap = readFile(SPIFFS, "/ap.txt");  //load the variable persistently stored in ap.txt and write it to ap
  if(ap.equals("befindet sich der Windsensor im Acces-Point-Modus")) { //check if the WiFiManager managed to connect to a network, if not, create an AP
    WiFi.softAP("Windsensor_AP", "123456789");  //create an AP with the SSID "Windsensor_AP" and the password "123456789"
    Serial.println("AP-Mode");
  }
  else{
  // Local intialization of WiFiManager. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;
    
  // fetches ssid and pass from eeprom and tries to connect
  // if it does not connect it starts an access point with the specified name
  // here  "Windsensor"
  // and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect("Windsensor");

  Serial.println("Connected.");  // if you get here you have connected to the WiFi
  }


  //--------------------------------------------------------
  // softAP - creates an AP if setup with wifiManager was
  // unsuccessful 
  //--------------------------------------------------------
  #ifdef ESP32
  if((WiFi.localIP().toString().equals("0.0.0.0"))) { //check if the WiFiManager managed to connect to a network, if not, create an AP
    WiFi.softAP("Windsensor_AP", "123456789");  //create an AP with the SSID "Windsensor_AP" and the password "123456789"
  }
  #else
  if((WiFi.localIP().toString().equals("(IP unset)"))) { //check if the WiFiManager managed to connect to a network, if not, create an AP
    WiFi.softAP("Windsensor_AP", "123456789");  //create an AP with the SSID "Windsensor_AP" and the password "123456789"
  }
  #endif


  //--------------------------------------------------------
  // ArduinoOTA - allows OTA-FW-flashing with the Arduino IDE
  // must be included in every flash, or OTA-capability is lost
  //--------------------------------------------------------

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  //--------------------------------------------------------
  // make pin 18 an interrupt-pin, so whenever this pin is high,
  // an interrupt-function (here: WinSensInterrupt()) will be 
  // called. Crutial for the calculation of the windspeed.
  //--------------------------------------------------------
  #ifdef ESP32
    attachInterrupt(digitalPinToInterrupt(18), WinSensInterupt, FALLING);  //define interrupt
  #else
    attachInterrupt(digitalPinToInterrupt(14), WinSensInterupt, FALLING);  //define interrupt
  #endif
  
  //--------------------------------------------------------
  // configuration of the HTTP-webserver
  // define how to deal with different URLs send or requested
  // by the clients
  //--------------------------------------------------------

  webserver.on("/config", handleCorrection);  //calls the function that sends the HTML code of the correction page, when the client goes to /config
  webserver.on("/action_page", handleForm);  //form action is handled here
  webserver.on("/ipblink", ipBlinkToggle);  //if the button to toggle the IPblinking is pressed, this calls the function to change it
  webserver.on("/", gui);  //calls the function that sends the HTML code to graphically display the wind data, when the client goes to the root of the webserver
  webserver.on("/data", data);  //when called, call the function that displays the current NMEA0813-sentence to be fetched by the API as plain text

  webserver.on("/sensor_jukolein", sensor_jukolein);  //call a function to change the sensor type to "jukolein"
  webserver.on("/sensor_ventus", sensor_ventus);  //call a function to change the sensor type to "ventus"
  webserver.on("/sensor_emos", sensor_emos);  //call a function to change the sensor type to "emos"

  webserver.on("/meters", ws_unitChange_meters);  //call the function to change the unit to meters per second
  webserver.on("/kilometers", ws_unitChange_kilometers);  //call the function to change the unit to kilometers per hour
  webserver.on("/knots", ws_unitChange_knots);  //call the function to change the unit to knots

  webserver.on("/language_de", language_de);  //call the function to change the language to german
  webserver.on("/language_en", language_en);  //call the function to change the language to english

  webserver.on("/ap", AP_Toggle);  //call the function to change the acces-point/WiFI-mode

  #ifdef ESP32
    webserver.on("/nmea2k", nmea2kToggle);  //call the function to change if NMEA2000-Data is processed and send or not
    webserver.on("/hum_temp", hum_tempToggle);  //call the function to change if humidity and temperature are measured and processed or not
  #endif
  
  webserver.begin();  //now that all is set up, start the HTTP-webserver
  server.begin();  //start the TCP-server that sends the NMEA0813 on port 8080
  server.setNoDelay(true); // disable sending small packets

  //--------------------------------------------------------
  // configuration of the mDNS service
  // makes the ESP accessible at the string passed over
  // in the constructor, here: "windsensor", ".local"
  //--------------------------------------------------------
  
 if (!MDNS.begin("windsensor")) {  // Start the mDNS responder for windsensor.local
    Serial.println("Error setting up MDNS responder!");
  }
  MDNS.addService("http", "tcp", 80);   //tell the mDNS service, what to manage, here: the TCP-webserver on Port 80
  MDNS.addService("nmea-0183", "tcp", 8080);  // NMEA0183 data service for AVnav
  //An additional service besides port 8080 is, as far as I can tell, not needed. At least Signal K could resolve with only this line

  
  //--------------------------------------------------------
  // processing of the stored correction-values (except ap)
  //--------------------------------------------------------

  offset = readFile(SPIFFS, "/offset.txt");  //load the persistent data as variable
  factor = readFile(SPIFFS, "/factor.txt");  //load the persistent data as variable
  blIPblink = readFile(SPIFFS, "/blipblink.txt");  //update the blIPblink state
  ws_unit = readFile(SPIFFS, "/ws_unit.txt");  //update the ws_unit state
  language = readFile(SPIFFS, "/language.txt");  //load the persistent data as variable
  numReadings_angle = readFile(SPIFFS, "/numReadings_angle.txt");  //update the hum_temp state
  numReadings_speed = readFile(SPIFFS, "/numReadings_speed.txt");  //update the hum_temp state
  #ifdef ESP32
    nmea2k = readFile(SPIFFS, "/nmea2k.txt");  //update the NMEA2000 state
    hum_temp = readFile(SPIFFS, "/hum_temp.txt");  //update the hum_temp state
    numReadings_temp = readFile(SPIFFS, "/numReadings_temp.txt");  //update the hum_temp state
    numReadings_humidity = readFile(SPIFFS, "/numReadings_humidity.txt");  //update the hum_temp state
    correction_factor_temp = readFile(SPIFFS, "/correction_factor_temp.txt");  //update the hum_temp state
    offset_temp = readFile(SPIFFS, "/offset_temp.txt");  //update the hum_temp state
    correction_factor_humidity = readFile(SPIFFS, "/correction_factor_humidity.txt");  //update the hum_temp state
    offset_humidity = readFile(SPIFFS, "/offset_humidity.txt");  //update the hum_temp state
    sensor = readFile(SPIFFS, "/sensor.txt");  //update the sensor type
  #endif
  
  //--------------------------------------------------------
  // initializing the stored correction-values
  // this only takes effect at the very first start
  //--------------------------------------------------------

  if (!(blIPblink.equals("geblinkt") or blIPblink.equals("nicht geblinkt"))) {  //initially set blIPblink to "geblinkt" ("true"), if it is neither "geblinkt" nor "nicht geblinkt"
    blIPblink = "geblinkt"; 
    writeFile(SPIFFS, "/blipblink.txt", blIPblink.c_str());}

 if (!(ap.equals("verbindet sich der Windsensor mit einem bestehendem Netzwerk") or ap.equals("befindet sich der Windsensor im Acces-Point-Modus"))) {  //
    ap = "verbindet sich der Windsensor mit einem bestehendem Netzwerk"; 
    writeFile(SPIFFS, "/ap.txt", ap.c_str());}

if (!(sensor.equals("Jukolein") or sensor.equals("Ventus") or sensor.equals("Emos"))) {  //initially set sensor type to jukolein
    sensor = "Jukolein"; 
    writeFile(SPIFFS, "/sensor.txt", sensor.c_str());}
    
  if (!(language.equals("en") or language.equals("de"))) {  //initially set language to en
    language = "de"; 
    writeFile(SPIFFS, "/language.txt", language.c_str());}

  if (!(ws_unit.equals("Meter pro Sekunde") or ws_unit.equals("Kilometer pro Stunde") or ws_unit.equals("Knoten"))) {  //if not yet set, initially set ws_unit to "Meter pro Sekunde" ("meters per second")
    ws_unit = "Meter pro Sekunde"; 
    writeFile(SPIFFS, "/ws_unit.txt", ws_unit.c_str());}

  if (offset.toInt() == 0) {  //initially set the offset to 0, if it is null or 0, as both get converted to 0 by toInt()
    offset = "0"; 
    writeFile(SPIFFS, "/offset.txt", offset.c_str());}
  
  if (factor.toFloat() == 0) {  //initially set the linear windspeed factor to 1 if it is null or 0, as both get converted to 0 by toInt(), but a factor of 0 would result in no windspeed at all
    factor = "1"; 
    writeFile(SPIFFS, "/factor.txt", factor.c_str());}

  if (numReadings_angle.toInt() == 0){  //if not set or 0, set the number of valuaes for the moving average of the wind angle to 1
    numReadings_angle = "1";
    writeFile(SPIFFS, "/numReadings_angle.txt", numReadings_angle.c_str());}
    
  if (numReadings_speed.toInt() == 0){  //if not set or 0, set the number of valuaes for the moving average of the wind speed to 1
    numReadings_speed = "1";
    writeFile(SPIFFS, "/numReadings_speed.txt", numReadings_speed.c_str());}

  #ifdef ESP32

    if (!(nmea2k.equals("NMEA2000-Daten werden nicht erstellt oder ausgegeben <input type=\"button\" onclick=\"location.href='/nmea2k';\" value=\"Ändern\" /> <br>  <br>") or 
      nmea2k.equals("NMEA2000-Daten werden erstellt und ausgegeben <input type=\"button\" onclick=\"location.href='/nmea2k';\" value=\"Ändern\" /> <br>  <br>"))) {  
      nmea2k = "NMEA2000-Daten werden nicht erstellt oder ausgegeben <input type=\"button\" onclick=\"location.href='/nmea2k';\" value=\"Ändern\" /> <br>  <br>";  //initilly don't process or send NMEA200 values
      writeFile(SPIFFS, "/nmea2k.txt", nmea2k.c_str());}

    if (!(hum_temp.equals("gemessen") or hum_temp.equals("nicht gemessen"))) {  //initially set hum_temp to "nicht gemessen" ("false"), if it is neither "gemessen" nor "nicht gemessen"
        hum_temp = "nicht gemessen"; 
        writeFile(SPIFFS, "/hum_temp.txt", hum_temp.c_str());}
    
    if (numReadings_temp.toInt() == 0){   //if not set or 0, set the number of valuaes for the moving average of the temperature to 10
      numReadings_temp = "10";
      writeFile(SPIFFS, "/numReadings_temp.txt", numReadings_temp.c_str());}
    
    if (numReadings_humidity.toInt() == 0){  //if not set or 0, set the number of valuaes for the moving average of the humidity to 1
      numReadings_humidity = "10";
      writeFile(SPIFFS, "/numReadings_humidity.txt", numReadings_humidity.c_str());}
          
    if (correction_factor_temp.toInt() == 0){  //if not set or 0, set the correction factor of the temperature to 1
      correction_factor_temp = "1";
      writeFile(SPIFFS, "/correction_factor_temp.txt", correction_factor_temp.c_str());}
    
    if (offset_temp.toInt() == 0){  //if not set or 0, set the offset of the temperature to 0
      offset_temp = "0";
      writeFile(SPIFFS, "/offset_temp.txt", offset_temp.c_str());}
    
    if (correction_factor_humidity.toInt() == 0){  //if not set or 0, set the correction factor of the humidity to 1
      correction_factor_humidity = "1";
      writeFile(SPIFFS, "/correction_factor_humidity.txt", correction_factor_temp.c_str());}
    
    if (offset_humidity.toInt() == 0){  //if not set or 0, set the offset of the humidity to 0
      offset_humidity = "0";
      writeFile(SPIFFS, "/offset_humidity.txt", offset_temp.c_str());}
      
  #endif


  //--------------------------------------------------------
  // these lines can trigger the bliniking of the IP address
  //--------------------------------------------------------

  if (blIPblink.equals("geblinkt") and !(WiFi.localIP().toString().equals("(IP unset)"))) {  //if wanted and there is an IP to be blinked, call the function to blink it
    ip_blink();  //call the function to blink out the IP
  };

  #ifdef ESP32
    xTaskCreatePinnedToCore(
                      hum_and_temp,   /* Task function. */
                      "Task1",     /* name of task. */
                      10000,       /* Stack size of task */
                      NULL,        /* parameter of the task */
                      1,           /* priority of the task */
                      &Task1,      /* Task handle to keep track of created task */
                      0);          /* pin task to core 0 */          
  #endif
          
  delay(500); 

  Serial.print("leave setup");
}


//********************************************************
// Loop
//********************************************************


void loop() {
  ArduinoOTA.handle(); //deals with the ArduinoOTA-stuff, must be kept in to preserve posibility of OTA-FW-updates

  webserver.handleClient();  //Handle client requests of the HTTP-webserver

  #ifdef ESP32
  #else 
    MDNS.update();  //Allow mDNS processing
  #endif

  Z = smooth((fmod(((convertRawAngleToDegrees(ams5600.getRawAngle()))*(1) + offset.toInt() + 720), 360)), "angle"); //stores the value of the angle mod 360 after calculating the moving average
//  Z = fmod((convertRawAngleToDegrees(random(260)) + offset.toInt() + 360), 360); //stores the value of the angle mod 360

if(times != 0) {  //avoid division by zero
  if(sensor.equals("Jukolein") or sensor.equals("Emos")){  //these sensors create two pulses per Round, so wo need times*2
  rate_of_rotation = 1000.0/(double)(2*times);  //calculate the rate of rotation in Hz
  }
  else{
    rate_of_rotation = 1000.0/(double)(times);  //calculate the rate of rotation in Hz
  }
}

  //--------------------------------------------------------
  // The wind speed can be calculated using the tip-speed-
  // ratio. This equation results in the formula
  // wind_speed = (2*Pi*rate_of_rotation[Hz]*radius[m])/lamda 
  // A wheel with 3 cups, like our sensors, results in a
  // lamda of 0.3. The factors are precalculated and 
  // introduced as the factor before the rate_of_rotation.
  //--------------------------------------------------------

  if(sensor.equals("Jukolein")){
     WS = smooth((0.9006*rate_of_rotation), "speed");  //calculating the wind speed
  }
  if(sensor.equals("Venuts")){
     WS = smooth((1.1519*rate_of_rotation), "speed");  //calculating the wind speed
  }
  if(sensor.equals("Emos")){
     WS = smooth((1.0472*rate_of_rotation), "speed");  //calculating the wind speed
  }

  //--------------------------------------------------------
  // Assembly of the MWV-NMEA0183-sentence to calculate the 
  // checksum Beware: sprintf works on raw storage. If the 
  // chars are not initialized long enough, it will cause an
  // buffer overflow
  //--------------------------------------------------------

  strcpy(MWVSentence, "WIMWV,");
  sprintf(e, "%d", Z);  //e represents the angle in degrees
  strcat(MWVSentence, e);
  strcat(MWVSentence, ",R,");
  sprintf(c, "%.2f", WS);  //c represents the wind speed. .2f means, that the float will be cut after two decimals
  strcat(MWVSentence, c);
  strcat(MWVSentence, ",M,A");
  
  checksum(MWVSentence).toCharArray(g, 85); //calculate checksum and store it

  //final assembly of the TCP-message to be send
  strcpy(result, "$");  //start with the dollar symbol
  strcat(result, MWVSentence); //append the MWVSentence
  strcat(result, "*"); //star-seperator for the CS
  strcat(result, g);  //append the CS

  #ifdef ESP32
    //--------------------------------------------------------
    // Assembly of the XDR-NMEA0183-sentence to calculate the 
    // checksum Beware: sprintf works on raw storage. If the 
    // chars are not initialized long enough, it will cause an
    // buffer overflow
    //--------------------------------------------------------
  
    strcpy(XDRSentence, "WIXDR,");
    strcat(XDRSentence, "C,");
    sprintf(temp_char, "%-.2f", (public_temperature));  //get the public temperature as signed float with two deciamls 
    strcat(XDRSentence, temp_char);
    strcat(XDRSentence, ",C,EMOS_TEMP,");
    strcat(XDRSentence, "H,");
    sprintf(hum_char, "%-.0f", public_hum); //get the public temperature as signed float with two deciamls 
    strcat(XDRSentence, hum_char);
    strcat(XDRSentence, ",P,EMOS_HUM,");
  
    checksum(XDRSentence).toCharArray(hum_temp_check, 85); //calculate checksum and store it
  
    //final assembly of the TCP-message to be send
    strcpy(hum_temp_result, "$");  //start with the dollar symbol
    strcat(hum_temp_result, XDRSentence); //append the MWVSentence
    strcat(hum_temp_result, "*"); //star-seperator for the CS
    strcat(hum_temp_result, hum_temp_check);  //append the CS
  #endif

  //--------------------------------------------------------
  // Assembly and sending of the NMEA0183-sentence via TCP
  //--------------------------------------------------------
  
  if (server.hasClient()) {  //only send if there are clients to receive
    for (int i = 0; i < MAX_CLIENTS; i++) {

      //added check for clients[i].status==0 to reuse connections
      if ( !(clients[i] && clients[i].connected() ) ) {
        if (clients[i]) {
          clients[i].stop(); // make room for new connection
        }
        clients[i] = server.available();
        continue;
      }
    }

    // No free spot or exceeded MAX_CLENTS so reject incoming connection
    server.available().stop();
  }

  // Broadcast NMEA0183 sentence to all clients
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i] && clients[i].connected()) {
      clients[i].println(result);  //make sure to use println and not write, at least it did not work for me
      #ifdef ESP32
        clients[i].println(hum_temp_result);  //make sure to use println and not write, at least it did not work for me
      #endif
    }
  }
  #ifdef ESP32
    if(nmea2k.equals("NMEA2000-Daten werden erstellt und ausgegeben <input type=\"button\" onclick=\"location.href='/nmea2k';\" value=\"Ändern\" /> <br>  <br>")) {  //only call the NMEA2000-functions if settings say so
      //--------------------------------------------------------
      // Assembly and sending of the NMEA2000-sentence via CAN
      //--------------------------------------------------------
      tN2kMsg N2kMsg;  //get local instance of N2KMsg for the MWV sentence
      tN2kMsg N2kMsg2;  //get local instance of N2KMsg for the XDR sentence
      SetN2kWindSpeed(N2kMsg, 1, WS, DegToRad(Z),N2kWind_Apprent);  //create the datagram with the current values of WS (windspeed) and Z (angle in degrees)
      SetN2kEnvironmentalParameters(N2kMsg2, 1, N2kts_OutsideTemperature, CToKelvin(public_temperature), N2khs_InsideHumidity, public_hum);  //create the datagram with the current values of temperature and humidity
      NMEA2000.SendMsg(N2kMsg);  //send out the MWV message via the CAN BUS
      NMEA2000.SendMsg(N2kMsg2);  //send out the XDR message via the CAN BUS

      NMEA2000.ParseMessages();

      CheckSourceAddressChange();
  
      if ( Serial.available() ) {  //Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
        Serial.read();
      }
    }
  #endif

  //--------------------------------------------------------
  // Sending of the NMEA0183-sentence via SERIAL 
  //--------------------------------------------------------
  Serial.print(result);
  Serial.print("\r");
  Serial.print("\n");
  Serial.println();

  #ifdef ESP32
    Serial.print(hum_temp_result);
    Serial.print("\r");
    Serial.print("\n");
    Serial.println();
  #endif
  
  // Wait for next reading
  for (int c = 0; c < DELAY_COUNT; c++) {
    delay(DELAY_MS);
  }

}
