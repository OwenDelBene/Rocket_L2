
// Import required libraries
#include <WiFi.h>
#include <String>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "Donatello";
const char* password = "SecretPassword"; 

  

bool drogeState = false;
const int drogePin = 22;

bool mainState = false;
const int mainPin = 23;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Donatello Controller Web Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <style>
  html {
    font-family: Arial, Helvetica, sans-serif;
    text-align: center;
  }
  h1 {
    font-size: 1.8rem;
    color: white;
  }
  h2{
    font-size: 1.5rem;
    font-weight: bold;
    color: #143642;
  }
  .topnav {
    overflow: hidden;
    background-color: #143642;
  }
  body {
    margin: 0;
  }
  .content {
    padding: 30px;
    max-width: 600px;
    margin: 0 auto;
  }
  .card {
    background-color: #F8F7F9;;
    box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);
    padding-top:10px;
    padding-bottom:20px;
  }
  .button {
    padding: 15px 50px;
    font-size: 24px;
    text-align: center;
    outline: none;
    color: #fff;
    background-color: #FFC0CB;
    border: none;
    border-radius: 5px;
    -webkit-touch-callout: none;
    -webkit-user-select: none;
    -khtml-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
    user-select: none;
    -webkit-tap-highlight-color: rgba(0,0,0,0);
   }
   /*.button:hover {background-color: #0f8b8d}*/
   .button:active {
     background-color: #0f8b8d;
     box-shadow: 2 2px #CDCDCD;
     transform: translateY(2px);
   }
   .state {
     font-size: 1.5rem;
     color:#8c8c8c;
     font-weight: bold;
   }
  </style>
<title>Donatello</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:,">
</head>
<body>
  <div class="topnav">
    <h1>Donatello Ejection Charge Controller</h1>
  </div>
  <div class="content">
    <div class="card">
      <h2>Eject main or droge</h2>
      <p class="state">Ejection Charge state: </p>
      <p><button id="Droge" class="button"><span id="drogeState">Droge: %STATE%</span></button></p>
      <p><button id="Main" class="button"><span id="mainState">Main: %STATE2%</span></button></p>
    </div>
    
  </div>
<script>
  var gateway = `ws://${window.location.hostname}/ws`;
  var websocket;
  window.addEventListener('load', onLoad);
  function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage; // <-- add this line
  }
  function onOpen(event) {
    console.log('Connection opened');
  }
  function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
  }
  function onMessage(event) {
    var ejectionState;
    if (event.data == "1"){
      ejectionState = "Deployed";
    }
    else{
      ejectionState = "Deploy";
    }
    document.getElementById('drogeState').innerHTML = ejectionState;
    document.getElementById('mainState').innerHTML = ejectionState;
  }
  function onLoad(event) {
    initWebSocket();
    initButton();
  }
  function initButton() {
    document.getElementById('Droge').addEventListener('click', toggleDroge);
    document.getElementById('Main').addEventListener('click', toggleMain);
  }
  function toggleDroge(){
    websocket.send('toggleDroge');
  }
  function toggleMain(){
    websocket.send('toggleMain');
  }
</script>
</body>
</html>
)rawliteral";



/*
void PrintLCDMessage()
{
lcd.clear();
lcd.setCursor(0,0);
int index = random(100)%messages.size();
message = messages.at(index);
}
 */
void notifyClients() {
  ws.textAll(String(ledState));
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    if (strcmp((char*)data, "toggleDroge") == 0) {
      mainState = !mainState;
      notifyClients();
    }
    if (strcmp((char*)data, "toggleMain") == 0) {
      drogeState = !drogeState;
      notifyClients();
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

String processor(const String& var){
  Serial.println(var);
  if(var == "STATE"){
    if (drogeState){
      return "ON";
    }
    else{
      return "OFF";
    }
  }
  return String();
}

//
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);


void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);

  pinMode(drogePin, OUTPUT);
  digitalWrite(drogePin, LOW);

  pinMode(mainPin, OUTPUT);
  digitalWrite(mainPin, LOW);
  /*
  // Connect to Wi-Fi
  //
 // WiFi.config(local_IP);
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }
  //
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
*/
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);  
  // Print ESP Local IP Address
  Serial.println(WiFi.softAPIP());
  Serial.println(WiFi.macAddress());
  initWebSocket();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Start server
  server.begin();
}

void loop() {
  ws.cleanupClients();
  digitalWrite(drogePin, drogeState);
  digitalWrite(mainPin, mainState);
  /*

  if (millis() >= time_now + period)
  {
    time_now+=period;
    
   }
   */

  
}
