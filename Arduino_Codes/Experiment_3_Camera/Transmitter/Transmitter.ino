#include <ESP8266WiFi.h>
#include <espnow.h>

uint8_t ReceiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};//Auto find the MAC of the receiver ESP8266

//Compose a message structure to contain data.
typedef struct struct_message {
  String d;
  String theta;
}

struct_message;

struct_message TransData; //Create an object called TransData

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);//10ms for read data from Matlab

  WiFi.mode(WIFI_STA);//station mode: the ESP32 connects to an access point. Set device as a Wi-Fi Station

  //(esp_now_init()to initialize ESP-NOW. You must initialize Wi-Fi before initializing ESP-NOW. Returns 0, if succeed.
  while (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  /*
    Set the role of the board as the controller with esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER).
    Then, we register the callback function we made earlier to trigger when data is sent to the slave.
  */
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

  //Finally, we pair with the Receiver device using esp_now_add_peer()
  esp_now_add_peer(ReceiverAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {
  /*
    Check if data is available. (Foexample: int rxlen = Serial.available(); //The number of bytes available in Serial buffer)
  */
  if (Serial.available() > 0)
  {
    String s = Serial.readString();//reads characters from the serial buffer into a String. The function terminates if it times out
    Serial.println(s);
    /*
      Creating substring of myString.substring(from, to). Take from Matlab
    */
    TransData.d = s.substring(s.indexOf('|') + 1, s.indexOf(';'));
    TransData.theta = s.substring(s.indexOf(';') + 1, s.indexOf('/'));
    esp_now_send(ReceiverAddress, (uint8_t *) &TransData, sizeof(TransData));
  }
}
