#include <ESP8266WiFi.h>
#include <espnow.h>

typedef struct struct_message {
  String d;
  String theta;
} struct_message;

struct_message ReceiveData; //Create an object called ReceiveData

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  /*
    If you want to copy one variable’s content to another, you can do that easily using the memcpy() function.
    This function takes three input arguments. The first argument is the variable where you want to paste data.
    The second parameter is the variable from which you want to copy data.
    The third parameter is the number of bytes you need to copy from the second variable.
  */
  memcpy(&ReceiveData, incomingData, sizeof(ReceiveData));
  String s = "d = " + ReceiveData.d + "; Theta = " + ReceiveData.theta + "/";
  Serial.println(s);
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);//Set device as a Wi-Fi Station

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  /*
    Set the role of the board as the controller with esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER).
    Then, we register the callback function we made earlier to trigger when data is sent to the slave.
  */
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);// get OnDataRecv packer info
}

void loop() {

}
