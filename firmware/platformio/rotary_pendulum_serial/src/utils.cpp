#include <Arduino.h>
#include <WiFi.h>
#include <nvs_flash.h>

bool connectToWiFi(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < 2000) {
    delay(100);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to " + String(ssid));
    return true;
  } else {
    Serial.println("\nFailed to connect to " + String(ssid));
    WiFi.disconnect();
    return false;
  }
}

void setupWiFi(const char* ssids[], const char* passwords[], int NUM_NETWORKS) {
  if (nvs_flash_init() != ESP_OK) {
    Serial.println("Error initializing NVS");
    return;
  }

  nvs_handle my_handle;
  int32_t networkIndex = -1;

  if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
    nvs_get_i32(my_handle, "networkIndex", &networkIndex);
    nvs_close(my_handle);
  }

  if (networkIndex >= 0 && networkIndex < NUM_NETWORKS) {
    if (connectToWiFi(ssids[networkIndex], passwords[networkIndex])) {
      return;
    }
  }

  for (int i = 0; i < NUM_NETWORKS; i++) {
    if (connectToWiFi(ssids[i], passwords[i])) {
      if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_i32(my_handle, "networkIndex", i);
        nvs_commit(my_handle);
        nvs_close(my_handle);
      }
      break;
    }
  }
}

double getTheta(long encoderPosition, long encoderSteps) {
  double theta = 0.0;
  long half_revolutions = encoderPosition/encoderSteps; 
  if (encoderPosition > 0) {
     if (half_revolutions > 1 && half_revolutions % 2 == 0){
       theta = map(encoderPosition, half_revolutions * encoderSteps, (half_revolutions + 1)*encoderSteps, 31415,0) / 1e4;
     }
    else if (half_revolutions > 1 && half_revolutions % 2 == 1){
       theta = map(encoderPosition, half_revolutions * encoderSteps, (half_revolutions + 1)*encoderSteps, 0, -31415) / 1e4;
     }
    else {
      theta = map(encoderPosition, 0, encoderSteps, 31415, 0) / 1e4;
    }
  }
  else { 
     if (half_revolutions < -1 && half_revolutions % 2 == 0){
       theta = map(encoderPosition, half_revolutions * encoderSteps, (half_revolutions - 1)*encoderSteps, -31415,0) / 1e4;
     }
    else if (half_revolutions < -1 && half_revolutions % 2 == -1){
       theta = map(encoderPosition, half_revolutions * encoderSteps, (half_revolutions - 1)*encoderSteps, 0, 31415) / 1e4;
     }
    else {
      theta = map(encoderPosition, 0, -encoderSteps, -31415, 0) / 1e4;
    }
  }
  return theta;
}

double clip(double n, double lower, double upper) {
  return max(lower, min(n, upper));
}