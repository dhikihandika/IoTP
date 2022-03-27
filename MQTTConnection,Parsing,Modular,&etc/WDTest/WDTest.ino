#include <esp_task_wdt.h>

//3 seconds WDT
#define WDT_TIMEOUT 3

void setup() {
 Serial.begin(115200);
 Serial.println("Configuring WDT...");
 esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
 esp_task_wdt_add(NULL); //add current thread to WDT watch

}

int i = 0;
int last = millis();

void loop() {
  // resetting WDT every 2s, 5 times only
  if (millis() - last >= 2000 && i < 5) {
    unsigned long startTask2 = micros();
    Serial.println("Resetting WDT...");
    esp_task_wdt_reset();
    last = millis();
    i++;
    if (i == 5) {
     Serial.println("Stopping WDT reset. CPU should reboot in 3s");
     unsigned long endTask2 = micros();
     unsigned long deltaTask2 = endTask2 - startTask2;
     Serial.print("exTime2:");Serial.println(deltaTask2);
    }
  } 
}
