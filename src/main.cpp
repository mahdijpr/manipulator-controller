#include <Arduino.h>

#include "app/application.h"
#include "common/config.h"

Application app;

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);

    app.begin();
}

void loop()
{
    app.update();
}
