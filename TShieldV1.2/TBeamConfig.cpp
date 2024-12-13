// TBeamConfig.hpp
#pragma once

#include <Arduino.h>
#include <AceButton.h>

namespace TBeam {
    void handleEvent(ace_button::AceButton* button, uint8_t eventType, uint8_t buttonState);
    void setFlag(void);
    void setupDisplay();
    void updateDisplay();
}