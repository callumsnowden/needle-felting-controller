struct Button
{
  uint8_t currentState = 0;
  uint8_t debounceDelay = 50;
  uint8_t pin;
  void update()
  {
    reading = digitalRead(pin);
    if(reading != lastButtonState)
    {
      lastDebounceTime = millis();
    }
    if((millis() - lastDebounceTime) > debounceDelay)
    {
      if(reading != buttonState)
      {
        buttonState = reading;

        currentState = buttonState;
      }
    }
    lastButtonState = reading;
  }
  private:
    uint8_t reading = 0;
    uint8_t buttonState = 0;
    uint8_t lastButtonState = LOW;
    unsigned long lastDebounceTime = 0;
};