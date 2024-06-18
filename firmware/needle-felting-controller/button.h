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
      if(reading != lastButtonState)
      {
        currentState = reading;
      }
    }
    lastButtonState = reading;
  }
  private:
    uint8_t reading;
    uint8_t lastButtonState;
    uint32_t lastDebounceTime;
};