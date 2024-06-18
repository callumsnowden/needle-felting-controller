struct Solenoid
{
  uint8_t activationTime = 100;
  uint8_t pin;
  void fire()
  {
    if(!cyclicMode)
    {
      digitalWrite(pin, HIGH);
      lastSingleActivationMillis = millis();
    }
  }
  void update()
  {
    if(!cyclicMode)
    {
      // Update single shot state
      if(millis() - lastSingleActivationMillis > activationTime)
      {
        digitalWrite(pin, LOW);
      }
    } else {
      // Open the valve and set the in cycle flag provided we're not in a stopped state
      if(millis() - lastCycleEndMillis > interPulseDelay && inCycle == false && stopped == false)
      {
        digitalWrite(pin, HIGH);
        inCycle = true;
      }

      // Close the valve if the cycle has completed, update end time and cycle flag provided we're not in a stopped state
      if(millis() - lastCycleEndMillis > interPulseDelay + (activationTime * 2) && inCycle == true && stopped == false)
      {
        lastCycleEndMillis = millis();
        digitalWrite(pin, LOW);
        inCycle = false;
      }
    }
  }
  void fireCycle(uint16_t frequency)
  {
    /*
     * Calculate a cycle delay
     * Frequency is specified in punches/min and we need milliseconds to wait
     * Accounts for 2x activation time to allow for piston extension and retraction (assuming rougly equivalent)
     */
    interPulseDelay = ((60 / frequency) * 1000) - activationTime * 2;
    cyclicMode = true;
    if(!stopped)
    {
      digitalWrite(pin, HIGH);
    }
    lastCycleEndMillis = millis() + (activationTime * 2);
  }
  bool getCycleState()
  {
    return inCycle;
  }
  void stop()
  {
    // Retain current pin state, cancel any other modes
    stopped = true;
    cyclicMode = false;
    inCycle = false;
  }
  void cancel()
  {
    stopped = false;
    cyclicMode = false;
    inCycle = false;
    digitalWrite(pin, LOW);
    lastSingleActivationMillis = millis();
  }
  private:
    // this variable keeps track of whether we've had an estop issued via stop()
    uint8_t stopped = false;

    uint8_t cyclicMode = false;
    uint8_t inCycle = false;
    uint32_t lastSingleActivationMillis;
    uint32_t lastCycleEndMillis;
    uint16_t interPulseDelay;
};