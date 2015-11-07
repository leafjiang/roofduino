// Interrupt service routines

void isr_enc_left()
{
  if (digitalRead(encLeftPin)) {
    // Rising edge
  
    // Debounce encoder
    // Call a change a bounce if it is close in time to the last change
    unsigned long time_diff = millis() - last_isr_enc_left;
    
    if (time_diff > 5) {
      // Valid interrupt.  Increment counter.
      // todo: check for overflow/underflow of counter?
      // seems unlikely to overflow, because long type stores
      // 32 bits (4 bytes), from -2,147,483,648 to 2,147,483,647.
      if (leftMotorForward)
        leftMotorCounter++;
      else
        leftMotorCounter--;
    }
  
    // Timestamp this interrupt
    last_isr_enc_left = millis();
  }
}

void isr_enc_right()
{
  if (digitalRead(encRightPin)) {
    // Rising edge
    
    // Debounce encoder
    // Call a change a bounce if it is close in time to the last change
    unsigned long time_diff = millis() - last_isr_enc_right;
  
    if (time_diff > 5) {
      // todo: check for overflow/underflow of counter?
      if (rightMotorForward)
        rightMotorCounter++;
      else
        rightMotorCounter--;
    }
  
    // Timestamp this interrupt
    last_isr_enc_right = millis();
  }
}

void isr_ir_remote()
{
  if (!digitalRead(irRemotePin)) {
    // Falling edge
    unsigned long time_diff = micros() - last_isr_ir_remote;

    update_ir_remote_state_machine(time_diff);
    
    // Timestamp this interrupt
    last_isr_ir_remote = micros();
  }
}

void update_ir_remote_state_machine(unsigned long time_diff)
{
  
  if (time_diff > 20000) {
    // Reset state machine
    // We are at the first falling edge
    msg_index = 0;
  } else {
    if (msg_index == 31) time31 = time_diff;
    if (msg_index == 32) time32 = time_diff;
    msg_index++;
  }
  if (msg_index == 33) {
    // Complete message received
    if ((time31 < 1000) && (time32 > 1500)) {
      // "2": time31 = 800, tiem32 = 1700
      irRemoteUp = true;
    } else if ((time31 > 1500) && (time32 > 1500)) {
      // "5": time31 = 1700, tiem32 = 1700
      irRemoteStop = true;
    } else if ((time31 > 1500) && (time32 < 1000)) {
      // "4": time31 = 1700, tiem32 = 800
      irRemoteLeft = true;
    } else if ((time31 > 1100) && (time32 > 1100)) {
      // "6": time31 = 1300, tiem32 = 1300
      irRemoteRight = true;
    }  
  }
}

