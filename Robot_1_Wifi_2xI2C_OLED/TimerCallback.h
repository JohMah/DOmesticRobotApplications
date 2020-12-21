void timerCallback(void *pArg) {
  int wasSDA, wasSCL;
  //btime = etime;
  btime = asm_ccount();
  ReductionCounter++;
  if (ReductionCounter >= Reduction){
    ReductionCounter = 0;
    Setpoint_left+= Increment;
    if (Setpoint_left >=  2000) Increment = -1* abs(Increment); //if (Setpoint > 4096 * 58 / 12) Increment = -1* abs(Increment);
    if (Setpoint_left <=     0) Increment = abs(Increment);     //if (Setpoint < -4096 * 58 / 12) Increment = abs(Increment);
  }
  Wire.getpins(&wasSDA,&wasSCL); //Remember the I2C Pins in use before interrupt
  Wire.begin(SDA1_PN, SCL1_PN); //First I2C bus
  raw_ang_left = ams5600.getRawAngle();
  //Simulate two reads. One read takes about 240 µs
  Wire.begin(SDA2_PN, SCL2_PN); //Second I2C bus
  raw_ang_right = 4095-ams5600.getRawAngle();// right wheel encoder is reversed: subtract from 4095
  //put SDA en SCL backt to what it was before interrupt callback
  Wire.begin(wasSDA, wasSCL);
  //Calculate position Left Wheel
  if (raw_ang_left > 4095) raw_ang_left = 4096;
  if (raw_ang_left < 0) raw_ang_left = 0;
  //Count revolutions. at 1 ms sampling rate works up to 250 rps = 15000 rpm on the encoder.
  if ((raw_ang_left > 3072) && (prev_raw_ang_left < 1024)) Revolutions_left--;
  if ((raw_ang_left < 1024) && (prev_raw_ang_left > 3072)) Revolutions_left++;
  prev_raw_ang_left = raw_ang_left;

  Position_left = raw_ang_left + Revolutions_left*4096;
  Input_left = Position_left;
  //Calculate speed Left Wheel
  Speed_left = (Position_left - Positions_left[TimerCallbackCounter])/10.0;
  //Update positions array
  Positions_left[TimerCallbackCounter] = Position_left;
  //Calculate PID for the left wheel
  myPID_left.Compute();
/*
  if (Output >= 0) {
    analogWrite(ENA, (int)abs(Output));  // Turn the LED on by making the voltage HIGH
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {
    analogWrite(ENA, (int)abs(Output));  // Turn the LED on by making the voltage HIGH
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
*/
  pwm+= IncPwm;
  if (pwm >= PWM_RANGE) IncPwm = -1* abs(IncPwm);
  if (pwm <= -PWM_RANGE) IncPwm = abs(IncPwm);

  if (pwm_left >= 0) {
    analogWrite(IN1, (int)(abs(pwm_left)));
    //digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {
    analogWrite(IN2, (int)(abs(pwm_left)));
    digitalWrite(IN1, LOW);
    //digitalWrite(IN2, HIGH);
  }

  //Calculate position Right Wheel
  if (raw_ang_right > 4095) raw_ang_right = 4096;
  if (raw_ang_right < 0) raw_ang_right = 0;
  //Count revolutions. at 1 ms sampling rate works up to 250 rps = 15000 rpm on the encoder.
  if ((raw_ang_right > 3072) && (prev_raw_ang_right < 1024)) Revolutions_right--;
  if ((raw_ang_right < 1024) && (prev_raw_ang_right > 3072)) Revolutions_right++;
  prev_raw_ang_right = raw_ang_right;

  Position_right = raw_ang_right + Revolutions_right*4096;
  Input_right = Position_right;
  //Calculate speed Right Wheel
  Speed_right = (Position_right - Positions_right[TimerCallbackCounter])/10.0;
  //Update positions array
  Positions_right[TimerCallbackCounter] = Position_right;
  //Calculate PID for the left wheel
  myPID_right.Compute();

  if (pwm_right >= 0) {
    analogWrite(IN3, (int)(abs(pwm_right)));
    //digitalWrite(IN1, HIGH);
    digitalWrite(IN4, LOW);
  }
  else {
    analogWrite(IN4, (int)(abs(pwm_right)));
    digitalWrite(IN3, LOW);
    //digitalWrite(IN2, HIGH);
  }

etime = asm_ccount();
TimerCallbackCounter++;
if (TimerCallbackCounter >= 10) TimerCallbackCounter = 0;
} // End of timerCallback
