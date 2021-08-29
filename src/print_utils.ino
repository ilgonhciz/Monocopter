
void print_all(){
    print_pitch();
    print_roll();
    //print_yaw();
    //print_time();
    //print_servo_offset();

    Serial.println("");

  }

void print_pitch(){
  /*Serial.print(" G_X:");
  Serial.print(gyro_x);
  Serial.print(" A_X:");
  Serial.print(acc_x);*/
  Serial.print(" P:");
  Serial.print(pitch);
  }

void print_roll(){
  /*Serial.print(" G_Y:");
  Serial.print(gyro_y);
  Serial.print(" A_Y:");
  Serial.print(acc_y);*/
  Serial.print(" R:");
  Serial.print(roll);
  }

void print_yaw(){
  Serial.print(" G_Z:");
  Serial.print(gyro_z);
  Serial.print(" A_Z:");
  Serial.print(acc_z);
  Serial.print(" Y:");
  Serial.print(yaw);
  }

void print_servo_offset(){
  Serial.print(" S_P:");
  Serial.print(servo_pitch_offset);
  Serial.print(" S_R:");
  Serial.print(servo_roll_offset);
  }
  
void print_time(){
    Serial.print(" T:");
    Serial.print(time_elapsed_IMU);   
  }
