float angle_limit = 10;

void mode_0(){
  float middle_point = 90;

  att_pitch_should = 90.0 - pitch_channel_value.getAngle();
  att_roll_should = roll_channel_value.map(-100, 100);

  att_pitch_should /= 2;
  att_roll_should /= 2;

  myservo_pitch.write(middle_point + att_pitch_should  ); 
  myservo_roll.write(middle_point + att_roll_should); 
}

void mode_1(){
  update_gyro();
  update_acc();
  compute_attitude();

  att_pitch_should = 90.0 - pitch_channel_value.getAngle();
  att_roll_should = roll_channel_value.map(-angle_limit, angle_limit);

  
  att_pitch_should /= 90/angle_limit;
  map_into_circle();

  servo_pitch_offset += computePID_p(att_pitch_should, pitch);
  servo_roll_offset += computePID_r(att_roll_should, roll);  
  
  apply_servo_limit(55);
  
  myservo_pitch.write(servo_pitch_offset); 
  myservo_roll.write(servo_roll_offset); 
}

void mode_2(){
  update_gyro();
  update_acc();
  compute_attitude();

  att_pitch_should = 90.0 - pitch_channel_value.getAngle();
  att_roll_should = roll_channel_value.map(-angle_limit, angle_limit);

  att_pitch_should /= 90/angle_limit;
  map_into_circle();
  compute_rotation_compensation();  
  
  
  servo_pitch_offset += computePID_p(att_pitch_should, pitch);
  servo_roll_offset += computePID_r(att_roll_should, roll);  
  
  apply_servo_limit(55);
  
  myservo_pitch.write(servo_pitch_offset); 
  myservo_roll.write(servo_roll_offset); 
}
