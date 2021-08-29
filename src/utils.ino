void rising()
{
  attachPCINT(digitalPinToPCINT(MY_PIN), falling, FALLING);
  prev_time = micros();
}
 
void falling() {
  attachPCINT(digitalPinToPCINT(MY_PIN), rising, RISING);
  channel_5_value = micros()-prev_time;
}

float beta = 0.2; 

void apply_servo_limit(int limit){
  if ((servo_pitch_offset - 90) > limit){
      servo_pitch_offset = servo_pitch_offset * (1 - beta) + (90 + limit)* beta;
    }
  if ((servo_pitch_offset - 90) < -limit){
      servo_pitch_offset = servo_pitch_offset * (1 - beta) + limit*beta;
    }
   if ((servo_roll_offset - 90) > limit){
      servo_roll_offset = servo_roll_offset * (1 - beta) + (90 + limit)*beta;
    }
  if ((servo_roll_offset - 90) < -limit){
      servo_roll_offset = servo_roll_offset * (1 - beta) + limit* beta;
    }
  }
