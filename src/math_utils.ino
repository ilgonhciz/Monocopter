void init_IMU()
{
  if(!accel.begin())
  {
    Serial.print("Problem acc");
    /* There was a problem detecting the LSM303 ... check your connections */
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.print("Problem mag");
    while(1);
  }
  
  if (!gyro.init())
  {
    Serial.print("Problem gyro");
    while (1);
  }
  int init_average_window = 20;
  for(int n = 0; n < init_average_window; n++){
      gyro.read();
      
      gyro_offset_x += gyro.g.x*0.00875;
      gyro_offset_y += gyro.g.y*0.00875;
      gyro_offset_z += gyro.g.z*0.00875;

      mag.getEvent(&mag_event);
        if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)){
            yaw_offset += orientation.heading;
        }
      }
    gyro_offset_x /= init_average_window;
    gyro_offset_y /= init_average_window;
    gyro_offset_z /= init_average_window; 
    yaw_offset /= init_average_window;
    acc_z = yaw = yaw_offset;
     
    gyro.enableDefault(); 
}

void update_gyro(){
    gyro.read();
    gyro_x_rate = -(gyro.g.y*0.00875 - gyro_offset_y) * time_elapsed_IMU/ 1000;
    gyro_y_rate = (gyro.g.x*0.00875 - gyro_offset_x) * time_elapsed_IMU/ 1000;
    gyro_z_rate = -(gyro.g.z*0.00875 - gyro_offset_z) * time_elapsed_IMU/ 1000;

    
    gyro_x += gyro_x_rate;
    gyro_y += gyro_y_rate;
    gyro_z += gyro_z_rate;
  }

void update_acc(){
    accel.getEvent(&accel_event);
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {
      acc_x = orientation.pitch;
      acc_y = orientation.roll;
    }
    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    { 
      if(orientation.heading != 0 ){
        if(orientation.heading > 330 && fmod(acc_z+abs(full_rotation_count)*360, 360) < 30){
          full_rotation_count --;
          }
        if(orientation.heading < 30 && fmod(acc_z+abs(full_rotation_count)*360, 360) > 330){
          full_rotation_count ++;
          } 
        acc_z = orientation.heading + full_rotation_count * 360;  
        } 
    }
  }

void compute_attitude(){
    pitch = (1- alpha) * (pitch + gyro_x_rate) + alpha * acc_x;
    roll = (1- alpha) * (roll + gyro_y_rate) + alpha * acc_y;
    yaw = (1- alpha) * (yaw + gyro_z_rate) + alpha * acc_z;
  }

void map_into_circle(){
    float vir_pitch_rad = att_pitch_should * PI/ 180;
    float vir_roll_rad = att_roll_should * PI/ 180;
    float x,y;

    x = compute_x_from_p_r(vir_pitch_rad,vir_roll_rad);
    y = compute_y_from_p_r(vir_pitch_rad,vir_roll_rad);

    //Serial.print(x); Serial.print(" ");
    //Serial.print(y); Serial.print(" ");
    
    float normal_length = sqrt(pow(x,2) + pow(y,2));
    if (normal_length != 0){
    float z = sqrt(1-pow(normal_length,2));

    x = x * sqrt(1- 0.5* pow(y/0.26,2));
    y = y * sqrt(1- 0.5* pow(x/0.26,2));


    Serial.print(x); Serial.print(" ");
    Serial.print(y); Serial.print(" ");
    
    att_pitch_should = acos(x/ z) * 180 /PI - 90;
    att_roll_should = acos(y/ z) * 180 /PI - 90;
    
    Serial.print(att_pitch_should); Serial.print(" ");
    Serial.print(att_roll_should); Serial.print(" ");
    
    }
   }

void compute_rotation_compensation(){
    float yaw_diff = yaw - yaw_offset;
    float vir_pitch_rad = att_pitch_should * PI/ 180;
    float vir_roll_rad = att_roll_should * PI/ 180;
    float yaw_diff_rad = yaw_diff * PI/ 180; 

    float x,y;
    float x_new, y_new;  

    x = compute_x_from_p_r(vir_pitch_rad,vir_roll_rad);
    y = compute_y_from_p_r(vir_pitch_rad,vir_roll_rad);
    x_new = compute_new_x_from_x(x,y,yaw_diff_rad);
    y_new = compute_new_y_from_y(x,y,yaw_diff_rad);
    
    att_pitch_should = acos(compute_pitch_from_new_x_y(x_new, y_new)) * 180 /PI - 90;
    att_roll_should = acos(compute_roll_from_new_x_y(x_new, y_new)) * 180 /PI - 90;

  }

float compute_pitch_from_new_x_y(float x, float y){
    float p;
    p = x / sqrt(1- pow(y,2));
    return p;
  }

float compute_roll_from_new_x_y(float x, float y){
    float r;
    r = y / sqrt(1- pow(x,2));
    return r;
  }

float compute_new_x_from_x(float x, float y, float theta){
    float x_new ;
    x_new = cos(theta)*x - sin(theta) *y;
    return x_new;
  }

float compute_new_y_from_y(float x, float y, float theta){
    float y_new;
    y_new = sin(theta)*x + cos(theta) *y;
    return y_new;
  }

float compute_x_from_p_r(float p, float r){
     float x;
     float N = - sin(p)* cos(r);
     float D = sqrt(1 - pow(sin(p)*sin(r),2));
     x = N/D;
     if(!isnan(x)){
          return x;
        }
     return 0; 
  }

float compute_y_from_p_r(float p, float r){
     float y;
     float N = - sin(r)* cos(p);
     float D = sqrt(1 - pow(sin(p)*sin(r),2));
     y = N/D;
     if(!isnan(y)){
          return y;
        }
     return 0; 
  }
