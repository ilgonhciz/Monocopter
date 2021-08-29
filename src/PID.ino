float computePID_p(float setpoint, float inp  ){     
        
        error_p = setpoint - inp;                                // determine error
        cumError_p += error_p * time_elapsed;                // compute integral
        rateError_p = (error_p - lastError_p)/time_elapsed;   // compute derivative
 
        float out = kp*error_p + ki*cumError_p + kd*rateError_p;                //PID output               
        lastError_p = error_p;                                //remember current error
        if(abs(out) > 20){
            out = abs(20 / out)*out; 
          }
        Serial.print(out); Serial.print(" ");
        if(!isnan(out)){
            return out;                                        //have function return the PID output  
          }
        return 0;       
}

float computePID_r(float setpoint, float inp  ){     
        
        error_r = setpoint - inp;                                // determine error
        cumError_r += error_r * time_elapsed;                // compute integral
        rateError_r = (error_r - lastError_r)/time_elapsed;   // compute derivative
 
        float out = kp*error_r + ki*cumError_r + kd*rateError_r;                //PID output               
 
        lastError_r = error_r;                                //remember current error
        if(abs(out) > 20){
            out = abs(20 / out)*out; 
          }
        if(!isnan(out)){
            return out;                                        //have function return the PID output  
          }
        return 0;                                       //have function return the PID output
}
