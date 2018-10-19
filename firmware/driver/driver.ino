#include <Servo.h>
#define ZEROPOINT 1500
#define BUFF_SIZE 64
//#define DEBUG

//drive 7 & 11, unused 12-13 & 44-46
//const char pwm[] = {2,3,4,5,6,7,11,12,13,44,45,46};
const char solenoid[] = {39,40,41,42,43,44};
const char analog[] = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};
const char digital[] = {22,23,24,25,26,27,28,29,30,31,32,33,34,35,36};
Servo servos[6];

//A1,B1,A2,B2
const char encoder[4] = {18,19,20,21};

char tx_buffer[BUFF_SIZE];
volatile int64_t encoderCounts[] = {0,0};

//PID vars
float kp[] = {0.02175,0.02175};
float ki[] = {0,0};
float kd[] = {0.000451,0.000451};
int prv[] = {0,0}; //previous error from PID
int throttle[] = {0,0}; //counts per second; error correction after running PID
float ierr[] = {0,0}; //elapsed error
float err[] = {0,0}; 
float sp[] = {0,0}; ///PID Set Point/////
char pid_flag[] = {0,0}; //set this after running an immediate PID so we have a reference point
uint32_t pid_time[] = {100000,100000}; //time elapsed since our last PID routine
uint32_t DT = 10000; //microseconds to wait before performing PID iteration
float vel[] =  {0,0};
uint64_t pos[] = {0,0};

char rcv_buffer[BUFF_SIZE];
char halt_flag = 1;

int simon_target = 0;

void handleSimon() {
  if(simon_target < 0) {
    if(!digitalRead(digital[0])) {
      servos[2].writeMicroseconds(ZEROPOINT + simon_target);
    } else {
      servos[2].writeMicroseconds(ZEROPOINT);
    }
  } else if(simon_target > 0) {
    if(!digitalRead(digital[1])) {
      servos[2].writeMicroseconds(ZEROPOINT + simon_target);
    } else {
      servos[2].writeMicroseconds(ZEROPOINT);
    }
  } else {
    servos[2].writeMicroseconds(ZEROPOINT);
  }
}

void setup()
{
  Serial.begin(1000000);
  while(!Serial); //wait for UART to initialize.
  Serial.println("Beep boop, I am a robot.");

  for(char i = 22; i < 36; i++)
  {
    pinMode(i, INPUT_PULLUP);
  }

  for(char i = 39; i < 45; i++)
  {
    pinMode(i, OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(encoder[0]), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder[1]), encoder1B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder[2]), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder[3]), encoder2B_ISR, CHANGE);

  buffer_Flush(rcv_buffer);

  for(int i = 13; i > 7; i--)
  {
    servos[13-i].attach(i);
  
  }
}

void loop()
{
    if(digitalRead(digital[2]) == 0)
    {
      halt_flag = 1;
      Serial.println("ESTOP!");
    }
    receiveBytes();
    if(!halt_flag)
    {
      pid0();
      pid1();
      handleSimon();
    }
    else
    {
       ierr[0] = 0;
       ierr[1] = 0;
       sp[0] = 0;
       sp[1] = 0;
  
       for(char i = 0; i < 6; i++)
       {
        servos[i].writeMicroseconds(ZEROPOINT);
       }
       digitalWrite(solenoid[0], LOW);
    }
}

void encoder1A_ISR()
{
  noInterrupts();
  if(digitalRead(encoder[0]) == HIGH)
  {
    if(digitalRead(encoder[1]) == LOW)
    {
      encoderCounts[0]++; //clockwise
    }
    else
    {
      encoderCounts[0]--; //counterclockwise
    }
  }
  else
  {
    if(digitalRead(encoder[1]) == HIGH)
    {
      encoderCounts[0]++; //clockwise
    }
    else
    {
      encoderCounts[0]--; //counterclockwise
    }
  }
  interrupts();
}

void encoder1B_ISR()
{
  noInterrupts();
  if(digitalRead(encoder[1]) == HIGH)
  {
    if(digitalRead(encoder[0]) == HIGH)
    {
      encoderCounts[0]++; //clockwise
    }
    else
    {
      encoderCounts[0]--; //counterclockwise
    }
  }
  else
  {
    if(digitalRead(encoder[0]) == LOW)
    {
      encoderCounts[0]++; //clockwise
    }
    else
    {
      encoderCounts[0]--; //counterclockwise
    }
  }
  interrupts();
}

void encoder2A_ISR()
{
  noInterrupts();
   if(digitalRead(encoder[2]) == HIGH)
  {
    if(digitalRead(encoder[3]) == LOW)
    {
      encoderCounts[1]++;
    }
    else
    {
      encoderCounts[1]--;
    }
  }
  else
  {
    if(digitalRead(encoder[3]) == HIGH)
    {
      encoderCounts[1]++;
    }
    else
    {
      encoderCounts[1]--;
    }
  }
  interrupts();
}

void encoder2B_ISR()
{
  noInterrupts();
  if(digitalRead(encoder[3]) == HIGH)
  {
    if(digitalRead(encoder[2]) == HIGH)
    {
      encoderCounts[1]++;
    }
    else
    {
      encoderCounts[1]--;
    }
  }
  else
  {
    if(digitalRead(encoder[2]) == LOW)
    {
      encoderCounts[1]++;
    }
    else
    {
      encoderCounts[1]--;
    }
  }
  interrupts();
}

void buffer_Flush(char *ptr)
{
  for(int i = 0; i < BUFF_SIZE; i++)
  {
    ptr[i] = 0;
  }
}

void pid0()
{
  float t;

  /*  The pid_flag variable is used to tell us our PID loop has recorded an initial sample.
   *  Since to calculate velocity we need two readings (read, wait, read), this flag lets us
   *  know that the first reading has happened so we can wait for a second reading before
   *  calculating the PID loop output.
   *  
   *  The time at which this reading is recorded so we can make accurate calculations later.
   */
  if(!pid_flag[0])
  {
    err[0] = encoderCounts[0];
    pid_flag[0] = 1;
    pid_time[0] = micros();
    
  }

  /*  We must check if enough time has passed since our initial reading for us to take another.
   *  Since it may have taken more than DT microseconds to run this loop again (interrupt stalls,
   *  serial parsing, etc. can slow it down.) we check if the difference between the current
   *  runtime and our pid_time sample are greater than the DT we need.
   * 
   */
  if((micros() - pid_time[0]) >= DT)
  {
    t = ((float)(micros() - pid_time[0]))/1000000.0; //compute time in seconds
    //Serial.print(t);
    //Serial.print(" ");
    err[0] = encoderCounts[0] - err[0]; //find difference in counts (ie. calculate change in position)
    pos[0] = (uint64_t)err[0];
    err[0] /= t; //find velocity by dividing by change in time (ie. compute derivative of position)
    vel[0] = err[0];
    /*Serial.print(8000.0);
    Serial.print(" ");
    Serial.print(sp[0]+250);
    Serial.print(" ");
    Serial.print(sp[0]-250);
    Serial.print(" ");
    Serial.print(0.0);
    Serial.print(" ");
    Serial.print(err[0]);
    Serial.print(" ");
    Serial.println(sp[0]);
    */
    err[0] = sp[0] - err[0]; //find error by seeing how far away from setpoint we are

   
    ierr[0] += err[0]*t; //integral error
    float output;

    /* Compute PID loop output
     * The output of a PID loop is the sum of it's errors compensated for by the 
     * chosen coefficients.
     * 
     * P term: controls how quickly you approach the set point
     * I term: controls how far above or below the setpoint you oscillate
     * D term: controls how much oscillation is allowed
     */
    output = kp[0] * err[0] + ki[0]*ierr[0] + kd[0]*((err[0] - prv[0])/t);

    /*  Clamp the output
     *  The output can not be allowed to change too sharply so the PID response
     *  is clamped. This slows the rate of change but prevents sharp or jittery
     *  responses when set points change or load changes.
     */
    if(output > 10)
    {
      output = 10;
      
    }
    if(output < -10)
    {
      output = -10;
      
    }

    throttle[0] += output; //adjust throttle with PID output
    
    /*  Clamp throttle
     *  Throttle is what we use to change in units from counts/sec to pulse
     *  width in microseconds for the ESC control signal. Some precision is lost
     *  since we change to integer but this can not be avoided.
     */
    if(throttle[0] >500)
    {
      throttle[0] = 500;
      
    }
     if(throttle[0] <-500)
    {
      throttle[0] = -500;
      
    }

    prv[0] = err[0]; //save previous error for use in next iteration.
    pid_flag[0] = 0; //reset PID flag
    servos[0].writeMicroseconds(ZEROPOINT+throttle[0]); //update motor output
  }  
}

void pid1()
{
  float t;
  float output;
  if(!pid_flag[1])
  {
    err[1] = encoderCounts[1];
    pid_flag[1] = 1;
    pid_time[1] = micros();
  }
  if(micros() - pid_time[1] >= DT)
  {
    t = ((float)(micros() - pid_time[1]))/1000000.0; //convert t to seconds
  
    err[1] = encoderCounts[1] - err[1];
    pos[1] = (uint64_t)err[1];
    err[1] /= t;
    vel[1] = err[1];
    
    err[1] = sp[1] - err[1];
    ierr[1] += err[1]*t;
    
    output = kp[1] * err[1] + ki[1]*ierr[1] + kd[1]*((err[1] - prv[1])/t);
    if(output > 10)
    {
      output = 10;
      
    }
    if(output < -10)
    {
      output = -10;
      
    }
    throttle[1] += output;
    
    prv[1] = err[1];
    pid_flag[1] = 0;

    if(throttle[1] > 500)
    {
      throttle[1] = 500;
    }
    if(throttle[1] < -500)
    {
      throttle[1] = -500;
    }
    servos[1].writeMicroseconds(ZEROPOINT+throttle[1]);
  } 
}

void receiveBytes()
{
  static byte index = 0;
  char terminator = '\r';
  while(Serial.available() > 0)
  {
    rcv_buffer[index] = Serial.read();
    if(rcv_buffer[index] == terminator)
    {
      index = 0;
      parseCommand();
      buffer_Flush(rcv_buffer);
    }
    else
    {
      index++;
      if(index >= 64)
      {
        Serial.println("buffer overflow");
        index = 0;
        buffer_Flush(rcv_buffer);
      }
    }
  }

}

void parseCommand()
{
    char command = rcv_buffer[0];

    int id;
    int value;
    char motor = 0;
    int pickSolenoid = 0;
    int solenoidState = 0;
    int pidMotor = 0;
    char pidInput = 0;
    float pidValue = 0;
    int speed;
    
    switch(command)
    {
        case 'A':
        case 'a':

        #ifdef DEBUG
          Serial.println("A switch");
        #endif
          
          int pinNum;
          sscanf(&rcv_buffer[1], " %d\r", &pinNum);
          Serial.println(analogRead(analog[pinNum]));
          break;
        case 'C':
        case 'c':
        
        #ifdef DEBUG
          Serial.println("C switch");
          #endif 
          
        //command e.g. PID control - c motorNum pidInput pidValue (pidInput can be P, D, I, or S (setpoint))
          int32_t fixedPoint;
          sscanf(&rcv_buffer[1], " %d %c %ld\r", &pidMotor, &pidInput, &fixedPoint);
          pidValue = fixedPoint / 1000.0;
          
          #ifdef DEBUG 
          Serial.println(pidMotor);
          Serial.println(pidInput);
          Serial.println(fixedPoint);
          Serial.println(pidValue);
          #endif
          
          if(pidInput == 'p' || pidInput == 'P')
          {
             kp[pidMotor] = pidValue;
          
          }else if(pidInput == 'D' || pidInput == 'd')
          {
            kd[pidMotor] = pidValue;  
          
          }else if(pidInput == 'I' || pidInput == 'i')
          {
             ki[pidMotor] = pidValue;
          
          }else if(pidInput == 'S' || pidInput == 's')
          {
            //Serial.println("Updating setpoint.");
            sp[pidMotor] =  pidValue;
          }
          else if(pidInput == 'T' || pidInput == 't')
          {
            DT = (uint32_t)pidValue;
          }
        
        break; 
       
        //Pulse Width Modulation (PWM)
        case 'P':
        case 'p':

          #ifdef DEBUG
          Serial.println("P switch");
          #endif
           
          sscanf(&rcv_buffer[1], " %d %d\r", &id, &value);
          analogWrite(pwm[id], value); // Write the value to the pin
        break;
       
        //encoder: -1 reads both encoder values
        case 'E':
        case 'e':
        
          #ifdef DEBUG
           Serial.println("E switch");
           #endif
           
           int val;
           sscanf(&rcv_buffer[1], " %d\r", &val);
           
           if(val == -1)
           {
             itoa(encoderCounts[0],tx_buffer,10); // integer to string
             Serial.print(tx_buffer);
             Serial.print(" ");
             Serial.print(vel[0]);
             Serial.print(" ");
             buffer_Flush(tx_buffer);          // 0 out everything in buffer
             itoa(encoderCounts[1],tx_buffer,10);
             Serial.print(tx_buffer);
             Serial.print(" ");
             Serial.println(vel[1]);
             buffer_Flush(tx_buffer);
             
           }else if(val == 0)
           {
             itoa(encoderCounts[0],tx_buffer,10);
             Serial.println(tx_buffer);
             Serial.print(" ");
             Serial.println(vel[0]);
             buffer_Flush(tx_buffer);
            
           }else if(val == 1)
           {
             itoa(encoderCounts[1],tx_buffer,10);
             Serial.print(" ");
             Serial.println(vel[1]);
             buffer_Flush(tx_buffer);
           }
           
          break;

        //solenoid
          case 'S':
          case 's':
          
           #ifdef DEBUG
            Serial.println("S switch");
            #endif
            
            sscanf(&rcv_buffer[1], " %d %d\r", &pickSolenoid, &solenoidState);
            digitalWrite(solenoid[pickSolenoid],solenoidState);
          break;

        //digital read
          case 'D':
          case 'd':
            int pinNumber;
            
            sscanf(&rcv_buffer[1], " %d\r", &pinNumber);
            
            #ifdef DEBUG
            Serial.println("D switch");
            Serial.println(digital[pinNumber]);
            #endif
            
            Serial.println(digitalRead(digital[pinNumber]));
          break;

        //motor
          case 'M':
          case 'm':
          
          #ifdef DEBUG
          Serial.println("M switch");
          Serial.println(motor);
          Serial.println(speed);
          #endif
          
          sscanf(&rcv_buffer[1], " %c %d\r", &motor, &speed);

          if(!halt_flag)
          {
            if(motor == '0')
            {
              servos[0].writeMicroseconds(ZEROPOINT + speed);
            }
            else if(motor == '1')
            {
              servos[1].writeMicroseconds(ZEROPOINT + speed);
            }
            else if(motor == '2')
            {
              servos[2].writeMicroseconds(ZEROPOINT + speed);                    
            }
           else if(motor == '3')
            {
              servos[3].writeMicroseconds(ZEROPOINT + speed);                    
            }
            else if(motor == '4')
            {
              servos[4].writeMicroseconds(ZEROPOINT + speed);                    
            }
            else if(motor == '5')
            {
              servos[5].writeMicroseconds(ZEROPOINT + speed);                    
            }
          }
           break;

           //Immediately stop the robot and reset PID
           case 'H':
           case 'h':

           #ifdef DEBUG
           Serial.println("H switch");
           #endif
           
           char state;
           sscanf(&rcv_buffer[1], " %d\r", &halt_flag);

           break;
          case 'N':
          case 'n':
             // while(Serial.available() < 2);
             sscanf(&rcv_buffer[1], " %d\r", &simon_target);
           break;
           
        default:
            Serial.println("Error, serial input incorrect  ");
            while (Serial.available() > 0) 
            {
              Serial.read();
            }
    }
}

