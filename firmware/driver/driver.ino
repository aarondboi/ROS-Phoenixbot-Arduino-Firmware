#include <Servo.h>
#define ZEROPOINT 1500
#define BUFF_SIZE 64

//drive 7 & 11, unused 12-13 & 44-46
const char pwm[] = {2,3,4,5,6,7,11,12,13,44,45,46};
const char solenoid[] = {39,40,41,42,43,44};
const char analog[] = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};
const char digital[] = {22,23,24,25,26,27,28,29,30,31,32,33,34,35,36};
Servo servos[6];

//A1,B1,A2,B2
const char encoder[4] = {18,19,20,21};

char buffer[BUFF_SIZE];
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
uint32_t DT = 10000; //microseconds to wait before performing PID iteration.

char halt_flag = 0;

void setup()
{
  Serial.begin(115200);
  while(!Serial); //wait for UART to initialize.
  Serial.println("Beep boop, I am a robot.");

  for(char i = 22; i < 36; i++)
  {
    pinMode(i, INPUT);
  }

  for(char i = 39; i < 45; i++)
  {
    pinMode(i, OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(encoder[0]), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder[1]), encoder1B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder[2]), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder[3]), encoder2B_ISR, CHANGE);

  for(int i = 13; i > 7; i--)
  {
    servos[13-i].attach(i);
  
  }
}

void loop()
{
    char command;

    if((command = Serial.read()) >= 0)
    {
        while(Serial.available() < 1);
        Serial.read(); //readout space
        int id;
        int value;
        char motor = 0;
        char pickSolenoid = 0;
        char solenoidState = 0;
        char pidMotor = 0;
        char pidInput = 0;
        float pidValue = 0;
        int speed;
        
        switch(command)
        {
            case 'A':
            case 'a':
              int pinNum;
              while(Serial.available() < 2);
              pinNum = (int)Serial.parseInt();
              Serial.read();//read \r
              Serial.println(analogRead(analog[pinNum]));
              break;
            case 'C':
            case 'c':
            //command e.g. PID control - c motorNum pidInput pidValue (pidInput can be P, D, I, or S (setpoint))
              while(Serial.available() < 5);
              pidMotor = Serial.parseInt();
              Serial.read();
              pidInput = Serial.read();   
              pidValue = Serial.parseFloat();
              
                /*Serial.write(pidMotor);
                Serial.print(" ");
                Serial.print(pidInput);
                Serial.print(" ");
                Serial.println(pidValue);*/

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
            
            Serial.read();
            break; 
           
            //Pulse Width Modulation (PWM)
            case 'P':
            case 'p':
            
              while(Serial.available() < 3);
              id = (int)Serial.parseInt(); //Read in int
              value = (int)Serial.parseInt(); //Read in second int
              analogWrite(pwm[id], value); // Write the value to the pin
              Serial.read(); //Read out extra \r
            break;
           
            //encoder: -1 reads both encoder values
            case 'E':
            case 'e':
            
              while(Serial.available() < 2);
               if(Serial.parseInt() == -1)
               {
                 itoa(encoderCounts[0],buffer,10); // integer to string
                 Serial.println(buffer);
                 buffer_Flush(buffer);          // 0 out everything in buffer
                 itoa(encoderCounts[1],buffer,10);
                 Serial.println(buffer);
                 buffer_Flush(buffer);
                 
               }else if(Serial.parseInt() == 0)
               {
                 itoa(encoderCounts[0],buffer,10);
                 Serial.println(buffer);
                 buffer_Flush(buffer);
                
               }else if(Serial.parseInt() == 1)
               {
                 itoa(encoderCounts[1],buffer,10);
                 Serial.println(buffer);
                 buffer_Flush(buffer);
               }
               
               Serial.read(); // eats the char return /r
               
              break;

            //solenoid
              case 'S':
              case 's':
                while(Serial.available() < 3);
                pickSolenoid = Serial.parseInt();
                solenoidState  = Serial.parseInt();
                digitalWrite(solenoid[pickSolenoid],solenoidState);
                Serial.read();
              break;

            //digital read
              case 'D':
              case 'd':
                int pinNumber;
                
                while(Serial.available() < 2);
                pinNumber = (int)Serial.parseInt();
                Serial.read();//read \r
                Serial.println(digitalRead(digital[pinNumber]));
              break;

            //motor
              case 'M':
              case 'm':

              while(Serial.available() < 3);
              motor = Serial.read();
              speed = (int)Serial.parseInt();
              
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
               Serial.read(); //Read out extra \r
               break;

               //Immediately stop the robot and reset PID
               case 'H':
               case 'h':
              
              while(Serial.available() < 1);
               ierr[0] = 0;
               ierr[1] = 0;
               sp[0] = 0;
               sp[1] = 0;
               servos[0].write(ZEROPOINT);
               servos[1].write(ZEROPOINT);
               halt_flag = 1;
               Serial.read(); //Read out extra \r

               break;
               
            default:
                Serial.println("Error, serial input incorrect  ");
                while (Serial.available() > 0) 
                {
                  Serial.read();
                }
        }
    }
    //TODO: Set up timer to interrupt every 10ms and determine velocity. Alternatively we can do this through the PID loop 
    if(!halt_flag)
    {
      pid0();
      pid1();
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

void buffer_Flush(char *pt)
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
    err[0] /= t; //find velocity by dividing by change in time (ie. compute derivative of position)

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
    err[1] /= t;
   
    /*Serial.print(8000.0);
    Serial.print(" ");
    Serial.print(sp[1]+250);
    Serial.print(" ");
    Serial.print(sp[1]-250);
    Serial.print(" ");
    Serial.print(0.0);
    Serial.print(" ");
    Serial.print(err[1]);
    Serial.print(" ");
    Serial.println(sp[1]);*/
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
