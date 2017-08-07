/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

//SetPointInfo leftPID, rightPID;
SetPointInfo wheel1PID, wheel2PID, wheel3PID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   wheel1PID.TargetTicksPerFrame = 0.0;
   wheel1PID.Encoder = readEncoder(WHEEL1);
   wheel1PID.PrevEnc = wheel1PID.Encoder;
   wheel1PID.output = 0;
   wheel1PID.PrevInput = 0;
   wheel1PID.ITerm = 0;

   wheel2PID.TargetTicksPerFrame = 0.0;
   wheel2PID.Encoder = readEncoder(WHEEL2);
   wheel2PID.PrevEnc = wheel2PID.Encoder;
   wheel2PID.output = 0;
   wheel2PID.PrevInput = 0;
   wheel2PID.ITerm = 0;
   
   wheel3PID.TargetTicksPerFrame = 0.0;
   wheel3PID.Encoder = readEncoder(WHEEL3);
   wheel3PID.PrevEnc = wheel3PID.Encoder;
   wheel3PID.output = 0;
   wheel3PID.PrevInput = 0;
   wheel3PID.ITerm = 0;
/*   
   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
*/
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  wheel1PID.Encoder = readEncoder(WHEEL1);
  wheel2PID.Encoder = readEncoder(WHEEL2);
  wheel3PID.Encoder = readEncoder(WHEEL3);
  //rightPID.Encoder = readEncoder(RIGHT);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (wheel1PID.PrevInput != 0 || wheel2PID.PrevInput != 0 || wheel3PID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  //doPID(&rightPID);
  doPID(&wheel1PID);
  doPID(&wheel2PID);
  doPID(&wheel3PID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(wheel1PID.output, wheel2PID.output, wheel3PID.output);
}

