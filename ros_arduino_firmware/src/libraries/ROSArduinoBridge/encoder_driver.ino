/* *************************************************************
   Encoder definitions

   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   ************************************************************ */

#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table

  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
  	static uint8_t enc_last=0;

	enc_last <<=2; //shift previous state two places
	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits

  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
        static uint8_t enc_last=0;

	enc_last <<=2; //shift previous state two places
	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits

  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else {
      right_enc_pos=0L;
      return;
    }
  }
#elif defined(ARDUINO_MY_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;

  void initEncoders(){
    pinMode(LEFT_ENC_A, INPUT);
    pinMode(LEFT_ENC_B, INPUT);
    pinMode(RIGHT_ENC_A, INPUT);
    pinMode(RIGHT_ENC_B, INPUT);

    attachInterrupt(0, leftEncoderEvent, CHANGE);
    attachInterrupt(2, rightEncoderEvent, CHANGE);
  }

  // encoder event for the interrupt call
  void leftEncoderEvent() {
    if (digitalRead(LEFT_ENC_A) == HIGH) {
      if (digitalRead(LEFT_ENC_B) == LOW) {
        left_enc_pos++;
      }
      else {
        left_enc_pos--;
      }
    }
    else {
      if (digitalRead(LEFT_ENC_B) == LOW) {
        left_enc_pos--;
      }
      else {
        left_enc_pos++;
      }
    }
  }

  // encoder event for the interrupt call
  void rightEncoderEvent() {
    if (digitalRead(RIGHT_ENC_A) == HIGH) {
      if (digitalRead(RIGHT_ENC_B) == LOW) {
        right_enc_pos++;
      }
      else {
        right_enc_pos--;
      }
    }
    else {
      if (digitalRead(RIGHT_ENC_B) == LOW) {
        right_enc_pos--;
      }
      else {
        right_enc_pos++;
      }
    }
  }

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return -right_enc_pos;    // It's just because my right encoder get reverse value so if yours is normal, don't add "-"
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else {
      right_enc_pos=0L;
      return;
    }
  }
#elif defined(DF_ARDUINO_MY_COUNTER)
  volatile long duration1 = 0L;                                  //the number of the pulses of Moter1 in the interval
  volatile long duration2 = 0L;                                  //the number of the pulses of Moter2 in the interval
  volatile long duration3 = 0L;                                  //the number of the pulses of Moter3 in the interval
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  byte encoder1PinALast;
  byte encoder2PinALast;
  byte encoder3PinALast;
  boolean Direction1;                              //the rotation Direction1 
  boolean Direction2;                              //the rotation Direction2 
  boolean Direction3;                              //the rotation Direction3

  void initEncoders(){
	  Direction1 = true;//default -> Forward  
	  Direction2 = true;//default -> Forward  
	  Direction3 = true;//default -> Forward  
	  pinMode(encoder1pinB,INPUT);  
	  pinMode(encoder2pinB,INPUT);  
	  pinMode(encoder3pinB,INPUT);  
	  attachInterrupt(5, wheelSpeed1, CHANGE);
	  attachInterrupt(4, wheelSpeed2, CHANGE);
	  attachInterrupt(2, wheelSpeed3, CHANGE);
  }

  void wheelSpeed1()  //motor1 speed count
  {
	  int Lstate = digitalRead(encoder1pinA);
	  if((encoder1PinALast == LOW) && Lstate==HIGH)
	  {
		  int val = digitalRead(encoder1pinB);
		  if(val == LOW && Direction1)
		  {
			  Direction1 = false; //Reverse
		  }
		  else if(val == HIGH && !Direction1)
		  {
			  Direction1 = true;  //Forward
		  }
	  }
	  encoder1PinALast = Lstate;
	  
	  if(!Direction1)  duration1++;
	  else  duration1--;
  }

  void wheelSpeed2()  //motor2 speed count
  {
	  int Lstate = digitalRead(encoder2pinA);
	  if((encoder2PinALast == LOW) && Lstate==HIGH)
	  {
		  int val = digitalRead(encoder2pinB);
		  if(val == LOW && Direction2)
		  {
			  Direction2 = false; //Reverse
		  }
		  else if(val == HIGH && !Direction2)
		  {
			  Direction2 = true;  //Forward
		  }
	  }
	  encoder2PinALast = Lstate;
	  
	  if(!Direction2)  duration2++;
	  else  duration2--;
  }
  
  void wheelSpeed3()  //motor3 speed count
  {
	  int Lstate = digitalRead(encoder3pinA);
	  if((encoder3PinALast == LOW) && Lstate==HIGH)
	  {
		  int val = digitalRead(encoder3pinB);
		  if(val == LOW && Direction3)
		  {
			  Direction3 = false; //Reverse
		  }
		  else if(val == HIGH && !Direction3)
		  {
			  Direction3 = true;  //Forward
		  }
	  }
	  encoder3PinALast = Lstate;
	  
	  if(!Direction3)  duration3++;
	  else  duration3--;
  }

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
	  if (i == WHEEL1) return duration1;
	  else if(i == WHEEL2) return duration2;    // It's just because my right encoder get reverse value so if yours is normal, don't add "-"
	  else return duration3;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
	  if (i == WHEEL1){
		  duration1=0L;
		  return;
	  } else if(i == WHEEL2){
		  duration2=0L;
		  return;
	  } else {
		  duration3=0L;
		  return;
	  }
  }
#else
  #error A encoder driver must be selected!
#endif

  /* Wrap the encoder reset function */
  void resetEncoders() {
    resetEncoder(WHEEL1);
    resetEncoder(WHEEL2);
	resetEncoder(WHEEL3);
  }

#endif
