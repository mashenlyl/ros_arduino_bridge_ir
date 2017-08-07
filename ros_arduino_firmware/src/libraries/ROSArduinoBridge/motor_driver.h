/***********************************************************************
   Motor driver function definitions - by James Nugen and Chaoyang Liu
   ***********************************************************************/

  void initMotorController();
  void setMotorSpeed(int i, int spd);
  void setMotorSpeeds(int wheel1Speed, int wheel2Speed, int wheel3Speed);
  #ifdef L298N_DUAL_HBRIDGE
    // motor one
    #define ENA 5
    #define IN1 7
    #define IN2 8
    // motor two
    #define ENB 6
    #define IN3 9
    #define IN4 10
  #endif
  #ifdef DF_DUAL_HBRIDGE
 //Motor Driver variables
    #define  M1   14     //M1 Direction Control
    #define  M2   15     //M2 Direction Control
    #define  M3   16     //M3 Direction Control
    #define  E1   2     //M1 Speed Control
    #define  E2   3     //M2 Speed Control
    #define  E3   4     //M3 Speed Control
  #endif
