/*
State machine m1 is for motion control
the states are as follows:
1 = Stopped
2 = Forward
3 = Avoiding Obstacle
4 = Turn Left
5 = Turn Right
6 = Reverse
MotionStop dictates how much time is spend in any state.
*/

State m1s1h(){ // m1s1 is --- Motion:Stopped
  Serial.println("Motion:Stopped (State 1)");
  halt();
}//m1s1h()

State m1s1b(){
  if(m1.Timeout(5000)){ // Maximum 5 seconds idle time
    m1.Set(m1s2h, m1s2b);
    Serial.println("changing to Motion:Forward (State 2)");
  };
}//m1s1b()
