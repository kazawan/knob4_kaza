#include <SimpleFOC.h>
float point = 0 ;
float point2 = 18.84;
int count = 0 ;
float temp;
float mAngle = 0;
float preMAngle = 0;


MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
//BLDCDriver3PWM driver = BLDCDriver3PWM(10, 5, 6, 8);
//BLDCDriver3PWM driver = BLDCDriver3PWM(11, 3, 10, 7);



void setup() {

  
  sensor.init();
  
  motor.linkSensor(&sensor);

  
  driver.init();
  driver.voltage_power_supply = 5;
  
  motor.linkDriver(&driver);
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 5;
  motor.PID_velocity.D = 0;

  motor.LPF_velocity.Tf = 0.01;

  motor.P_angle.P = 30;

  motor.velocity_limit = 5;


  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  Serial.begin(115200);
//  while(!Serial);
  motor.useMonitoring(Serial);
   motor.init();
 
  motor.initFOC();

   Serial.println("Motor ready.");
  _delay(1000);
//  motor.move(0);
  
}

void loop() {

  motor.loopFOC();
  
  

 
  
  if( motor.shaft_angle <= point ){
   //Serial.print("start");Serial.println(motor.shaft_angle);
    motor.move(5*(point-motor.shaft_angle));
    
    
  }else if(motor.shaft_angle >= point2 ){
    motor.move(5*(point2-motor.shaft_angle));
    //Serial.print("end");Serial.println(motor.shaft_angle);
  }else if(point2 > motor.shaft_angle > point){
    //Serial.print("free1");Serial.println(motor.shaft_angle);
    motor.move();
  }


  
  angleCheck();
  
  
  

  
  
}



void angleCheck(){
  
  if(motor.shaft_angle > point2){
    
    mAngle = point2;
  }else if(motor.shaft_angle < point){
    
    mAngle = point;
  }else{
    mAngle = motor.shaft_angle;
  }

  count = map(mAngle,point,point2,0,1024);
  Serial.println(count);
}


//int encoder(){
//  temp = angleCheck();
//  if(temp - preMAngle > 0.2){
//    count++;
//    countCheck();
//    Serial.print("+++++");
//    Serial.println(count);
//    preMAngle = temp;
//  }else if(temp - preMAngle < -0.2){
//    count--;
//    countCheck();
//    Serial.print("-----");
//    Serial.println(count);
//    preMAngle = temp;
//  }
//  
//}
//
//
//void countCheck(){
//  if(count > 127){
//    count = 127;
//  }else if(count < 0 ){
//    count = 0;
//  }
//}
