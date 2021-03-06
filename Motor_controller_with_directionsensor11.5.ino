#include <ps2dev.h>
#include <Wire.h>
#include <JY901.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 6); 

  int spd;
  int spd1;
  int Char1;
  int Char2;
  int Char_1;
  int Char_2;
  int Mtrspd_Char1;
  int Mtrspd_Char2;
  int Mtrspd_Char_1;
  int Mtrspd_Char_2;
  int pre_dir = 0;
  int dir=0;                  // initialize, dir must stay in void loop in order to initialize the dir for every loop
  int dir_leftright=0;
  int VL=0;
  int VR=0;
  float referenceData;
  float robotDirection;
  float adjustmentA=0;
  float adjustmentB=0;
  float A=0;
  float B=0;
  float mean_myData=0;
  float coe=0.2;
  float gain=1;
  int cycle=1;
  int num=0;                // in getAngle() loop, for differentiating 1st cycle and other cycle runing sequence
  int num1=0;               // in compareData() loop, for differentiating 1st cycle and other cycle runing sequence
  int num2=0;
  int num3=0;
  int num4=0;
  char mtrA_dir=0;
  char mtrB_dir=0;
  char gain_check;
  char speed;
  
void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600,SERIAL_7E1);
  Serial1.begin(9600,SERIAL_7E1);
  mySerial.begin(9600);
  JY901.StartIIC();
  speed = 100 ; 
  Serial.println("Start");
  pinMode(4,INPUT);      //1 
  pinMode(5,INPUT);
  pinMode(7,INPUT);  //interrupt pin
  pinMode(9,INPUT);
  pinMode(10,INPUT);
  pinMode(11,INPUT);
  pinMode(12,INPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  //attachInterrupt(digitalPinToInterrupt(7),Input_checking , LOW);
}
 /*This function need to be realised 
  1, auto direction correction in forward and reverse based on the feedback value
  2, continuous input checking if not input data set every thing
  3, the bigger the angle error the larger the speed correction
  4, left turn and right turn function; after right turn reset the reference direction
  5. conbine with remote control mode
  */
  
void loop() {      
    detectData();                       // continuous checking the input if no input set speed to zero; 
                                             // 'cycle' for cycle count; receive: Serial.read(),cycle; send: dir,speed   
    moto_control();
    while(Serial1.available()>0){     
      Serial.write(Serial1.read());
    }
    delay(1000);
}

void moto_control(){
    if(dir==1||dir==2){                //Forward or Back
      getAngle();                        // get the direction data from direction sensor;
                                                     //'num' for cycle count; send: referenceData,robotDirection.
                                                     
      compareData();                    // compare angle data;  
                                                   //'num1' for cycle count;  receive: referenceData,robotDirection; send: adjustmentA,adjustmentB.
                                                   
      computeMotorValue();             // compute speed and convert the value to character; receive: speed,adjustmentA,adjustmentB.
      sendtoMotor();                  // write motor speed to AX3500 controller 
      //Serial.print("Cycle ");
      //Serial.println(cycle,DEC);
      cycle++;
    }
                                                           
    else if(dir>=3&&dir<=6){                //left or right
      A=0;
      B=0;
      computeMotorValue();             // compute speed and convert the value to character; receive: dir,speed
      sendtoMotor();                  // write motor speed to AX3500 controller 
      //Serial.print("Cycle ");
      //Serial.println(cycle,DEC);
      cycle++;   
    }
    else if(dir==7){                //Stop
      Stop();   
    }
  /////////////////////////////////////////////////////////////////////////     
     else if(dir==8||dir==9){  
      computeMotorValue();
      VL=1;
      A=0;
     }
    else if(dir==10||dir==11){
      computeMotorValue();            
      VR=1;
      B=0;
     }    
    else if(dir==12&VL==1&VR==1){
      sendtoMotor();                  // write motor speed to AX3500 controller 
      Serial.print("Cycle ");
      Serial.println(cycle,DEC);
      cycle++;
     } 
    
}

int detectData(){                                    //variable speed ,dir,cycle
   if (mySerial.available()>0){
    switchcase();
   }
   
   //Input_checking();
   gain_checking();
   
}

// get the direction data from direction sensor;
float getAngle(){                //variable robotDirection, referenceData, num
   //Serial.println("Get angle Mode");
   float myData[16];
   float AngleValue;
   float sum=0;

  if(num==0){                               // only run for the 1st cycle unless no input coming in for 5s 
   for (int i=1;i<16;i++){
   JY901.GetAngle();
   AngleValue=(float)JY901.stcAngle.Angle[2]/32768*180;
   myData[i]=AngleValue;
   //Serial.print(myData[i]);   Serial.print("  ");
   sum=sum+myData[i];
   }
   mean_myData=sum/15;
   //Serial.println("");
   //Serial.print("The mean value is " );
   //Serial.println(mean_myData);
   num++;
   referenceData=mean_myData;
   //Serial.println("");
  }
  else if(num>=1) {                      // after 1st cycle, program will direct go to here  
   num=1;
   for (int i=1;i<2;i++){
   JY901.GetAngle();
   AngleValue=(float)JY901.stcAngle.Angle[2]/32768*180;
   myData[i]=AngleValue;
   //Serial.print(myData[i]);   Serial.print("  ");
   sum=sum+myData[i];
   }
   mean_myData=sum/1;
   //Serial.println("");
   //Serial.print("The mean value is " );
   //Serial.println(mean_myData);
   robotDirection=mean_myData;
   //Serial.println("");
   }
}

void compareData(){                                  //variable adjustmentA,adjustmentB,num1,num2
     float difference;
     float angle_difference;
     float convertedDirection;
     //Serial.println("Compare data Mode ");
     angle_difference=abs(referenceData-robotDirection);
     if (angle_difference<180){
     difference=referenceData-robotDirection;
     }
     else if (referenceData>0&&angle_difference>=180){
     difference=referenceData-(361+robotDirection);
     convertedDirection=361+robotDirection;
     }
     else if (referenceData<0&&angle_difference>=180){
     difference=referenceData-(-361+robotDirection);
     convertedDirection=-361+robotDirection;
     } 
     float error=difference;                           // The error line must be below the difference line
     
 ///////////////
    int angle_error;
    float angle_error1;
    float angle_error2;

    if (num2==0){    
      angle_error=mean_myData*10;
      angle_error1=angle_error/10;
      num2=1;
      //Serial.print("The angle_error value is " );
      //Serial.println(angle_error,DEC);
      //Serial.print("The angle_error1 value is " );
      //Serial.println(angle_error1,DEC);
      
     } 
     
    else if (num2==1){
        num2=0;
        angle_error=mean_myData*10;
        angle_error2=angle_error/10;
      //Serial.print("The angle_error2 value is " );
      //Serial.println(angle_error2,DEC);
        if (error<-1&&angle_error1>angle_error2){
        coe=coe+0.1;
        delay(80);
        }
        if (error>1&&angle_error1<angle_error2){
        coe=coe+0.1;
        delay(80);
        }
        if (coe>1.5){
        coe=1.5;          
        }
        if (error>-1&error<1){
          coe=0.2;
        }
     }  
     ////////////////
     if(num1==0){
      adjustmentA=0; 
      adjustmentB=0;
      num1++;
     }
     else if(num1>=1){                                 // after 1st cycle, program will direct go to here  
      if(error==0)  {adjustmentA=0; adjustmentB=0;} 
      else if(error>0&&error<1) {adjustmentA=-(0.25*speed*error); adjustmentB=0;} 
      else if(error>1) {adjustmentA=-(0.27*speed*error*coe); adjustmentB=0;} 
      else if(error<0&&error>-1) {adjustmentA=0; adjustmentB=(0.25*speed*error);}
      else if(error<-1) {adjustmentA=0; adjustmentB=(0.27*speed*error*coe);}
      num1=1;
     }
     //Serial.print("The num2 is " );
     //Serial.println(num2,DEC);
     //Serial.print("The coefficient value is " );
     //Serial.println(coe,DEC);
     //Serial.print("The error value is " );
     //Serial.println(error,DEC);
     //Serial.print("The referenceData value is " );
     //Serial.println(referenceData,DEC);
     //Serial.print("The robotDirection value is " );
     //Serial.println(robotDirection,DEC);
     //Serial.println("");
}

int computeMotorValue(){                          //variable adjustmentA,adjustmentB, spd,spd1, Mtrspd_Char1, Mtrspd_Char1, Mtrspd_Char_1, Mtrspd_Char_2
     //Serial.println("Compute motor value mode");  
     switch (dir) { 
      case 1:                                       //FOWARD MODE  
      A=adjustmentA;
      B=adjustmentB; 
    //A motor speed conversion
      mtrA_dir=65;            //A    
      MotorA_Spd();      
    //B motor speed conversion
      mtrB_dir=98;            //b   
      MotorB_Spd();
      break; 
 /////////////////////////////////////////////////////////////////   
      case 2:                                     //REVERSE MODE                          
      A=adjustmentB;
      B=adjustmentA; 
    //A motor speed conversion
      mtrA_dir=97;            //a    
      MotorA_Spd(); 
      
    //B motor speed conversion
      mtrB_dir=66;            //B    
      MotorB_Spd(); 
      break;
 /////////////////////////////////////////////////////////////////
      case 3:                                    //LEFT MODE(left forward)
        if(pre_dir != dir){
            pre_dir = dir;
            Serial.println("The gain value is");
            Serial.println(gain,DEC);    
            mtrA_dir=65;            //A    
            MotorA_SpdwithGain();     
            //B motor speed conversion
            mtrB_dir=98;            //b   
            MotorB_Spd();  
        }
      break;
 /////////////////////////////////////////////////////////////////
      case 4:                                    //LEFT MODE(left back)
        if(pre_dir != dir){
            pre_dir = dir;
            Serial.println("The gain value is");
            Serial.println(gain,DEC);    
            mtrA_dir=97;            //a    
            MotorA_SpdwithGain();        
            //B motor speed conversion
            mtrB_dir=66;            //B   
            MotorB_Spd(); 
        }
      break;
 /////////////////////////////////////////////////////////////////
      case 5:                                    //RIGHT MODE(right forward)
        if(pre_dir != dir){
            pre_dir = dir;
            Serial.println("The gain value is");
            Serial.println(gain,DEC);
            //A motor speed conversion
            mtrA_dir=65;            //A    
            MotorA_Spd(); 
      
            //B motor speed conversion
            mtrB_dir=98;            //b    
            MotorB_SpdwithGain(); 
        }
      break;
 /////////////////////////////////////////////////////////////////
      case 6:                                    //RIGHT MODE(right back)
        if(pre_dir != dir){
            pre_dir = dir;
            Serial.println("The gain value is");
            Serial.println(gain,DEC);
            //A motor speed conversion
            mtrA_dir=97;            //a    
            MotorA_Spd(); 
      
            //B motor speed conversion
            mtrB_dir=66;            //B    
            MotorB_SpdwithGain();
        }
      break;
 /////////////////////////////////////////////////////////////////
      case 8:                                    //MotorA-F mode 
      if (VR==0){
      Serial.println("Please enter value for MotorB");
      }
      else if(VR==1){
      Serial.println("Ready to run");        
      }
    //A motor speed conversion
      MotorA_Spd();
      mtrA_dir=65;            //A       
      break;
 /////////////////////////////////////////////////////////////////
      case 9:                                    //MotorA-R mode 
      if (VR==0){
      Serial.println("Please enter value for MotorB");
      }
      else if(VR==1){
      Serial.println("Ready to run");        
      }
    //A motor speed conversion
     MotorA_Spd();
     mtrA_dir=97;            //a      
      break;
 /////////////////////////////////////////////////////////////////
      case 10:                                    //MotorB-F mode
      if (VL==0){
      Serial.println("Please enter value for MotorA");
      }
      else if(VL==1){
      Serial.println("Ready to run");        
      }
    //B motor speed conversion     
      MotorB_Spd();
      mtrB_dir=98;            //b   
      break;
 /////////////////////////////////////////////////////////////////
      case 11:                                    //MotorB-R mode
      if (VL==0){
      Serial.println("Please enter value for MotorA");
      }
      else if(VL==1){
      Serial.println("Ready to run");        
      }
    //B motor speed conversion     
      MotorB_Spd();
      mtrB_dir=66;            //B    
      break;
 /////////////////////////////////////////////////////////////////
     } 
}
int sendtoMotor(){
  switch (dir) { 
      case 1:
      Serial.println("Forward Mode");
      break;    
      case 2:
      Serial.println("Reverse Mode");
      break;
      case 3:
      Serial.println("Left forward Mode");
      break;
      case 4:
      Serial.println("Left back Mode");
      break;
      case 5:
      Serial.println("Right Forward Mode");
      break;
      case 6:
      Serial.println("Right Back Mode");  
      break;
      case 12:
      Serial.println("Independent motor control mode start to run");
      break;
  }
     MotorA_run();
     MotorB_run();
}

void Stop(){ 
      num=0;
      num1=0;
      num2=0;
      cycle=1;
      dir=0;
      //speed=0;
      gain=1;
      VL=0;
      VR=0;
      robotDirection=0;
      referenceData=0;
      mtrA_dir=0;
      mtrB_dir=0;
      gain_check=0;
      Serial1.println("!A00");
      Serial1.println("!B00");
      Serial.println(" "); 
      Serial.println("Cycle reset");           // reset everything
}
void switchcase() { 
  Serial.print("Switchcase mode");
  int rf_data;         
  rf_data=int(mySerial.read());
  Serial.println(rf_data);
  
     switch (rf_data) { 
	  case 14:
        if(dir != 1){
            pre_dir = dir;
            dir = 1;
            Serial.println("Button right UP Pressed"); 
        }
	  break;
	  
	  case 15:
        if(dir != 2){
            pre_dir = dir;
            dir = 2;
            Serial.println("Button right Down Pressed"); 
        }
	  break;
	  
	  case 16:
        if(dir != 3){
            pre_dir = dir;
            dir = 3;
            Serial.println("Button right LEFT Pressed"); 
        }
	  break;
	  
	  case 17:
        if(dir != 5){
            pre_dir = dir;
            dir = 5;
            Serial.println("Button right RIGHT Pressed");
        }
	  break;
	  
	  case 19:
        if(dir != 7){
            pre_dir = dir;
            dir = 7;
            Serial.println("Button right STOP Pressed");
        }
	  break;
	 
      case 22:                                            
      if (num3==0){                     //PC to RC
      Serial1.write(94);          //^
      Serial1.println("00 00");
      delay(50);                  //delay must have,otherwise ax3500 will not recognize the commands below
      /*Serial1.write(37);          //%
      Serial1.println("rrrrrr");
      delay(100); */
      num3=1;
      Serial.print("PC to RC");

     } 
     else if(num3==1){                     //RC to PC
      Serial1.write(0xD);         // 10 Enters
      Serial1.write(0xD);
      Serial1.write(0xD);
      Serial1.write(0xD);
      Serial1.write(0xD);
      Serial1.write(0xD);
      Serial1.write(0xD);
      Serial1.write(0xD);
      Serial1.write(0xD);
      Serial1.write(0xD);
      Serial1.write(94);          //^
      Serial1.println("00 01");
      delay(50);                  //delay must have,otherwise ax3500 will not recognize the commands below
      Serial1.write(37);          //%
      Serial1.println("rrrrrr");
      num3=0;
     }     
      break; 
      
      case 24:   
       if(num4==0){ 
        Serial1.write(94);          //^
        Serial1.println("04 00");    // set pin 15 to e-stop condition // ^ 04 00 causes emergency stop
        num4=1;
       }
       else if (num4==1){                                          
        Serial1.write(37);          //%
        Serial1.println("rrrrrr");  //Reset
        num4=0;
       }
      break; 
	  
	  default: 
		dir= 0;
		break;
   }   
}

void MotorA_Spd(){
//A motor speed conversion
     spd = speed+A;             // set the first 4 bits to 0, then move 3 bits in left direction, 0000 xxxx to 0xxx x000
     if (spd<0){
      spd=0;
      }
     Char1=spd>>4;                                //convert the motor value to the characters that motor understands
     Mtrspd_Char1=Char1+0x30;
     Char2=spd&0x0f; 
     if(Char2<=0x09){              
     Mtrspd_Char2=Char2+0x30;
     }
     if(Char2>0x09){
     Mtrspd_Char2=Char2+0x37; 
     }
     //Serial.print("The A motor spd value in Decimal is " );
     //Serial.println(spd,DEC);
     //Serial.print("The A motor adjustment value is" );
     //Serial.println(A,DEC);
  
}
void MotorB_Spd(){
      //B motor speed conversion     
     spd1 =speed+B;             // set the first 4 bits to 0, then move 3 bits in left direction, 0000 xxxx to 0xxx x000 
     if (spd1<0){
      spd1=0;
      }   
     Char_1=spd1>>4;                               //convert the motor value to the characters that motor understands;reduce B motor speed to 30% of A motor
     Mtrspd_Char_1=Char_1+0x30;
     Char_2=spd1&0x0f; 
     if(Char_2<=0x09){       
     Mtrspd_Char_2=Char_2+0x30;
     }
     if (Char_2>9){
     Mtrspd_Char_2=Char_2+0x37; 
     }
    
     //Serial.print("The B motor spd1 value in Decimal is " );
     //Serial.println(spd1,DEC);
     //Serial.print("The B motor adjustment value is " );
     //Serial.println(B,DEC);
     //Serial.println("");
}
  
void MotorA_SpdwithGain(){
    //A motor speed conversion
     spd =speed*gain;                           // set the first 4 bits to 0, then move 3 bits in left direction, 0000 xxxx to 0xxx x000
     if (spd<0){
      spd=0;
      }
     Char1=spd>>4;                                //convert the motor value to the characters that motor understands
     Mtrspd_Char1=Char1+0x30;
     Char2=spd&0x0f; 
     if(Char2<=0x09){              
        Mtrspd_Char2=Char2+0x30;
     }
     if(Char2>0x09){
        Mtrspd_Char2=Char2+0x37; 
     }
     Serial.print("The A motor spd value in Decimal is " );
     Serial.println(spd,DEC);
     Serial.print("The A motor adjustment value is" );
     Serial.println(A,DEC);
     Serial.println("");
}

void MotorB_SpdwithGain(){    
    //B motor speed conversion     
     spd1 =speed*gain;             // set the first 4 bits to 0, then move 3 bits in left direction, 0000 xxxx to 0xxx x000  
     Char_1=spd1>>4;             //convert the motor value to the characters that motor understands;reduce B motor speed to 30% of A motor
     Mtrspd_Char_1=Char_1+0x30;
     Char_2=spd1&0x0f; 
     if(Char_2<=0x09){       
       Mtrspd_Char_2=Char_2+0x30;
     }
     if (Char_2>9){
       Mtrspd_Char_2=Char_2+0x37; 
     }
    
     Serial.print("The B motor spd1 value in Decimal is " );
     Serial.println(spd1,DEC);
     Serial.print("The B motor adjustment value is " );
     Serial.println(B,DEC);
     Serial.println("");
}

void MotorA_run(){
       Serial1.print("!");
       Serial1.write(mtrA_dir);
       Serial1.write(Mtrspd_Char1);
       Serial1.write(Mtrspd_Char2);
       Serial1.println("");
}
void MotorB_run(){
       Serial1.print("!");
       Serial1.write(mtrB_dir);
       Serial1.write(Mtrspd_Char_1);
       Serial1.write(Mtrspd_Char_2);
       Serial1.println("");  
}
void gain_checking(){
  if(dir<=6&&dir>=3){
     if (dir!=gain_check){
        gain=1;
      }  
      gain_check=dir;    
      gain=gain-0.1;
      if (gain<0.1){
        gain=0.1;
      } 
  }       
}
void Input_checking() { 
  Serial.println("Detect IO port data"); 
  int bit1=digitalRead(A0);      //1 
  int bit2=digitalRead(5)<<1;
  int bit3=digitalRead(10)<<2;
  int bit4=digitalRead(9)<<3;
  int bit5=digitalRead(12)<<4;
  int bit6=digitalRead(4)<<5;
  int bit7=digitalRead(11)<<6;
  ////////////////////////////////
  int bit8=digitalRead(A1);
  int bit9=digitalRead(A2)<<1;
  int bit10=digitalRead(A3)<<2;
  int bit11=digitalRead(A4)<<3;
  speed=bit1|bit2|bit3|bit4|bit5|bit6|bit7;
  pre_dir = dir;
  dir=bit8|bit9|bit10|bit11;
  Serial.print("The input speed value in BIN is ");
  Serial.println(speed,BIN); 
  Serial.print("The input dir value in BIN is ");
  Serial.println(dir,BIN);
  delay(10000);
}
