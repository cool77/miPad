/*
this class handels the transformation from a point(x,y,z) to angles between
 the arm segments.
 it recives on cunstruction the phisica parmeters of the arm.
 its also needed to set the arms position reletivly to the paper using setOffset()
 */


class RArm {

  int segLength1, segLength2;//arms length
  float R;
  float angle1, angle2, azimuth;//servo1, servo2, baseServo
  int x0, y0, z0; //arm hinge rel to base position
  int[] offset = {0, 0, 0};


  RArm(int L1, int L2, int x, int y, int z) {
    segLength1 = L1;
    segLength2 = L2;
    x0 = x;
    y0 = y;
    z0 = z; 
    angle1=0;
    angle2=0;
    azimuth=0;
    
    println("Arm object created:");
    println("x0= "+ x0);
    println("y0= "+ y0);
    println("z0= "+ z0);
  }

  /*  void setOffset(int x, int y, int z){
   offset[0]=x;
   offset[1]=y;
   offset[2]=z;
   }
   */
  void attach(int servoBasePin, int servo1Pin, int servo2Pin) {
  
  }


  void setAngles(PVector pos) {//possition of reached point reletivly to arm base.

    //float dh= pos[2] - z0;
    PVector hinge1_pos = new PVector(x0*sin(azimuth)+y0*cos(azimuth), x0*cos(azimuth)-y0*sin(azimuth));
    R=dist(0,0, pos.x, pos.y)-x0;
    //float segV=dist(0, 0, R, dh);
    //println("segLength1= " + segLength1 + "  segLength1= " +segLength2+ "   segV= "+ segV);

    float segV=dist(x0, z0, R, pos.z);
    angle1= acos( (sq(segLength1) + sq(segV) - sq(segLength2)) / (2* segLength1*segV))+asin((pos.z-z0)/segV);//
    angle2= acos( (sq(segLength1) - sq(segV) + sq(segLength2)) / (2* segLength1*segLength2));
    //println("angle 1 is  " + map(angle1, 0, PI, 0 ,180 )+ "  angle 2 is " + map(angle2, 0, PI, 0 ,180 ));
    
     azimuth=atan2(pos.y, pos.x);
    //println("azimuth is  " + map(azimuth, 0, PI, 0 ,180 ));
    
  }

/*  void setAzimuth(PVector pos) {
    azimuth=atan2(pos.y, pos.x);
    //println("azimuth is  " + map(azimuth, 0, PI, 0 ,180 ));
  }
*/
  float[] getAnglesRad() {
    float[] angles= {azimuth, angle1, angle2};
    return angles;
  }

  int[] getAngles() {  
    int[] angles=new int[3];
    angles[0]=int(map(azimuth, 0, PI, 0, 180 ));
    angles[1]=int(map(angle1, 0, PI, 0, 180 ));
    angles[2]=int(map(angle2, 0, PI, 0, 180 ));
    return angles;
  }

 

  void sideView( PVector pos, int thickness) {

    int[] angle = new int[3];
    angle= getAngles();

   pushStyle();
    fill(200);

   pushMatrix(); 
    translate(pos.x, pos.y);

   pushStyle();
    fill(0);
    ellipse(0, 0, 10, 10);

    rect(0, 0, -x0, -z0);
    translate(-x0, -z0);
   popStyle();
    stroke(200);
    strokeWeight(thickness);

    rotate(radians(180 + angle[1]));
    line(0, 0, segLength1, 0);
    translate(segLength1, 0);

    rotate(radians(180 + angle[2]));
    line(0, 0, segLength2, 0);
   popMatrix();
   popStyle();
  }

  void topView(PVector pos, int thickness) {

   pushStyle();
  
   pushMatrix();
    translate(pos.x, pos.y);
    rotate(azimuth);
    fill(0);
   ellipse(0, 0, 10, 10);
   rect(0,0, -x0, -y0);
   translate(-x0, -y0);
    
      stroke(180);
      strokeWeight(thickness);
    line(0, 0, -R, 0);
   popMatrix();
   popStyle();
  }
}    

