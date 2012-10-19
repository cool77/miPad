/*
  a bit about the interface(ofcourse one day it would be graphic and fun but for now):
   
   *notice the new "override_camera" boolean(line 4). true for mouse control, false for camera detection!
   
    at frst when u start the program you should click the four corners of the paper, on the camera view.
    the corners should be clicked starting the top-left clockwise. notice it shows a blue dot where u clicked.
    you also need to set cameras memory image. it done by pressing th space key.
    then you can set the threshold and minimum size of blob to be recogized using 9,0,- and + keys. notice the values are printed to the consule
    
    when four points r choosen, it waits for the rangefinder to be triggered. for now a key 'r' simulates this.
    once the rangefinder is triggered, it will print: "mode 2: led on". it doesnt, realy sends any massage to arduino, this needs to be fixed.
    now it waits for a blob to be detected by the camera. when a blob is detected. (in case of the camera is overriden a mouse click will simulate blob detection) it will print:"mode 3: ;)" and show the relevant point on the sheet of paper.
    it does all the same the other way around, when blob is lost, and randefinder goes false.
    
    in order not to load on the serial transmition, the arduino should be doing the signal processing from the rangefinder, and notify the programan only on events.
  
  

  TODO list:
   - turn Serial on in the setup before trying it with the arm.
   - corrently the range events are simulated by mouse press. to be set from SerialEvent
   - corrently the program does not acctualy sends Serial messages to turn led and heat on and off. just prints it. the reason is I didnt understand why, but they seems to flip the all thing out.
   - it might be a good idea to and an option to rechoose a calabration poind, it a bit annoing if u mis-click u have to start all over again
   - it didnt look like needed, but it's possible to filter the blob found, to limit it just to blob inside the area between the calabration points

*/


import processing.serial.*;
import hypermedia.video.*;
import java.awt.*;


boolean override_camera=false;//do u want to control arm with mouse?

//camera stuff
OpenCV opencv;

int w = 320;//capturing size
int h = 240;////capturing size

int threshold = 120; //this two r the important suff when recognizing blob from camera
int min_blob=100;   //these are just setup value, they can be changed in real-time using 9,0,-,+ keys.
                    //notice the vchanging alues displayed in the consule.



Serial myPort; 

//arm parameters
int[] ang_factor= {0,0,0};//factor to normalize pysical servo 0 point;


int[] paper_size={260,300};//the size of the arms active area, use this values 
                           //to scale-fit arms movment to screen size.

PVector paperOffset=new PVector(310, 150,30);//arm base from paper corner,
                               //use this values to move-fit arm movmement to 
                               //screen position.
                               //if I rember right in the corrent hardware setup x axis if positive to the right, y upwards. z I dont really use. use the "hi" below instead.

int[] armOffset={-40, 0, 30};//hinge1 from arm base(x0, y0 ,z0)
int[] sagment={170,220};//length of arms parts


int hi=20;//distance from paper


RArm arm = new RArm(sagment[0], sagment[1],armOffset[0], armOffset[1], armOffset[2]);//creat arm object

//int[] Ocorner= new int[2];//display pos
int[] ang=new int[3];





//ppt parameters
 
 PlanarProjectiveTransform ppt;
 
 //to make the calabration 2 sets of 4 point r needed. 4 foints on camera view, and 4 matching points on the real paper.
double[][] real_pts={{0,0},                         
                     {paper_size[0], 0},
                     {paper_size[0],paper_size[1]},
                     {0,paper_size[1]}};              //this is the array of real point
                     
double[][] proj_pts=new double[4][2];                //here the points from camera r to saved.
PVector[] verts=new PVector[4];                      //this is bit stupid, left over from using mose instead of camera. some of the fonctions I use demand PVector. It just a copy of "proj_pts"


//some genaral varebles
                     
boolean calaborated=false;

Point2D ppt_input= new Point2D();//point2D is a privat class(my fathe rcreated it, not beenig familier with Java's native classes. I didn't mass with its code. i just handle the transormation between point2D, Pvector, and arrays)
                                //ppt_input is the point recognized by the camera and sent to be trasformed into a point on our paper.
Point2D ppt_output= new Point2D();// ppt_output is were I save the output of the transformation.

int n_pts=0;//point used for calabration
int i=0;


 boolean firstContact = false;
 int mode=0;//0-calaboration  
 boolean modeChange=false;
 boolean blob_found=false; //this one is not used for now. it might be good idea(from estetic point of u
                          //to use it instead of the negitive value code I use for representing that the screen is not beeing touched.
                          
 boolean range_detect=false;// to be set from inside OnSerialEvent() function
 
 int[] send_arm2pos=new int[3];// these is an arraye always containing the wished position of the arm [x,y,z]. in paper coordinats.
 PVector arm_coord_pos = new PVector();// position of the arm [x,y,z]. in arm's coordinats (taking arm's offset in account. it is fed the arm class methods(setAngle & setAzimuth) every loop after calabration ends. 
 PVector finger_pos= new PVector(-1,-1);//contains the x & y 
 PVector SBpos= new PVector(paper_size[0]/2, paper_size[1]/2, -40);//i dont dont why its PVector???????????
 
 
 //displaying stuff
 int width=1100;
 int height=600;
 
 PVector sheet_pos=new PVector((width-paper_size[0])/2, height-10-paper_size[1]);     //paper sheet represantation pos on screen

 PVector side_view_pos = new PVector(200, 200);
 int arm_stroke_weidth = 5;
 
 PVector top_view_pos = PVector.add(sheet_pos, paperOffset);
 
 //Point cam_norm_pos=new Point(10,10); //position of narmal cam view
 PVector cam_view_pos=new PVector((width-w)/2,10);                      //position of after filltering cam view
 

//-----------------------------------------setup----------------------------------------


void setup(){
  //frameRate(10);
  size(width, height);
  background(50);
  fill(255);
  
   //arm.setOffset(paperOffset[0],paperOffset[1], paperOffset[2]);//set the arm's position relativly to the paper
 
   println(Serial.list());
   String portName = Serial.list()[1];
 //  myPort = new Serial(this, portName, 9600); //if this line is out of use(has // at the begining) it becous i have to do it fot compiling here. IT IS NEEDED!!!
   
   //camera stuff
   opencv = new OpenCV( this );
   opencv.capture(w,h);
   
     noStroke();  
}

//----------------------------------------draw---------------------------------------------
void draw(){
  
   background(50);
   fill(255);
   rect(sheet_pos.x, sheet_pos.y, paper_size[0], paper_size[1]);//draw the represantation of the paper on screen
 
 
  opencv.read(); //read from camera
  //image( opencv.image(OpenCV.GRAY), cam_norm_pos.x, cam_norm_pos.y ); //show the image
  opencv.absDiff();//substruct image in memory from it
  opencv.threshold(threshold); //fillter it
  image( opencv.image(OpenCV.GRAY), cam_view_pos.x, cam_view_pos.y );   // show it after substraction and filtering
  
  findBlob();//sets finger_pos to the blobs position, and the blob_found to true or false
  showBlob();
  showDrawingPoint();
  
  if(mode!=0){ //eny time after calabration
    showActiveArea();

    arm_coord_pos.x = paperOffset.x - paper_size[0]+ send_arm2pos[0];
    arm_coord_pos.y = paperOffset.y - send_arm2pos[1];
    arm_coord_pos.z = paperOffset.z - send_arm2pos[2];
    
    println(arm_coord_pos.x +" ,  "+arm_coord_pos.y+ " ,  "+arm_coord_pos.z);
    //arm_coord_pos.sub(paperOffset.x, paperOffset.y, paperOffset.z); 
   
    arm.setAngles(arm_coord_pos);// update the the angles of the arm oject
    //arm.setAzimuth(arm_coord_pos);//update the the angles of the arm oject, no real reason for two saperated methods

      arm.sideView(side_view_pos, arm_stroke_weidth );//( PVector pos, int thickness)
      arm.topView(top_view_pos, arm_stroke_weidth); //( PVector pos, int thickness)
  }

  switch(mode)
  {
    case 0://-----------------calabration----------------------------
      
      
       //call calabate() function on mouse click
       // points should be picked starting the top left, clockwise.

      //show the points
     for(int i=0;i<n_pts;i++){
       pushMatrix();
      translate(cam_view_pos.x, cam_view_pos.y);
      fill(0,0,255);
      ellipse((float)proj_pts[i][0], (float)proj_pts[i][1],5,5);
       popMatrix();
    }
     
     
     if(n_pts>3){ //if 4 calebration points were picked
        ppt= new  PlanarProjectiveTransform(4, real_pts, proj_pts);//creat the PlanarProjectiveTransformatin object
        
        for(int i=0;i<4;i++){
          verts[i]=new PVector((float)proj_pts[i][0],(float)proj_pts[i][1]);// stupid over head. i explained it above.
          println(verts[i].x+".,"+verts[i].y);
         
        }
      
        calaborated=true;
        mode=1;
        modeChange=true;
        
     }
    break; //---------------calabration end------------------------   
   
   case 1:
   // READY - basicly just turn led off and wait for a rangefinder detection
   
     delay(150);
      send_arm2pos[0] =  (int)SBpos.x;
      send_arm2pos[1] =  (int)SBpos.y;
      send_arm2pos[2] =  (int)SBpos.z; //set the wanted arm pos to standbay pos.
      
      if(modeChange) {//if u just entered this mode
        myPort.write('L');
        myPort.write(0);  //turn led off
        println("mode 1: led OFF");
        
        myPort.write('H');
        myPort.write(0);  //heat led off
        println("mode 1: heat OFF");

        delay(15);
        modeChange=false;
      }
       
       if(range_detect){
         mode=2;
         modeChange=true;
       }
     
       break;
       
   
   case 2:
   //GET-SET - turn led on and wait for screen to get thouched
   
     send_arm2pos[0] = (int)SBpos.x;
     send_arm2pos[1] =  (int)SBpos.y;
     send_arm2pos[2] =  (int)SBpos.z;//set the wanted arm pos to standbay pos.
     
     if(modeChange) {
        myPort.write('L');
        myPort.write(1);  //turn led on
        println("mode 2: led ON");
        
        myPort.write('H');
        myPort.write(1);  //heat  on
        println("mode 2: heat ON");

        delay(15);
        
        delay(15);
        modeChange=false;
      }
      if(blob_found){
        mode=3;
        modeChange=true;
      }
      else if(!range_detect){
         mode=1;
         modeChange=true;
      }
      
       break;
   
   
     
   case 3:
   //GO!
   
        if(modeChange) {
        println("mode 3:  (; ");
        modeChange=false;
        }
      
      ppt_input.set(finger_pos.x, finger_pos.y);//just converting from PVector to Point2D
      ppt_output=ppt.ProjToReal(ppt_input);// transform from projected camera X,Y to X,Y on actual screen/paper.
      
      send_arm2pos[0] =  paper_size[0]-(int)ppt_output.m_x;
      send_arm2pos[1] = (int)ppt_output.m_y;  //set the wanted position to that point
      send_arm2pos[2] =  hi;                  //and wanted height
      
      
      
      
     if(!blob_found) {//what to do if no blob recognized????????????
       mode=2;
       modeChange=true;
     }
     
     break;
  }
}
//---------------------------------------------------end of LOOP-------------------------------------------------


void mouseClicked(){
  
  if(mode==0) Calaborate();// use mouse click to choose calabration points.
  else {  // this elese limulates range finder events. we should get rid of it later.
    blob_found=!blob_found;
    println("blob_found set to: "+ blob_found);
  }
}


void Calaborate(){
  proj_pts[n_pts][0]=mouseX-cam_view_pos.x;
  proj_pts[n_pts][1]=mouseY-cam_view_pos.y;//write the coordinates of the pickd point to the array of calabration points 
  ellipse(finger_pos.x, finger_pos.y, 5,5);//show as blue dot
  n_pts++;//move to next calabration point
  
}



void serialEvent(Serial myPort) {//-------------------------------SerialEvent---------------------------
  int inByte = myPort.read();
  myPort.clear();
  
 if(mode==0){}//don't respond while calabration.
 
 else if(firstContact == false) {//first thing after calabration? esteblish connection.
    if(inByte == 'A') { // you've had first contact from the microcontroller
      myPort.write('A');
      firstContact = true;   
    
      println("got an A, first connection!!!");
    } 
    else println("Invailed signal!  expected A!!!!!!!!!!!! got: "+inByte);
  }
 
  else {
   ang= arm.getAngles();
    if(inByte == 'B'){
      myPort.write(ang_factor[0]+ang[0]);
    //  println("got B: sent Azimueth:  "+ ang[0]);

    }
    else if(inByte == 'C'){
      myPort.write(ang_factor[1] + ang[1]);
    // println("got C: sent Angle 1:  "+ ang[1]);

    }
    else if(inByte == 'D'){
      myPort.write (ang_factor[2] + ang[2]);
      //println("got D: sent Angle 2:  "+ ang[2]);
     // println();
   
    }
    
   //add another block of code handling rangefiner events.
         
    else println("Invailed signal!!!!!!!!!!!!  "+inByte); 
 
 }
    
}


void keyPressed() {
    if ( key==' ' ) opencv.remember();
     if ( key=='=' ) {
       min_blob+=5;
       println("min_blob= "+min_blob);
     }
     
      if ( key=='-' ) {
       min_blob-=5;
       println("min_blob= "+min_blob);
     }
     
     
      if ( key=='0' ) {
     threshold += 5;
    println(" threshold = "+ threshold);
      }
      
        if ( key=='9' ) {
     threshold -= 5;
    println(" threshold = "+ threshold);
      }
       if ( key=='p' ) {
    println(" ppt output: "+ (int)ppt_output.m_x+", "+(int)ppt_output.m_y);
      }
      
        if ( key=='r' ) {
       range_detect=!range_detect;
    println("range_detect set to: "+ range_detect);
        }
        
          if ( keyCode==LEFT ) {
            paperOffset.x +=1;
          }
          
          if ( keyCode == RIGHT ) {
             paperOffset.x -=1;
          }
          
          if ( keyCode == DOWN ) {
             paperOffset.y -=1;
          }
          
          if ( keyCode == UP ) {
             paperOffset.y -=1;
          }
}


public void stop() {
    opencv.stop();
    super.stop();
}


void findBlob(){
  if (!override_camera){
 // working with blobs
    Blob[] blobs = opencv.blobs( min_blob, w*h/3, 1, true );//look for blobs, only one in this case
    if(blobs.length>0){                                     //if found any
      finger_pos.x=blobs[0].centroid.x;
      finger_pos.y=blobs[0].centroid.y;
      blob_found=true;
    }
   else{                                                    //else
     blob_found=false;
      // finger_pos.x=-1;
      // finger_pos.y=-1;
    }
  }
  
 else{
   finger_pos.x=mouseX-cam_view_pos.x;
   finger_pos.y=mouseY-cam_view_pos.y;
 }
}

void showBlob(){
     pushMatrix();
    translate( cam_view_pos.x, cam_view_pos.y);//draw on the cam view
    if(blob_found) fill(255,0,0);//red if the blob is now detected
    else  fill(0,255,0);         //green if no real time detection (last detected point)
    ellipse(finger_pos.x,finger_pos.y,10,10);//draw the dot
     popMatrix();

}

void showDrawingPoint(){
   pushMatrix(); 
  translate(sheet_pos.x,sheet_pos.y); //draw on sheet of paper
  fill(0);
  //if(mode==3) ellipse((float)ppt_output.m_x,(float) ppt_output.m_y, 5, 5);
  /*if(mode==1 || mode==2)*/ ellipse( paper_size[0]-send_arm2pos[0], send_arm2pos[1], 5, 5);
    popMatrix();
  
}

void showActiveArea(){
     pushMatrix();
    translate(cam_view_pos.x,cam_view_pos.y);//draw on the cam view
    fill(200,100);                           //with transparent gray
    quad(verts[0].x,verts[0].y,verts[1].x,verts[1].y, verts[2].x, verts[2].y, verts[3].x, verts[3].y);//mark the active area(area difined by calabration points)
     popMatrix();
}
