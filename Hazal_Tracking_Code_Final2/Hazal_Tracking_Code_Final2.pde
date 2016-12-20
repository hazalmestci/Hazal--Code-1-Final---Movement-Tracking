
/* Hazal Mestci, Final for Code 1 ----------------------------------
 * date: 12/20/2016
 * Works with only Processing 2
 * Example code by Max Rheiner / Interaction Design / Zhdk / http://iad.zhdk.ch/
 * Special thanks to Bryan Ma and Michael Braverman
 * SimpleOpenNI User Test
 * --------------------------------------------------------------------------
 * Processing Wrapper for the OpenNI/Kinect library
 * http://code.google.com/p/simple-openni
 * --------------------------------------------------------------------------
 * 
 * ----------------------------------------------------------------------------
 */

import SimpleOpenNI.*;

int formResolution = 10;
float stepSize = 2;
float initRadius = 150;
float centerX, centerY;
float[] x = new float[formResolution];
float[] y = new float[formResolution];

float[] xOffset = new float[formResolution];
float[] yOffset = new float[formResolution];

boolean filled = false;
boolean freeze = false;
int mode = 0;

  

PGraphics    canvas;
color[]      userClr = new color[]
{
  color(255, 0, 0), 
  color(0, 255, 0), 
  color(0, 0, 255), 
  color(255, 255, 0), 
  color(255, 0, 255), 
  color(0, 255, 255)
};
int xPos, yPos; 
PVector com = new PVector();                                   
PVector com2d = new PVector();                                   

// --------------------------------------------------------------------------------
//  CAMERA IMAGE SENT VIA SYPHON
// --------------------------------------------------------------------------------
int kCameraImage_RGB = 1;                // rgb camera image
int kCameraImage_IR = 2;                 // infra red camera image
int kCameraImage_Depth = 3;              // depth without colored bodies of tracked bodies
int kCameraImage_User = 4;               // depth image with colored bodies of tracked bodies

int kCameraImageMode = kCameraImage_User; // << Set thie value to one of the kCamerImage constants above

// --------------------------------------------------------------------------------
//  SKELETON DRAWING
// --------------------------------------------------------------------------------
boolean kDrawSkeleton = true; // << set to true to draw skeleton, false to not draw the skeleton

// --------------------------------------------------------------------------------
//  OPENNI (KINECT) SUPPORT
// --------------------------------------------------------------------------------

import SimpleOpenNI.*;           // import SimpleOpenNI library

SimpleOpenNI     context;

private void setupOpenNI()
{
  context = new SimpleOpenNI(this);
  if (context.isInit() == false) {
    println("Can't init SimpleOpenNI, maybe the camera is not connected!"); 
    exit();
    return;
  }   

  // enable depthMap generation 
  context.enableDepth();
  context.enableUser();

  // disable mirror
  context.setMirror(false);
}

private void setupOpenNI_CameraImageMode()
{
  println("kCameraImageMode " + kCameraImageMode);

  switch (kCameraImageMode) {
  case 1: // kCameraImage_RGB:
    context.enableRGB();
    println("enable RGB");
    break;
  case 2: // kCameraImage_IR:
    context.enableIR();
    println("enable IR");
    break;
  case 3: // kCameraImage_Depth:
    context.enableDepth();
    println("enable Depth");
    break;
  case 4: // kCameraImage_User:
    context.enableUser();
    println("enable User");
    break;
  }
}

private void OpenNI_DrawCameraImage()
{
  switch (kCameraImageMode) {
  case 1: // kCameraImage_RGB:
    canvas.image(context.rgbImage(), 0, 0);
    // println("draw RGB");
    break;
  case 2: // kCameraImage_IR:
    canvas.image(context.irImage(), 0, 0);
    // println("draw IR");
    break;
  case 3: // kCameraImage_Depth:
    canvas.image(context.depthImage(), 0, 0);
    // println("draw DEPTH");
    break;
  case 4: // kCameraImage_User:
    //    canvas.image(context.userImage(), 0, 0);
    // println("draw DEPTH");
    break;
  }
}

// --------------------------------------------------------------------------------
//  OSC SUPPORT
// --------------------------------------------------------------------------------

import oscP5.*;                  // import OSC library
import netP5.*;                  // import net library for OSC

OscP5            oscP5;                     // OSC input/output object
NetAddress       oscDestinationAddress;     // the destination IP address - 127.0.0.1 to send locally
int              oscTransmitPort = 1234;    // OSC send target port; 1234 is default for Isadora
int              oscListenPort = 9000;      // OSC receive port number

private void setupOSC()
{
  // init OSC support, lisenting on port oscTransmitPort
  oscP5 = new OscP5(this, oscListenPort);
  oscDestinationAddress = new NetAddress("127.0.0.1", oscTransmitPort);
}

private void sendOSCSkeletonPosition(String inAddress, int inUserID, int inJointType)
{
  // create the OSC message with target address
  OscMessage msg = new OscMessage(inAddress);

  PVector p = new PVector();
  float confidence = context.getJointPositionSkeleton(inUserID, inJointType, p);

  // add the three vector coordinates to the message
  msg.add(p.x);
  msg.add(p.y);
  msg.add(p.z);

  // send the message
  oscP5.send(msg, oscDestinationAddress);
}

private void sendOSCSkeleton(int inUserID)
{
  sendOSCSkeletonPosition("/head", inUserID, SimpleOpenNI.SKEL_HEAD);
  sendOSCSkeletonPosition("/neck", inUserID, SimpleOpenNI.SKEL_NECK);
  sendOSCSkeletonPosition("/torso", inUserID, SimpleOpenNI.SKEL_TORSO);

  sendOSCSkeletonPosition("/left_shoulder", inUserID, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  sendOSCSkeletonPosition("/left_elbow", inUserID, SimpleOpenNI.SKEL_LEFT_ELBOW);
  sendOSCSkeletonPosition("/left_hand", inUserID, SimpleOpenNI.SKEL_LEFT_HAND);

  sendOSCSkeletonPosition("/right_shoulder", inUserID, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  sendOSCSkeletonPosition("/right_elbow", inUserID, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  sendOSCSkeletonPosition("/right_hand", inUserID, SimpleOpenNI.SKEL_RIGHT_HAND);

  sendOSCSkeletonPosition("/left_hip", inUserID, SimpleOpenNI.SKEL_LEFT_HIP);
  sendOSCSkeletonPosition("/left_knee", inUserID, SimpleOpenNI.SKEL_LEFT_KNEE);
  sendOSCSkeletonPosition("/left_foot", inUserID, SimpleOpenNI.SKEL_LEFT_FOOT);

  sendOSCSkeletonPosition("/right_hip", inUserID, SimpleOpenNI.SKEL_RIGHT_HIP);
  sendOSCSkeletonPosition("/right_knee", inUserID, SimpleOpenNI.SKEL_RIGHT_KNEE);
  sendOSCSkeletonPosition("/right_foot", inUserID, SimpleOpenNI.SKEL_RIGHT_FOOT);
}

// --------------------------------------------------------------------------------
//  SYPHON SUPPORT
// --------------------------------------------------------------------------------

import codeanticode.syphon.*;    // import syphon library

SyphonServer     server;     

private void setupSyphonServer(String inServerName)
{
  // Create syhpon server to send frames out.
  server = new SyphonServer(this, inServerName);
}

// --------------------------------------------------------------------------------
//  EXIT HANDLER
// --------------------------------------------------------------------------------
// called on exit to gracefully shutdown the Syphon server
private void prepareExitHandler()
{
  Runtime.getRuntime().addShutdownHook(
  new Thread(
  new Runnable()
  {
    public void run () {
      try {
        if (server.hasClients()) {
          server.stop();
        }
      } 
      catch (Exception ex) {
        ex.printStackTrace(); // not much else to do at this point
      }
    }
  }
  )
    );
}

// --------------------------------------------------------------------------------
//  MAIN PROGRAM
// --------------------------------------------------------------------------------
void setup()
{
  size(840, 680, P3D);
  canvas = createGraphics(840, 680, P3D);

  println("Setup Canvas");
  
  frameRate (30);

  // canvas.background(200, 0, 0);
//  canvas.stroke(255, random(10,255));
//  canvas.strokeWeight(3);
//  canvas.smooth();
  println("-- Canvas Setup Complete");

  // setup Syphon server
  println("Setup Syphon");
  setupSyphonServer("Depth");

  // setup Kinect tracking
  println("Setup OpenNI");
  setupOpenNI();
  setupOpenNI_CameraImageMode();

  // setup OSC
  println("Setup OSC");
  setupOSC();

  // setup the exit handler
  println("Setup Exit Handerl");
  prepareExitHandler();

  for (int i=0; i<formResolution; i++) {
    xOffset[i] = 0;
    yOffset[i] = 0;
  }
  perspective(radians(45), 
  float(width)/float(height), 
  10, 150000);
}

void draw()
{
//  background(0);
  // update the cam
  context.update();
//  beginDraw();
    
  fill(cos(millis()/5000.1234)*255, cos(millis()/6000.245)*255, sin(millis()/3000.234)*255, 5);
  rect(0,0,width, height);

  translate(width/2, height/2, 0);
  rotateY(radians(frameCount * 0.5));
  rotateX(PI);
  scale(0.3);


  translate(0, 0, -1000);




  // draw image
  OpenNI_DrawCameraImage();

  // draw the skeleton if it's available
  if (kDrawSkeleton) {

    int[] userList = context.getUsers();
    for (int i=0; i<userList.length; i++)
    {
      if (context.isTrackingSkeleton(userList[i]))
      {
        canvas.stroke(userClr[ (userList[i] - 1) % userClr.length ] );

        drawSkeleton(userList[i]);

        if (userList.length == 1) {
          sendOSCSkeleton(userList[i]);
        }
      }      

      //      //draw the center of mass
      //            if (context.getCoM(userList[i], com))
      //            {
      //              context.convertRealWorldToProjective(com, com2d);
      //      
      //              canvas.stroke(100, 255, 0);
      //              canvas.strokeWeight(1);
      //              canvas.beginShape(LINES);
      //              canvas .vertex(com2d.x, com2d.y - 5);
      //              canvas.vertex(com2d.x, com2d.y + 5);
      //              canvas.vertex(com2d.x - 5, com2d.y);
      //              canvas.vertex(com2d.x + 5, com2d.y);
      //              canvas.endShape();
      //      
      ////              canvas.fill(0, 255, 100);
      //              canvas.text(Integer.toString(userList[i]), com2d.x, com2d.y);
      //            }
    }
  }

  // canvas.endDraw();

  // send image to syphon
  server.sendImage(canvas);
}

// draw the skeleton with the selected joints
void drawLimb(int userId, int inJoint1)
{
}

void drawJoint(int userId, int jointType) {
  float  confidence;

  // draw the joint position
  PVector a_3d = new PVector();
  confidence = context.getJointPositionSkeleton(userId, jointType, a_3d);

  PVector a_2d = new PVector();
  context.convertRealWorldToProjective(a_3d, a_2d);

  //  canvas.stroke(random(255), random(255), random(255));
  canvas.fill(random(255),random(255),random(255), random(255));

  PVector screen_a = new PVector(0, 0);
  screen_a.x = map(a_2d.x, 0, 640, 0, width);
  screen_a.y = map(a_2d.y, 0, 480, 0, height);


  //  canvas.ellipse(a_3d.x, a_3d.y, 20, 20);
  canvas.pushMatrix();
  canvas.translate(a_3d.x, a_3d.y, a_3d.z);
//  canvas.box(50);
  canvas.popMatrix();


  /*
  centerX = a_2d.x/2; 
   centerY = a_2d.y/2;
   float angle = radians(360/float(formResolution));
   for (int i=0; i<formResolution; i++) {
   x[i] = cos(angle*i) * initRadius;
   y[i] = sin(angle*i) * initRadius;
   
   xOffset[i] += random(-1,1);
   yOffset[i] += random(-1,1);
   }
   
   
   canvas.strokeWeight(0.75);
   if (filled) canvas.fill(random(255));
   else canvas.noFill();
   
   if (mode == 0) {
   canvas.beginShape();
   // start controlpoint
   canvas.curveVertex(x[formResolution-1]+centerX+xOffset[formResolution-1], y[formResolution-1]+centerY+yOffset[formResolution-1]);
   
   // only these points are drawn
   for (int i=0; i<formResolution; i++) {
   canvas.curveVertex(x[i]+centerX+xOffset[i], y[i]+centerY+yOffset[i]);
   }
   canvas.curveVertex(x[0]+centerX + xOffset[0], y[0]+centerY + yOffset[0]);
   
   // end controlpoint
   canvas.curveVertex(x[1]+centerX + xOffset[1], y[1]+centerY + yOffset[1]);
   canvas.endShape();
   }
   
   if (mode == 1) {
   canvas.beginShape();
   // start controlpoint
   canvas.curveVertex(x[0]+centerX, y[0]+centerY);
   
   // only these points are drawn
   for (int i=0; i<formResolution; i++) {
   canvas.curveVertex(x[i]+centerX, y[i]+centerY);
   }
   
   // end controlpoint
   canvas.curveVertex(x[formResolution-1]+centerX, y[formResolution-1]+centerY);
   canvas.endShape();
   }
   */
}

// draw the skeleton with the selected joints
void drawSkeleton(int userId)
{
  //  canvas.stroke(255, 255, 255, 255);
  canvas.strokeWeight(3);

  drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_HAND);
  drawJoint(userId, SimpleOpenNI.SKEL_LEFT_HAND);
//  drawJoint(userId, SimpleOpenNI.SKEL_LEFT_KNEE);
//  drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_KNEE);

  drawLimb(userId, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK);
  drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND);
  //
  drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND);

  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO);
  //
  drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_HIP, SimpleOpenNI.SKEL_LEFT_KNEE);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT);
  //
  drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT);
}

void drawLimb(int userId, int jointType1, int jointType2)
{
  float  confidence;

  // draw the joint position
  PVector a_3d = new PVector();
  confidence = context.getJointPositionSkeleton(userId, jointType1, a_3d);
  PVector b_3d = new PVector();
  confidence = context.getJointPositionSkeleton(userId, jointType2, b_3d);

  PVector a_2d = new PVector();
  context.convertRealWorldToProjective(a_3d, a_2d);
  PVector b_2d = new PVector();
  context.convertRealWorldToProjective(b_3d, b_2d);


  stroke(255);
  strokeWeight(5);
  line(a_3d.x, a_3d.y, a_3d.z, b_3d.x, b_3d.y, b_3d.z);
  //  canvas.ellipse(a_2d.x, a_2d.y, 2, 2);
  //  canvas.ellipse(b_2d.x, b_2d.y, 2, 2);
}

// -----------------------------------------------------------------
// SimpleOpenNI events

void onNewUser(SimpleOpenNI curContext, int userId)
{
  println("onNewUser - userId: " + userId);
  println("\tstart tracking skeleton");

  curContext.startTrackingSkeleton(userId);
}

void onLostUser(SimpleOpenNI curContext, int userId)
{
  println("onLostUser - userId: " + userId);
}

void onVisibleUser(SimpleOpenNI curContext, int userId)
{
  //println("onVisibleUser - userId: " + userId);
}


void keyPressed()
{
  switch(key)
  {
  case ' ':
    context.setMirror(!context.mirror());
    println("Switch Mirroring");
    break;
  }
}  

