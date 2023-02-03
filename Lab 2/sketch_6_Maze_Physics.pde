/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import java.util.Random;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 0; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;

/* define maze blocks */
FBox              b1;
FBox              b2;
FBox              b3;
FBox              b4;
FBox              b5;
FBox              b6;
FBox              b7;
FBox              b8;
FBox              b9;
FBox              b10;
FBox              b11;
FBox              b12;
FBox              b13;
FBox              b14;
FBox              b15;
FBox              b16;
FBox              b17;
FBox              b18;
FBox              b19;
FBox              b20;

int fillColour;

/* define start and stop buttons */
FCircle           c1;
FCircle           c2;
FCircle           c3;
FCircle           c4;

///* define game balls */
//FCircle           g2;
//FCircle           g3;
//FCircle           g4;

/* define game start */
boolean           gameStart                           = false;

/* text font */
PFont             f;

/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400);
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);

  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, "/dev/cu.usbmodem144401", 0);
  //haplyBoard          = new Board(this, Serial.list()[0], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  
  /* Set maze barriers */
  b1                  = new FBox(4.0, 0.2);
  b1.setPosition(edgeTopLeftX+worldWidth/4.0-4.5, edgeTopLeftY+worldHeight/2-3); 
  b1.setFill(0);
  b1.setNoStroke();
  b1.setStaticBody(true);
  world.add(b1);
  
  b2 = new FBox(0.2, 5);
  b2.setPosition(edgeTopLeftX+worldWidth/4.0-1.5, edgeTopLeftY+worldHeight/2-2.5);
  b2.setFill(0);
  b2.setNoStroke();
  b2.setStaticBody(true);
  world.add(b2);
  
  b3 = new FBox(0.2, 5);
  b3.setPosition(edgeTopLeftX+worldWidth/4.0-2.5, edgeTopLeftY+worldHeight/2-0.6);
  b3.setFill(0);
  b3.setNoStroke();
  b3.setStaticBody(true);
  world.add(b3);
  
  b4                  = new FBox(2, 0.2);
  b4.setPosition(edgeTopLeftX+worldWidth/4.0-4.8, edgeTopLeftY+worldHeight/2-2); 
  b4.setFill(0);
  b4.setNoStroke();
  b4.setStaticBody(true);
  world.add(b4);
  
  b5                  = new FBox(2, 0.2);
  b5.setPosition(edgeTopLeftX+worldWidth/4.0-3.6, edgeTopLeftY+worldHeight/2-1); 
  b5.setFill(0);
  b5.setNoStroke();
  b5.setStaticBody(true);
  world.add(b5);
  
  b6                  = new FBox(2.0, 0.2);
  b6.setPosition(edgeTopLeftX+worldWidth/4.0-4.8, edgeTopLeftY+worldHeight/2-0); 
  b6.setFill(0);
  b6.setNoStroke();
  b6.setStaticBody(true);
  world.add(b6);
  
  b7                  = new FBox(2.0, 0.2);
  b7.setPosition(edgeTopLeftX+worldWidth/4.0-3.6, edgeTopLeftY+worldHeight/2+1); 
  b7.setFill(0);
  b7.setNoStroke();
  b7.setStaticBody(true);
  world.add(b7);
  
  b8                  = new FBox(2.0, 0.2);
  b8.setPosition(edgeTopLeftX+worldWidth/4.0-4.8, edgeTopLeftY+worldHeight/2+2); 
  b8.setFill(0);
  b8.setNoStroke();
  b8.setStaticBody(true);
  world.add(b8);
  
  b9 = new FBox(0.2, 2.6);
  b9.setPosition(edgeTopLeftX+worldWidth/4.0-4.0, edgeTopLeftY+worldHeight/2+4.3);
  b9.setFill(0);
  b9.setNoStroke();
  b9.setStaticBody(true);
  world.add(b9);
  
  b10                  = new FBox(1.0, 0.2);
  b10.setPosition(edgeTopLeftX+worldWidth/4.0-1, edgeTopLeftY+worldHeight/2+1); 
  b10.setFill(0);
  b10.setNoStroke();
  b10.setStaticBody(true);
  world.add(b10);
  
  b11                  = new FBox(15.0, 0.2);
  b11.setPosition(edgeTopLeftX+worldWidth/4.0+6.0, edgeTopLeftY+worldHeight/2-2); 
  b11.setFill(0);
  b11.setNoStroke();
  b11.setStaticBody(true);
  world.add(b11);
  
  b12 = new FBox(0.2, 5);
  b12.setPosition(edgeTopLeftX+worldWidth/4.0+13.5, edgeTopLeftY+worldHeight/2-2.5);
  b12.setFill(0);
  b12.setNoStroke();
  b12.setStaticBody(true);
  world.add(b12);
  
  b13 = new FBox(0.2, 5);
  b13.setPosition(edgeTopLeftX+worldWidth/4.0+0.5, edgeTopLeftY+worldHeight/2+0.5);
  b13.setFill(0);
  b13.setNoStroke();
  b13.setStaticBody(true);
  world.add(b13);
  
  b14 = new FBox(0.2, 2);
  b14.setPosition(edgeTopLeftX+worldWidth/4.0-0.5, edgeTopLeftY+worldHeight/2+0.10);
  b14.setFill(0);
  b14.setNoStroke();
  b14.setStaticBody(true);
  world.add(b14);
  
  b15 = new FBox(1, 0.2);
  b15.setPosition(edgeTopLeftX+worldWidth/4.0-0, edgeTopLeftY+worldHeight/2+0.10);
  b15.setFill(0);
  b15.setNoStroke();
  b15.setStaticBody(true);
  world.add(b15);

  b16 = new FBox(2, 0.2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0-0.5, edgeTopLeftY+worldHeight/2+2);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(0.2, 2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0-2.5, edgeTopLeftY+worldHeight/2+3.95);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(2.1, 0.2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0-0.45, edgeTopLeftY+worldHeight/2+3);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);

  b16 = new FBox(0.2, 5.5);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+1.5, edgeTopLeftY+worldHeight/2+1.8);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(0.2, 3.5);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+2.5, edgeTopLeftY+worldHeight/2+1.6);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(6, 0.2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+4.4, edgeTopLeftY+worldHeight/2-1);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(0.2, 3);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+8.4, edgeTopLeftY+worldHeight/2-0.5);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(3, 0.2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+7, edgeTopLeftY+worldHeight/2-0.1);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(0.2, 3.5);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+3.5, edgeTopLeftY+worldHeight/2+1.55);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(1, 0.2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+3.0, edgeTopLeftY+worldHeight/2+1.55);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(0.2, 3.5);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+5.5, edgeTopLeftY+worldHeight/2+1.55);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(1, 0.2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+5.0, edgeTopLeftY+worldHeight/2+1.55);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(0.2, 3.5);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+4.5, edgeTopLeftY+worldHeight/2+1.55);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(1.2, 0.2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+2, edgeTopLeftY+worldHeight/2+3.3);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(1.2, 0.2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+6, edgeTopLeftY+worldHeight/2+3.2);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(0.2, 4);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+7.5, edgeTopLeftY+worldHeight/2+3.2);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(1, 0.2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+7, edgeTopLeftY+worldHeight/2+2.2);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(0.2, 1);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+6.5, edgeTopLeftY+worldHeight/2+0.5);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(0.2, 0.5);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+7.5, edgeTopLeftY+worldHeight/2+0.1);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(3, 0.2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+9, edgeTopLeftY+worldHeight/2+2.2);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(0.2, 2);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+9.4, edgeTopLeftY+worldHeight/2-0);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b16 = new FBox(0.2, 4);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+10.4, edgeTopLeftY+worldHeight/2+1);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b17 = new FBox(1, 0.2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+9.8, edgeTopLeftY+worldHeight/2);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(1.5, 0.2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+8.2, edgeTopLeftY+worldHeight/2+3.3);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(0.2, 1.2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+9, edgeTopLeftY+worldHeight/2+3.8);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(0.2, 5);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+11.5, edgeTopLeftY+worldHeight/2+0.5);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(1, 0.2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+12, edgeTopLeftY+worldHeight/2+1);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(0.2, 5);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+11.5, edgeTopLeftY+worldHeight/2+0.5);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(1, 0.2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+13, edgeTopLeftY+worldHeight/2-0.8);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(0.2, 3);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+13.5, edgeTopLeftY+worldHeight/2+2.2);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(6, 0.2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+15.5, edgeTopLeftY+worldHeight/2+2.2);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(0.2, 2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+14.5, edgeTopLeftY+worldHeight/2+4.2);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(0.2, 1.3);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+15.5, edgeTopLeftY+worldHeight/2+2.8);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(0.2, 2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+16.5, edgeTopLeftY+worldHeight/2+4.2);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(0.2, 6);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+14.5, edgeTopLeftY+worldHeight/2-0.4);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(3, 0.2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+17, edgeTopLeftY+worldHeight/2-3.4);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(3, 0.2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+17, edgeTopLeftY+worldHeight/2-2.4);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(3, 0.2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+16, edgeTopLeftY+worldHeight/2-1.4);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(3, 0.2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+17, edgeTopLeftY+worldHeight/2-0.4);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(3, 0.2);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+17, edgeTopLeftY+worldHeight/2+1.4);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(0.2, 1);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+15.5, edgeTopLeftY+worldHeight/2+0);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b17 = new FBox(0.2, 1);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+16.5, edgeTopLeftY+worldHeight/2+0.8);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  /* Start Button */
  c1                  = new FCircle(0.5); // diameter is 2
  c1.setPosition(edgeTopLeftX+1.2, edgeTopLeftY+worldHeight/2.0-3.8);
  c1.setFill(0, 255, 0);
  c1.setStaticBody(true);
  world.add(c1);
  
  /* Finish Button - Blue */
  c2                  = new FCircle(0.5);
  c2.setPosition(worldWidth-1.2, edgeTopLeftY+worldHeight/2.0+3.8);
  c2.setFill(0,0,200);
  c2.setStaticBody(true);
  c2.setSensor(true);
  world.add(c2);
  
  /* Finish Button - Red */
  c3                  = new FCircle(0.5);
  c3.setPosition(worldWidth-1.2, edgeTopLeftY+worldHeight/2.0);
  c3.setFill(200,0,0);
  c3.setStaticBody(true);
  c3.setSensor(true);
  world.add(c3);
  
  /* Finish Button - Pink */
  c4                  = new FCircle(0.5);
  c4.setPosition(worldWidth-1.2, edgeTopLeftY+worldHeight/2.0-3.8);
  c4.setFill(200,0,200);
  c4.setStaticBody(true);
  c4.setSensor(true);
  world.add(c4);
  
  ///* Game Ball - Blue */
  //g2                  = new FCircle(0.2);
  //g2.setPosition(1, 3.5);
  //g2.setDensity(1000);
  //g2.setFill(0,0,255);
  //g2.setName("Widget");
  //g2.setGrabbable(true);
  //world.add(g2);
  
  ///* Game Ball - Red */
  //g3                 = new FCircle(0.2);
  //g3.setPosition(2, 3.5);
  //g3.setDensity(1000);
  //g3.setFill(255,0,0);
  //g3.setName("Widget");
  //world.add(g3);
  
  ///* Game Ball - Pink */
  //g4                = new FCircle(0.2);
  //g4.setPosition(3,3.5);
  //g4.setDensity(1000);
  //g4.setFill(255,0,255);
  //g4.setName("Widget");
  //world.add(g4);
  
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  Random rand = new Random();
  int high = 3;
  int colour = rand.nextInt(high);
  s                   = new HVirtualCoupling((0.4)); 
  s.h_avatar.setDensity(1); 
  if (colour == 0)  {
    s.h_avatar.setFill(255, 0, 0);
    fillColour = 0;
  }
  else if (colour == 1)  {
    s.h_avatar.setFill(0, 0, 255);
    fillColour = 1;
  }
  else  {
    s.h_avatar.setFill(255, 0, 255);
    fillColour = 2;
  }
  //s.h_avatar.setFill(255,0,0); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.9);
  world.setEdgesFriction(0);
  

 
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 22);
 
    if(gameStart){
      fill(0, 0, 255);
      textAlign(CENTER);
      text("Move your circle to the corresponding colored exit", width/2, 60);
      textAlign(CENTER);
      text("Touch the green circle to reset", width/2, 90);
    
      b1.setFill(0, 0, 0);
    
    }
    else{
      fill(128, 128, 128);
      textAlign(CENTER);
      text("Touch the green circle to start the maze", width/2, 60);
    }
  
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
 
 
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    
    if (s.h_avatar.isTouchingBody(c1)){
      gameStart = true;
      //g3.setPosition(edgeTopLeftX+2.2, edgeTopLeftY+worldHeight/2.0-3.8);
      //g2.setPosition(edgeTopLeftX+3.2, edgeTopLeftY+worldHeight/2.0-3.8);
      //g4.setPosition(edgeTopLeftX+4.2, edgeTopLeftY+worldHeight/2.0-3.8);
      s.h_avatar.setSensor(false);
    }
  
    if (s.h_avatar.isTouchingBody(c3) && fillColour == 0)
    {
      gameStart = false;
      s.h_avatar.setSensor(true);
      exit();
    }
    
    if(s.h_avatar.isTouchingBody(c2) && fillColour == 1){
      gameStart = false;
      s.h_avatar.setSensor(true);
      exit();
    }
    
    if(s.h_avatar.isTouchingBody(c4) && fillColour == 2){
      gameStart = false;
      s.h_avatar.setSensor(true);
      exit();
    }
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* Alternate bouyancy of fluid on avatar and gameball helper functions, comment out
 * "Bouyancy of fluid on avatar and gameball section" in simulation and uncomment 
 * the helper functions below to test
 */
 
/*
void contactPersisted(FContact contact){
  float size;
  float b_s;
  float bm_d;
  
  if(contact.contains("Water", "Widget")){
    size = 2*sqrt(contact.getBody2().getMass()/contact.getBody2().getDensity()/3.1415);
    bm_d = contact.getBody2().getY()-contact.getBody1().getY()+l1.getHeight()/2;
    
    if(bm_d + size/2 >= size){
      b_s = size;
    }
    else{
      b_s = bm_d + size/2;
    }
    
    contact.getBody2().addForce(0, contact.getBody1().getDensity()*sq(b_s)*300*-1);
    contact.getBody2().setDamping(20);
  }
  
}


void contactEnded(FContact contact){
  if(contact.contains("Water", "Widget")){
    contact.getBody2().setDamping(0);
  }
}
*/

/* End Alternate Bouyancy of fluid on avatar and gameball helper functions */

/* end helper functions section ****************************************************************************************/
