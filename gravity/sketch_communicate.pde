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
boolean           gameStart                          = false;
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
float             worldWidth                          = 30.0;  
float             worldHeight                         = 20.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 0; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;

/* define blocks for 20 sawteeth, 3 separate viscous layers, a planet, a satellite and the starting circle */
FCircle           planet;
FCircle           satellite;
FCircle           c1;

/* define the end-effector position coordinates, as well as planet position coordinates */
float effector_x_coordinate, effector_y_coordinate;
float planet_x_coordinate, planet_y_coordinate;
float mass_planet = 333000000;
float satellite_distance_squared;
boolean render_satellite = false;

int fillColour;

/* text font */
PFont             f;

/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1200, 800);
  
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
  haplyBoard          = new Board(this, "/dev/cu.usbmodem144301", 0);
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
  
  
  
  /* Set up placement for planet with gravity */
  /* the idea is to make up a force that obeys the inverse square law */
  planet = new FCircle(0.5);
  planet.setPosition(edgeTopLeftX+12, edgeTopLeftY+worldHeight/2.0+0);
  planet.setFill(0, 0, 255);
  planet.setStaticBody(true);
  world.add(planet);
  
  
  /* Set up placement for the satellite orbiting the planet */
  satellite = new FCircle(0.2);
  satellite.setPosition(edgeTopLeftX+12, edgeTopLeftY+worldHeight/2.0-5);
  satellite.setFill(255, 255, 0);
  satellite.setVelocity(sqrt(6.67e-8 * mass_planet / 5.0)+6, 0);
  world.add(satellite);
  
  
  /* Set up the circle (used to start the experience) */
  c1                  = new FCircle(0.5);
  c1.setPosition(edgeTopLeftX+1.2, edgeTopLeftY+worldHeight/2.0-5);
  c1.setFill(0, 255, 0);
  c1.setStaticBody(true);
  world.add(c1);
  
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.4)); 
  s.h_avatar.setDensity(1); 
  s.h_avatar.setFill(255,0,0); 
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
      //for (int i = 0; i < 20; i++)  {
      //  blocks[i].setFill(0, 0, 0);
      //}
    
    }
    else{
      fill(128, 128, 128);
      textAlign(CENTER);
      text("Touch the green circle to start", width/2, 60);
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
  
    /* to control experience start */
    if (s.h_avatar.isTouchingBody(c1) && gameStart == false){
      gameStart = true;
      s.h_avatar.setSensor(false);
    }
    
    s.h_avatar.setDamping(0);
        
    /* implement planet gravitation */
    planet_x_coordinate = edgeTopLeftX + 12;
    planet_y_coordinate = edgeTopLeftY + worldHeight/2.0 + 0;
    effector_x_coordinate = s.h_avatar.getX();
    effector_y_coordinate = s.h_avatar.getY();
    satellite_distance_squared = (planet_x_coordinate - satellite.getX()) * (planet_x_coordinate - satellite.getX()) + (planet_y_coordinate - satellite.getY()) * (planet_y_coordinate - satellite.getY());
    float distance_squared = (planet_x_coordinate - effector_x_coordinate) * (planet_x_coordinate - effector_x_coordinate) + (planet_y_coordinate - effector_y_coordinate) * (planet_y_coordinate - effector_y_coordinate);
    /* only exert influence if the effector is to the right of the viscous layers */
    if (true)  {
      float satellite_force_x, satellite_force_y;
      float force_x, force_y;
      float satellite_cos_theta = abs(planet_x_coordinate - satellite.getX()) / sqrt(satellite_distance_squared);
      float satellite_sin_theta = abs(planet_y_coordinate - satellite.getY()) / sqrt(satellite_distance_squared);
      float cos_theta = abs(planet_x_coordinate - effector_x_coordinate) / sqrt(distance_squared);
      float sin_theta = abs(planet_y_coordinate - effector_y_coordinate) / sqrt(distance_squared);
      float gravitation_constant = 6.67e-8;
      float mass_effector = 100000; // in grams
      float mass_satellite = 1; // in grams
      mass_planet = 333000000; // in grams
      float force = gravitation_constant * mass_effector * mass_planet / (distance_squared + 5);
      float satellite_force = gravitation_constant * mass_satellite * mass_planet / (satellite_distance_squared + 50);
      satellite_force_x = satellite_force * satellite_cos_theta;
      satellite_force_y = satellite_force * satellite_sin_theta;
      force_x = force * cos_theta;
      force_y = force * sin_theta;
      
      if (!render_satellite)  {
      /* handle cases where signs flip */
      if (planet_x_coordinate >= effector_x_coordinate && planet_y_coordinate < effector_y_coordinate)  {
        // bottom left quadrant
        // y-axis force acts in negative direction
        s.h_avatar.addForce(force_x, -1*force_y); 
      }
      else if (planet_x_coordinate >= effector_x_coordinate && planet_y_coordinate >= effector_y_coordinate)  {
        // top left quadrant
        // both forces act in positive direction
        s.h_avatar.addForce(force_x, force_y);
      }
      else if (planet_x_coordinate < effector_x_coordinate && planet_y_coordinate < effector_y_coordinate)  {
        // bottom right quadrant
        // both forces act in negative direction
        s.h_avatar.addForce(-1*force_x, -1*force_y);
      }
      else  {
        // top right quadrant
        // x-axis force acts in negative direction
        s.h_avatar.addForce(-1*force_x, force_y);
      }
      }
      
      print("Satellite x velocity: " + satellite.getVelocityX() + "\n");
      print("Satellite y velocity: " + satellite.getVelocityY() + "\n");
      float c = 1000000.0;
      /* handle sign flips for satellite */
      if (planet_x_coordinate >= satellite.getX() && planet_y_coordinate < satellite.getY())  {
        // bottom left quadrant
        // y-axis force is negative
        satellite.addForce(satellite_force_x, -1*satellite_force_y);
        if (render_satellite)  {
          s.h_avatar.addForce(satellite_force_x*c, -1*satellite_force_y*c);
        }
        //print("X Force: " + satellite_force_x + "\n");
        //print("Y Force: " + -1*satellite_force_y + "\n");
      }
      else if (planet_x_coordinate >= satellite.getX() && planet_y_coordinate >= satellite.getY())  {
        // top left quadrant
        // both forces act in positive direction
        satellite.addForce(satellite_force_x, satellite_force_y);
        if (render_satellite)  {
          s.h_avatar.addForce(satellite_force_x*c, satellite_force_y*c);
        }
        //print("X Force: " + satellite_force_x + "\n");
        //print("Y Force: " + satellite_force_y + "\n");
      }
      else if (planet_x_coordinate < satellite.getX() && planet_y_coordinate < satellite.getY())  {
        // bottom right quadrant
        // both forces act in negative direction
        satellite.addForce(-1*satellite_force_x, -1*satellite_force_y);
        if (render_satellite)  {
          s.h_avatar.addForce(-1*satellite_force_x*c, -1*satellite_force_y*c);
        }
        //print("X Force: " + -1*satellite_force_x + "\n");
        //print("Y Force: " + -1*satellite_force_y + "\n");
      }
      else  {
        // top right quadrant
        // x-axis force is negative
        satellite.addForce(-1*satellite_force_x, satellite_force_y);
        if (render_satellite)  {
          s.h_avatar.addForce(-1*satellite_force_x*c, satellite_force_y*c);
        }
        //print("X Force: " + -1*satellite_force_x + "\n");
        //print("Y Force: " + satellite_force_y + "\n");
      }
    }
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

void keyPressed()  {
  if (key == 116)  {
    // toggle
    render_satellite = !render_satellite;
    if (render_satellite)  {
      s.h_avatar.setFill(255, 255, 255);
    }
    else  {
      s.h_avatar.setFill(255, 0, 0);
    }
  }
}

/* end helper functions section ****************************************************************************************/
