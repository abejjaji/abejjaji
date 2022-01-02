// I2C device) demonstration Processing sketch for MPU6G05 Gyroscope output developped by STEAM NOOR


import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

import processing.sound.*;
SoundFile file;

// Planets, by Andres Colubri
// Sun and mercury textures from http://planetpixelemporium.com
// Star field picture from http://www.galacticimages.com/

PImage starfield;

PShape sun;
PImage suntex;

PShape planet1;
PImage surftex1;
PImage cloudtex;

PShape planet2;
PImage surftex2;
//PImage airplane;
PShape airplane;

// NOTE: requires ToxicLibs to be installed in order to run properly.
// 1. Download from http://toxiclibs.org/downloads
// 2. Extract into [userdir]/Processing/libraries
//    (location may be different on Mac/Linux)
// 3. Run and bask in awesomeness

ToxiclibsSupport gfx;

Serial port;                         // The serial port
char[] teapotPacket = new char[14];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int synced = 0;
int interval = 0;

float[] q = new float[4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];

void setup() {
  
  size(1200, 800, P3D);
  
 starfield = loadImage("starfield.jpg");
  suntex = loadImage("sun.jpg");  
  surftex1 = loadImage("planet.jpg"); 
  airplane = loadShape("airplane.obj");
    
  surftex2 = loadImage("mercury.jpg"); 
    

  noStroke();
  fill(255);
  sphereDetail(40);

  sun = createShape(SPHERE, 100);
  sun.setTexture(suntex);  

  planet1 = createShape(SPHERE, 120);
  planet1.setTexture(surftex1);
  
  planet2 = createShape(SPHERE, 40);
  planet2.setTexture(surftex2);
 //airplane=loadImage("airplane.png");
  
 // Load a soundfile from the data folder of the sketch and play it back in a loop
  file = new SoundFile(this, "sample.mp3");
  file.loop();
  
  // 300px square viewport using OpenGL rendering
  // size(300, 300, OPENGL);
    gfx = new ToxiclibsSupport(this);

    // setup lights and antialiasing
    lights();
    smooth();
  
    // display serial port list for debugging/clarity
    println(Serial.list());

    // get the first available port (use EITHER this OR the specific port code below)
   // String portName = Serial.list()[0];
    
    // get a specific serial port (use EITHER this OR the first-available code above)
    String portName = "COM16";
    
    // open the serial port
    port = new Serial(this, portName, 115200);
    
    // send single character to trigger DMP init/start
    // (expected by MPU6050_DMP6 example Arduino sketch)
    port.write('r');
}

void draw() {
  
  
  // background to clear the screen anyways, otherwise A3D will think
  // you want to keep each drawn frame in the framebuffer, which results in 
  // slower rendering.
  
     
  background(0);
  
  pushMatrix();
  
  hint(DISABLE_DEPTH_MASK);
  hint(ENABLE_DEPTH_MASK);
  
   
   
  // Disabling writing to the depth mask so the 
  // background image doesn't occludes any 3D object.
  
   hint(DISABLE_DEPTH_MASK);
   image(starfield, 0, 0, width, height);
   hint(ENABLE_DEPTH_MASK);
  //image(airplane,-680,480,3000,400); 
   pushMatrix();
  translate(width/2,( height/2)-105, -300);  
    
  pushMatrix();
  rotateY(PI * frameCount / 1000);
  shape(sun);
  popMatrix();
  

  pointLight(255,  255,  255,  0,  0,  0);  
  rotateY(PI * frameCount /300);
  translate(0, 0, 400);
  shape(planet2);  
  
 popMatrix();
  
 noLights();
 pointLight(255,  255,  255,  0,  0,  -150);  
 translate(0.75 * width,  0.6 * height,  50);
 shape(planet1);
 popMatrix();

    if (millis() - interval > 1000) {
        // resend single character to trigger DMP init/start
        // in case the MPU is halted/reset while applet is running
        port.write('r');
        interval = millis();
    }
    
    // black background
    //background(0);
    
    // translate everything to the middle of the viewport
   pushMatrix();
    //translate(width / 2,( height / 2)+300);

    // 3-step rotation from yaw/pitch/roll angles (gimbal lock!)
    // ...and other weirdness I haven't figured out yet
    //rotateY(-ypr[0]);
    //rotateZ(-ypr[1]);
    //rotateX(-ypr[2]);

    // toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
    // (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
    // different coordinate system orientation assumptions between Processing
    // and InvenSense DMP)
    float[] axis = quat.toAxisAngle();
    rotate(axis[0], -axis[1], axis[3], axis[2]);
    
    
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI/2);
    hint(DISABLE_DEPTH_MASK);
    hint(ENABLE_DEPTH_MASK);         
           
  //ambientLight(2, 8,201);
  //pointLight(255,  255,  255,  0,  0, 1); 
  //smooth();
  camera(0,0, height * .86602,    0,-230, 0, 0,1,0);
  scale(0.12);
  //Uncomment this line to stop rotation
  rotateY(PI * frameCount / 1000);
  rotateX(PI/3);
  shape(airplane);
  popMatrix();     
  popMatrix();    

}


void serialEvent(Serial port) {
    interval = millis();
    while (port.available() > 0) {
        int ch = port.read();

        if (synced == 0 && ch != '$') return;   // initial synchronization - also used to resync/realign if needed
        synced = 1;
        print ((char)ch);

        if ((serialCount == 1 && ch != 2)
            || (serialCount == 12 && ch != '\r')
            || (serialCount == 13 && ch != '\n'))  {
            serialCount = 0;
            synced = 0;
            return;
        }

        if (serialCount > 0 || ch == '$') {
            teapotPacket[serialCount++] = (char)ch;
            if (serialCount == 14) {
                serialCount = 0; // restart packet byte position
                
                // get quaternion from data packet
                q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
                q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
                q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
                q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
                for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];
                
                // set our toxilibs quaternion to new data
                quat.set(q[0], q[1], q[2], q[3]);

               
            }
        }
    }
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = PI / sides;
    //beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
        //vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
        //vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
        // hint(DISABLE_DEPTH_MASK);
           // hint(ENABLE_DEPTH_MASK);         
           // lights();
           // smooth();
    pushMatrix();         
  
            hint(DISABLE_DEPTH_MASK);
           hint(ENABLE_DEPTH_MASK);         
           
  //ambientLight(2, 8,201);
  //pointLight(255,  255,  255,  0,  0, 1); 
  //smooth();
  camera(0,0, height * .86602,    0,-230, 0, 0,1,0);
  scale(0.1);
  //rotateY(PI * frameCount / 1000);
  //rotateZ(PI/7);
  shape(airplane);
  
  popMatrix();
             // translate(-width/4, -height/2,50);
              // rotateZ(PI/3);
              // rotateY(PI/3);
               // rotateX(PI/3);
            // popMatrix();
        angle += angleIncrement;
    }
    endShape();
    
    // If it is not a cone, draw the circular top cap
    if (topRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
        
        // Center point
        vertex(0, 0, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
  
    // If it is not a cone, draw the circular bottom cap
    if (bottomRadius != 0) {
        angle = 0;
       // beginShape(TRIANGLE_FAN);
    
        // Center point
        //vertex(0, tall, 0);
        for (int i = 0; i < sides + 1; i++) {
            angle += angleIncrement;            
           
            
        }
        endShape();
    }
}
