// Learning Processing
// Daniel Shiffman
// http://www.learningprocessing.com
import processing.video.*;
import processing.sound.*;
import ddf.minim.*;
import boofcv.processing.*;
import boofcv.struct.image.*;
import java.util.*;
import boofcv.alg.filter.binary.*;
import georegression.struct.point.*;

// Declaring a variable of type PImage
// PImage is a class available from the Processing core library.
PImage img, imgContour, imgBlobs, redImg; 
PVector center = new PVector(0,0);
Joint jN, jE, jS, jW;
int p1X, p1Y, p2X, p2Y, p3X, p3Y, p4X, p4Y;
float qom, speed, accel, timeEffort, flow;
float qomFiltered, timeEffortFiltered, flowFiltered;
int medianFilterOrder = 10;
int bufferIndex = 0;
float [] qomBuffer = new float [medianFilterOrder];
float [] timeEffortBuffer = new float [medianFilterOrder];
float [] flowBuffer = new float [medianFilterOrder];
double threshold;
Capture cam;
float windowSize;
float currTime, timeLastWave;
float waveThreshold;
Table table;
SoundFile [] firstLevelWaves, secondLevelWaves, thirdLevelWaves, firstLevelBirds, secondLevelBirds;
AudioPlayer glue;
Minim minim;

//float eins, zwei, drei, vier, funf;
void setup() {
  frameRate(60);
  // set window size to count how many waves per window size are triggered
  waveThreshold = 10000; // set lowest wave frequency threshold to 10 seconds
  currTime = millis();
  // create for reference Joints 
  jN = new Joint();
  jE = new Joint();
  jS = new Joint();
  jW = new Joint();
  qom = 0;
  
  // prepare file to write data (movement descriptors)
  table = new Table();
  table.addColumn("QoM");
  table.addColumn("Speed");
  table.addColumn("Acc");
  table.addColumn("TiE");
  table.addColumn("Flow");
  table.addColumn("qomFiltered");
  table.addColumn("timeEffortFiltered");
  table.addColumn("flowFiltered");
  // load sound files
  // waves and birds respectively must be of same lengthe otherwise the selectors might access out of bounds
  firstLevelWaves = new SoundFile [] {new SoundFile(this, "smallWave.wav"), new SoundFile(this, "smallWave2.wav"),
                               new SoundFile(this, "mediumWave2.wav"), new SoundFile(this, "mediumWave.wav"),
                               new SoundFile(this, "smallWaveWithSeaGull.wav")};
  secondLevelWaves = new SoundFile[]{new SoundFile(this, "mediumWave2.wav"), new SoundFile(this, "mediumWave.wav"),
                               new SoundFile(this, "smallWaveWithSeaGull.wav"), new SoundFile(this, "bigWave.wav"),
                               new SoundFile(this, "smallWave.wav")};
  thirdLevelWaves =  new SoundFile[]{new SoundFile(this, "mediumWave2.wav"), new SoundFile(this, "mediumWave.wav"),
                               new SoundFile(this, "bigWave.wav"), new SoundFile(this, "bigWave2.wav"),
                               new SoundFile(this, "smallWaveWithSeaGull.wav")};
  firstLevelBirds = new SoundFile[]{new SoundFile(this, "smallBird1.wav"), new SoundFile(this, "smallDoubleSeagull.wav"), 
                                    new SoundFile(this, "smallSeagull.wav")};                                    
  secondLevelBirds = new SoundFile[]{new SoundFile(this, "bigBird1.wav"), new SoundFile(this, "bigBird2.wav"),
                                     new SoundFile(this, "bigBird3.wav")};
                               
  //waveSoundFiles =new SoundFile [] {new SoundFile(this, "bigWave.wav"), new SoundFile(this, "bigWave2.wav"),
  //                             new SoundFile(this, "smallWave.wav"), new SoundFile(this, "smallWave2.wav"),
  //                             new SoundFile(this, "mediumWave2.wav"), new SoundFile(this, "mediumWave.wav"),
  //                             new SoundFile(this, "smallWaveWithSeaGull.wav")};
  // create background noise
  minim = new Minim(this);
  glue = minim.loadFile("glue.wav");
  glue.loop();
  
  // setup Camera
  String[] cameras = Capture.list();
  if (cameras.length == 0) {
  println("There are no cameras available for capture.");
  exit();
  } else {
  println("Available cameras:");
  for (int i = 0; i < cameras.length; i++) {
  println(cameras[i]);
  }
    
  // The camera can be initialized directly using an 
  // element from the array returned by list():
  cam = new Capture(this, cameras[1]);
  cam.start(); 
  }
  p1X = 1000;
  size(800, 600);
}

void draw() {
  background(0);
  if (cam.available()) {
  cam.read();
  }
  image(cam, 0, 0);
  
  SimpleGray gray = Boof.gray(cam,ImageDataType.U8);
  // if too much noise, blur the image -> less accurate contour but less noise
  //gray = gray.blurMean(5);
  // find blobs and contour of the particles
  // some image processing that could be applied
  //gray = gray.enhanceSharpen4();
  //gray = gray.blurGaussian(20, 8);
  
  // Threshold the image using its mean value
  threshold = gray.mean();
  //ResultsBlob results = gray.threshold(threshold,true).erode8(1).contour();
  
  // other thresholding methods
  ResultsBlob results = gray.thresholdSquare(40, .85,true).erode8(1).contour();
  // ResultsBlob results = gray.thresholdSauvola(10, .3,true).erode8(1).contour();
  // ResultsBlob results = gray.thresholdGaussian(40, .8,true).erode8(1).contour();

  // Visualize the results
  imgContour = results.getContours().visualize();
  
  // The image() function displays the image at a location
  // in this case the point (0,0).
  List <Contour> list = results.contour;
  
  // extract contours and perform maximization and minimization 
  // to extract reference points
  for(Contour c: list){
    for(Point2D_I32 p : c.external){
       if(p1X>p.x){
         p1X = p.x;
         p1Y = p.y;
       } else if(p2X<p.x){
         p2X = p.x;
         p2Y = p.y;
       } else if(p3Y<p.y){
         p3X = p.x;
         p3Y = p.y;
       } else if(p4Y>p.y){
         p4X = p.x;
         p4Y = p.y;
       }
    }  
  }
  jW.setNewPosition(p1X,p1Y); //<>//
  jE.setNewPosition(p2X,p2Y);
  jS.setNewPosition(p3X,p3Y);
  jN.setNewPosition(p4X,p4Y);
  
  //eins = jW.getPosPlus2().x;
  //zwei = jW.getPosPlus1().x;
  //drei = jW.getPos().x;
  //vier = jW.getPosMinus1().x;
  //funf = jW.getPosMinus2().x;
  //// draw circle around reference points
  ellipse(p1X, p1Y, 40, 40);
  ellipse(p2X, p2Y, 20, 20);
  //ellipse(p3X, p3Y, 30, 30);
  //ellipse(p4X, p4Y, 10, 10);
  if(mousePressed){
  redImg = gray.convert();
  image(redImg, 0, 0);
  } else{
  image(imgContour,0,0);
  tint(255, 127);
  }
  
  // Computation of movement descriptors
  // test with Joints
  //float qom = computeQuantityOfMotion(jN);

  // test with mouse
  //jN.setNewPosition(mouseX,mouseY);
  //print("currpos "+jN.getPosition()+"\n");
  //print("-1pos "+jN.getLastPosition()+"\n");
  //print("-2currpos "+jN.getPositionBeforeLast()+"\n");
  //jS.setNewPosition(mouseX,mouseY);
  qom = computeQuantityOfMotion(jW,jE);
  timeEffort = computeTimeEffort(jW,jE);
  flow = computeFlow(jN,jW,jE);
  
  qomBuffer[bufferIndex]=qom;
  timeEffortBuffer[bufferIndex]=timeEffort;
  flowBuffer[bufferIndex]=flow;
  bufferIndex += 1;
  bufferIndex = bufferIndex%medianFilterOrder;
  
  qomFiltered = medianFilter(qomBuffer);
  timeEffortFiltered = medianFilter(timeEffortBuffer);
  flowFiltered = medianFilter(flowBuffer);
  
  //speed = computeSpeed(jN)+computeSpeed(jS)+computeSpeed(jW)+computeSpeed(jE);
  //accel = computeAccelerationScalar(jN)+computeAccelerationScalar(jS)+
          //computeAccelerationScalar(jW)+computeAccelerationScalar(jE);

  //print("Velocity " + computeVelocity(jN) + "\n");
  //print("Speed: " + computeSpeed(jN) + "\n");
  print("QuantityOfMotion " + qom + "\n");
  //print("Acceleration " + accel + "\n");
  print("TimeEffort " + timeEffort + "\n");
  println("Flow" + flow);
  //print(timeLastWave);

  
  // write data into csv file
  TableRow row = table.addRow();
  //TableRow row1 = table.addRow();
  //TableRow row2 = table.addRow();
  //TableRow row3 = table.addRow();
  row.setFloat("QoM", qom);
  row.setFloat("Speed", speed);
  row.setFloat("Acc", accel);
  row.setFloat("TiE", timeEffort);
  row.setFloat("Flow", flow);
  row.setFloat("qomFiltered", qomFiltered);
  row.setFloat("timeEffortFiltered", timeEffortFiltered);
  row.setFloat("flowFiltered", flowFiltered);
  saveTable(table, "data.csv");

  // make randomizer range smaller for better flow and longer for less flow

  // sound mapping 
  float flowMapping = (flowFiltered/600)*1000;
  float randomizerWave = random(0, flowMapping);   // randomize threshold to not play the waves back with a specific metrum
  float randomizerBird = random(0, flowMapping);
  int waveSelector = int(random(0, firstLevelWaves.length)); // randomize waveSelector to not play back the same waves all the time
  int birdSelector = int(random(0, firstLevelBirds.length));
  timeLastWave = millis()-currTime; // calculate when the lastWave was played back
  
  //Trigger Wave Sounds
  if(qom<60 && timeLastWave>waveThreshold-randomizerWave){
    firstLevelWaves[waveSelector].play();
    print("playfirst");
    currTime = millis();
  } else if(qom<120 && qom>60 && timeLastWave>(waveThreshold/2)-randomizerWave){
    secondLevelWaves[waveSelector].play();
    firstLevelBirds[birdSelector].play();
    print("playsecond");
    currTime = millis();
  } else if(qom>120 && timeLastWave>(waveThreshold/5)-randomizerWave){
    thirdLevelWaves[waveSelector].play();
    secondLevelBirds[birdSelector].play();
    currTime = millis();
  }  
  //Trigger bird sounds
  if(qom<120 && qom>60 && timeLastWave>(waveThreshold/2)-randomizerBird){
    firstLevelBirds[birdSelector].play();
    currTime = millis();
  } else if(qom>120 && timeLastWave>(waveThreshold/5)-randomizerBird){
    secondLevelBirds[birdSelector].play();
    currTime = millis();
  }
  
  // reinitialize starting values for each frame
  p1X=5000; 
  p1Y=0; 
  p2X=0;
  p2Y=0; 
  p3X=0; 
  p3Y=0;
  p4X=0;
  p4Y=5000;
}

PVector computeVelocity(Joint j){
  float vx = ((j.getPosPlus1().x-j.getPosMinus1().x)/2);
  float vy = ((j.getPosPlus1().y-j.getPosMinus1().y)/2);
  PVector velocity = new PVector(vx,vy);
  return (velocity);
}

PVector computeAcceleration(Joint j){
  float ax = j.getPosPlus1().x-2*j.getPos().x+j.getPosMinus1().x;
  float ay = j.getPosPlus1().y-2*j.getPos().y+j.getPosMinus1().y;
  PVector accel = new PVector(ax,ay);
  return accel;
}

PVector computeJerk(Joint j){
  float jx = ((j.getPosPlus2().x-2*j.getPosPlus1().x+2*j.getPosMinus1().x-j.getPosMinus2().x)/2);
  float jy = ((j.getPosPlus2().y-2*j.getPosPlus1().y+2*j.getPosMinus1().y-j.getPosMinus2().y)/2);
  PVector jerk = new PVector(jx,jy);
  return jerk;
}

float computeSpeed(Joint j){
  PVector velocity = computeVelocity(j);
  float speed = sqrt(pow(velocity.x,2)+pow(velocity.y,2));
  return speed;
}

float computeAccelerationScalar(Joint j){
  PVector acceleration = computeAcceleration(j);
  float accelScalar = sqrt(pow(acceleration.x,2)+pow(acceleration.y,2));
  return accelScalar;
}

float computeJerkScalar(Joint j){
  PVector jerk = computeJerk(j);
  float jerkScalar = sqrt(pow(jerk.x,2)+pow(jerk.y,2));
  return jerkScalar;
}

// this method computes the quantity of motion assumed weights w are 1 for each joint
float computeQuantityOfMotion(Joint... j){
  float quantityOfMotion = 0;
  for(int i = 0; i<j.length; i++){
    quantityOfMotion = computeSpeed(j[i])+quantityOfMotion;
  }
  return (quantityOfMotion/j.length);
}

// this method computes the time effort, weights are 1 for each joint and T = 1
float computeTimeEffort(Joint... j){
  float timeEffort = 0;
  for(int i = 0; i<j.length; i++){
    timeEffort = computeAccelerationScalar(j[i])+timeEffort;
  }
  return (timeEffort);
}

//flow with Timewindow 1
float computeFlow(Joint... j){
  float flow = 0;
  for(int i = 0; i<j.length; i++){
    flow = computeJerkScalar(j[i])+flow;
  }
  return (flow);
}

float medianFilter(float [] buffer){
  float [] bufferSorted = sort(buffer);
  return bufferSorted[buffer.length/2];
}