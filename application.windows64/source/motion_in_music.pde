// Adapted from Kinect Physics Example by Amnon Owed (09/15/12)

// import libraries
import processing.opengl.*; // opengl
import SimpleOpenNI.*; // kinect
import blobDetection.*; // blobs
import toxi.geom.*; // toxiclibs shapes and vectors
import toxi.processing.*; // toxiclibs display
import shiffman.box2d.*; // shiffman's jbox2d helper library
import org.jbox2d.collision.shapes.*; // jbox2d
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.common.*; // jbox2d
import org.jbox2d.dynamics.*; // jbox2d
import ddf.minim.*;
import ddf.minim.analysis.FFT;
import ddf.minim.analysis.*;
import java.util.Collections;

float spectrumScale = 2;
float STROKE_MAX = 10;
float STROKE_MIN = 2;
float strokeMultiplier = 1;
float audioThresh = 1;
float DECAY_RATE = 0;
float globalX, globalY;
float reScale;
float[] circles = new float[29];
int kinectWidth = 640;    // the kinect's dimensions to be used later on for calculations, to center and rescale from 640x480 to higher custom resolutions
int kinectHeight = 480;
boolean musicChange = false; 
SimpleOpenNI context;    // declare SimpleOpenNI object
BlobDetection theBlobDetection;    // declare BlobDetection object
ToxiclibsSupport gfx;    // ToxiclibsSupport for displaying polygons
PolygonBlob poly;    // declare custom PolygonBlob object (see class for more info)
Minim minim;
AudioPlayer music1;
AudioPlayer music2;
BeatDetect beat;
BeatListener bl;
PImage cam = createImage(640, 480, RGB);
PImage blobs;    // PImage to hold incoming imagery and smaller one for blob detection
color blobColor;    // blob color
String[] palettes = {
  "-1117720,-13683658,-8410437,-9998215,-1849945,-5517090,-4250587,-14178341,-5804972,-3498634", 
  "-67879,-9633503,-8858441,-144382,-4996094,-16604779,-588031", 
  "-1978728,-724510,-15131349,-13932461,-4741770,-9232823,-3195858,-8989771,-2850983,-10314372"
};    // three color palettes (artifact from me storingmany interesting color palettes as strings in an external data file ;-)
color[] colorPalette;
ArrayList<CustomShape> customShapes = new ArrayList<CustomShape>();    // list to hold all the custom shapes (circles, polygons)
Particle[] flow = new Particle[2800];
Box2DProcessing box2d;    // the main PBox2D object in which all the physics-based stuff is happening
FFT fftLog;

void setup() {
  size(1440, 720, OPENGL);  // it's possible to customize this, for example 1920x1080
  frameRate(60);
  context = new SimpleOpenNI(this);
  poly = new PolygonBlob();
  minim = new Minim(this);
  music1 = minim.loadFile("saika.mp3");
  music1.rewind();
  music1.play();
  music1.loop();
  fftLog = new FFT(music1.bufferSize(), music1.sampleRate());
  fftLog.logAverages(22, 3);
  beat = new BeatDetect(music1.bufferSize(), music1.sampleRate());
  beat.setSensitivity(300);
  bl = new BeatListener(beat, music1);
  String[] paletteStrings = split(palettes[int(random(palettes.length))], ",");    // turn a palette into a series of strings
  colorPalette = new color[paletteStrings.length];        // turn strings into colors
  for (int i = 0; i < paletteStrings.length; i++) {
    colorPalette[i] = int(paletteStrings[i]);
  }
  if (!context.enableDepth() || !context.enableUser()) {      // initialize SimpleOpenNI object
    println("Kinect not connected!");        // if it returns false then the Kinect is not working correctly, make sure the green light is blinking
    exit();
  } else {
    context.setMirror(true);      // mirror the image to be more intuitive
    reScale = (float) width / kinectWidth;    // calculate the reScale value, currently it's rescaled to fill the complete width (cuts of top-bottom), it's also possible to fill the complete height (leaves empty sides)
    blobs = createImage(kinectWidth/3, kinectHeight/3, RGB);    // create a smaller blob image for speed and efficiency
    theBlobDetection = new BlobDetection(blobs.width, blobs.height);     // initialize blob detection object to the blob image dimensions
    theBlobDetection.setThreshold(0.3);
    gfx = new ToxiclibsSupport(this);    // initialize ToxiclibsSupport object
    box2d = new Box2DProcessing(this);    // setup box2d, create world, set gravity
    box2d.createWorld();
    box2d.setGravity(0, -100);
    setRandomColors(1);     // set random colors (background, blob)
    setupFlowfield();
    float gapWidth = kinectWidth/21;
    for (int i = 0; i < 20; i++)
    {
      drawString(gapWidth*(i+1), 2, 30);
    }
  }
}


void draw() {
  background(200);
  fftLog.forward(music1.left);      // detect the background music beat
  noStroke();

  // draw the background effect
  for (int i = 0; i < circles.length; i+=2) {
    float amplitude = fftLog.getAvg(i);        // What is the average height in relation to the screen height?
    if (amplitude < audioThresh) {        // If we hit a threshhold, then set the circle radius to new value
      circles[i] = amplitude*(height/2);
    } else { // Otherwise, decay slowly
      circles[i] = max(0, min(height, circles[i]-DECAY_RATE));
    }
    float centerFrequency = fftLog.getAverageCenterFrequency(i);        // What is the centerpoint of the this frequency band?
    float averageWidth = fftLog.getAverageBandWidth(i);        // What is the average width of this freqency?
    float lowFreq = centerFrequency - averageWidth/2;        // Get the left and right bounds of the frequency
    float highFreq = centerFrequency + averageWidth/2;
    noStroke();
    fill(100+amplitude*1.5, 100);
    rect(width/30*i, 0, width/15, height);

    if (frameCount % 2 == 0 && amplitude > 20) {    // create objects according to rythm type
      float size = map(amplitude, 30, 300, 10, 20);
      if (beat.isKick()) {
        color c = color(100, map(i, 0, 29, 255, 0), 200, constrain(amplitude*20+i, 0, 200));
        CustomShape shape1 = new CustomShape(width/30*(i+1), 0, -1, BodyType.DYNAMIC, c) ;
        customShapes.add(shape1);
      } else if (beat.isSnare()) {
        color c = color(200, 0, map(i, 0, 29, 0, 200), constrain(amplitude*20+i, 0, 100));
        CustomShape shape2 = new CustomShape(width/30*(i+1), 0, size/1.5, BodyType.DYNAMIC, c);
        customShapes.add(shape2);
      } else if (beat.isHat()) {
        color c = color(map(i, 0, 29, 0, 150), 0, map(i, 0, 29, 255, 0), constrain(amplitude*20+i, 0, 100));
        CustomShape shape2 = new CustomShape(width/30*(i+1), 0, size/1.5, BodyType.DYNAMIC, c);
        customShapes.add(shape2);
      }
    }
  }

  // update the SimpleOpenNI object
  context.update();  
  cam = context.userImage();
  cam.loadPixels();
  color black = color(0, 0, 0);
  for (int i = 0; i < cam.pixels.length; i++) {  // filter out grey pixels (mixed in depth image) 
    color pix = cam.pixels[i];
    int blue = pix & 0xff;
    if (blue == ((pix >> 8) & 0xff) && blue == ((pix >> 16) & 0xff)) {    // if g = b and r = b then it means it's grey
      cam.pixels[i] = black;  //set grey pixels to black
    }
  }
  cam.updatePixels();

  // the actual drawing part
  blobs.copy(cam, 0, 0, cam.width, cam.height, 0, 0, blobs.width, blobs.height);      // copy the image into the smaller blob image
  blobs.filter(BLUR, 2);      // blur the blob image
  theBlobDetection.computeBlobs(blobs.pixels);      // detect the blobs
  poly = new PolygonBlob();      // initialize a new polygon
  poly.createPolygon();    // create the polygon from the blobs (custom functionality, see class)
  poly.createBody();      // create the box2d body from the polygon
  updateAndDrawBox2D();      // update and draw everything (see method)
  poly.destroyBody();      // destroy the person's body (important!)
  drawFlowfield();      // set the colors randomly every 240th frame
}

void setupFlowfield() {
  strokeWeight(0.5);
  for (int i = 0; i < flow.length; i++) {
    color c = color(map(i, 0, flow.length, 0, 150), 0, map(i, 0, flow.length, 255, 0));
    flow[i] = new Particle(i/10000.0, c);
  }
  setRandomColors(1);
}

void drawFlowfield() {
  translate(0, (height - kinectHeight*reScale)/2);
  scale(reScale);
  globalX = noise(frameCount * 0.01) * width/2 + width/4;
  globalY = noise(frameCount * 0.005 + 5) * height;
  for (Particle p : flow) {
    p.updateAndDisplay();
  }
}


void updateAndDrawBox2D() {
  box2d.step();      // take one step in the box2d physics world
  translate(0, (height - kinectHeight*reScale)/2);  // center and reScale from Kinect to custom dimensions
  scale(reScale);
  noStroke();
  fill(blobColor);
  gfx.polygon2D(poly);
  for (int i= customShapes.size()-1; i >= 0; i--) {      // display all the shapes (circles, polygons) and go backwards to allow removal of shapes
    CustomShape cs = customShapes.get(i);
    if (cs.done()) {        // if the shape is off-screen remove it (see class for more info)
      customShapes.remove(i);
    } else {          // otherwise update (keep shape outside person) and display (circle or polygon)
      cs.update();
      cs.display();
    }
  }
}

void setRandomColors(int nthFrame) {    // sets the colors every nth frame
  if (frameCount % nthFrame == 0) {  // set background color to first color from palette  // set blob color to second color from palette
    blobColor = color(50, 55, 100);
    for (CustomShape cs : customShapes) {        // set all shape colors randomly
      cs.col = getRandomColor();
    }
  }
}

void drawString(float x, float size, int cards) {
  float gapHeight = kinectHeight / cards;      // cards improve the figure shape but it also causes problem like polygons too close
  color c = color(200);
  CustomShape last_shape = new CustomShape(x, -40, size, BodyType.DYNAMIC, c);
  customShapes.add(last_shape);
  CustomShape next_shape;
  for (int i = 0; i < cards; i++) {
    float y = -20 + gapHeight * (i+1);
    next_shape = new CustomShape(x, -20 + gapHeight * (i+1), size, BodyType.DYNAMIC, c);
    DistanceJointDef jd = new DistanceJointDef();
    Vec2 c1 = last_shape.body.getWorldCenter();
    Vec2 c2 = next_shape.body.getWorldCenter();
    // offset the anchors so the cards hang vertically
    c1.y = c1.y + size / 5;
    c2.y = c2.y - size / 5;
    jd.initialize(last_shape.body, next_shape.body, c1, c2);
    jd.length = box2d.scalarPixelsToWorld(gapHeight - 1);
    box2d.createJoint(jd);
    customShapes.add(next_shape);
    last_shape = next_shape;
  }
}

color getRandomColor() {    // returns a random color from the palette (excluding first aka background color)
  return colorPalette[int(random(1, colorPalette.length))];
}

