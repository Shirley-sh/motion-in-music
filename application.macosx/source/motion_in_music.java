import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.opengl.*; 
import SimpleOpenNI.*; 
import blobDetection.*; 
import toxi.geom.*; 
import toxi.processing.*; 
import shiffman.box2d.*; 
import org.jbox2d.collision.shapes.*; 
import org.jbox2d.dynamics.joints.*; 
import org.jbox2d.common.*; 
import org.jbox2d.dynamics.*; 
import ddf.minim.*; 
import ddf.minim.analysis.FFT; 
import ddf.minim.analysis.*; 
import java.util.Collections; 
import java.util.List; 
import java.util.Arrays; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class motion_in_music extends PApplet {

// Adapted from Kinect Physics Example by Amnon Owed (09/15/12)

// import libraries
 // opengl
 // kinect
 // blobs
 // toxiclibs shapes and vectors
 // toxiclibs display
 // shiffman's jbox2d helper library
 // jbox2d

 // jbox2d
 // jbox2d





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
int blobColor;    // blob color
String[] palettes = {
  "-1117720,-13683658,-8410437,-9998215,-1849945,-5517090,-4250587,-14178341,-5804972,-3498634", 
  "-67879,-9633503,-8858441,-144382,-4996094,-16604779,-588031", 
  "-1978728,-724510,-15131349,-13932461,-4741770,-9232823,-3195858,-8989771,-2850983,-10314372"
};    // three color palettes (artifact from me storingmany interesting color palettes as strings in an external data file ;-)
int[] colorPalette;
ArrayList<CustomShape> customShapes = new ArrayList<CustomShape>();    // list to hold all the custom shapes (circles, polygons)
Particle[] flow = new Particle[2800];
Box2DProcessing box2d;    // the main PBox2D object in which all the physics-based stuff is happening
FFT fftLog;

public void setup() {
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
  String[] paletteStrings = split(palettes[PApplet.parseInt(random(palettes.length))], ",");    // turn a palette into a series of strings
  colorPalette = new int[paletteStrings.length];        // turn strings into colors
  for (int i = 0; i < paletteStrings.length; i++) {
    colorPalette[i] = PApplet.parseInt(paletteStrings[i]);
  }
  if (!context.enableDepth() || !context.enableUser()) {      // initialize SimpleOpenNI object
    println("Kinect not connected!");        // if it returns false then the Kinect is not working correctly, make sure the green light is blinking
    exit();
  } else {
    context.setMirror(true);      // mirror the image to be more intuitive
    reScale = (float) width / kinectWidth;    // calculate the reScale value, currently it's rescaled to fill the complete width (cuts of top-bottom), it's also possible to fill the complete height (leaves empty sides)
    blobs = createImage(kinectWidth/3, kinectHeight/3, RGB);    // create a smaller blob image for speed and efficiency
    theBlobDetection = new BlobDetection(blobs.width, blobs.height);     // initialize blob detection object to the blob image dimensions
    theBlobDetection.setThreshold(0.3f);
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


public void draw() {
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
    fill(100+amplitude*1.5f, 100);
    rect(width/30*i, 0, width/15, height);

    if (frameCount % 2 == 0 && amplitude > 20) {    // create objects according to rythm type
      float size = map(amplitude, 30, 300, 10, 20);
      if (beat.isKick()) {
        int c = color(100, map(i, 0, 29, 255, 0), 200, constrain(amplitude*20+i, 0, 200));
        CustomShape shape1 = new CustomShape(width/30*(i+1), 0, -1, BodyType.DYNAMIC, c) ;
        customShapes.add(shape1);
      } else if (beat.isSnare()) {
        int c = color(200, 0, map(i, 0, 29, 0, 200), constrain(amplitude*20+i, 0, 100));
        CustomShape shape2 = new CustomShape(width/30*(i+1), 0, size/1.5f, BodyType.DYNAMIC, c);
        customShapes.add(shape2);
      } else if (beat.isHat()) {
        int c = color(map(i, 0, 29, 0, 150), 0, map(i, 0, 29, 255, 0), constrain(amplitude*20+i, 0, 100));
        CustomShape shape2 = new CustomShape(width/30*(i+1), 0, size/1.5f, BodyType.DYNAMIC, c);
        customShapes.add(shape2);
      }
    }
  }

  // update the SimpleOpenNI object
  context.update();  
  cam = context.userImage();
  cam.loadPixels();
  int black = color(0, 0, 0);
  for (int i = 0; i < cam.pixels.length; i++) {  // filter out grey pixels (mixed in depth image) 
    int pix = cam.pixels[i];
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

public void setupFlowfield() {
  strokeWeight(0.5f);
  for (int i = 0; i < flow.length; i++) {
    int c = color(map(i, 0, flow.length, 0, 150), 0, map(i, 0, flow.length, 255, 0));
    flow[i] = new Particle(i/10000.0f, c);
  }
  setRandomColors(1);
}

public void drawFlowfield() {
  translate(0, (height - kinectHeight*reScale)/2);
  scale(reScale);
  globalX = noise(frameCount * 0.01f) * width/2 + width/4;
  globalY = noise(frameCount * 0.005f + 5) * height;
  for (Particle p : flow) {
    p.updateAndDisplay();
  }
}


public void updateAndDrawBox2D() {
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

public void setRandomColors(int nthFrame) {    // sets the colors every nth frame
  if (frameCount % nthFrame == 0) {  // set background color to first color from palette  // set blob color to second color from palette
    blobColor = color(50, 55, 100);
    for (CustomShape cs : customShapes) {        // set all shape colors randomly
      cs.col = getRandomColor();
    }
  }
}

public void drawString(float x, float size, int cards) {
  float gapHeight = kinectHeight / cards;      // cards improve the figure shape but it also causes problem like polygons too close
  int c = color(200);
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

public int getRandomColor() {    // returns a random color from the palette (excluding first aka background color)
  return colorPalette[PApplet.parseInt(random(1, colorPalette.length))];
}

class BeatListener implements AudioListener
{
  private BeatDetect beat;
  private AudioPlayer source;

  BeatListener(BeatDetect beat, AudioPlayer source)
  {
    this.source = source;
    this.source.addListener(this);
    this.beat = beat;
  }

  public void samples(float[] samps)
  {
    beat.detect(source.mix);
  }

  public void samples(float[] sampsL, float[] sampsR)
  {
    beat.detect(source.mix);
  }
}
/*class Boundary {
  float x, y, w, h;
  Body b;

  Boundary(float x_, float y_, float w_, float h_) {
    x = x_;
    y = y_;
    w = w_;
    h = h_;

    // Define the polygon
    PolygonShape sd = new PolygonShape();
    float box2dW = box2d.scalarPixelsToWorld(w/2);
    float box2dH = box2d.scalarPixelsToWorld(h/2);
    sd.setAsBox(box2dW, box2dH);

    // Create the body
    BodyDef bd = new BodyDef();
    bd.type = BodyType.STATIC;
    bd.position.set(box2d.coordPixelsToWorld(x, y));
    b = box2d.createBody(bd);

    // Attach the shape to the body using a Fixture
    b.createFixture(sd, 1);
    b.setUserData(this);
  }

  void display() {
    fill(0);
    stroke(0);
    rectMode(CENTER);
    rect(x, y, w, h);
  }
}
*/

// Class for falling objects



class CustomShape {
  Body body;  // to hold the box2d body
  Polygon2D toxiPoly;  // to hold the Toxiclibs polygon shape
  int col;  // custom color for each shape
  float r;  // radius (also used to distinguish between circles and polygons in this combi-class
  CustomShape(float x, float y, float r, BodyType type, int c) {
    this.r = r;
    makeBody(x, y, type, c);        // create a body (polygon or circle based on the r)
    col = c;    // get a random color
  }

  public void makeBody(float x, float y, BodyType type, int c) {    // define a dynamic body positioned at xy in box2d world coordinates, create it and set the initial values for this box2d body's speed and angle
    BodyDef bd = new BodyDef();
    bd.type = type;
    bd.position.set(box2d.coordPixelsToWorld(new Vec2(x, y)));
    body = box2d.createBody(bd);
    body.setLinearVelocity(new Vec2(0, -50));
    body.setAngularVelocity(random(-5, 5));
    body.setBullet(true);

    if (r == -1) {
      PolygonShape sd = new PolygonShape();          // box2d polygon shape
      FixtureDef fd = new FixtureDef();
      toxiPoly = new Circle(random(5, 20)).toPolygon2D(PApplet.parseInt(random(3, 6)));          // toxiclibs polygon creator (triangle, square, etc)
      Vec2[] vertices = new Vec2[toxiPoly.getNumPoints()];          // place the toxiclibs polygon's vertices into a vec2d array
      for (int i=0; i < vertices.length; i++) {
        Vec2D v = toxiPoly.vertices.get(i);
        vertices[i] = box2d.vectorPixelsToWorld(new Vec2(v.x, v.y));
      }
      sd.set(vertices, vertices.length);          // put the vertices into the box2d shape
      fd.shape = sd;
      fd.restitution = 0.3f;
      fd.friction = 0.1f;
      fd.density = 1.0f;
      body.createFixture(fd);          // create the fixture from the shape (deflect things based on the actual polygon shape)
    } else {
      CircleShape cs = new CircleShape();
      cs.m_radius = box2d.scalarPixelsToWorld(r);          // box2d circle shape of radius r
      FixtureDef fd = new FixtureDef();          // tweak the circle's fixture def a little bit
      fd.shape = cs;
      fd.density = 1;
      fd.friction = 0.01f;
      fd.restitution = 0.5f;
      body.createFixture(fd);          // create the fixture from the shape's fixture def (deflect things based on the actual circle shape)
    }
  }




  public void update() {      // method to loosely move shapes outside a person's polygon (alternatively you could allow or remove shapes inside a person's polygon)

    Vec2 posScreen = box2d.getBodyPixelCoord(body);    // get the screen position from this shape (circle of polygon)
    Vec2D toxiScreen = new Vec2D(posScreen.x, posScreen.y);    // turn it into a toxiclibs Vec2D
    boolean inBody = poly.containsPoint(toxiScreen);    // check if this shape's position is inside the person's polygon
    if (inBody) {        // if a shape is inside the person
      Vec2D closestPoint = toxiScreen;   // find the closest point on the polygon to the current position
      float closestDistance = 9999999;
      for (Vec2D v : poly.vertices) {
        float distance = v.distanceTo(toxiScreen);
        if (distance < closestDistance) {
          closestDistance = distance;
          closestPoint = v;
        }
      }
      Vec2 contourPos = new Vec2(closestPoint.x, closestPoint.y);      // create a box2d position from the closest point on the polygon
      Vec2 posWorld = box2d.coordPixelsToWorld(contourPos);
      float angle = body.getAngle();
      body.setTransform(posWorld, angle);      // set the box2d body's position of this CustomShape to the new position (use the current angle)
    }
  }

  public void display() {  // display the customShape
    Vec2 pos = box2d.getBodyPixelCoord(body);    // get the pixel coordinates of the body
    pushMatrix();
    translate(pos.x, pos.y);    // translate to the position
    noStroke();
    fill(col);      // use the shape's custom color
    if (r == -1) {
      float a = body.getAngle();          // rotate by the body's angle
      rotate(-a); // minus!
      gfx.polygon2D(toxiPoly);
    } else {
      ellipse(0, 0, r*2, r*2);
    }
    popMatrix();
  }


  public boolean done() {    // if the shape moves off-screen, destroy the box2d body (important!) and return true (which will lead to the removal of this CustomShape object)
    Vec2 posScreen = box2d.getBodyPixelCoord(body);
    boolean offscreen = posScreen.y > height;
    if (offscreen) {
      box2d.destroyBody(body);
      return true;
    }
    return false;
  }
}


class Particle {
  // unique id
  float id;
  // current positions
  float x = width/2;
  float y = height/2;
  // previous positions
  float xp, yp;
  // speed
  float sx, sy;
  // distance;
  float d;
  // color
  int col;

  Particle (float id, int particleColor) {
    this.id = id;
    this.col = particleColor;
    sx = random(-3, 3);
    sy = random(-1.5f, 1.5f);
  }

  public void updateAndDisplay() {
    id += 0.01f;
    d = (noise(id, x/globalY, y/globalY) - 0.5f) * globalX;
    x += cos(radians(d)) * sx;
    y += sin(radians(d)) * sy;


    // constrain to boundaries
    if (x < -10) {
      x = kinectWidth - 10;
      xp = kinectWidth - 10;
    } 
    if (x > kinectWidth + 10) {
      x = 10;
      xp = 10;
    } 
    if (y < -10) {
      y = kinectHeight - 10;
      yp = kinectHeight - 10;
    }
    if (y > kinectHeight + 10) {
      y = 10;
      yp = 10;
    }
    // if there is a polygon (more than 0 points)
    //    if (poly.getNumPoints() > 0) {
    //      // if polygon contains this particle
    //      if (!poly.vertices.contains(new Vec2D(x, y))) {
    ////        while (!poly.vertices.contains(new Vec2D(x, y))) {
    ////          x = random(kinectWidth);
    ////          y = random(kinectHeight);
    ////        }
    //        xp = x;
    //        yp = y;
    //      }
    //    }
    stroke(col, 75);
    strokeWeight(0.5f);
    line(xp, yp, x, y);
    xp = x;
    yp = y;
  }
}

class PolygonBlob extends Polygon2D {

  // to hold the box2d body
  Body body;

  public void createPolygon() {    // an ArrayList of the ArrayLists of PVectors, which is basically the person's contours (almost but not completely in a polygon-correct order)
    ArrayList<ArrayList<PVector>> contours = new ArrayList<ArrayList<PVector>>();
    int selectedContour = 0;
    int selectedPoint = 0;

    for (int n=0; n<theBlobDetection.getBlobNb (); n++) {        // create contours from blobs
      Blob b = theBlobDetection.getBlob(n);
      if (b != null && b.getEdgeNb() > 100) {
        ArrayList<PVector> contour = new ArrayList<PVector>();
        for (int m=0; m<b.getEdgeNb (); m++) {
          EdgeVertex eA = b.getEdgeVertexA(m);
          EdgeVertex eB = b.getEdgeVertexB(m);
          if (eA != null && eB != null) {
            EdgeVertex fn = b.getEdgeVertexA((m+1) % b.getEdgeNb());
            EdgeVertex fp = b.getEdgeVertexA((max(0, m-1)));
            float dn = dist(eA.x*kinectWidth, eA.y*kinectHeight, fn.x*kinectWidth, fn.y*kinectHeight);
            float dp = dist(eA.x*kinectWidth, eA.y*kinectHeight, fp.x*kinectWidth, fp.y*kinectHeight);
            if (dn > 15 || dp > 15) {
              if (contour.size() > 0) {
                contour.add(new PVector(eB.x*kinectWidth, eB.y*kinectHeight));
                contours.add(contour);
                contour = new ArrayList<PVector>();
              } else {
                contour.add(new PVector(eA.x*kinectWidth, eA.y*kinectHeight));
              }
            } else {
              contour.add(new PVector(eA.x*kinectWidth, eA.y*kinectHeight));
            }
          }
        }
      }
    }

    while (contours.size () > 0) {        // find next contour
      float distance = 999999999;
      if (getNumPoints() > 0) {
        Vec2D vecLastPoint = vertices.get(getNumPoints()-1);
        PVector lastPoint = new PVector(vecLastPoint.x, vecLastPoint.y);
        for (int i=0; i<contours.size (); i++) {
          ArrayList<PVector> c = contours.get(i);
          PVector fp = c.get(0);
          PVector lp = c.get(c.size()-1);
          if (fp.dist(lastPoint) < distance) { 
            distance = fp.dist(lastPoint); 
            selectedContour = i; 
            selectedPoint = 0;
          }
          if (lp.dist(lastPoint) < distance) { 
            distance = lp.dist(lastPoint); 
            selectedContour = i; 
            selectedPoint = 1;
          }
        }
      } else {
        PVector closestPoint = new PVector(width, height);
        for (int i=0; i<contours.size (); i++) {
          ArrayList<PVector> c = contours.get(i);
          PVector fp = c.get(0);
          PVector lp = c.get(c.size()-1);
          if (fp.y > kinectHeight-5 && fp.x < closestPoint.x) { 
            closestPoint = fp; 
            selectedContour = i; 
            selectedPoint = 0;
          }
          if (lp.y > kinectHeight-5 && lp.x < closestPoint.y) { 
            closestPoint = lp; 
            selectedContour = i; 
            selectedPoint = 1;
          }
        }
      }
      ArrayList<PVector> contour = contours.get(selectedContour);          // add contour to polygon
      if (selectedPoint > 0) {
        Collections.reverse(contour);
      }
      for (PVector p : contour) {
        add(new Vec2D(p.x, p.y));
      }
      contours.remove(selectedContour);
    }
  }

  public void createBody() {    // creates a shape-deflecting physics chain in the box2d world from this polygon
    BodyDef bd = new BodyDef();        // for stability the body is always created (and later destroyed)
    body = box2d.createBody(bd);
    // if there are more than 0 points (aka a person on screen)...
    if (getNumPoints() > 0) {
      // create a vec2d array of vertices in box2d world coordinates from this polygon
      Vec2[] verts = new Vec2[getNumPoints()/2];
      for (int i = 0; i < getNumPoints ()/2; i++) {
        Vec2D v = vertices.get(i * 2);
        verts[i] = box2d.coordPixelsToWorld(v.x, v.y);
      }
      ChainShape chain = new ChainShape();          // create a chain from the array of vertices
      chain.createChain(verts, verts.length);          // create fixture in body from the chain (this makes it actually deflect other shapes)
      body.createFixture(chain, 1);
    }
  }

  public void destroyBody() {      // destroy the box2d body (important!)
    box2d.destroyBody(body);
  }
}

  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "--full-screen", "--bgcolor=#666666", "--stop-color=#cccccc", "motion_in_music" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
