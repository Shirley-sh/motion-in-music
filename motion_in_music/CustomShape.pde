// Class for falling objects
import java.util.List;
import java.util.Arrays;

class CustomShape {
  Body body;  // to hold the box2d body
  Polygon2D toxiPoly;  // to hold the Toxiclibs polygon shape
  color col;  // custom color for each shape
  float r;  // radius (also used to distinguish between circles and polygons in this combi-class
  CustomShape(float x, float y, float r, BodyType type, color c) {
    this.r = r;
    makeBody(x, y, type, c);        // create a body (polygon or circle based on the r)
    col = c;    // get a random color
  }

  void makeBody(float x, float y, BodyType type, color c) {    // define a dynamic body positioned at xy in box2d world coordinates, create it and set the initial values for this box2d body's speed and angle
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
      toxiPoly = new Circle(random(5, 20)).toPolygon2D(int(random(3, 6)));          // toxiclibs polygon creator (triangle, square, etc)
      Vec2[] vertices = new Vec2[toxiPoly.getNumPoints()];          // place the toxiclibs polygon's vertices into a vec2d array
      for (int i=0; i < vertices.length; i++) {
        Vec2D v = toxiPoly.vertices.get(i);
        vertices[i] = box2d.vectorPixelsToWorld(new Vec2(v.x, v.y));
      }
      sd.set(vertices, vertices.length);          // put the vertices into the box2d shape
      fd.shape = sd;
      fd.restitution = 0.3;
      fd.friction = 0.1;
      fd.density = 1.0;
      body.createFixture(fd);          // create the fixture from the shape (deflect things based on the actual polygon shape)
    } else {
      CircleShape cs = new CircleShape();
      cs.m_radius = box2d.scalarPixelsToWorld(r);          // box2d circle shape of radius r
      FixtureDef fd = new FixtureDef();          // tweak the circle's fixture def a little bit
      fd.shape = cs;
      fd.density = 1;
      fd.friction = 0.01;
      fd.restitution = 0.5;
      body.createFixture(fd);          // create the fixture from the shape's fixture def (deflect things based on the actual circle shape)
    }
  }




  void update() {      // method to loosely move shapes outside a person's polygon (alternatively you could allow or remove shapes inside a person's polygon)

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

  void display() {  // display the customShape
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


  boolean done() {    // if the shape moves off-screen, destroy the box2d body (important!) and return true (which will lead to the removal of this CustomShape object)
    Vec2 posScreen = box2d.getBodyPixelCoord(body);
    boolean offscreen = posScreen.y > height;
    if (offscreen) {
      box2d.destroyBody(body);
      return true;
    }
    return false;
  }
}

