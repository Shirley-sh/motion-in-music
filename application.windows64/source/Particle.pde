
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
  color col;

  Particle (float id, color particleColor) {
    this.id = id;
    this.col = particleColor;
    sx = random(-3, 3);
    sy = random(-1.5, 1.5);
  }

  void updateAndDisplay() {
    id += 0.01;
    d = (noise(id, x/globalY, y/globalY) - 0.5) * globalX;
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
    strokeWeight(0.5);
    line(xp, yp, x, y);
    xp = x;
    yp = y;
  }
}

