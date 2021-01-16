float manhattan_distance(Vertex a, Vertex b) {
  return max(abs(a.x - b.x), abs(a.y - b.y));
}

float euclidean_distance(Vertex a, Vertex b) {
  return sqrt(sq(a.x - b.x) + sq(a.y - b.y));
}

float angle(Vertex a, Vertex b) {
  return atan2(a.y - b.y, a.x - b.x);
}

class Vertex {
  float x;
  float y;
  float fx = 0.0;
  float fy = 0.0;
  boolean station = true;
  
  Vertex(float x, float y, boolean station) {
    this.x = x;
    this.y = y;
    this.station = station;
  }
  
  Vertex(float x, float y) {
   this.x = x;
    this.y = y;
    this.station = false;
  }
  
  void addForce(float fx, float fy) {
    this.fx += fx;
    this.fy += fy;
  }
  
  void applyForce(float scale) {
    this.x += scale * this.fx;
    this.y += scale * fy;
    this.fx = 0.0;
    this.fy = 0.0;
  }
  
  void applyForce() {
    applyForce(1.0);
  }
}

class Edge {
  Vertex a;
  Vertex b;
  Edge(Vertex a, Vertex b) {
    this.a = a;
    this.b = b;
  }
  
  void optimize(float weight) {
    float phi = angle(a, b);
    float moment = weight * (round(4 * phi / PI) - 4 * phi / PI);
    
    a.addForce(-moment * sin(phi), moment * cos(phi));
    b.addForce(moment * sin(phi), -moment * cos(phi));
  }
}

int octilinear_section(float phi) {
  phi = phi % (2 * PI);
  float segment = PI / 8;
  if(15 * segment <= phi && phi < segment) {
    return 0;
  } else if(segment <= phi && phi < 3 * segment) {
    return 1;
  }else if(3 * segment <= phi && phi < 5 * segment) {
    return 2;
  }else if(5 * segment <= phi && phi < 7 * segment) {
    return 3;
  }else if(7 * segment <= phi && phi < 9 * segment) {
    return 4;
  }else if(9 * segment <= phi && phi < 11 * segment) {
    return 5;
  }else if(11 * segment <= phi && phi < 13 * segment) {
    return 6;
  }else if(13 * segment <= phi && phi < 15 * segment) {
    return 7;
  }
  return 0;
}

float manhattan_hookes(Vertex a, Vertex b, float k, float x0) {
  float x = manhattan_distance(a, b);
  float phi = angle(a, b);
  int section = octilinear_section(phi);
  
  if(x < x0) {
    return k*(x-x0);
  } else if (x > sqrt(2) * x0 && (section == 1 || section == 3 || section == 5 || section == 7)) {
    return k*(x-x0*(sqrt(2)-1));
  } 
  return 0.0;
}
