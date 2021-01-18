final int N = 10;

ArrayList<Vertex> vertices = new ArrayList<Vertex>();
ArrayList<Vertex> positions = new ArrayList<Vertex>();
ArrayList<Edge> edges = new ArrayList<Edge>();

float minimum_distance;
float dt = .01;
float iteration = 0;

float accuracy_weight = 1.0;
float grid_weight = 100.0;
float distance_weight = 10.0;
float octilinearity_weight = 1000.0;
void setup() {
  size(600, 600);
  float off = 0.0;
  float x = 0.0 + random(width/(N+1));
  float y = height/2 + height/(N+1) - random(height/(N+1));
  for(int i=0; i< N; i++) {
    positions.add(new Vertex(x, y));
    vertices.add(new Vertex(x, y));
    if(i > 0) {
      edges.add(new Edge(vertices.get(i-1), vertices.get(i)));
    }
    float dx=0, dy=0;
    while(dx < width/(2*N)) {
      float phi = (0.5 - noise(off)) * 2*PI;
      float r = random(width/(1.5*N), 1.5*width/N);
      off += 1;
      dx = r * cos(phi); //width/N + (0.5 - noise(off)) * width / N;
      dy = r * sin(phi); //(0.5 - noise(off + 360)) * 2 * height / N;
    }
    x += dx;
    y += dy;
  }
  minimum_distance = width/N;
  noLoop();
}

boolean running = false;

void mouseClicked() {
  if(running) {
    noLoop();
  } else {
    loop();
  }
  running = !running;
}

void draw() {
  iteration++;
  background(255);
  fill(0);
  stroke(0);
  // Draw the grid
  for(int i = 0; i < N; i++) {
    for(int j = 0; j < N; j++) {
      ellipse(i*width/(N), j*height/(N), 3, 3);
    }
  }
  // Draw the lines
  for(int i=0; i<vertices.size()-1; i++) {
    // Draw the refined path
    stroke(0);
    line(vertices.get(i).x, vertices.get(i).y, vertices.get(i+1).x, vertices.get(i+1).y);
    // Draw the original path
    stroke(128);
    line(positions.get(i).x, positions.get(i).y, positions.get(i+1).x, positions.get(i+1).y);
  }
  
  fill(255, 0, 0);
  stroke(255, 0, 0);
  
  for(int i=0; i<edges.size(); i++) {
    edges.get(i).optimize(octilinearity_weight);
  }
  
  // Calculate and apply forces
  float dx, dy, f, phi;
  for(int i=0; i<vertices.size(); i++) {
    // Draw the vertex 
    Vertex c = vertices.get(i);
    ellipse(c.x, c.y, 10, 10);
    
    // Snap the vertices to a grid
    //dx = round(c.x / (width/N)) - c.x / (width/N);
    //dy = round(c.y / (height/N)) - c.y / (height/N);
    //
    //f = grid_weight / (1+(sq(dx) + sq(dy))); // ~ 1/r^2
    //phi = atan2(dy, dx);
    //c.addForce(f * cos(phi), f * sin(phi));
    
    
    // Pull of original positions
    dx = positions.get(i).x - c.x;
    dy = positions.get(i).y - c.y;
    f = accuracy_weight * euclidean_distance(positions.get(i), c); //sqrt(sq(dx) + sq(dy));
    phi = atan2(dy, dx);
    c.addForce(f * cos(phi), f * sin(phi));
    
    
    if(i > 0) {
      // Apply a force based on the distance between this and the previous vertex
      Vertex w = vertices.get(i-1);
      dx = w.x - c.x;
      dy = w.y - c.y;
      f =  manhattan_hookes(c, w, distance_weight, minimum_distance);
      phi = atan2(dy, dx);
      c.addForce(f * cos(phi), f * sin(phi));
   }
    
   if(i < vertices.size()-1) {
      // Apply a force based on the distance between this and the next vertex
      Vertex e = vertices.get(i+1);
      dx = e.x - c.x;
      dy = e.y - c.y;
      f = manhattan_hookes(c, e, distance_weight, minimum_distance); //distance_weight * (distance(e, c) - minimum_distance);
      phi = atan2(dy, dx);
      c.addForce(f * cos(phi), f * sin(phi)); 
    }
    c.applyForce(dt);
  }
  
  text("iteration=" + str(iteration), 10, height - 10);
}
