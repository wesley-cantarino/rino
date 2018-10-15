import peasy.*;  //para poder rotacionar a visão do observador no R3
PeasyCam cam;

/****************/
import processing.serial.*;
Serial myPort;
String one ="";
String dado ="";
String data="";
int index1=0;

int tipo;
float DADO;
/****************/

float dist = 0, X = 0, Y = 0, Z = 0;
int escala = 10;  /* escala = tamanho_desenho/tamanho_real */

void setup (){
  surface.setTitle("draw");
  size(1280, 720, P3D);

  myPort = new Serial(this, Serial.list()[0], 115200);

  lights();
  cam = new PeasyCam(this, width);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(5000);

  colorMode(HSB);
}

void draw (){
  background (#e1e1e1);

  if(myPort.available() > 10){
    data = myPort.readStringUntil(';');
    data = data.substring(0, data.length()-1);

    index1 = data.indexOf(",");
    one = data.substring(0, index1);
    dado = data.substring(index1+1, data.length());

    tipo = int(one);
    DADO = float(dado);

    if(tipo == 1)
      dist = DADO;
    else if(tipo == 2)
      X = DADO;
    else if(tipo == 3)
      Y = DADO;
    else if(tipo == 4)
      Z = DADO;

    //println(dist + " _ " + X + " _ " + Y + " _ " + Z);
  }

  pushMatrix();
    rotateY(radians(map(X, -16000, 16000, -90, 90)));
    rotateX(radians(map(Y, -16000, 16000, -90, 90)));

    draw_object();
  popMatrix();
}

void draw_object (){
  /*****draw_box****/
  fill(#bbb5b5);
  stroke(#555353);
  box(10 * escala, 10 * escala, 5 * escala);
  /*****************/

  /*****draw_arc****/
  fill(#ffb691);
  noStroke();

  arc(0, 0, 2*dist*escala, 2*dist*escala, radians(-15), radians(15), OPEN);
  beginShape();
    vertex(0, 0);
    vertex((dist*escala)*cos(radians(15)), -(dist*escala)*sin(radians(15)));
    vertex((dist*escala)*cos(radians(15)), (dist*escala)*sin(radians(15)));
  endShape(OPEN);

  fill(#050606);
  textSize(20);
  textAlign(CENTER, CENTER);
  pushMatrix();
    rotate(PI/2);
    text(int(dist) + "cm", 0, -(dist*escala + 20));
  popMatrix();
  /*****************/
}
