import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import peasy.*; 
import processing.serial.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class processing_3D_visor extends PApplet {

  //para poder rotacionar a visão do observador no R3
PeasyCam cam;

/****************/

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

public void setup (){
  surface.setTitle("draw");
  

  myPort = new Serial(this, Serial.list()[0], 115200);

  lights();
  cam = new PeasyCam(this, width);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(5000);

  colorMode(HSB);
}

public void draw (){
  background (0xffe1e1e1);

  if(myPort.available() > 10){
    data = myPort.readStringUntil(';');
    data = data.substring(0, data.length()-1);

    index1 = data.indexOf(",");
    one = data.substring(0, index1);
    dado = data.substring(index1+1, data.length());

    tipo = PApplet.parseInt(one);
    DADO = PApplet.parseFloat(dado);

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

public void draw_object (){
  /*****draw_box****/
  fill(0xffbbb5b5);
  stroke(0xff555353);
  box(10 * escala, 10 * escala, 5 * escala);
  /*****************/

  /*****draw_arc****/
  fill(0xffffb691);
  noStroke();

  arc(0, 0, 2*dist*escala, 2*dist*escala, radians(-15), radians(15), OPEN);
  beginShape();
    vertex(0, 0);
    vertex((dist*escala)*cos(radians(15)), -(dist*escala)*sin(radians(15)));
    vertex((dist*escala)*cos(radians(15)), (dist*escala)*sin(radians(15)));
  endShape(OPEN);

  fill(0xff050606);
  textSize(20);
  textAlign(CENTER, CENTER);
  pushMatrix();
    rotate(PI/2);
    text(PApplet.parseInt(dist) + "cm", 0, -(dist*escala + 20));
  popMatrix();
  /*****************/
}
  public void settings() {  size(1280, 720, P3D); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "processing_3D_visor" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
