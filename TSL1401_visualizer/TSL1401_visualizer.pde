import processing.serial.*;

final int LINELEN = 128*3 + 3;
final int SCROLLHT = 256;
final int NPIXELS = 128;
PImage img;
Serial duino;
boolean Synced = false;
byte[] byteArray = new byte[LINELEN + 1];
int[] intArray = new int[LINELEN + 1];
byte[] brightness = new byte[3];

int noLED = -1;

double calibFactor = 31.38912669 / 2;

PFont f;

void setup ()
{
  println ("<START>");
  println (Serial.list());
  println ("<END>");
  
  int serialNo = Serial.list().length - 1;

  // Open serial port to Arduino at 115200 baud
  duino = new Serial (this, Serial.list()[serialNo], 115200);
  
  while (duino.available () == 0)
  {
    print("X");
  }
  
  delay(5000);
  duino.write('v');
  
    while (duino.available () == 0)
  {
    print("Y");
  }
  
  // Window is same width as sensor, but long enough to scroll
  //size (LINELEN, SCROLLHT + 50);
  size (387, 316); 

  // Image is same size as window
  img = createImage (LINELEN, SCROLLHT, RGB);

  // Initialise image to a shade of blue
  img.loadPixels ();

  for (int i = 0; i < img.pixels.length; i++) {
    img.pixels[i] = color (0, 90, 102);
  }

  img.updatePixels ();

  // Choose image update rate
  frameRate (5);
  
  f = loadFont("consolas16.vlw");
  textFont(f,16); // Step 4: Specify font to be used
}

void exit()
{
  duino.write('f');
  super.exit();
}
 

void draw ()
{
  int i;
  int ch;
  int nbufs;
  int b;
  int maxi;
  int maxpx;

  background(0);

print("A");

  // Synchronise
  if (Synced) {
    nbufs = duino.available () / (LINELEN + 1);
  } else {
    do {
      while (duino.available () == 0)
      {
        print("B");
      }
        ;

      ch = duino.read ();
    } 
    while (ch != 0);
    nbufs = 0;
    Synced = true;
  }

  // Load the image pixels in preparation for next row(s)
  img.loadPixels ();

  for (b = 0; b < nbufs; b++) {
    // Scroll the old image data down the window
    for (i = img.pixels.length - LINELEN - 1; i >= 0; i--) {
      img.pixels[i + LINELEN] = img.pixels[i];
    }

    // Read 128 pixels from image sensor, via Arduino
    duino.readBytes (byteArray);

    // Check we're still in sync
    if (byteArray[LINELEN] != 0) {
      print ("UNSYNC ");
      Synced = false;
    }
    
    print(byteArray[0]);

    maxi = 0;
    maxpx = 0;

    // Transfer incoming pixels to image
    for (i = 0; i < LINELEN; i++) {
      ch = byteArray[i];
      intArray[i] = int(byteArray[i]) * 4;

      if (ch < 0)
        ch += 256;

      if (ch > maxpx) {   // Look for brightest pixel
        maxi = i;
        maxpx = ch;
      }

      img.pixels[i] = color (ch, ch, ch);
    }
    img.pixels[NPIXELS] = color(0, 0, 0);
    img.pixels[NPIXELS*2] = color(0, 0, 0);

    brightness[0] = byteArray[LINELEN - 3];
    brightness[1] = byteArray[LINELEN - 2];
    brightness[2] = byteArray[LINELEN - 1];

    //println((brightness[0]&0xFF)+", "+(brightness[1]&0xFF)+", "+(brightness[2]&0xFF));   

    processImage(0);
    processImage(1);
    processImage(2);
  }

  // We're done updating the image, so re-display it
  img.updatePixels ();
  image (img, 0, 0);
  
  if(noLED == 0) { fill(255, 0, 0); } else { fill(255); }
  text("(0) "+(brightness[0]&0xFF), 10, SCROLLHT + 20);
  if(noLED == 1) { fill(255, 0, 0); } else { fill(255); }
  text("(1) "+(brightness[1]&0xFF), 128 + 10, SCROLLHT + 20);
  if(noLED == 2) { fill(255, 0, 0); } else { fill(255); }  
  text("(2) "+(brightness[2]&0xFF), 128*2 + 10, SCROLLHT + 20);
  
  fill(255);
  text("* (j) decrease (k)increase (c) save", 10, SCROLLHT + 40);
}

void keyPressed()
{
  duino.write(key);
  if(key == '0')
    noLED = 0;
  if(key == '1')
    noLED = 1;
  if(key == '2')
    noLED = 2;
}


double widthsubpixellp = 2;

void processImage(int led)
{
  double x0, x1, x2, x3;
  double minstep, maxstep;
  int minsteploc, maxsteploc;
  int ct;
  int ad_image;
  int offset = led * NPIXELS;

  double a1, b1, c1, a2, b2, c2, m1, m2; //sub pixel quadratic interpolation variables
  double widthsubpixel; 

  int filWidth = 0;

  int startPos = 0, endPos = 0;
  
  int thresh = 600;
  boolean seenedge;

  seenedge = false;
  for (int i=3; i<NPIXELS; i++)
  {
    if (intArray[offset+i-2] < thresh && intArray[offset+i-1] < thresh && intArray[offset+i] < thresh)
      seenedge = true;
    
    //if (intArray[offset+i-3] > thresh && intArray[offset+i-2] > thresh && intArray[offset+i-1] > thresh && intArray[offset+i] > thresh)
    if (seenedge && intArray[offset+i-2] > thresh && intArray[offset+i-1] > thresh && intArray[offset+i] > thresh)
    {
      startPos = i;
      break;
    }
  }
  
  seenedge = false;
  for (int i=NPIXELS-3; i>=startPos;i--)
  {
    if (intArray[offset+i+2] < thresh && intArray[offset+i+1] < thresh && intArray[offset+i] < thresh)
      seenedge = true;
    
    //if (intArray[offset+i+3] > thresh && intArray[offset+i+2] > thresh && intArray[offset+i+1] > thresh && intArray[offset+i] > thresh)
    if (seenedge && intArray[offset+i+2] > thresh && intArray[offset+i+1] > thresh && intArray[offset+i] > thresh)
    {
      endPos = i;
      break;
    }
  }
  
  minstep = maxstep = 0;
  minsteploc = maxsteploc = 255;
  //clear the sub-pixel buffers
  x0 = x1 = x2 = x3 = 0;
  a1 = b1 = c1 = a2 = b2 = c2 = m1 = m2 = 0;
  widthsubpixel = 0;
  ct = startPos-2;  //index to count samples  need to load buffer for 2 steps to subtract x2-x1

  for (int i=startPos; i<endPos; i++)
  {
    x3=x2;  
    x2=x1;
    x1=x0;
    x0=intArray[offset+i];
    ct = ct + 1;

    if (ct > startPos+1 && ct < endPos-2)
    {
      if (x1+10<x2)
      {
        if (minstep<x2-x1)
        {
          minstep=x2-x1;
          minsteploc=ct;
          c1=x1-x0;
          b1=x2-x1;
          a1=x3-x2;
        }
      } else if(x1 > x2+10)
      {
        if (maxstep<x1-x2)
        {
          maxstep=x1-x2;
          maxsteploc=ct;
          c2=x1-x0;
          b2=x2-x1;
          a2=x3-x2;
        }
      }
    }
  }

  img.pixels[offset+minsteploc] = color (0, 255, 0); // Mark brightest in green
  img.pixels[offset+maxsteploc] = color (255, 0, 0); // Mark brightest in green

  if (minstep>16 && maxstep>16)  //check for significant threshold
  {
    filWidth=maxsteploc-minsteploc;
  } else
    filWidth=0;
  if (filWidth>103)  //check for width overflow or out of range (15.7pixels per mm, 65535/635=103)
    filWidth=0;

  m1=((a1-c1) / (a1+c1-(b1*2)))/2;
  m2=((a2-c2) / (a2+c2-(b2*2)))/2;

  if (filWidth>15)    //check for a measurement > 1mm  otherwise treat as noise
    widthsubpixel=(double)filWidth+m2-m1; 
  else
    widthsubpixel=0;
    
  //widthsubpixellp = ((widthsubpixel - widthsubpixellp) * 0.1) + widthsubpixellp; 
  widthsubpixellp = widthsubpixellp * 0.9 + widthsubpixel * 0.1;

  if (widthsubpixel > 0)
  {
    double mmWidth = widthsubpixellp / calibFactor;

    //println(widthsubpixellp + "\t" + mmWidth);
  }
}
