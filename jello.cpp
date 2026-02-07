/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

  Your name:
  <write your name here>

*/

#include "jello.h"
#include "showCube.h"
#include "input.h"
#include "physics.h"

#include <sys/time.h>

// camera parameters
double Theta = pi / 6;
double Phi = pi / 6;
double R = 6;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;

// number of images saved to disk so far
int sprite=0;

// these variables control what is displayed on screen
int shear=0, bend=0, structural=1, pause=0, viewingMode=0, saveScreenToFile=0;

// texture mode: 0 = off (default material), 1 = Fresnel effect
int textureMode = 0;

struct world jello;

int windowWidth, windowHeight;

// FPS tracking
double currentFPS = 0.0;
struct timeval lastTime;
int frameCount = 0;

double getTimeInSeconds()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + tv.tv_usec / 1000000.0;
}

void renderFPS()
{
  // Switch to 2D orthographic projection
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, windowWidth, 0, windowHeight);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  // Render FPS text in top-left corner
  glColor3f(1.0, 1.0, 1.0);
  glRasterPos2i(10, windowHeight - 20);

  char fpsText[32];
  sprintf(fpsText, "FPS: %.1f", currentFPS);

  for (char *c = fpsText; *c != '\0'; c++)
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);

  // Restore matrices
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void myinit()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90.0,1.0,0.01,1000.0);

  // set background color to sky blue
  glClearColor(0.53, 0.81, 0.92, 0.0);

  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  return; 
}

void reshape(int w, int h) 
{
  // Prevent a divide by zero, when h is zero.
  // You can't make a window of zero height.
  if(h == 0)
    h = 1;

  glViewport(0, 0, w, h);

  // Reset the coordinate system before modifying
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // Set the perspective
  double aspectRatio = 1.0 * w / h;
  gluPerspective(60.0f, aspectRatio, 0.01f, 1000.0f);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); 

  windowWidth = w;
  windowHeight = h;

  glutPostRedisplay();
}

void display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // camera parameters are Phi, Theta, R
  gluLookAt(R * cos(Phi) * cos (Theta), R * sin(Phi) * cos (Theta), R * sin (Theta),
	        0.0,0.0,0.0, 0.0,0.0,1.0);


  /* Lighting */
  /* Personalized cinematic three-point lighting setup for red/orange jello */

  // Global ambient light - warm sunlight
  GLfloat aGa[] = { 0.25, 0.22, 0.12, 1.0 };

  // Light 0: Key light - warm orange/gold (main illumination)
  GLfloat lKa0[] = { 0.05, 0.03, 0.01, 1.0 };
  GLfloat lKd0[] = { 1.0, 0.85, 0.6, 1.0 };
  GLfloat lKs0[] = { 1.0, 0.95, 0.8, 1.0 };
  GLfloat lP0[] = { 3.0, -2.0, 3.0, 1.0 };

  // Light 1: Fill light - cool blue (softens shadows)
  GLfloat lKa1[] = { 0.01, 0.02, 0.03, 1.0 };
  GLfloat lKd1[] = { 0.3, 0.4, 0.6, 1.0 };
  GLfloat lKs1[] = { 0.2, 0.3, 0.5, 1.0 };
  GLfloat lP1[] = { -3.0, -1.0, 1.0, 1.0 };

  // Light 2: Rim light - pale yellow (edge definition)
  GLfloat lKa2[] = { 0.02, 0.02, 0.01, 1.0 };
  GLfloat lKd2[] = { 0.8, 0.8, 0.6, 1.0 };
  GLfloat lKs2[] = { 0.9, 0.9, 0.7, 1.0 };
  GLfloat lP2[] = { 0.0, 3.0, 2.0, 1.0 };

  // Light 3: Under light - subtle purple (depth)
  GLfloat lKa3[] = { 0.01, 0.0, 0.02, 1.0 };
  GLfloat lKd3[] = { 0.2, 0.1, 0.3, 1.0 };
  GLfloat lKs3[] = { 0.15, 0.1, 0.2, 1.0 };
  GLfloat lP3[] = { 0.0, 0.0, -3.0, 1.0 };

  // Lights 4-7: Disabled (black)
  GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd4[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKs4[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lP4[] = { 0.0, 0.0, 0.0, 1.0 };

  GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd5[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKs5[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lP5[] = { 0.0, 0.0, 0.0, 1.0 };

  GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd6[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKs6[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lP6[] = { 0.0, 0.0, 0.0, 1.0 };

  GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd7[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKs7[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lP7[] = { 0.0, 0.0, 0.0, 1.0 };

  // Jelly material - red translucent appearance with sunlight highlights
  GLfloat mKa[] = { 0.2, 0.02, 0.02, 1.0 };   // Ambient: deep red tint
  GLfloat mKd[] = { 0.9, 0.1, 0.1, 1.0 };     // Diffuse: bright red
  GLfloat mKs[] = { 1.0, 0.95, 0.4, 1.0 };    // Specular: yellow sunlight highlights
  GLfloat mKe[] = { 0.05, 0.0, 0.0, 1.0 };    // Emissive: subtle red glow

  /* set up lighting */
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, aGa);
  glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

  // set up cube color
  glMaterialfv(GL_FRONT, GL_AMBIENT, mKa);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mKd);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mKs);
  glMaterialfv(GL_FRONT, GL_EMISSION, mKe);
  glMaterialf(GL_FRONT, GL_SHININESS, 80);  // Slightly less shiny for jello look

  // macro to set up light i
  #define LIGHTSETUP(i)\
  glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
  glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##i);\
  glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##i);\
  glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##i);\
  glEnable(GL_LIGHT##i)

  LIGHTSETUP (0);
  LIGHTSETUP (1);
  LIGHTSETUP (2);
  LIGHTSETUP (3);
  LIGHTSETUP (4);
  LIGHTSETUP (5);
  LIGHTSETUP (6);
  LIGHTSETUP (7);

  // enable lighting
  glEnable(GL_LIGHTING);    
  glEnable(GL_DEPTH_TEST);

  // show the cube
  showCube(&jello);

  glDisable(GL_LIGHTING);

  // show the bounding box
  showBoundingBox();

  // show the inclined plane (if present)
  showInclinedPlane(&jello);

  // show FPS counter
  renderFPS();

  glutSwapBuffers();
}

void doIdle()
{
  // FPS calculation at the start of idle handler
  static double lastFrameTime = 0.0;
  static int fpsFrameCount = 0;
  static double fpsAccumTime = 0.0;

  double currentTime = getTimeInSeconds();
  if (lastFrameTime > 0.0)
  {
    double elapsed = currentTime - lastFrameTime;
    fpsAccumTime += elapsed;
    fpsFrameCount++;

    // Update FPS every 0.5 seconds for stable display
    if (fpsAccumTime >= 0.5)
    {
      currentFPS = fpsFrameCount / fpsAccumTime;
      fpsFrameCount = 0;
      fpsAccumTime = 0.0;
    }
  }
  lastFrameTime = currentTime;

  char s[20]="picxxxx.ppm";
  int i;

  // save screen to file
  s[3] = 48 + (sprite / 1000);
  s[4] = 48 + (sprite % 1000) / 100;
  s[5] = 48 + (sprite % 100 ) / 10;
  s[6] = 48 + sprite % 10;

  if (saveScreenToFile==1)
  {
    saveScreenshot(windowWidth, windowHeight, s);
    saveScreenToFile=0; // save only once, change this if you want continuos image generation (i.e. animation)
    sprite++;
  }

  if (sprite >= 300) // allow only 300 snapshots
  {
    exit(0);	
  }

  if (pause == 0)
  {
    // Perform n steps of simulation per frame
    for (int step = 0; step < jello.n; step++)
    {
      if (strcmp(jello.integrator, "RK4") == 0)
        RK4(&jello);
      else if (strcmp(jello.integrator, "Euler") == 0)
        Euler(&jello);
    }

    // Blow-up detection: check if any point has gone too far
    for (int i = 0; i <= 7; i++)
      for (int j = 0; j <= 7; j++)
        for (int k = 0; k <= 7; k++)
        {
          if (fabs(jello.p[i][j][k].x) > 10.0 ||
              fabs(jello.p[i][j][k].y) > 10.0 ||
              fabs(jello.p[i][j][k].z) > 10.0)
          {
            printf("Simulation blew up! Point [%d][%d][%d] at (%.2f, %.2f, %.2f)\n",
                   i, j, k, jello.p[i][j][k].x, jello.p[i][j][k].y, jello.p[i][j][k].z);
            exit(1);
          }
        }
  }

  glutPostRedisplay();
}

int main (int argc, char ** argv)
{
  if (argc<2)
  {  
    printf ("Oops! You didn't say the jello world file!\n");
    printf ("Usage: %s [worldfile]\n", argv[0]);
    exit(0);
  }

  readWorld(argv[1],&jello);

  glutInit(&argc,argv);
  
  /* double buffered window, use depth testing, 640x480 */
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  
  windowWidth = 640;
  windowHeight = 480;
  glutInitWindowSize (windowWidth, windowHeight);
  glutInitWindowPosition (0,0);
  glutCreateWindow ("Jello cube");

  /* tells glut to use a particular display function to redraw */
  glutDisplayFunc(display);

  /* replace with any animate code */
  glutIdleFunc(doIdle);

  /* callback for mouse drags */
  glutMotionFunc(mouseMotionDrag);

  /* callback for window size changes */
  glutReshapeFunc(reshape);

  /* callback for mouse movement */
  glutPassiveMotionFunc(mouseMotion);

  /* callback for mouse button changes */
  glutMouseFunc(mouseButton);

  /* register for keyboard events */
  glutKeyboardFunc(keyboardFunc);

  /* do initialization */
  myinit();

  /* forever sink in the black hole */
  glutMainLoop();

  return(0);
}

