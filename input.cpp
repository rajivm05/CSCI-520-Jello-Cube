/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "input.h"

/* Write a screenshot, in the PPM format, to the specified filename, in PPM format */
void saveScreenshot(int windowWidth, int windowHeight, char *filename)
{
  if (filename == NULL)
    return;

  // Allocate a picture buffer 
  Pic * in = pic_alloc(windowWidth, windowHeight, 3, NULL);

  printf("File to save to: %s\n", filename);

  for (int i=windowHeight-1; i>=0; i--) 
  {
    glReadPixels(0, windowHeight-i-1, windowWidth, 1, GL_RGB, GL_UNSIGNED_BYTE,
      &in->pix[i*in->nx*in->bpp]);
  }

  if (ppm_write(filename, in))
    printf("File saved Successfully\n");
  else
    printf("Error in Saving\n");

  pic_free(in);
}

/* converts mouse drags into information about rotation/translation/scaling */
void mouseMotionDrag(int x, int y)
{
  int vMouseDelta[2] = {x-g_vMousePos[0], y-g_vMousePos[1]};

  if (g_iRightMouseButton) // handle camera rotations
  {
    Phi += vMouseDelta[0] * 0.01;
    Theta += vMouseDelta[1] * 0.01;
    
    if (Phi>2*pi)
      Phi -= 2*pi;
    
    if (Phi<0)
      Phi += 2*pi;
    
    if (Theta>pi / 2 - 0.01) // dont let the point enter the north pole
      Theta = pi / 2 - 0.01;
    
    if (Theta<- pi / 2 + 0.01)
      Theta = -pi / 2 + 0.01;
    
    g_vMousePos[0] = x;
    g_vMousePos[1] = y;
  }
}

void mouseMotion (int x, int y)
{
  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
}

// Compute the axis-aligned bounding box of the jello cube
void getJelloBoundingBox(double *minX, double *maxX, double *minY, double *maxY, double *minZ, double *maxZ)
{
  *minX = *minY = *minZ = 1e10;
  *maxX = *maxY = *maxZ = -1e10;

  for (int i = 0; i <= 7; i++)
    for (int j = 0; j <= 7; j++)
      for (int k = 0; k <= 7; k++)
      {
        if (jello.p[i][j][k].x < *minX) *minX = jello.p[i][j][k].x;
        if (jello.p[i][j][k].x > *maxX) *maxX = jello.p[i][j][k].x;
        if (jello.p[i][j][k].y < *minY) *minY = jello.p[i][j][k].y;
        if (jello.p[i][j][k].y > *maxY) *maxY = jello.p[i][j][k].y;
        if (jello.p[i][j][k].z < *minZ) *minZ = jello.p[i][j][k].z;
        if (jello.p[i][j][k].z > *maxZ) *maxZ = jello.p[i][j][k].z;
      }
}

// Ray-AABB intersection test
// Returns intersection distance t (negative if no intersection)
// hitPoint will contain the intersection point if t >= 0
double rayIntersectsAABB(double rayOrigX, double rayOrigY, double rayOrigZ,
                         double rayDirX, double rayDirY, double rayDirZ,
                         double minX, double maxX, double minY, double maxY, double minZ, double maxZ,
                         double *hitX, double *hitY, double *hitZ)
{
  double tmin = -1e10, tmax = 1e10;

  // X slab
  if (fabs(rayDirX) > 1e-10)
  {
    double t1 = (minX - rayOrigX) / rayDirX;
    double t2 = (maxX - rayOrigX) / rayDirX;
    if (t1 > t2) { double tmp = t1; t1 = t2; t2 = tmp; }
    if (t1 > tmin) tmin = t1;
    if (t2 < tmax) tmax = t2;
  }
  else if (rayOrigX < minX || rayOrigX > maxX)
    return -1.0;

  // Y slab
  if (fabs(rayDirY) > 1e-10)
  {
    double t1 = (minY - rayOrigY) / rayDirY;
    double t2 = (maxY - rayOrigY) / rayDirY;
    if (t1 > t2) { double tmp = t1; t1 = t2; t2 = tmp; }
    if (t1 > tmin) tmin = t1;
    if (t2 < tmax) tmax = t2;
  }
  else if (rayOrigY < minY || rayOrigY > maxY)
    return -1.0;

  // Z slab
  if (fabs(rayDirZ) > 1e-10)
  {
    double t1 = (minZ - rayOrigZ) / rayDirZ;
    double t2 = (maxZ - rayOrigZ) / rayDirZ;
    if (t1 > t2) { double tmp = t1; t1 = t2; t2 = tmp; }
    if (t1 > tmin) tmin = t1;
    if (t2 < tmax) tmax = t2;
  }
  else if (rayOrigZ < minZ || rayOrigZ > maxZ)
    return -1.0;

  if (tmax < tmin || tmax < 0)
    return -1.0;

  // Use tmin if in front of camera, otherwise tmax
  double t = (tmin >= 0) ? tmin : tmax;

  // Compute hit point
  *hitX = rayOrigX + t * rayDirX;
  *hitY = rayOrigY + t * rayDirY;
  *hitZ = rayOrigZ + t * rayDirZ;

  return t;
}

// Find the closest surface control point to a given world position
// Returns indices via pointers
void findClosestSurfacePoint(double hitX, double hitY, double hitZ,
                             int *bestI, int *bestJ, int *bestK)
{
  double minDist = 1e10;
  *bestI = *bestJ = *bestK = 0;

  for (int i = 0; i <= 7; i++)
    for (int j = 0; j <= 7; j++)
      for (int k = 0; k <= 7; k++)
      {
        // Only consider surface points (at least one index is 0 or 7)
        if (i != 0 && i != 7 && j != 0 && j != 7 && k != 0 && k != 7)
          continue;

        double dx = jello.p[i][j][k].x - hitX;
        double dy = jello.p[i][j][k].y - hitY;
        double dz = jello.p[i][j][k].z - hitZ;
        double dist = dx*dx + dy*dy + dz*dz;

        if (dist < minDist)
        {
          minDist = dist;
          *bestI = i;
          *bestJ = j;
          *bestK = k;
        }
      }
}

// Set up click force to be applied in physics loop
void applyClickForce(int i, int j, int k, double dirX, double dirY, double dirZ, double magnitude)
{
  clickForceActive = 1;
  clickForceI = i;
  clickForceJ = j;
  clickForceK = k;
  clickForceDirX = dirX;
  clickForceDirY = dirY;
  clickForceDirZ = dirZ;
  clickForceMagnitude = magnitude;
}

// Convert screen coordinates to world ray
void screenToWorldRay(int screenX, int screenY,
                      double *rayOrigX, double *rayOrigY, double *rayOrigZ,
                      double *rayDirX, double *rayDirY, double *rayDirZ)
{
  // Camera position (same as in display())
  double camX = R * cos(Phi) * cos(Theta);
  double camY = R * sin(Phi) * cos(Theta);
  double camZ = R * sin(Theta);

  // Camera looks at origin
  double lookX = 0.0, lookY = 0.0, lookZ = 0.0;

  // Up vector
  double upX = 0.0, upY = 0.0, upZ = 1.0;

  // Compute camera basis vectors
  double forwardX = lookX - camX;
  double forwardY = lookY - camY;
  double forwardZ = lookZ - camZ;
  double fLen = sqrt(forwardX*forwardX + forwardY*forwardY + forwardZ*forwardZ);
  forwardX /= fLen; forwardY /= fLen; forwardZ /= fLen;

  // Right = forward x up
  double rightX = forwardY * upZ - forwardZ * upY;
  double rightY = forwardZ * upX - forwardX * upZ;
  double rightZ = forwardX * upY - forwardY * upX;
  double rLen = sqrt(rightX*rightX + rightY*rightY + rightZ*rightZ);
  rightX /= rLen; rightY /= rLen; rightZ /= rLen;

  // Recompute up = right x forward
  double trueUpX = rightY * forwardZ - rightZ * forwardY;
  double trueUpY = rightZ * forwardX - rightX * forwardZ;
  double trueUpZ = rightX * forwardY - rightY * forwardX;

  // Convert screen coords to normalized device coords (-1 to 1)
  double ndcX = (2.0 * screenX / windowWidth) - 1.0;
  double ndcY = 1.0 - (2.0 * screenY / windowHeight);  // flip Y

  // Field of view (60 degrees from reshape())
  double fovY = 60.0 * pi / 180.0;
  double aspect = (double)windowWidth / windowHeight;
  double tanHalfFovY = tan(fovY / 2.0);

  // Ray direction in world space
  *rayDirX = forwardX + ndcX * aspect * tanHalfFovY * rightX + ndcY * tanHalfFovY * trueUpX;
  *rayDirY = forwardY + ndcX * aspect * tanHalfFovY * rightY + ndcY * tanHalfFovY * trueUpY;
  *rayDirZ = forwardZ + ndcX * aspect * tanHalfFovY * rightZ + ndcY * tanHalfFovY * trueUpZ;

  // Normalize ray direction
  double dLen = sqrt((*rayDirX)*(*rayDirX) + (*rayDirY)*(*rayDirY) + (*rayDirZ)*(*rayDirZ));
  *rayDirX /= dLen; *rayDirY /= dLen; *rayDirZ /= dLen;

  // Ray origin is camera position
  *rayOrigX = camX;
  *rayOrigY = camY;
  *rayOrigZ = camZ;
}

void mouseButton(int button, int state, int x, int y)
{
  switch (button)
  {
    case GLUT_LEFT_BUTTON:
      g_iLeftMouseButton = (state==GLUT_DOWN);

      // Check for click on jello cube and apply force
      if (state == GLUT_DOWN)
      {
        double rayOrigX, rayOrigY, rayOrigZ;
        double rayDirX, rayDirY, rayDirZ;
        screenToWorldRay(x, y, &rayOrigX, &rayOrigY, &rayOrigZ,
                         &rayDirX, &rayDirY, &rayDirZ);

        double minX, maxX, minY, maxY, minZ, maxZ;
        getJelloBoundingBox(&minX, &maxX, &minY, &maxY, &minZ, &maxZ);

        double hitX, hitY, hitZ;
        double t = rayIntersectsAABB(rayOrigX, rayOrigY, rayOrigZ,
                                     rayDirX, rayDirY, rayDirZ,
                                     minX, maxX, minY, maxY, minZ, maxZ,
                                     &hitX, &hitY, &hitZ);

        if (t >= 0)
        {
          // Find closest surface point to hit location
          int hitI, hitJ, hitK;
          findClosestSurfacePoint(hitX, hitY, hitZ, &hitI, &hitJ, &hitK);

          // Apply impulse force in ray direction
          double forceMagnitude = 10.0;  // Adjust this value to control force strength
          applyClickForce(hitI, hitJ, hitK, rayDirX, rayDirY, rayDirZ, forceMagnitude);

          printf("Force applied on cube at point [%d][%d][%d], hit=(%.2f, %.2f, %.2f)\n",
                 hitI, hitJ, hitK, hitX, hitY, hitZ);
        }
      }
      break;

    case GLUT_MIDDLE_BUTTON:
      g_iMiddleMouseButton = (state==GLUT_DOWN);
      break;
    case GLUT_RIGHT_BUTTON:
      g_iRightMouseButton = (state==GLUT_DOWN);
      break;
  }

  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
}

// gets called whenever a key is pressed
void keyboardFunc (unsigned char key, int x, int y)
{
  switch (key)
  {
    case 27:
      exit(0);
      break;

    case 'e':
      Theta = pi / 6;
      Phi = pi / 6;
      viewingMode = 0;
      break;

    case 'v':
      viewingMode = 1 - viewingMode;
      break;

    case 'h':
      shear = 1 - shear;
      break;

    case 's':
      structural = 1 - structural;
      break;

    case 'b':
      bend = 1 - bend;
      break;

    case 'p':
      pause = 1 - pause;
      break;

    case 'z':
      R -= 0.2;
      if (R < 0.2)
        R = 0.2;
      break;

    case 'x':
      R += 0.2;
      break;

    case ' ':
      saveScreenToFile = 1 - saveScreenToFile;
      break;

    case 't':
      textureMode = (textureMode + 1) % 2;  // cycle through 0 (off) and 1 (Fresnel)
      break;
  }
}

/* reads the world parameters from a world file */
/* fileName = string containing the name of the world file, ex: jello1.w */
/* function fills the structure 'jello' with parameters read from file */
/* structure 'jello' will typically be declared (probably statically, not on the heap)
   by the caller function */
/* function aborts the program if can't access the file */
void readWorld (char * fileName, struct world * jello)
{
  int i,j,k;
  FILE * file;
  
  file = fopen(fileName, "r");
  if (file == NULL) {
    printf ("can't open file\n");
    exit(1);
  }
 
/* 

  File should first contain a line specifying the integrator (EULER or RK4).
  Example: EULER
  
  Then, follows one line specifying the size of the timestep for the integrator, and
  an integer parameter n specifying  that every nth timestep will actually be drawn
  (the other steps will only be used for internal calculation)
  
  Example: 0.001 5
  Now, timestep equals 0.001. Every fifth time point will actually be drawn,
  i.e. frame1 <--> t = 0
  frame2 <--> t = 0.005
  frame3 <--> t = 0.010
  frame4 <--> t = 0.015
  ...
  
  Then, there should be two lines for physical parameters and external acceleration.
  Format is:
    kElastic dElastic kCollision dCollision
    mass
  Here
    kElastic = elastic coefficient of the spring (same for all springs except collision springs)
    dElastic = damping coefficient of the spring (same for all springs except collision springs)
    kCollision = elastic coefficient of collision springs (same for all collision springs)
    dCollision = damping coefficient of collision springs (same for all collision springs)
    mass = mass in kilograms for each of the 512 mass points 
    (mass assumed to be the same for all the points; total mass of the jello cube = 512 * mass)
  
  Example:
    10000 25 10000 15
    0.002
  
  Then, there should be one or two lines for the inclined plane, with the obvious syntax. 
  If there is no inclined plane, there should be only one line with a 0 value. There
  is no line for the coefficient. Otherwise, there are two lines, first one containing 1,
  and the second one containing the coefficients.
  Note: there is no inclined plane in this assignment (always 0).
  Example:
    1
    0.31 -0.78 0.5 5.39
  
  Next is the forceField block, first with the resolution and then the data, one point per row.
  Example:
    30
    <here 30 * 30 * 30 = 27 000 lines follow, each containing 3 real numbers>
  
  After this, there should be 1024 lines, each containing three floating-point numbers.
  The first 512 lines correspond to initial point locations.
  The last 512 lines correspond to initial point velocities.
  
  There should no blank lines anywhere in the file.

*/
       
  /* read integrator algorithm */ 
  fscanf(file,"%s\n",&jello->integrator);

  /* read timestep size and render */
  fscanf(file,"%lf %d\n",&jello->dt,&jello->n);

  /* read physical parameters */
  fscanf(file, "%lf %lf %lf %lf\n", 
    &jello->kElastic, &jello->dElastic, &jello->kCollision, &jello->dCollision);

  /* read mass of each of the 512 points */
  fscanf(file, "%lf\n", &jello->mass);

  /* read info about the plane */
  fscanf(file, "%d\n", &jello->incPlanePresent);
  if (jello->incPlanePresent == 1)
    fscanf(file, "%lf %lf %lf %lf\n", &jello->a, &jello->b, &jello->c, &jello->d);

  /* read info about the force field */
  fscanf(file, "%d\n", &jello->resolution);
  jello->forceField = 
    (struct point *)malloc(jello->resolution*jello->resolution*jello->resolution*sizeof(struct point));
  if (jello->resolution != 0)
    for (i=0; i<= jello->resolution-1; i++)
      for (j=0; j<= jello->resolution-1; j++)
        for (k=0; k<= jello->resolution-1; k++)
          fscanf(file, "%lf %lf %lf\n", 
             &jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].x, 
             &jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].y, 
             &jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].z);
             
  
  /* read initial point positions */
  for (i= 0; i <= 7 ; i++)
  {
    for (j = 0; j <= 7; j++)
    {
      for (k = 0; k <= 7; k++)
        fscanf(file, "%lf %lf %lf\n", 
          &jello->p[i][j][k].x, &jello->p[i][j][k].y, &jello->p[i][j][k].z);
    }
  }
      
  /* read initial point velocities */
  for (i = 0; i <= 7 ; i++)
  {
    for (j = 0; j <= 7; j++)
    {
      for (k = 0; k <= 7; k++)
        fscanf(file, "%lf %lf %lf\n", 
          &jello->v[i][j][k].x, &jello->v[i][j][k].y, &jello->v[i][j][k].z);
    }
  }

  fclose(file);
  
  return;
}

/* writes the world parameters to a world file on disk*/
/* fileName = string containing the name of the output world file, ex: jello1.w */
/* function creates the output world file and then fills it corresponding to the contents
   of structure 'jello' */
/* function aborts the program if can't access the file */
void writeWorld (char * fileName, struct world * jello)
{
  int i,j,k;
  FILE * file;
  
  file = fopen(fileName, "w");
  if (file == NULL) {
    printf ("can't open file\n");
    exit(1);
  }

  /* write integrator algorithm */ 
  fprintf(file,"%s\n",jello->integrator);

  /* write timestep */
  fprintf(file,"%lf %d\n",jello->dt,jello->n);

  /* write physical parameters */
  fprintf(file, "%lf %lf %lf %lf\n", 
    jello->kElastic, jello->dElastic, jello->kCollision, jello->dCollision);

  /* write mass */
  fprintf(file, "%lf\n", 
    jello->mass);

  /* write info about the plane */
  fprintf(file, "%d\n", jello->incPlanePresent);
  if (jello->incPlanePresent == 1)
    fprintf(file, "%lf %lf %lf %lf\n", jello->a, jello->b, jello->c, jello->d);

  /* write info about the force field */
  fprintf(file, "%d\n", jello->resolution);
  if (jello->resolution != 0)
    for (i=0; i<= jello->resolution-1; i++)
      for (j=0; j<= jello->resolution-1; j++)
        for (k=0; k<= jello->resolution-1; k++)
          fprintf(file, "%lf %lf %lf\n", 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].x, 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].y, 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].z);
  


  /* write initial point positions */
  for (i = 0; i <= 7 ; i++)
  {
    for (j = 0; j <= 7; j++)
    {
      for (k = 0; k <= 7; k++)
        fprintf(file, "%lf %lf %lf\n", 
          jello->p[i][j][k].x, jello->p[i][j][k].y, jello->p[i][j][k].z);
    }
  }
      
  /* write initial point velocities */
  for (i = 0; i <= 7 ; i++)
  {
    for (j = 0; j <= 7; j++)
    {
      for (k = 0; k <= 7; k++)
        fprintf(file, "%lf %lf %lf\n", 
          jello->v[i][j][k].x, jello->v[i][j][k].y, jello->v[i][j][k].z);
    }
  }

  fclose(file);
  
  return;
}

