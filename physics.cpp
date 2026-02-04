/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

/* Computes the external force at a given position using trilinear interpolation
   of the force field grid. Force field spans -2 to +2 in each dimension. */
void computeForceField(struct world * jello, struct point * position, struct point * force)
{
  // Initialize force to zero
  pMAKE(0.0, 0.0, 0.0, *force);

  // No force field if resolution is 0
  if (jello->resolution == 0)
    return;

  int n = jello->resolution;

  // Convert position to grid coordinates
  // Force field spans -2 to +2 (size 4) in each dimension
  // Grid has n points, so n-1 intervals
  double gridX = (position->x + 2.0) * (n - 1) / 4.0;
  double gridY = (position->y + 2.0) * (n - 1) / 4.0;
  double gridZ = (position->z + 2.0) * (n - 1) / 4.0;

  // Clamp to valid grid range [0, n-1]
  if (gridX < 0) gridX = 0;
  if (gridX > n - 1) gridX = n - 1;
  if (gridY < 0) gridY = 0;
  if (gridY > n - 1) gridY = n - 1;
  if (gridZ < 0) gridZ = 0;
  if (gridZ > n - 1) gridZ = n - 1;

  // Get integer indices of the cell containing the point
  int i0 = (int)gridX;
  int j0 = (int)gridY;
  int k0 = (int)gridZ;

  // Clamp to ensure we have valid i1, j1, k1
  int i1 = (i0 < n - 1) ? i0 + 1 : i0;
  int j1 = (j0 < n - 1) ? j0 + 1 : j0;
  int k1 = (k0 < n - 1) ? k0 + 1 : k0;

  // Compute interpolation weights (fractional part)
  double fx = gridX - i0;
  double fy = gridY - j0;
  double fz = gridZ - k0;

  // Get the 8 corner forces of the cell
  // Force field is indexed as: forceField[i * n * n + j * n + k]
  struct point * f000 = &jello->forceField[i0 * n * n + j0 * n + k0];
  struct point * f001 = &jello->forceField[i0 * n * n + j0 * n + k1];
  struct point * f010 = &jello->forceField[i0 * n * n + j1 * n + k0];
  struct point * f011 = &jello->forceField[i0 * n * n + j1 * n + k1];
  struct point * f100 = &jello->forceField[i1 * n * n + j0 * n + k0];
  struct point * f101 = &jello->forceField[i1 * n * n + j0 * n + k1];
  struct point * f110 = &jello->forceField[i1 * n * n + j1 * n + k0];
  struct point * f111 = &jello->forceField[i1 * n * n + j1 * n + k1];

  // Trilinear interpolation
  // Interpolate along z axis first
  struct point f00, f01, f10, f11;
  f00.x = f000->x * (1 - fz) + f001->x * fz;
  f00.y = f000->y * (1 - fz) + f001->y * fz;
  f00.z = f000->z * (1 - fz) + f001->z * fz;

  f01.x = f010->x * (1 - fz) + f011->x * fz;
  f01.y = f010->y * (1 - fz) + f011->y * fz;
  f01.z = f010->z * (1 - fz) + f011->z * fz;

  f10.x = f100->x * (1 - fz) + f101->x * fz;
  f10.y = f100->y * (1 - fz) + f101->y * fz;
  f10.z = f100->z * (1 - fz) + f101->z * fz;

  f11.x = f110->x * (1 - fz) + f111->x * fz;
  f11.y = f110->y * (1 - fz) + f111->y * fz;
  f11.z = f110->z * (1 - fz) + f111->z * fz;

  // Interpolate along y axis
  struct point f0, f1;
  f0.x = f00.x * (1 - fy) + f01.x * fy;
  f0.y = f00.y * (1 - fy) + f01.y * fy;
  f0.z = f00.z * (1 - fy) + f01.z * fy;

  f1.x = f10.x * (1 - fy) + f11.x * fy;
  f1.y = f10.y * (1 - fy) + f11.y * fy;
  f1.z = f10.z * (1 - fy) + f11.z * fy;

  // Interpolate along x axis to get final force
  force->x = f0.x * (1 - fx) + f1.x * fx;
  force->y = f0.y * (1 - fx) + f1.y * fx;
  force->z = f0.z * (1 - fx) + f1.z * fx;
}

/* Computes acceleration to every control point of the jello cube,
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  int i, j, k;
  struct point forceField;

  // Initialize all accelerations to zero
  for (i = 0; i <= 7; i++)
    for (j = 0; j <= 7; j++)
      for (k = 0; k <= 7; k++)
      {
        pMAKE(0.0, 0.0, 0.0, a[i][j][k]);
      }

  // Accumulate forces for each point
  for (i = 0; i <= 7; i++)
    for (j = 0; j <= 7; j++)
      for (k = 0; k <= 7; k++)
      {
        struct point force;
        pMAKE(0.0, 0.0, 0.0, force);

        // 1. External force field
        computeForceField(jello, &jello->p[i][j][k], &forceField);
        pSUM(force, forceField, force);

        // TODO: 2. Spring forces (structural, shear, bend)

        // TODO: 3. Collision forces (bounding box, inclined plane)

        // Convert force to acceleration: a = F / m
        pMULTIPLY(force, 1.0 / jello->mass, a[i][j][k]);
      }
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
