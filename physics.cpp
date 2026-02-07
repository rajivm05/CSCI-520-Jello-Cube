/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

#include <thread>
#include <algorithm>
#include <vector>

// Number of control points (8x8x8 = 512)
#define TOTAL_POINTS 512

// Convert flat index to 3D indices
static inline void flatTo3D(int flat, int &i, int &j, int &k)
{
  i = flat / 64;        // flat / (8*8)
  j = (flat / 8) % 8;
  k = flat % 8;
}

// Number of hardware threads for parallelization
static const unsigned int NUM_THREADS = std::max(1u, std::thread::hardware_concurrency());

// Helper: run a function in parallel over flat point indices [0, TOTAL_POINTS)
// func(start, end) is called for each thread's assigned range
template <typename Func>
static void parallelForPoints(Func func)
{
  if (NUM_THREADS <= 1)
  {
    func(0, TOTAL_POINTS);
    return;
  }

  std::vector<std::thread> threads;
  threads.reserve(NUM_THREADS);

  int blockSize = TOTAL_POINTS / NUM_THREADS;
  for (unsigned int t = 0; t < NUM_THREADS; t++)
  {
    int start = t * blockSize;
    int end = (t == NUM_THREADS - 1) ? TOTAL_POINTS : (t + 1) * blockSize;
    threads.emplace_back(func, start, end);
  }

  for (auto& thread : threads)
    thread.join();
}

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

/* Computes spring force between two points using Hooke's Law and damping.

   Hooke's Law: F_hook = -k * (|L| - R) * (L / |L|)
   Damping:     F_damp = -d * ((vA - vB) · (L / |L|)) * (L / |L|)

   Where L = pA - pB (vector from B to A), R = rest length
   Force is accumulated into forceAccum (force on point A) */
void computeSpringForce(struct point * pA, struct point * pB,
                        struct point * vA, struct point * vB,
                        double kElastic, double dElastic,
                        double restLength, struct point * forceAccum)
{
  struct point L;        // Vector from B to A
  struct point Lnorm;    // Normalized L
  struct point vDiff;    // Velocity difference vA - vB
  double length;         // Required by pNORMALIZE macro
  double currentLength;  // |L|
  double extension;      // |L| - R (how much spring is stretched)
  double dotProduct;     // (vA - vB) · Lnorm

  // L = pA - pB
  pDIFFERENCE(*pA, *pB, L);

  // Compute current spring length |L|
  currentLength = sqrt(L.x * L.x + L.y * L.y + L.z * L.z);

  // Avoid division by zero for coincident points
  if (currentLength < 1e-10)
    return;

  // Normalized direction vector
  Lnorm.x = L.x / currentLength;
  Lnorm.y = L.y / currentLength;
  Lnorm.z = L.z / currentLength;

  // Extension: positive when stretched, negative when compressed
  extension = currentLength - restLength;

  // Hooke's Law: F = -k * extension * direction
  // When stretched (extension > 0), force pulls A toward B (negative direction)
  // When compressed (extension < 0), force pushes A away from B (positive direction)
  struct point hookForce;
  hookForce.x = -kElastic * extension * Lnorm.x;
  hookForce.y = -kElastic * extension * Lnorm.y;
  hookForce.z = -kElastic * extension * Lnorm.z;

  // Damping force: F = -d * ((vA - vB) · Lnorm) * Lnorm
  // Damps relative motion along the spring direction
  pDIFFERENCE(*vA, *vB, vDiff);
  dotProduct = vDiff.x * Lnorm.x + vDiff.y * Lnorm.y + vDiff.z * Lnorm.z;

  struct point dampForce;
  dampForce.x = -dElastic * dotProduct * Lnorm.x;
  dampForce.y = -dElastic * dotProduct * Lnorm.y;
  dampForce.z = -dElastic * dotProduct * Lnorm.z;

  // Accumulate forces
  forceAccum->x += hookForce.x + dampForce.x;
  forceAccum->y += hookForce.y + dampForce.y;
  forceAccum->z += hookForce.z + dampForce.z;
}

/* Computes acceleration to every control point of the jello cube,
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  // Initialize all accelerations to zero
  for (int i = 0; i <= 7; i++)
    for (int j = 0; j <= 7; j++)
      for (int k = 0; k <= 7; k++)
      {
        pMAKE(0.0, 0.0, 0.0, a[i][j][k]);
      }

  // Accumulate forces for each point in parallel
  // Each thread processes a range of flat indices [start, end)
  // Safe because each point only writes to its own a[i][j][k] entry
  parallelForPoints([&](int start, int end) {
    for (int flat = start; flat < end; flat++)
    {
      int i, j, k;
      flatTo3D(flat, i, j, k);

      struct point force;
      pMAKE(0.0, 0.0, 0.0, force);

      // 1. External force field
      struct point forceField;
      computeForceField(jello, &jello->p[i][j][k], &forceField);
      pSUM(force, forceField, force);

      // 2. Spring forces

      // Rest lengths (grid spans 0 to 1 with 7 intervals, so spacing = 1/7)
      double restStructural = 1.0 / 7.0;

      // Structural springs: connect to immediate neighbors
      int structuralOffsets[6][3] = {
        {-1, 0, 0}, {1, 0, 0},
        {0, -1, 0}, {0, 1, 0},
        {0, 0, -1}, {0, 0, 1}
      };

      for (int s = 0; s < 6; s++)
      {
        int ni = i + structuralOffsets[s][0];
        int nj = j + structuralOffsets[s][1];
        int nk = k + structuralOffsets[s][2];

        if (ni >= 0 && ni <= 7 && nj >= 0 && nj <= 7 && nk >= 0 && nk <= 7)
        {
          computeSpringForce(&jello->p[i][j][k], &jello->p[ni][nj][nk],
                             &jello->v[i][j][k], &jello->v[ni][nj][nk],
                             jello->kElastic, jello->dElastic,
                             restStructural, &force);
        }
      }

      // Shear springs: face diagonals
      double restShearFace = sqrt(2.0) / 7.0;

      int shearFaceOffsets[12][3] = {
        {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}, {1, 1, 0},
        {-1, 0, -1}, {-1, 0, 1}, {1, 0, -1}, {1, 0, 1},
        {0, -1, -1}, {0, -1, 1}, {0, 1, -1}, {0, 1, 1}
      };

      for (int s = 0; s < 12; s++)
      {
        int ni = i + shearFaceOffsets[s][0];
        int nj = j + shearFaceOffsets[s][1];
        int nk = k + shearFaceOffsets[s][2];

        if (ni >= 0 && ni <= 7 && nj >= 0 && nj <= 7 && nk >= 0 && nk <= 7)
        {
          computeSpringForce(&jello->p[i][j][k], &jello->p[ni][nj][nk],
                             &jello->v[i][j][k], &jello->v[ni][nj][nk],
                             jello->kElastic, jello->dElastic,
                             restShearFace, &force);
        }
      }

      // Shear springs: body diagonals
      double restShearBody = sqrt(3.0) / 7.0;

      int shearBodyOffsets[8][3] = {
        {-1, -1, -1}, {-1, -1, 1}, {-1, 1, -1}, {-1, 1, 1},
        {1, -1, -1}, {1, -1, 1}, {1, 1, -1}, {1, 1, 1}
      };

      for (int s = 0; s < 8; s++)
      {
        int ni = i + shearBodyOffsets[s][0];
        int nj = j + shearBodyOffsets[s][1];
        int nk = k + shearBodyOffsets[s][2];

        if (ni >= 0 && ni <= 7 && nj >= 0 && nj <= 7 && nk >= 0 && nk <= 7)
        {
          computeSpringForce(&jello->p[i][j][k], &jello->p[ni][nj][nk],
                             &jello->v[i][j][k], &jello->v[ni][nj][nk],
                             jello->kElastic, jello->dElastic,
                             restShearBody, &force);
        }
      }

      // Bend springs: neighbors 2 steps away
      double restBend = 2.0 / 7.0;

      int bendOffsets[6][3] = {
        {-2, 0, 0}, {2, 0, 0},
        {0, -2, 0}, {0, 2, 0},
        {0, 0, -2}, {0, 0, 2}
      };

      for (int s = 0; s < 6; s++)
      {
        int ni = i + bendOffsets[s][0];
        int nj = j + bendOffsets[s][1];
        int nk = k + bendOffsets[s][2];

        if (ni >= 0 && ni <= 7 && nj >= 0 && nj <= 7 && nk >= 0 && nk <= 7)
        {
          computeSpringForce(&jello->p[i][j][k], &jello->p[ni][nj][nk],
                             &jello->v[i][j][k], &jello->v[ni][nj][nk],
                             jello->kElastic, jello->dElastic,
                             restBend, &force);
        }
      }

      // 3. Collision forces

      // Bounding box collision: box spans -2 to +2
      double boxMin = -2.0;
      double boxMax = 2.0;

      struct point * pos = &jello->p[i][j][k];
      struct point * vel = &jello->v[i][j][k];

      // X-axis
      if (pos->x < boxMin)
      {
        double penetration = boxMin - pos->x;
        force.x += jello->kCollision * penetration;
        force.x += -jello->dCollision * vel->x;
      }
      else if (pos->x > boxMax)
      {
        double penetration = pos->x - boxMax;
        force.x += -jello->kCollision * penetration;
        force.x += -jello->dCollision * vel->x;
      }

      // Y-axis
      if (pos->y < boxMin)
      {
        double penetration = boxMin - pos->y;
        force.y += jello->kCollision * penetration;
        force.y += -jello->dCollision * vel->y;
      }
      else if (pos->y > boxMax)
      {
        double penetration = pos->y - boxMax;
        force.y += -jello->kCollision * penetration;
        force.y += -jello->dCollision * vel->y;
      }

      // Z-axis
      if (pos->z < boxMin)
      {
        double penetration = boxMin - pos->z;
        force.z += jello->kCollision * penetration;
        force.z += -jello->dCollision * vel->z;
      }
      else if (pos->z > boxMax)
      {
        double penetration = pos->z - boxMax;
        force.z += -jello->kCollision * penetration;
        force.z += -jello->dCollision * vel->z;
      }

      // Inclined plane collision
      if (jello->incPlanePresent == 1)
      {
        double F = jello->a * pos->x + jello->b * pos->y + jello->c * pos->z + jello->d;
        double normalLength = sqrt(jello->a * jello->a + jello->b * jello->b + jello->c * jello->c);

        if (F < 0 && normalLength > 1e-10)
        {
          double penetration = -F / normalLength;
          double nx = jello->a / normalLength;
          double ny = jello->b / normalLength;
          double nz = jello->c / normalLength;

          force.x += jello->kCollision * penetration * nx;
          force.y += jello->kCollision * penetration * ny;
          force.z += jello->kCollision * penetration * nz;

          double vDotN = vel->x * nx + vel->y * ny + vel->z * nz;
          force.x += -jello->dCollision * vDotN * nx;
          force.y += -jello->dCollision * vDotN * ny;
          force.z += -jello->dCollision * vDotN * nz;
        }
      }

      // 4. Click force (user interaction)
      if (clickForceActive && i == clickForceI && j == clickForceJ && k == clickForceK)
      {
        force.x += clickForceMagnitude * clickForceDirX;
        force.y += clickForceMagnitude * clickForceDirY;
        force.z += clickForceMagnitude * clickForceDirZ;
      }

      // Convert force to acceleration: a = F / m
      pMULTIPLY(force, 1.0 / jello->mass, a[i][j][k]);
    }
  });
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
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
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
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
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
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
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
