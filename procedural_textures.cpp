/*
  Procedural Textures for Jello Cube

  Add new texture effects by:
  1. Increment NUM_TEXTURE_MODES in procedural_textures.h
  2. Add a case in getTextureModeName()
  3. Add a case in computeProceduralColor()
  4. Add a case in textureModeNeedsBlending() if it uses transparency
*/

#include "procedural_textures.h"
#include "jello.h"
#include <math.h>

// Camera parameters (extern from jello.cpp)
extern double Theta, Phi, R;

const char* getTextureModeName(int mode)
{
  switch (mode)
  {
    case 0:  return "Default";
    case 1:  return "Fresnel";
    case 2:  return "Image Texture";
    default: return "Unknown";
  }
}

int textureModeNeedsBlending(int mode)
{
  switch (mode)
  {
    case 1:  return 1;  // Fresnel uses transparency
    default: return 0;
  }
}

// Helper: compute view direction from camera to point
static void computeViewDirection(double px, double py, double pz,
                                 double *vx, double *vy, double *vz)
{
  // Camera position (same as in display())
  double camX = R * cos(Phi) * cos(Theta);
  double camY = R * sin(Phi) * cos(Theta);
  double camZ = R * sin(Theta);

  *vx = camX - px;
  *vy = camY - py;
  *vz = camZ - pz;

  double len = sqrt((*vx)*(*vx) + (*vy)*(*vy) + (*vz)*(*vz));
  if (len > 0)
  {
    *vx /= len;
    *vy /= len;
    *vz /= len;
  }
}

// Fresnel base color (extern from jello.cpp)
extern float fresnelBaseColor[3];

//
// Texture 1: Fresnel Effect
// Edges are more opaque/bright, center is more transparent
//
static void computeFresnel(double px, double py, double pz,
                           double nx, double ny, double nz,
                           float *r, float *g, float *b, float *a)
{
  double vx, vy, vz;
  computeViewDirection(px, py, pz, &vx, &vy, &vz);

  double NdotV = fabs(nx*vx + ny*vy + nz*vz);
  double fresnel = pow(1.0 - NdotV, 3.0);

  // Blend between translucent center (base color) and opaque edge (bright/white)
  *r = fresnelBaseColor[0] + (1.0f - fresnelBaseColor[0]) * 0.5f * fresnel;
  *g = fresnelBaseColor[1] + (1.0f - fresnelBaseColor[1]) * 0.8f * fresnel;
  *b = fresnelBaseColor[2] + (1.0f - fresnelBaseColor[2]) * 0.8f * fresnel;
  *a = 0.4f + 0.5f * fresnel;
}

//
// Main dispatch function
//
void computeProceduralColor(int mode,
                            double px, double py, double pz,
                            double nx, double ny, double nz,
                            float *r, float *g, float *b, float *a)
{
  switch (mode)
  {
    case 1:
      computeFresnel(px, py, pz, nx, ny, nz, r, g, b, a);
      break;

    default:
      // Default: solid red (shouldn't be called for mode 0)
      *r = 0.9f;
      *g = 0.1f;
      *b = 0.1f;
      *a = 1.0f;
      break;
  }
}
