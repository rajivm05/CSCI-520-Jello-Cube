/*
  Procedural Textures for Jello Cube

  This module provides procedural texture/shading effects that can be
  applied to the jello cube. Press 't' to cycle through available textures.
*/

#ifndef _PROCEDURAL_TEXTURES_H_
#define _PROCEDURAL_TEXTURES_H_

#include "jello.h"

// Number of available texture modes (0 = off, 1 = Fresnel, 2 = Blue swirl, 3 = Wood)
#define NUM_TEXTURE_MODES 4

// Get the name of the current texture mode (for display)
const char* getTextureModeName(int mode);

// Compute per-vertex color for the current texture mode
// Parameters:
//   mode: texture mode (1, 2, 3, ...)
//   px, py, pz: vertex position in world space
//   nx, ny, nz: vertex normal (normalized)
//   r, g, b, a: output color values (0.0 - 1.0)
void computeProceduralColor(int mode,
                            double px, double py, double pz,
                            double nx, double ny, double nz,
                            float *r, float *g, float *b, float *a);

// Check if the current texture mode requires blending
int textureModeNeedsBlending(int mode);

#endif
