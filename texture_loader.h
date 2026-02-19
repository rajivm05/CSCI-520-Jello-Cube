/*
  Texture Loader for Jello Cube
  Uses stb_image to load image files into OpenGL textures
*/

#ifndef _TEXTURE_LOADER_H_
#define _TEXTURE_LOADER_H_

#include "openGL-headers.h"

// Load a texture from file and return the OpenGL texture ID
// Returns 0 on failure
GLuint loadTexture(const char* filename);

// Texture IDs for different image textures
extern GLuint textureIDs[2];  // 0 = blue swirl, 1 = wood

// Initialize all textures (call once at startup)
int initTextures();

#endif
