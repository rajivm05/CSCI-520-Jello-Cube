/*
  Texture Loader for Jello Cube
  Uses stb_image to load image files into OpenGL textures
*/

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "texture_loader.h"
#include <stdio.h>
#include <stdlib.h>

// Maximum texture size to avoid GPU memory issues
#define MAX_TEXTURE_SIZE 1024

// Global texture ID
GLuint jelloTextureID = 0;

// Simple box filter downsampling
static unsigned char* downsampleImage(unsigned char* src, int srcW, int srcH, int channels, int targetW, int targetH)
{
  unsigned char* dst = (unsigned char*)malloc(targetW * targetH * channels);
  if (!dst) return NULL;

  float scaleX = (float)srcW / targetW;
  float scaleY = (float)srcH / targetH;

  for (int y = 0; y < targetH; y++)
  {
    for (int x = 0; x < targetW; x++)
    {
      // Sample from source using nearest neighbor (simple and fast)
      int srcX = (int)(x * scaleX);
      int srcY = (int)(y * scaleY);

      if (srcX >= srcW) srcX = srcW - 1;
      if (srcY >= srcH) srcY = srcH - 1;

      int srcIdx = (srcY * srcW + srcX) * channels;
      int dstIdx = (y * targetW + x) * channels;

      for (int c = 0; c < channels; c++)
        dst[dstIdx + c] = src[srcIdx + c];
    }
  }

  return dst;
}

GLuint loadTexture(const char* filename)
{
  int width, height, channels;

  // Flip image vertically (OpenGL expects bottom-left origin)
  stbi_set_flip_vertically_on_load(1);

  // Load image data
  unsigned char* data = stbi_load(filename, &width, &height, &channels, 0);

  if (!data)
  {
    printf("Failed to load texture: %s\n", filename);
    printf("stb_image error: %s\n", stbi_failure_reason());
    return 0;
  }

  printf("Loaded texture: %s (%dx%d, %d channels)\n", filename, width, height, channels);

  // Downsample if too large
  unsigned char* texData = data;
  int texW = width;
  int texH = height;

  if (width > MAX_TEXTURE_SIZE || height > MAX_TEXTURE_SIZE)
  {
    texW = (width > MAX_TEXTURE_SIZE) ? MAX_TEXTURE_SIZE : width;
    texH = (height > MAX_TEXTURE_SIZE) ? MAX_TEXTURE_SIZE : height;

    printf("Downsampling texture to %dx%d...\n", texW, texH);
    texData = downsampleImage(data, width, height, channels, texW, texH);

    if (!texData)
    {
      printf("Failed to downsample texture\n");
      stbi_image_free(data);
      return 0;
    }
  }

  // Generate OpenGL texture
  GLuint textureID;
  glGenTextures(1, &textureID);
  glBindTexture(GL_TEXTURE_2D, textureID);

  // Set texture parameters - use simple linear filtering (no mipmaps)
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // Upload texture data based on number of channels
  GLenum format;
  if (channels == 1)
    format = GL_LUMINANCE;
  else if (channels == 3)
    format = GL_RGB;
  else if (channels == 4)
    format = GL_RGBA;
  else
  {
    printf("Unsupported number of channels: %d\n", channels);
    if (texData != data) free(texData);
    stbi_image_free(data);
    return 0;
  }

  // Upload texture directly
  glTexImage2D(GL_TEXTURE_2D, 0, format, texW, texH, 0, format, GL_UNSIGNED_BYTE, texData);

  printf("Texture uploaded to GPU (%dx%d)\n", texW, texH);

  // Free image data (now stored in GPU memory)
  if (texData != data) free(texData);
  stbi_image_free(data);

  // Unbind texture
  glBindTexture(GL_TEXTURE_2D, 0);

  return textureID;
}

int initJelloTexture(const char* filename)
{
  jelloTextureID = loadTexture(filename);
  return (jelloTextureID != 0) ? 1 : 0;
}
