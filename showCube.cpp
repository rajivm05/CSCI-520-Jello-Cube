/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "showCube.h"
#include "procedural_textures.h"
#include "texture_loader.h"

int pointMap(int side, int i, int j)
{
  int r;

  switch (side)
  {
  case 1: //[i][j][0] bottom face
    r = 64 * i + 8 * j;
    break;
  case 6: //[i][j][7] top face
    r = 64 * i + 8 * j + 7;
    break;
  case 2: //[i][0][j] front face
    r = 64 * i + j;
    break;
  case 5: //[i][7][j] back face
    r = 64 * i + 56 + j;
    break;
  case 3: //[0][i][j] left face
    r = 8 * i + j;
    break;
  case 4: //[7][i][j] right face
    r = 448 + 8 * i + j;
    break;
  }

  return r;
}

void showCube(struct world * jello)
{
  int i,j,k,ip,jp,kp;
  point r1,r2,r3; // aux variables
  
  /* normals buffer and counter for Gourad shading*/
  struct point normal[8][8];
  int counter[8][8];

  int face;
  double faceFactor, length;

  if (fabs(jello->p[0][0][0].x) > 10)
  {
    printf ("Your cube somehow escaped way out of the box.\n");
    exit(0);
  }

  
  #define NODE(face,i,j) (*((struct point * )(jello->p) + pointMap((face),(i),(j))))

  
  #define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if\
    (!( (ip>7) || (ip<0) ||\
      (jp>7) || (jp<0) ||\
    (kp>7) || (kp<0) ) && ((i==0) || (i==7) || (j==0) || (j==7) || (k==0) || (k==7))\
       && ((ip==0) || (ip==7) || (jp==0) || (jp==7) || (kp==0) || (kp==7))) \
    {\
      glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
      glVertex3f(jello->p[ip][jp][kp].x,jello->p[ip][jp][kp].y,jello->p[ip][jp][kp].z);\
    }\

 
  if (viewingMode==0) // render wireframe
  {
    glLineWidth(1);
    glPointSize(5);
    glDisable(GL_LIGHTING);
    for (i=0; i<=7; i++)
      for (j=0; j<=7; j++)
        for (k=0; k<=7; k++)
        {
          if (i*j*k*(7-i)*(7-j)*(7-k) != 0) // not surface point
            continue;

          glBegin(GL_POINTS); // draw point
            glColor4f(0,0,0,0);  
            glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);        
          glEnd();

          //
          //if ((i!=7) || (j!=7) || (k!=7))
          //  continue;

          glBegin(GL_LINES);      
          // structural
          if (structural == 1)
          {
            glColor4f(0,0,1,1);
            PROCESS_NEIGHBOUR(1,0,0);
            PROCESS_NEIGHBOUR(0,1,0);
            PROCESS_NEIGHBOUR(0,0,1);
            PROCESS_NEIGHBOUR(-1,0,0);
            PROCESS_NEIGHBOUR(0,-1,0);
            PROCESS_NEIGHBOUR(0,0,-1);
          }
          
          // shear
          if (shear == 1)
          {
            glColor4f(0,1,0,1);
            PROCESS_NEIGHBOUR(1,1,0);
            PROCESS_NEIGHBOUR(-1,1,0);
            PROCESS_NEIGHBOUR(-1,-1,0);
            PROCESS_NEIGHBOUR(1,-1,0);
            PROCESS_NEIGHBOUR(0,1,1);
            PROCESS_NEIGHBOUR(0,-1,1);
            PROCESS_NEIGHBOUR(0,-1,-1);
            PROCESS_NEIGHBOUR(0,1,-1);
            PROCESS_NEIGHBOUR(1,0,1);
            PROCESS_NEIGHBOUR(-1,0,1);
            PROCESS_NEIGHBOUR(-1,0,-1);
            PROCESS_NEIGHBOUR(1,0,-1);

            PROCESS_NEIGHBOUR(1,1,1)
            PROCESS_NEIGHBOUR(-1,1,1)
            PROCESS_NEIGHBOUR(-1,-1,1)
            PROCESS_NEIGHBOUR(1,-1,1)
            PROCESS_NEIGHBOUR(1,1,-1)
            PROCESS_NEIGHBOUR(-1,1,-1)
            PROCESS_NEIGHBOUR(-1,-1,-1)
            PROCESS_NEIGHBOUR(1,-1,-1)
          }
          
          // bend
          if (bend == 1)
          {
            glColor4f(1,0,0,1);
            PROCESS_NEIGHBOUR(2,0,0);
            PROCESS_NEIGHBOUR(0,2,0);
            PROCESS_NEIGHBOUR(0,0,2);
            PROCESS_NEIGHBOUR(-2,0,0);
            PROCESS_NEIGHBOUR(0,-2,0);
            PROCESS_NEIGHBOUR(0,0,-2);
          }           
          glEnd();
        }
    glEnable(GL_LIGHTING);
  }
  
  else
  {
    glPolygonMode(GL_FRONT, GL_FILL);

    // Enable blending and color material for procedural textures
    if (textureMode > 0 && textureModeNeedsBlending(textureMode))
    {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_COLOR_MATERIAL);
      glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    }

    // Enable image texture mapping for mode 2
    if (textureMode == 2 && jelloTextureID != 0)
    {
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, jelloTextureID);
      // Replace color with texture (ignore material color)
      glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    }

    for (face=1; face <= 6; face++) 
      // face == face of a cube
      // 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
    {
      
      if ((face==1) || (face==3) || (face==5))
        faceFactor=-1; // flip orientation
      else
        faceFactor=1;
      

      for (i=0; i <= 7; i++) // reset buffers
        for (j=0; j <= 7; j++)
        {
          normal[i][j].x=0;normal[i][j].y=0;normal[i][j].z=0;
          counter[i][j]=0;
        }

      /* process triangles, accumulate normals for Gourad shading */
  
      for (i=0; i <= 6; i++)
        for (j=0; j <= 6; j++) // process block (i,j)
        {
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i,j),r1); // first triangle
          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i,j),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i][j],r3,normal[i][j]);
          counter[i][j]++;

          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i+1,j+1),r1); // second triangle
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i+1,j+1),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i+1][j+1],r3,normal[i+1][j+1]);
          counter[i+1][j+1]++;
        }

      
        /* the actual rendering */
        for (j=1; j<=7; j++)
        {

          if (faceFactor  > 0)
            glFrontFace(GL_CCW); // the usual definition of front face
          else
            glFrontFace(GL_CW); // flip definition of orientation

          glBegin(GL_TRIANGLE_STRIP);
          for (i=0; i<=7; i++)
          {
            // Vertex 1: (i, j)
            double nx1 = normal[i][j].x / counter[i][j];
            double ny1 = normal[i][j].y / counter[i][j];
            double nz1 = normal[i][j].z / counter[i][j];

            if (textureMode == 1)
            {
              // Fresnel procedural texture
              float r, g, b, a;
              computeProceduralColor(textureMode,
                                     NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z,
                                     nx1, ny1, nz1,
                                     &r, &g, &b, &a);
              glColor4f(r, g, b, a);
            }
            else if (textureMode == 2)
            {
              // Image texture - set UV coordinates
              glTexCoord2f(i / 7.0f, j / 7.0f);
            }

            glNormal3f(nx1, ny1, nz1);
            glVertex3f(NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z);

            // Vertex 2: (i, j-1)
            double nx2 = normal[i][j-1].x / counter[i][j-1];
            double ny2 = normal[i][j-1].y / counter[i][j-1];
            double nz2 = normal[i][j-1].z / counter[i][j-1];

            if (textureMode == 1)
            {
              // Fresnel procedural texture
              float r, g, b, a;
              computeProceduralColor(textureMode,
                                     NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z,
                                     nx2, ny2, nz2,
                                     &r, &g, &b, &a);
              glColor4f(r, g, b, a);
            }
            else if (textureMode == 2)
            {
              // Image texture - set UV coordinates
              glTexCoord2f(i / 7.0f, (j - 1) / 7.0f);
            }

            glNormal3f(nx2, ny2, nz2);
            glVertex3f(NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z);
          }
          glEnd();
        }
        
        
    }

    // Disable blending and color material after procedural texture rendering
    if (textureMode > 0 && textureModeNeedsBlending(textureMode))
    {
      glDisable(GL_COLOR_MATERIAL);
      glDisable(GL_BLEND);
    }

    // Disable image texture mapping
    if (textureMode == 2)
    {
      glBindTexture(GL_TEXTURE_2D, 0);
      glDisable(GL_TEXTURE_2D);
    }
  } // end for loop over faces
  glFrontFace(GL_CCW);
}

void showInclinedPlane(struct world * jello)
{
  if (jello->incPlanePresent == 0)
    return;

  double a = jello->a, b = jello->b, c = jello->c, d = jello->d;

  // Bounding box corners
  double bMin = -2.0, bMax = 2.0;
  double verts[8][3] = {
    {bMin,bMin,bMin}, {bMax,bMin,bMin}, {bMin,bMax,bMin}, {bMax,bMax,bMin},
    {bMin,bMin,bMax}, {bMax,bMin,bMax}, {bMin,bMax,bMax}, {bMax,bMax,bMax}
  };

  // 12 edges of the bounding box (pairs of vertex indices)
  int edges[12][2] = {
    {0,1}, {2,3}, {4,5}, {6,7},   // along x
    {0,2}, {1,3}, {4,6}, {5,7},   // along y
    {0,4}, {1,5}, {2,6}, {3,7}    // along z
  };

  // Find intersection points: vertices on the plane + edge crossings
  double pts[20][3];
  int numPts = 0;

  // First, add any bounding box vertices that lie on the plane
  for (int v = 0; v < 8; v++)
  {
    double f = a*verts[v][0] + b*verts[v][1] + c*verts[v][2] + d;
    if (fabs(f) < 1e-6)
    {
      pts[numPts][0] = verts[v][0];
      pts[numPts][1] = verts[v][1];
      pts[numPts][2] = verts[v][2];
      numPts++;
    }
  }

  // Then, add strict edge crossings (both endpoints off the plane, opposite sides)
  for (int e = 0; e < 12; e++)
  {
    double *p1 = verts[edges[e][0]];
    double *p2 = verts[edges[e][1]];

    double f1 = a*p1[0] + b*p1[1] + c*p1[2] + d;
    double f2 = a*p2[0] + b*p2[1] + c*p2[2] + d;

    // Only strict crossings (neither endpoint on plane)
    if (f1 * f2 < 0)
    {
      double t = f1 / (f1 - f2);
      pts[numPts][0] = p1[0] + t*(p2[0]-p1[0]);
      pts[numPts][1] = p1[1] + t*(p2[1]-p1[1]);
      pts[numPts][2] = p1[2] + t*(p2[2]-p1[2]);
      numPts++;
    }
  }

  if (numPts < 3)
    return;

  // Compute centroid of intersection polygon
  double cx=0, cy=0, cz=0;
  for (int i = 0; i < numPts; i++)
  { cx += pts[i][0]; cy += pts[i][1]; cz += pts[i][2]; }
  cx /= numPts; cy /= numPts; cz /= numPts;

  // Sort vertices by angle around centroid (needed for correct polygon winding)
  double nLen = sqrt(a*a + b*b + c*c);
  double nx = a/nLen, ny = b/nLen, nz = c/nLen;

  // Reference direction: centroid to first point
  double refX = pts[0][0]-cx, refY = pts[0][1]-cy, refZ = pts[0][2]-cz;

  double angles[12];
  for (int i = 0; i < numPts; i++)
  {
    double dx = pts[i][0]-cx, dy = pts[i][1]-cy, dz = pts[i][2]-cz;
    double dot = dx*refX + dy*refY + dz*refZ;
    // Cross product projected onto plane normal gives sin component
    double crossDotN = (refY*dz - refZ*dy)*nx
                     + (refZ*dx - refX*dz)*ny
                     + (refX*dy - refY*dx)*nz;
    angles[i] = atan2(crossDotN, dot);
  }

  // Sort by angle (bubble sort, max 12 points)
  for (int i = 0; i < numPts-1; i++)
    for (int j = i+1; j < numPts; j++)
      if (angles[j] < angles[i])
      {
        double tmp;
        tmp = angles[i]; angles[i] = angles[j]; angles[j] = tmp;
        tmp = pts[i][0]; pts[i][0] = pts[j][0]; pts[j][0] = tmp;
        tmp = pts[i][1]; pts[i][1] = pts[j][1]; pts[j][1] = tmp;
        tmp = pts[i][2]; pts[i][2] = pts[j][2]; pts[j][2] = tmp;
      }

  // Render filled polygon (semi-transparent)
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_CULL_FACE);

  glColor4f(0.8, 0.6, 0.2, 0.4);
  glBegin(GL_POLYGON);
  for (int i = 0; i < numPts; i++)
    glVertex3f(pts[i][0], pts[i][1], pts[i][2]);
  glEnd();

  // Render outline
  glLineWidth(2);
  glColor4f(0.9, 0.7, 0.2, 0.9);
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < numPts; i++)
    glVertex3f(pts[i][0], pts[i][1], pts[i][2]);
  glEnd();

  glEnable(GL_CULL_FACE);
  glDisable(GL_BLEND);
}

void showBoundingBox()
{
  int i,j;

  glColor4f(0.6,0.6,0.6,0);

  glBegin(GL_LINES);

  // front face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,-2,-2);
    glVertex3f(i,-2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(2,-2,j);
  }

  // back face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,2,-2);
    glVertex3f(i,2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,2,j);
    glVertex3f(2,2,j);
  }

  // left face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(-2,i,-2);
    glVertex3f(-2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(-2,2,j);
  }

  // right face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(2,i,-2);
    glVertex3f(2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(2,-2,j);
    glVertex3f(2,2,j);
  }

  glEnd();

  return;
}

