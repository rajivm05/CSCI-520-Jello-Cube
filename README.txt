CSCI 520, Assignment 1: Jello Cube

Rajiv Murali

================

WHAT I ACCOMPLISHED
-------------------

I implemented a complete jello cube physics simulation with all required features:

1. Mass-Spring Physics: Hooke's Law with damping for structural, shear (face and
   body diagonal), and bend springs. Forces correctly restore the cube to rest shape.

2. Numerical Integration: Both RK4 and Euler methods implemented. RK4 provides
   superior stability for stiff springs.

3. Force Field: Trilinear interpolation of 3D force field grid for smooth external
   forces (gravity, wind).

4. Bounding Box Collision: Penalty-based collision for all six faces (-2 to +2).
   Restoring forces proportional to penetration depth with damping.

5. Inclined Plane Collision: Collision with arbitrary plane (ax + by + cz + d = 0)
   using same penalty method.

6. Rendering: Wireframe mode (color-coded springs) and Gouraud-shaded mode with
   proper vertex normals.

7. Performance: 40+ FPS at 640x480, meeting interactive frame rate requirement.


EXTRA CREDIT IMPLEMENTED
------------------------

1. INTERACTIVE MOUSE DRAGGING
   - Left-click and drag to move the jello cube in real-time
   - Physics continues running during drag (cube deforms on wall collision)
   - Converts 2D mouse movement to 3D using camera's view plane vectors
   - All 512 points translated equally, maintaining cube integrity

2. FRESNEL PROCEDURAL TEXTURE (press 't' to enable)
   - View-dependent transparency: edges opaque, center transparent
   - Uses fresnel = pow(1.0 - |N.V|, 3.0) for realistic light behavior
   - COLOR CHANGES ON COLLISION: cycles through 8 colors (red, orange, yellow,
     green, cyan, blue, purple, pink) each time cube hits a wall

3. IMAGE TEXTURE MAPPING (press 't' to cycle)
   - Two textures: Blue Swirl and Wood
   - Loaded using stb_image single-header library
   - UV coordinates mapped (0,0) to (1,1) on each face
   - Automatic downsampling for textures > 1024x1024
   - Textures deform naturally with the jello cube

4. CUSTOM CINEMATIC LIGHTING
   - Replaced default 8-light setup with professional three-point lighting:
     * Key light: Warm orange/gold at (3, -2, 3) - main illumination
     * Fill light: Cool blue at (-3, -1, 1) - softens shadows
     * Rim light: Pale yellow at (0, 3, 2) - edge definition
     * Under light: Subtle purple at (0, 0, -3) - adds depth
   - Custom red/orange jello material with specular highlights

5. INCLINED PLANE VISUALIZATION
   - Renders collision plane as semi-transparent polygon
   - Automatically clips to bounding box using plane-edge intersection
   - Handles edge cases (vertices exactly on plane)
   - Sorts intersection points by angle for correct polygon winding

6. MULTITHREADED PHYSICS
   - Parallelized computeAcceleration() using C++11 std::thread
   - No external dependencies (no OpenMP required)
   - Block-based work distribution across available CPU cores
   - Thread-safe: each point writes only to its own acceleration entry
   - Automatic fallback to single-threaded if only 1 core


CONTROLS
--------
v       - Toggle wireframe/shaded view
t       - Cycle texture modes (Default, Fresnel, Blue Swirl, Wood)
p       - Pause/resume simulation
s/h/b   - Toggle structural/shear/bend springs (wireframe mode)
z/x     - Zoom in/out
e       - Reset camera
Space   - Save screenshot
ESC     - Exit

Left-click + drag  - Drag jello cube
Right-click + drag - Rotate camera
