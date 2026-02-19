# Jello Cube Physics Simulation

**CSCI 520 - Computer Animation and Simulation**
**Assignment 1: Jello Cube**

## Overview

This project implements a physically-based simulation of a deformable jello cube using mass-spring dynamics. The cube consists of 512 control points (8x8x8 grid) connected by structural, shear, and bend springs. The simulation supports real-time interaction, multiple rendering modes, and procedural/image textures.

---

## Build Instructions

### macOS / Linux
```bash
make clean
make
./jello world/jello.w
```

---

## Controls

| Key | Action |
|-----|--------|
| `v` | Toggle view mode (wireframe / shaded) |
| `t` | Cycle texture mode (Default, Fresnel, Blue Swirl, Wood) |
| `p` | Pause / resume simulation |
| `s` | Toggle structural springs (wireframe mode) |
| `h` | Toggle shear springs (wireframe mode) |
| `b` | Toggle bend springs (wireframe mode) |
| `z` | Zoom in |
| `x` | Zoom out |
| `e` | Reset camera to default position |
| `Space` | Save screenshot (PPM format) |
| `ESC` | Exit |

| Mouse | Action |
|-------|--------|
| **Left-click + drag** | Drag the jello cube |
| **Right-click + drag** | Rotate camera |

---

## Features Implemented

### 1. Core Physics Engine

#### Spring System
The jello cube uses three types of springs to maintain its shape:

| Spring Type | Connections | Rest Length | Purpose |
|-------------|-------------|-------------|---------|
| Structural | 6 neighbors (+-1 in x,y,z) | 1/7 | Maintains grid structure |
| Shear (face) | 12 face diagonal neighbors | sqrt(2)/7 | Resists shearing |
| Shear (body) | 8 body diagonal neighbors | sqrt(3)/7 | Resists volume change |
| Bend | 6 neighbors at distance 2 | 2/7 | Resists bending |

#### Force Computation
- **Hooke's Law:** `F = -k * (|L| - R) * (L/|L|)`
- **Damping:** `F = -d * ((vA - vB) . L/|L|) * (L/|L|)`

#### Integration Methods
- **RK4 (4th-order Runge-Kutta):** More stable, recommended for stiff springs
- **Euler:** Simpler but less stable

#### Multithreading
Physics computation is parallelized using `std::thread` for improved performance on multi-core systems. The `computeAcceleration()` function distributes work across available CPU cores.

---

### 2. Force Field

The simulation supports arbitrary 3D force fields loaded from world files. Forces are interpolated using **trilinear interpolation** for smooth force application between grid points.

- Force field spans -2 to +2 in each dimension
- Supports any grid resolution (specified in world file)
- Used for gravity, wind, and other external forces

---

### 3. Collision Detection and Response

#### Bounding Box
- Box spans -2 to +2 in x, y, z
- Penalty-based collision response using `kCollision` and `dCollision`
- Collision force: `F = kCollision * penetration * normal`
- Damping prevents oscillation at boundaries

#### Inclined Plane
- Supports arbitrary plane equation: `ax + by + cz + d = 0`
- Enabled when `incPlanePresent = 1` in world file
- Visual rendering of plane clipped to bounding box
- Same penalty-based response as bounding box

---

### 4. Interactive Mouse Dragging

**Feature:** Users can drag the jello cube in real-time using left-click + drag.

**Implementation:**
- Mouse movement is converted to 3D world-space movement
- Movement follows camera's view plane (intuitive left/right, up/down)
- All 512 points are translated equally
- Physics continues running during drag (cube deforms on wall collision)

**Technical Details:**
- Camera's right and up vectors computed from spherical coordinates (Phi, Theta)
- 2D mouse delta mapped to 3D delta using these vectors
- Sensitivity factor controls drag speed

```cpp
// Convert 2D mouse delta to 3D world movement
double rightX = -sin(Phi);
double rightY = cos(Phi);
double upX = -cos(Phi) * sin(Theta);
double upY = -sin(Phi) * sin(Theta);
double upZ = cos(Theta);

deltaX = (mouseX * rightX - mouseY * upX) * sensitivity;
deltaY = (mouseX * rightY - mouseY * upY) * sensitivity;
deltaZ = (mouseX * rightZ - mouseY * upZ) * sensitivity;
```

---

### 5. Rendering System

#### Wireframe Mode
- Displays spring network with color coding:
  - **Blue:** Structural springs
  - **Green:** Shear springs
  - **Red:** Bend springs
- Toggle individual spring types with `s`, `h`, `b` keys

#### Shaded Mode (Gouraud)
- Smooth-shaded triangles
- Vertex normals averaged from adjacent faces
- Supports procedural and image textures

#### Inclined Plane Visualization
- Semi-transparent polygon
- Automatically clipped to bounding box boundaries
- Rendered with outline for visibility

---

### 6. Custom Lighting

Implemented a **cinematic three-point lighting** setup:

| Light | Color | Position | Purpose |
|-------|-------|----------|---------|
| Key (0) | Warm orange/gold | (3, -2, 3) | Main illumination |
| Fill (1) | Cool blue | (-3, -1, 1) | Softens shadows |
| Rim (2) | Pale yellow | (0, 3, 2) | Edge definition |
| Under (3) | Subtle purple | (0, 0, -3) | Adds depth |

Material properties configured for red/orange jello appearance with specular highlights.

---

### 7. Procedural Textures

#### Fresnel Effect (Mode 1)
- View-dependent transparency effect
- Edges appear more opaque, center more transparent
- **Color changes on collision!** Cycles through 8 colors:
  - Red, Orange, Yellow, Green, Cyan, Blue, Purple, Pink

**Implementation:**
```cpp
double NdotV = dot(normal, viewDirection);
double fresnel = pow(1.0 - NdotV, 3.0);
// Blend between center color and edge highlight
color = baseColor + (1 - baseColor) * fresnel;
```

---

### 8. Image Texture Mapping

Two image textures are supported, loaded using **stb_image**:

| Mode | Texture | File |
|------|---------|------|
| 2 | Blue Swirl | `assets/swirls-paint-blue-liquid.jpg` |
| 3 | Wood | `assets/wood_table_diff_4k.jpg` |

**Features:**
- UV coordinates mapped to each face (0,0) to (1,1)
- Same texture applied to all 6 faces
- Automatic downsampling for large textures (max 1024x1024)
- Texture deforms naturally with jello cube

---

### 9. FPS Counter

Real-time frame rate display in top-left corner:
- Updated every 0.5 seconds for stability
- Uses `gettimeofday()` for timing
- Rendered using GLUT bitmap fonts

---

## World File Format

```
Line 1:     Integrator          "RK4" or "Euler"
Line 2:     dt n                timestep and render frequency
Line 3:     kElastic dElastic kCollision dCollision
Line 4:     mass                mass per point
Line 5:     incPlanePresent     0 or 1
Line 6:     a b c d             plane equation (if present)
Line 7:     resolution          force field grid size
Next N^3:   force vectors       (if resolution > 0)
Next 512:   positions           initial point positions
Next 512:   velocities          initial point velocities
```

---

## File Structure

| File | Description |
|------|-------------|
| `jello.cpp` | Main loop, display, lighting, FPS counter |
| `physics.cpp` | Spring forces, integration, collision detection |
| `showCube.cpp` | Rendering (wireframe, shaded, textures) |
| `input.cpp` | Keyboard/mouse handling, world file I/O |
| `procedural_textures.cpp` | Fresnel effect implementation |
| `texture_loader.cpp` | Image texture loading (stb_image) |
| `createWorld.cpp` | Utility to generate world files |

---

## Performance

- Achieves **40+ FPS** at 640x480 on modern hardware
- Multithreaded physics computation
- Texture downsampling prevents GPU memory issues

---

## Known Limitations

1. **Topology Inversion:** Under extreme deformation (aggressive dragging), the jello can become permanently deformed. This is a known limitation of mass-spring systems.

2. **No Jello-Jello Collision:** Multiple jello cubes would pass through each other (not implemented).

3. **OpenGL Deprecation:** Uses legacy OpenGL (fixed-function pipeline). Generates deprecation warnings on macOS but functions correctly.

---

## Extra Credit Features

1. **Interactive Mouse Dragging** - Drag the jello cube in real-time with physics still running

2. **Fresnel Procedural Texture** - View-dependent transparency with color change on collision

3. **Multiple Image Textures** - Two image textures (blue swirl, wood) with automatic downsampling

4. **Custom Cinematic Lighting** - Three-point lighting setup for visually appealing renders

5. **FPS Counter** - Real-time performance monitoring

6. **Inclined Plane Visualization** - Rendered with proper clipping to bounding box

7. **Multithreaded Physics** - Parallel computation using std::thread

---

## References

- CSCI 520 Course Materials
- OpenGL Programming Guide (Red Book)
- stb_image library: https://github.com/nothings/stb

---

## Author

Rajiv Murali
USC Viterbi School of Engineering
CSCI 520 - Computer Animation and Simulation
