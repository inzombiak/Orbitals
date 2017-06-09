# Orbitals
WIP 3D Physics Engine

A 3D physics engine. ~~I got sidetracked by [MinkowskiSum](https://github.com/inzombiak/MinkowskiSum), but I hope to come back to this soon~~ 
Back on track!

## Controls:
  * WASD - move camera
  * Click and drag - rotate camera
  
## Current Features:
* 3D rendering
* Support for spheres and boxes
* Camera controls
* Broadphase collision detection using AABBs
* Narrowphase collision detection using ~~GJK/EPA~~(need to add manifodl generation) or SAT
* Contact manifold generation
* Collision resolution using seqential impulse solver
* Friction/Rrstitution

## Build Requirements:
GLM for math, GLEW 2.0 or higher for OpenGL

## References:
 * Online resources
   * General
     * Allen Chou, Game Physics Series - [allenchou.net](http://allenchou.net/game-physics-series/)
     * Jake Tyndall, 3D Convex Collision Detection - [hacktank.net](http://hacktank.net/blog/?p=93)
     * Erin Catto, Numerous publications - [box2d.org](http://box2d.org/downloads/)
     * Nilson Souto, Constrained Rigid Body Simulation - [toptal.com](https://www.toptal.com/game/video-game-physics-part-iii-constrained-rigid-body-simulation)
   * SAT Algorithm
      * Dirk Gregorius, The Seperating Axis Test Between Convex Polyhedra - [link](http://twvideo01.ubm-us.net/o1/vault/gdc2013/slides/822403Gregorius_Dirk_TheSeparatingAxisTest.pdf)
   * Collision Resolution
     * Randy Gaul, Modelling and Solving Physical Constraints - [tutsplus.com](https://gamedevelopment.tutsplus.com/tutorials/modelling-and-solving-physical-constraints--gamedev-12578) (use [QuickLaTeX](http://quicklatex.com/) if equations don't show)
     * Chappuis, Daniel. "Constraints derivation for rigid body simulation in 3D." (2013). -[link](https://pdfs.semanticscholar.org/8db7/334a726806d41c04f34bda382bb3e465e31f.pdf) 
     * Catto, Erin. "Iterative dynamics with temporal coherence." Game developer conference. Vol. 2. No. 4. 2005. -[link](https://pdfs.semanticscholar.org/30b2/74ff5e3ac27e5264e70441c7f94bfab18b7b.pdf)
   
 * Books
   * Ericson, Christer. *Real-time collision detection*. CRC Press, 2004.
   * Eberly, David H. *Game physics*. CRC Press, 2010.
   * van den Bergen, G. *Collision detection in interactive 3D environments*. 2004.
   
## See Also:
* [Minkowski sum calculator](https://github.com/inzombiak/MinkowskiSum)
* [GJK distance calculator](https://github.com/inzombiak/GJK)
