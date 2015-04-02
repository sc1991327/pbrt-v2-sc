
/*
    pbrt source code Copyright(c) 1998-2012 Matt Pharr and Greg Humphreys.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


// main/pbrt.cpp*
#include "stdafx.h"
#include "api.h"
#include "probes.h"
#include "parser.h"
#include "parallel.h"

// main program
int main(int argc, char *argv[]) {
    Options options;
    vector<string> filenames;
    // Process command-line arguments
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--ncores")) options.nCores = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--outfile")) options.imageFile = argv[++i];
        else if (!strcmp(argv[i], "--quick")) options.quickRender = true;
        else if (!strcmp(argv[i], "--quiet")) options.quiet = true;
        else if (!strcmp(argv[i], "--verbose")) options.verbose = true;
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            printf("usage: pbrt [--ncores n] [--outfile filename] [--quick] [--quiet] "
                   "[--verbose] [--help] <filename.pbrt> ...\n");
            return 0;
        }
        else filenames.push_back(argv[i]);
    }

    // Print welcome banner
    if (!options.quiet) {
        printf("pbrt version %s of %s at %s [Detected %d core(s)]\n",
               PBRT_VERSION, __DATE__, __TIME__, NumSystemCores());
        printf("Copyright (c)1998-2014 Matt Pharr and Greg Humphreys.\n");
        printf("The source code to pbrt (but *not* the book contents) is covered by the BSD License.\n");
        printf("See the file LICENSE.txt for the conditions of the license.\n");
        fflush(stdout);
    }

	// INITIAL
    pbrtInit(options);
	// PROCESSING
    // Process scene description
    PBRT_STARTED_PARSING();
    if (filenames.size() == 0) {
        // Parse scene from standard input
        ParseFile("-");
    } else {
        // Parse scene from input files
        for (u_int i = 0; i < filenames.size(); i++)
            if (!ParseFile(filenames[i]))
                Error("Couldn't open scene file \"%s\"", filenames[i].c_str());
    }
	// CLEAN
    pbrtCleanup();
    return 0;
}

// PBRT WORKING PIPELINE:
/*
STEP 1: create a scene - scene.h/scene.cpp
	create primitive(aggregate), lights, (volumes), BBox : initial in makeScene()		- api.h/api.cpp

STEP 2: rendering - renderer.h/renderer/cpp; renders fonder are all implements
	all are virtual methods, makeRender() call implement object directly				- api.h/api/cpp
	input a scene, return an image or a set of measurement
	EXAMPLE: samplerRenderer:Renderer
		SamplerRenderer		: handle for an image(vector<Task>)
		SamplerRendererTask : handle for a single task

*/

// BASE GEOMETRY ARTHITECTURE - geometry.h/geometry.cpp
/*
Vector: public float x, y, z;
Point:	public float x, y, z;
Normal: public float x, y, z;
Ray:	public Point o; Vector d; mutable float mint,maxt; float time; int depth;
	RayDifferential: a useful ray
BBox:	public Point pmin, pmax;
*/

// BASE TRANSFROMATIONS - transform.h/transform.cpp
/*
struct Matrix4x4;
Transform: private Matrix4x4 m, mInv; -- this use to define all matrix transform methods
*/

// SHAPE -	shape.h/shape.cpp		
/*
Target And Role: input a ray and output intersect informations.
Primitive = Shape + Material
All are virtual methods
DIFFERENTIAL GEOMETRY:		- INFO STORE ARCHITECTURE		- diffgem.h/diffgem.cpp
	representation for a particular point on a surface
	target: all the other operators in pbrt can be executed without referring to the original shape.
EXAMPLES:
	1. sphere:shape			- sphere.h/sphere.cpp
	2. cylinder:shape		- cylinder.h/cylinder.cpp
	3. trianglemesh:shape	- trianglemesh.h/trianglemesh.cpp [very useful]
*/

// PRIMITIVE AND ACCELERATION - primitive.h/primitive.cpp
/*
Primitive = Shape + Material
INTERSECTION:				- INFO STORE ARCHITECTURE		- intersection.h/intersection.cpp
	include:DifferentialGeometry, primitive pointer, transform, shapeId, primitiveId, rayEpsilon.
			Get material methods.
			Le Method.
CLASSIFICATION:
	GeometricPrimitive:		General Geometry Objects 
		include: shape, material, light
		use Shape's methods to implement intersect and other functions
	TransformedPrimitive:	A large number of Duplicate Objects in the same scene.
		include: primitive, worldToPrimitive
		use Primitive's methods to implement intersect and other functions in local coordinate, then transform to world coordinate.
	Aggregate:				acceleration architecture.
		Acceleration is a heart component of a ray tracer because ray/scene intersection accounts for the majority of execution time.
		Goal: reduce the number of ray/primitive intersections by quick simultaneous rejection of groups of primitives
			and the fact that nearby intersections are likely to be found first.
		0. Base: Ray-Box intersection		- geometry.h/geometry.cpp
			BBox::IntersectP()
			return intersection or not(false/true), intersection points(near/far)
		1. GridAccel : Aggregate			- grid.h/grid.cpp
			Structure: Voxel: include vector<Reference<Primitive>> primitives
					   GridAccel: Implement by Voxel's intersect() and intersectP()
			Steps: create voxels, ray in and calculate intersect between ray and voxels one by one.
		2. BVHAccel : Aggregate				- bvh.h/bvh.cpp
			Build: dataPrepare[BVHPrimitiveInfo]; BVHTree[BVHBuildNode]; StoredBVHTree[LinearBVHNode];
		3. KdTreeAccel : Aggregate			- kdtreeaccel.h/kdtreeaccel.cpp
*/