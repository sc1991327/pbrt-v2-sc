
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

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_ACCELERATORS_GRID_H
#define PBRT_ACCELERATORS_GRID_H

// accelerators/grid.h*
#include "pbrt.h"
#include "primitive.h"

// ------ Handle Single Voxel ------

// GridAccel Forward Declarations
struct Voxel;

// Voxel Declarations
// -- Include multi-primitives and intersect one by one
struct Voxel {
    // Voxel Public Methods
    uint32_t size() const { return primitives.size(); }
    Voxel() { }
    Voxel(Reference<Primitive> op) {
        allCanIntersect = false;
        primitives.push_back(op);
    }
    void AddPrimitive(Reference<Primitive> prim) {
        primitives.push_back(prim);
    }

	// For each vector<Primitive>: canIntersect - call primitive's Intersect(); else - call primitive's Refine()
    bool Intersect(const Ray &ray, Intersection *isect, RWMutexLock &lock);
	// For each vector<Primitive>: canIntersect - call primitive's IntersectP(); else - call primitive's Refine()
    bool IntersectP(const Ray &ray, RWMutexLock &lock);
private:
    vector<Reference<Primitive> > primitives;	// all primitives in the voxel
    bool allCanIntersect;
};

// ------ Handle Scene (All Voxels) ------

// GridAccel Declarations
class GridAccel : public Aggregate {
public:
    // GridAccel Public Methods
	// create Voxels and it's primitives
    GridAccel(const vector<Reference<Primitive> > &p, bool refineImmediately);
    BBox WorldBound() const;
    bool CanIntersect() const { return true; }
    ~GridAccel();
	// ray in and find relation Voxels to calculate intersect, Inplement by Voxel->Intersect()
    bool Intersect(const Ray &ray, Intersection *isect) const;	
	// ray in and find relation Voxels to calculate intersect, Inplement by Voxel->IntersectP()
    bool IntersectP(const Ray &ray) const;
private:
    // GridAccel Private Methods
	// give primitive position to calculate it in which VoxelID.
    int posToVoxel(const Point &P, int axis) const {
        int v = Float2Int((P[axis] - bounds.pMin[axis]) *
                          invWidth[axis]);
        return Clamp(v, 0, nVoxels[axis]-1);
    }
	// give VoxelID to calculate primitive start position.
    float voxelToPos(int p, int axis) const {
        return bounds.pMin[axis] + p * width[axis];
    }
	// offset for use 1D array store 3D array data.
    inline int offset(int x, int y, int z) const {
        return z*nVoxels[0]*nVoxels[1] + y*nVoxels[0] + x;
    }

    // GridAccel Private Data
    vector<Reference<Primitive> > primitives;	// All primitives
    int nVoxels[3];				// Voxel Position: to decide which Voxel
    BBox bounds;				// Base BBox
    Vector width, invWidth;		// Voxel Width
    Voxel **voxels;				// Voxel Pointers' Array Pointer
    MemoryArena voxelArena;	
    mutable RWMutex *rwMutex;
};


GridAccel *CreateGridAccelerator(const vector<Reference<Primitive> > &prims,
        const ParamSet &ps);

#endif // PBRT_ACCELERATORS_GRID_H
