
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

#ifndef PBRT_SHAPES_FASTHEIGHTFIELD_H
#define PBRT_SHAPES_FASTHEIGHTFIELD_H

// shapes/Fastheightfield.h*
#include "shape.h"
#include <map>
using std::map;

class HFTriangle;
struct HFVoxel;

// ------------ Fastheightfield ------------------------------------------------

// Fastheightfield Declarations
class Fastheightfield : public Shape {
public:
	// Fastheightfield Public Methods
	// Create Fast Height Field(o2,w2o,ro:extend shape, nu,nv:field size, zs:height info list)
	Fastheightfield(const Transform *o2, const Transform *w2o, bool ro, int nu, int nv, const float *zs);
	~Fastheightfield();

	bool CanIntersect() const { return true; }
	bool Intersect(const Ray &ray, float *tHit, float *rayEpsilon, DifferentialGeometry *dg) const;
	bool IntersectP(const Ray &ray) const;

	BBox ObjectBound() const;

	void GetUVs(float uv[3][2]) const;

	// BASE
	float *z;	// height data array
	int nx, ny;	// numbers of point position

	// DATAs
	int ntris, nverts;		// numbers
	// for basic triangle mesh
	int *vertexIndex;				// point index
	Point *p;						// point position
	Normal *n;						// normal
	Vector *s;						// tangent
	float *uvs;						// UV

};

Fastheightfield *CreateFastheightfieldShape(const Transform *o2w, const Transform *w2o,
        bool reverseOrientation, const ParamSet &params);

#endif // PBRT_SHAPES_FASTHEIGHTFIELD_H
