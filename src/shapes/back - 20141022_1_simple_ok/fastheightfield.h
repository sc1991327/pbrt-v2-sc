
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

// Fastheightfield Declarations
class Fastheightfield : public Shape {
public:
    // Fastheightfield Public Methods
	// Create Fast Height Field(o2,w2o,ro:extend shape, nu,nv:field size, zs:height info list)
	Fastheightfield(const Transform *o2, const Transform *w2o, bool ro, int nu, int nv, const float *zs);
	~Fastheightfield();

	bool CanIntersect() const { return false; }
	// Split a vector of triangles to calculate ray tracing.
    void Refine(vector<Reference<Shape> > &refined) const;
	//virtual bool Intersect(const Ray &ray, float *tHit,
	//	float *rayEpsilon, DifferentialGeometry *dg) const; // slow and get full intersect informations.
	//virtual bool IntersectP(const Ray &ray) const;			// fast and only get intersect position.

	BBox ObjectBound() const;

private:
	void createTriangles(const Transform *o2w, const Transform *w2o,
		bool reverseOrientation, const ParamSet &params,
		map<string, Reference<Texture<float> > > *floatTextures);
private:
    // Fastheightfield Private Data
    float *z;	// height data array
    int nx, ny;	// numbers of point position
};


// HeightfieldMesh Declarations
class HeightfieldMesh : public Shape {
public:
	// METHODs
	HeightfieldMesh(const Transform *o2w, const Transform *w2o, bool ro,
		int ntris, int nverts, const int *vptr,
		const Point *P, const float *uv);
	~HeightfieldMesh();

	BBox ObjectBound() const;
	BBox WorldBound() const;

	bool CanIntersect() const { return false; }
	void Refine(vector<Reference<Shape> > &refined) const;

	// DATAs
	int ntris, nverts;		// numbers
	// for basic triangle mesh
	int *vertexIndex;				// point index
	Point *p;						// point position
	Normal *n;						// normal
	Vector *s;						// tangent
	float *uvs;						// UV
};

// Handle Ray-Triangle Intersect
class HFTriangle : public Shape {
public:
	// HFTriangle Public Methods
	HFTriangle(const Transform *o2w, const Transform *w2o, bool ro, HeightfieldMesh *m, int n) : Shape(o2w, w2o, ro) {
		p1 = m->p[m->vertexIndex[3 * n]];
		p2 = m->p[m->vertexIndex[3 * n + 1]];
		p3 = m->p[m->vertexIndex[3 * n + 2]];
	}

	BBox ObjectBound() const;
	BBox WorldBound() const;

	bool Intersect(const Ray &ray, float *tHit, float *rayEpsilon, DifferentialGeometry *dg) const;
	bool IntersectP(const Ray &ray) const;

	void GetUVs(float uv[3][2]) const;

private:
	// HFTriangle Private Data
	Point p1, p2, p3;

};

Fastheightfield *CreateFastheightfieldShape(const Transform *o2w, const Transform *w2o,
        bool reverseOrientation, const ParamSet &params);

#endif // PBRT_SHAPES_FASTHEIGHTFIELD_H
