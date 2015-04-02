
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


// shapes/fastheightfield.cpp*
#include "stdafx.h"
#include "shapes/fastheightfield.h"
#include "texture.h"
#include "textures/constant.h"
#include "paramset.h"
#include "montecarlo.h"

// Fastheightfield Method Definitions
Fastheightfield::Fastheightfield(const Transform *o2w, const Transform *w2o,
        bool ro, int x, int y, const float *zs)
    : Shape(o2w, w2o, ro) {
    nx = x;
    ny = y;
    z = new float[nx*ny];
    memcpy(z, zs, nx*ny*sizeof(float));
}


Fastheightfield::~Fastheightfield() {
    delete[] z;
}

// the height field BBox
BBox Fastheightfield::ObjectBound() const {
    float minz = z[0], maxz = z[0];
    for (int i = 1; i < nx*ny; ++i) {
        if (z[i] < minz) minz = z[i];
        if (z[i] > maxz) maxz = z[i];
    }
    return BBox(Point(0,0,minz), Point(1,1,maxz));
}

// Height Field to Vector Triangles
void Fastheightfield::Refine(vector<Reference<Shape> > &refined) const {

    int ntris = 2*(nx-1)*(ny-1);	// the number of triangles output.
    refined.reserve(ntris);			// Requests refined is at least enough to contain n elements.
    int *verts = new int[3*ntris];		// use to create triangle mesh 1 - point index
    Point *P = new Point[nx*ny];		// use to create triangle mesh 2 - point position
    float *uvs = new float[2*nx*ny];	// use to create triangle mesh 3 - point UV
    int nverts = nx*ny;
    int x, y;
    // Compute Fastheightfield vertex positions
    int pos = 0;
    for (y = 0; y < ny; ++y) {
        for (x = 0; x < nx; ++x) {
            P[pos].x = uvs[2*pos]   = (float)x / (float)(nx-1);
            P[pos].y = uvs[2*pos+1] = (float)y / (float)(ny-1);
            P[pos].z = z[pos];
            ++pos;
        }
    }

    // Fill in Fastheightfield vertex offset array
    int *vp = verts;
    for (y = 0; y < ny-1; ++y) {
        for (x = 0; x < nx-1; ++x) {
#define VERT(x,y) ((x)+(y)*nx)
            *vp++ = VERT(x, y);
            *vp++ = VERT(x+1, y);
            *vp++ = VERT(x+1, y+1);
    
            *vp++ = VERT(x, y);
            *vp++ = VERT(x+1, y+1);
            *vp++ = VERT(x, y+1);
        }
#undef VERT
    }

	// Create Object Directly
	refined.push_back(new HeightfieldMesh(ObjectToWorld, WorldToObject, ReverseOrientation, ntris, nverts, verts, P, uvs));

    delete[] P;
    delete[] uvs;
    delete[] verts;
}


Fastheightfield *CreateFastheightfieldShape(const Transform *o2w, const Transform *w2o,
        bool reverseOrientation, const ParamSet &params) {
    int nu = params.FindOneInt("nu", -1);
    int nv = params.FindOneInt("nv", -1);
    int nitems;
    const float *Pz = params.FindFloat("Pz", &nitems);
    Assert(nitems == nu*nv);
    Assert(nu != -1 && nv != -1 && Pz != NULL);
	return new Fastheightfield(o2w, w2o, reverseOrientation, nu, nv, Pz);
}


// TriangleMesh Method Definitions
HeightfieldMesh::HeightfieldMesh(const Transform *o2w, const Transform *w2o,
	bool ro, int nt, int nv, const int *vi, const Point *P,
	const float *uv)
	: Shape(o2w, w2o, ro){
	
	ntris = nt;
	nverts = nv;
	vertexIndex = new int[3 * ntris];
	memcpy(vertexIndex, vi, 3 * ntris * sizeof(int));
	
	// Copy _uv_, _N_, and _S_ vertex data, if present
	uvs = new float[2 * nverts];
	memcpy(uvs, uv, 2 * nverts*sizeof(float));
	p = new Point[nverts];
	n = NULL;
	s = NULL;

	// Transform mesh vertices to world space
	for (int i = 0; i < nverts; ++i)
		p[i] = (*ObjectToWorld)(P[i]);
}

// ------------------ HeightfielldMesh --------------------------------------------------------------------------------------------

HeightfieldMesh::~HeightfieldMesh() {
	delete[] vertexIndex;
	delete[] p;
	delete[] s;
	delete[] n;
	delete[] uvs;
}


BBox HeightfieldMesh::ObjectBound() const {
	BBox objectBounds;
	for (int i = 0; i < nverts; i++)
		objectBounds = Union(objectBounds, (*WorldToObject)(p[i]));
	return objectBounds;
}


BBox HeightfieldMesh::WorldBound() const {
	BBox worldBounds;
	for (int i = 0; i < nverts; i++)
		worldBounds = Union(worldBounds, p[i]);
	return worldBounds;
}

void HeightfieldMesh::Refine(vector<Reference<Shape> > &refined) const {
	for (int i = 0; i < ntris; ++i)
		refined.push_back(new HFTriangle(ObjectToWorld,
		WorldToObject, ReverseOrientation,
		(HeightfieldMesh *)this, i));
}

// --------- HTTriangle -----------------------------------------------------------------------------------------------------

BBox HFTriangle::ObjectBound() const {
	// Get triangle vertices in _p1_, _p2_, and _p3_
	const Point &p1 = mesh->p[v[0]];
	const Point &p2 = mesh->p[v[1]];
	const Point &p3 = mesh->p[v[2]];
	return Union(BBox((*WorldToObject)(p1), (*WorldToObject)(p2)),
		(*WorldToObject)(p3));
}


BBox HFTriangle::WorldBound() const {
	// Get triangle vertices in _p1_, _p2_, and _p3_
	const Point &p1 = mesh->p[v[0]];
	const Point &p2 = mesh->p[v[1]];
	const Point &p3 = mesh->p[v[2]];
	return Union(BBox(p1, p2), p3);
}


bool HFTriangle::Intersect(const Ray &ray, float *tHit, float *rayEpsilon,
	DifferentialGeometry *dg) const {
	PBRT_RAY_TRIANGLE_INTERSECTION_TEST(const_cast<Ray *>(&ray), const_cast<Triangle *>(this));
	// Compute $\VEC{s}_1$

	// Get triangle vertices in _p1_, _p2_, and _p3_
	// Triangle points
	const Point &p1 = mesh->p[v[0]];
	const Point &p2 = mesh->p[v[1]];
	const Point &p3 = mesh->p[v[2]];
	// Triangle edges
	Vector e1 = p2 - p1;
	Vector e2 = p3 - p1;
	Vector s1 = Cross(ray.d, e2);
	float divisor = Dot(s1, e1);

	if (divisor == 0.)
		return false;
	float invDivisor = 1.f / divisor;

	// Compute first barycentric coordinate
	Vector s = ray.o - p1;
	float b1 = Dot(s, s1) * invDivisor;
	if (b1 < 0. || b1 > 1.)
		return false;

	// Compute second barycentric coordinate
	Vector s2 = Cross(s, e1);
	float b2 = Dot(ray.d, s2) * invDivisor;
	if (b2 < 0. || b1 + b2 > 1.)
		return false;

	// Compute _t_ to intersection point
	float t = Dot(e2, s2) * invDivisor;
	if (t < ray.mint || t > ray.maxt)
		return false;

	// Compute triangle partial derivatives
	Vector dpdu, dpdv;
	float uvs[3][2];
	GetUVs(uvs);

	// Compute deltas for triangle partial derivatives
	float du1 = uvs[0][0] - uvs[2][0];
	float du2 = uvs[1][0] - uvs[2][0];
	float dv1 = uvs[0][1] - uvs[2][1];
	float dv2 = uvs[1][1] - uvs[2][1];
	Vector dp1 = p1 - p3, dp2 = p2 - p3;
	float determinant = du1 * dv2 - dv1 * du2;
	if (determinant == 0.f) {
		// Handle zero determinant for triangle partial derivative matrix
		CoordinateSystem(Normalize(Cross(e2, e1)), &dpdu, &dpdv);
	}
	else {
		float invdet = 1.f / determinant;
		dpdu = (dv2 * dp1 - dv1 * dp2) * invdet;
		dpdv = (-du2 * dp1 + du1 * dp2) * invdet;
	}

	// Interpolate $(u,v)$ triangle parametric coordinates
	float b0 = 1 - b1 - b2;
	float tu = b0*uvs[0][0] + b1*uvs[1][0] + b2*uvs[2][0];
	float tv = b0*uvs[0][1] + b1*uvs[1][1] + b2*uvs[2][1];

	// Fill in _DifferentialGeometry_ from triangle hit
	*dg = DifferentialGeometry(ray(t), dpdu, dpdv,
		Normal(0, 0, 0), Normal(0, 0, 0),
		tu, tv, this);
	*tHit = t;
	*rayEpsilon = 1e-3f * *tHit;
	PBRT_RAY_TRIANGLE_INTERSECTION_HIT(const_cast<Ray *>(&ray), t);
	return true;
}


bool HFTriangle::IntersectP(const Ray &ray) const {
	PBRT_RAY_TRIANGLE_INTERSECTIONP_TEST(const_cast<Ray *>(&ray), const_cast<Triangle *>(this));
	// Compute $\VEC{s}_1$

	// Get triangle vertices in _p1_, _p2_, and _p3_
	const Point &p1 = mesh->p[v[0]];
	const Point &p2 = mesh->p[v[1]];
	const Point &p3 = mesh->p[v[2]];
	Vector e1 = p2 - p1;
	Vector e2 = p3 - p1;
	Vector s1 = Cross(ray.d, e2);
	float divisor = Dot(s1, e1);

	if (divisor == 0.)
		return false;
	float invDivisor = 1.f / divisor;

	// Compute first barycentric coordinate
	Vector d = ray.o - p1;
	float b1 = Dot(d, s1) * invDivisor;
	if (b1 < 0. || b1 > 1.)
		return false;

	// Compute second barycentric coordinate
	Vector s2 = Cross(d, e1);
	float b2 = Dot(ray.d, s2) * invDivisor;
	if (b2 < 0. || b1 + b2 > 1.)
		return false;

	// Compute _t_ to intersection point
	float t = Dot(e2, s2) * invDivisor;
	if (t < ray.mint || t > ray.maxt)
		return false;

	PBRT_RAY_TRIANGLE_INTERSECTIONP_HIT(const_cast<Ray *>(&ray), t);
	return true;
}


float HFTriangle::Area() const {
	// Get triangle vertices in _p1_, _p2_, and _p3_
	const Point &p1 = mesh->p[v[0]];
	const Point &p2 = mesh->p[v[1]];
	const Point &p3 = mesh->p[v[2]];
	return 0.5f * Cross(p2 - p1, p3 - p1).Length();
}


void HFTriangle::GetShadingGeometry(const Transform &obj2world,
	const DifferentialGeometry &dg,
	DifferentialGeometry *dgShading) const {
	if (!mesh->n && !mesh->s) {
		*dgShading = dg;
		return;
	}
	// Initialize _Triangle_ shading geometry with _n_ and _s_

	// Compute barycentric coordinates for point
	float b[3];

	// Initialize _A_ and _C_ matrices for barycentrics
	float uv[3][2];
	GetUVs(uv);
	float A[2][2] =
	{ { uv[1][0] - uv[0][0], uv[2][0] - uv[0][0] },
	{ uv[1][1] - uv[0][1], uv[2][1] - uv[0][1] } };
	float C[2] = { dg.u - uv[0][0], dg.v - uv[0][1] };
	if (!SolveLinearSystem2x2(A, C, &b[1], &b[2])) {
		// Handle degenerate parametric mapping
		b[0] = b[1] = b[2] = 1.f / 3.f;
	}
	else
		b[0] = 1.f - b[1] - b[2];

	// Use _n_ and _s_ to compute shading tangents for triangle, _ss_ and _ts_
	Normal ns;
	Vector ss, ts;
	if (mesh->n) ns = Normalize(obj2world(b[0] * mesh->n[v[0]] +
		b[1] * mesh->n[v[1]] +
		b[2] * mesh->n[v[2]]));
	else   ns = dg.nn;
	if (mesh->s) ss = Normalize(obj2world(b[0] * mesh->s[v[0]] +
		b[1] * mesh->s[v[1]] +
		b[2] * mesh->s[v[2]]));
	else   ss = Normalize(dg.dpdu);

	ts = Cross(ss, ns);
	if (ts.LengthSquared() > 0.f) {
		ts = Normalize(ts);
		ss = Cross(ts, ns);
	}
	else
		CoordinateSystem((Vector)ns, &ss, &ts);
	Normal dndu, dndv;

	// Compute $\dndu$ and $\dndv$ for triangle shading geometry
	if (mesh->n) {
		float uvs[3][2];
		GetUVs(uvs);
		// Compute deltas for triangle partial derivatives of normal
		float du1 = uvs[0][0] - uvs[2][0];
		float du2 = uvs[1][0] - uvs[2][0];
		float dv1 = uvs[0][1] - uvs[2][1];
		float dv2 = uvs[1][1] - uvs[2][1];
		Normal dn1 = mesh->n[v[0]] - mesh->n[v[2]];
		Normal dn2 = mesh->n[v[1]] - mesh->n[v[2]];
		float determinant = du1 * dv2 - dv1 * du2;
		if (determinant == 0.f)
			dndu = dndv = Normal(0, 0, 0);
		else {
			float invdet = 1.f / determinant;
			dndu = (dv2 * dn1 - dv1 * dn2) * invdet;
			dndv = (-du2 * dn1 + du1 * dn2) * invdet;
		}
	}
	else
		dndu = dndv = Normal(0, 0, 0);
	*dgShading = DifferentialGeometry(dg.p, ss, ts,
		obj2world(dndu), obj2world(dndv),
		dg.u, dg.v, dg.shape);
	dgShading->dudx = dg.dudx;  dgShading->dvdx = dg.dvdx;
	dgShading->dudy = dg.dudy;  dgShading->dvdy = dg.dvdy;
	dgShading->dpdx = dg.dpdx;  dgShading->dpdy = dg.dpdy;
}


Point HFTriangle::Sample(float u1, float u2, Normal *Ns) const {
	float b1, b2;
	UniformSampleTriangle(u1, u2, &b1, &b2);
	// Get triangle vertices in _p1_, _p2_, and _p3_
	const Point &p1 = mesh->p[v[0]];
	const Point &p2 = mesh->p[v[1]];
	const Point &p3 = mesh->p[v[2]];
	Point p = b1 * p1 + b2 * p2 + (1.f - b1 - b2) * p3;
	Normal n = Normal(Cross(p2 - p1, p3 - p1));
	*Ns = Normalize(n);
	if (ReverseOrientation) *Ns *= -1.f;
	return p;
}
