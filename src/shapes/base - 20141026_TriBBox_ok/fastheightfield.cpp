
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
        bool ro, int xin, int yin, const float *zs)
    : Shape(o2w, w2o, ro) {
    nx = xin;
    ny = yin;
    z = new float[nx*ny];
    memcpy(z, zs, nx*ny*sizeof(float));

	ntris = 2 * (nx - 1)*(ny - 1);
	nverts = nx*ny;
	ngrid = ny - 1;
	vertexIndex = new int[3 * ntris];
	p = new Point[nx*ny];
	n = new Normal[nx*ny];
	s = new Vector[nx*ny];
	uvs = new float[2 * nx*ny];
	triangleworldbound = new BBox[ntris];
	boxworldbound = new BBox[(nx - 1)*(ny - 1)];
	gridworldbound = new BBox[ngrid];
	int x, y;

	// Compute Fastheightfield vertex positions
	int pos = 0;
	for (y = 0; y < ny; ++y) {
		for (x = 0; x < nx; ++x) {
			p[pos].x = uvs[2 * pos] = (float)x / (float)(nx - 1);
			p[pos].y = uvs[2 * pos + 1] = (float)y / (float)(ny - 1);
			p[pos].z = z[pos];
			n[pos] = Normal(0, 0, 0);
			++pos;
		}
	}

	// Fill in Fastheightfield vertex offset array
	int *vp = vertexIndex;
	int index0, index1, index2;
	Vector p0, p1, p2;
	Normal *sn = new Normal[ntris];
	int tri_counter = 0;
	for (y = 0; y < ny - 1; ++y) {
		for (x = 0; x < nx - 1; ++x) {
#define VERT(x,y) ((x)+(y)*nx)
			*vp++ = index0 = VERT(x, y);
			*vp++ = index1 = VERT(x + 1, y);
			*vp++ = index2 = VERT(x + 1, y + 1);

			p0 = (Vector)p[index0];
			p1 = (Vector)p[index1];
			p2 = (Vector)p[index2];
			sn[tri_counter] = (Normal)Normalize(Cross((p0 - p2), (p1 - p2)));
			n[index0] += sn[tri_counter];
			n[index1] += sn[tri_counter];
			n[index2] += sn[tri_counter];

			triangleworldbound[tri_counter] = Union(BBox((*ObjectToWorld)(p[index0]), (*ObjectToWorld)(p[index1])), (*ObjectToWorld)(p[index2]));
			tri_counter++;

			*vp++ = index0 = VERT(x, y);
			*vp++ = index1 = VERT(x + 1, y + 1);
			*vp++ = index2 = VERT(x, y + 1);

			p0 = (Vector)p[index0];
			p1 = (Vector)p[index1];
			p2 = (Vector)p[index2];
			sn[tri_counter] = (Normal)Normalize(Cross((p0 - p2), (p1 - p2)));
			n[index0] += sn[tri_counter];
			n[index1] += sn[tri_counter];
			n[index2] += sn[tri_counter];

			triangleworldbound[tri_counter] = Union(BBox((*ObjectToWorld)(p[index0]), (*ObjectToWorld)(p[index1])), (*ObjectToWorld)(p[index2]));
			tri_counter++;

			boxworldbound[tri_counter / 2 - 1] = Union(triangleworldbound[tri_counter - 2], triangleworldbound[tri_counter - 1]);

			gridworldbound[y] = Union(gridworldbound[y], boxworldbound[tri_counter / 2 - 1]);
		}
#undef VERT
	}

	pos = 0;
	for (y = 0; y < ny; ++y) {
		for (x = 0; x < nx; ++x) {
			n[pos] = Normalize(n[pos]);
			pos++;
		}
	}
	delete[] sn;

	// Transform mesh vertices to world space
	for (int i = 0; i < nverts; ++i)
		p[i] = (*ObjectToWorld)(p[i]);

	//// calculate bbox
	//int gx = 0, gy = 0;
	//for (int i = 0; i < ntris / 2; ++i)
	//{
	//	Point p1 = p[vertexIndex[6 * i]];
	//	Point p2 = p[vertexIndex[6 * i + 1]];
	//	Point p3 = p[vertexIndex[6 * i + 2]];
	//	triangleworldbound[2 * i] = Union(BBox(p1, p2), p3);

	//	Point p4 = p[vertexIndex[6 * i + 3]];
	//	Point p5 = p[vertexIndex[6 * i + 4]];
	//	Point p6 = p[vertexIndex[6 * i + 5]];
	//	triangleworldbound[2 * i + 1] = Union(BBox(p4, p5), p6);

	//	boxworldbound[i] = Union(triangleworldbound[2 * i], triangleworldbound[2 * i + 1]);		

	//}

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

bool Fastheightfield::Intersect(const Ray &ray, float *tHit, float *rayEpsilon, DifferentialGeometry *dg) const{

	// STEP 0 : check bound and ray

	for (int i = 0; i < ntris; ++i)
	{
		if ( !triangleworldbound[i].IntersectP(ray) ){
			continue;
		}

		// STEP 1: get data
		int itri = i;

		Point p1 = p[vertexIndex[3 * itri]];
		Point p2 = p[vertexIndex[3 * itri + 1]];
		Point p3 = p[vertexIndex[3 * itri + 2]];

		// Get triangle vertices in _p1_, _p2_, and _p3_
		// Triangle edges
		Vector e1 = p2 - p1;					// E1
		Vector e2 = p3 - p1;					// E2
		Vector s1 = Cross(ray.d, e2);			// P
		float divisor = Dot(s1, e1);			// P * E1
		if (divisor == 0.)
			continue;
		float invDivisor = 1.f / divisor;		// 1/(P*E1)

		// STEP 2: check intersect

		// Compute first barycentric coordinate
		Vector s = ray.o - p1;					// T
		float b1 = Dot(s, s1) * invDivisor;		// u = (T * P) / (P * E1)
		if (b1 < 0. || b1 > 1.)
			continue;
		// Compute second barycentric coordinate
		Vector s2 = Cross(s, e1);				// Q
		float b2 = Dot(ray.d, s2) * invDivisor;	// v = (Q * D) / (P * E1)
		if (b2 < 0. || b1 + b2 > 1.)
			continue;
		// Compute _t_ to intersection point
		float t = Dot(e2, s2) * invDivisor;		// (Q * E2) / (P * E1)
		if (t < ray.mint || t > ray.maxt)
			continue;

		// STEP 3: store info

		// Compute triangle partial derivatives
		Vector dpdu, dpdv;
		float uv[3][2];
		uv[0][0] = uvs[vertexIndex[3 * itri]];
		uv[0][1] = uvs[vertexIndex[3 * itri] + 1];
		uv[1][0] = uvs[vertexIndex[3 * itri + 1]];
		uv[1][1] = uvs[vertexIndex[3 * itri + 1] + 1];
		uv[2][0] = uvs[vertexIndex[3 * itri + 2]];
		uv[2][1] = uvs[vertexIndex[3 * itri + 2] + 1];
		// Interpolate $(u,v)$ triangle parametric coordinates
		float b0 = 1 - b1 - b2;
		float tu = b0*uv[0][0] + b1*uv[1][0] + b2*uv[2][0];
		float tv = b0*uv[0][1] + b1*uv[1][1] + b2*uv[2][1];

		Normal n0 = Normalize(n[vertexIndex[3 * itri]]);
		Normal n1 = Normalize(n[vertexIndex[3 * itri + 1]]);
		Normal n2 = Normalize(n[vertexIndex[3 * itri + 2]]);
		Vector avg_n = (Vector)(n0 * b0 + n1 * b1 + n2 * b2);

		Vector temp_t(-avg_n.y, avg_n.x, 0);
		Vector temp_s(1, 0, 0);
		avg_n = Normalize(avg_n);
		temp_s = Normalize(Cross(avg_n, temp_t));

		dpdu = temp_t;
		dpdv = temp_s;

		Normal dndu = Normal(0, 0, 0);
		Normal dndv = Normal(0, 0, 0);

		// Fill in _DifferentialGeometry_ from triangle hit
		*dg = DifferentialGeometry(ray(t), (*ObjectToWorld)(dpdu), (*ObjectToWorld)(dpdv), (*ObjectToWorld)(dndu), (*ObjectToWorld)(dndv), tu, tv, this);
		*tHit = t;
		*rayEpsilon = 1e-3f * *tHit;

		return true;

	}

	return false;

}

bool Fastheightfield::IntersectP(const Ray &ray) const{

	for (int i = 0; i < ntris; ++i)
	{
		// STEP 1: get data

		Point p1 = p[vertexIndex[3 * i]];
		Point p2 = p[vertexIndex[3 * i + 1]];
		Point p3 = p[vertexIndex[3 * i + 2]];
		// Get triangle vertices in _p1_, _p2_, and _p3_
		// Triangle edges
		Vector e1 = p2 - p1;
		Vector e2 = p3 - p1;
		Vector s1 = Cross(ray.d, e2);
		float divisor = Dot(s1, e1);
		if (divisor == 0.)
			continue;
		float invDivisor = 1.f / divisor;

		// STEP 2: check intersect

		// Compute first barycentric coordinate
		Vector s = ray.o - p1;
		float b1 = Dot(s, s1) * invDivisor;
		if (b1 < 0. || b1 > 1.)
			continue;
		// Compute second barycentric coordinate
		Vector s2 = Cross(s, e1);
		float b2 = Dot(ray.d, s2) * invDivisor;
		if (b2 < 0. || b1 + b2 > 1.)
			continue;
		// Compute _t_ to intersection point
		float t = Dot(e2, s2) * invDivisor;
		if (t < ray.mint || t > ray.maxt)
			continue;

		return true;
	}

	return false;

}

// ----------------------------------------------------------------------------------

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