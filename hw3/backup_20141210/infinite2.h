
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

#ifndef PBRT_LIGHTS_INFINITE2_H
#define PBRT_LIGHTS_INFINITE2_H

// lights/infinite.h*
#include "pbrt.h"
#include "light.h"
#include "texture.h"
#include "shape.h"
#include "scene.h"
#include "mipmap.h"

struct LightRegion
{
	LightRegion(int sx, int sy)
		: Start_x(sx), Start_y(sy){}
	LightRegion(int sx, int sy, int w, int h, float energy)
		: Start_x(sx), Start_y(sy), Width(w), Height(h), Energy(energy){}
	int Start_x;
	int Start_y;
	int Width;
	int Height;
	float Energy;
};

struct LightInfo
{
	LightInfo(float posx, float posy, float pdf, RGBSpectrum spectrum) 
		: Pos_x(Pos_x), Pos_y(posy), Pdf(pdf), Spectrum(spectrum){}
	float Pos_x;
	float Pos_y;
	float Pdf;
	RGBSpectrum Spectrum;
};

// InfiniteAreaLight Declarations
class EnvironmentLight : public Light {
public:
    // InfiniteAreaLight Public Methods
	/* hw3 - need to change 
		Do MedianCut algorithm to generate a set of lights with
		roughly equal energy
	*/
	EnvironmentLight(const Transform &light2world, const Spectrum &power, int ns,
        const string &texmap);
	~EnvironmentLight();

    Spectrum Power(const Scene *) const;			// the total of the power for the light
	Spectrum Le(const RayDifferential &r) const;	// input the direction, output the intensity
	/* hw3 - need to change
		Tell pbrt whether this light can be sampled or not? 
	*/
    bool IsDeltaLight() const { return true; }
    
	// get the high contribution direction for the Point
	/* hw3 - need to change 
		When pbrt asks a sample from environment light, uniformly
		select one from all lights and return its direction, intensity,
		and PDF
		- Pbrt only asks one sample at a time.
		- All lights have roughly equal energy.
		- We can simply random choose one from n lights and return PDF with 1/n
	*/
	Spectrum Sample_L(const Point &p, float pEpsilon, const LightSample &ls,
        float time, Vector *wi, float *pdf, VisibilityTester *visibility) const;
    Spectrum Sample_L(const Scene *scene, const LightSample &ls, float u1, float u2,
        float time, Ray *ray, Normal *Ns, float *pdf) const;

    float Pdf(const Point &, const Vector &) const;
    void SHProject(const Point &p, float pEpsilon, int lmax, const Scene *scene,
        bool computeLightVis, float time, RNG &rng, Spectrum *coeffs) const;
private:

    // InfiniteAreaLight Private Data
    MIPMap<RGBSpectrum> *radianceMap;
    Distribution2D *distribution;
	vector<LightInfo> lightInfo;
};


EnvironmentLight *CreateEnvironmentLight(const Transform &light2world,
        const ParamSet &paramSet);

#endif // PBRT_LIGHTS_INFINITE2_H
