
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


// lights/infinite.cpp*
#include "stdafx.h"
#include "lights/infinite2.h"
#include "sh.h"
#include "montecarlo.h"
#include "paramset.h"
#include "imageio.h"

// InfiniteAreaLight Utility Classes
struct InfiniteAreaCube {
    // InfiniteAreaCube Public Methods
	InfiniteAreaCube(const EnvironmentLight *l, const Scene *s,
                     float t, bool cv, float pe)
        : light(l), scene(s), time(t), pEpsilon(pe), computeVis(cv) { }
    Spectrum operator()(int, int, const Point &p, const Vector &w) {
        Ray ray(p, w, pEpsilon, INFINITY, time);
        if (!computeVis || !scene->IntersectP(ray))	// can not intersect anything, return Le() result
            return light->Le(RayDifferential(ray));
        return 0.f;
    }
	const EnvironmentLight *light;
    const Scene *scene;
    float time, pEpsilon;
    bool computeVis;
};

// InfiniteAreaLight Method Definitions
EnvironmentLight::~EnvironmentLight() {
    delete distribution;
    delete radianceMap;
}


EnvironmentLight::EnvironmentLight(const Transform &light2world,
        const Spectrum &L, int ns, const string &texmap)
    : Light(light2world, ns) {
    
	// -- need: texmap
	// -- get: radianceMap(environment map), texels, width, height
	int width = 0, height = 0;
	RGBSpectrum *texels = NULL;
    if (texmap != "") {
        texels = ReadImage(texmap, &width, &height);
        if (texels)
            for (int i = 0; i < width * height; ++i)
                texels[i] *= L.ToRGBSpectrum();
    }
    if (!texels) {
        width = height = 1;
        texels = new RGBSpectrum[1];
        texels[0] = L.ToRGBSpectrum();
    }
	radianceMap = new MIPMap<RGBSpectrum>(width, height, texels);	/* -- The output data -- */
	Warning("Width: %i", width);
	Warning("Height: %i", height);

	// -- need: radianceMap, width, height
    // -- get: img(scalar-valued image)
	float SolidAngle = ((2.f * M_PI) / (width - 1)) * (M_PI / (1.f * (height - 1)));
    float filter = 1.f / max(width, height);
	float *img = new float[width*height];
    for (int v = 0; v < height; ++v) {
        float vp = (float)v / (float)height;
		float sinTheta = sinf(M_PI * float(v + 0.5f) / float(height));
        for (int u = 0; u < width; ++u) {
            float up = (float)u / (float)width;
			img[u + v*width] = radianceMap->Lookup(up, vp, filter).y();
			img[u + v*width] *= sinTheta;
			texels[u + v*width] *= (SolidAngle * sinTheta);
        }
    }

	// -- need: img, width, height
	// -- get: itgimg(integrate image)
	float *itgimg = new float[width*height];
	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			int pos_now = i * width + j;	
			int pos_up = (i - 1) * width + j;
			int pos_left = i * width + (j - 1);
			int pos_leftup = (i - 1) * width + (j - 1);
			if (i > 0 && j > 0){
				itgimg[pos_now] = itgimg[pos_left] + itgimg[pos_up] - itgimg[pos_leftup] + img[pos_now] * SolidAngle;
			}
			else{
				if (i==0 && j==0){
					itgimg[pos_now] = img[pos_now] * SolidAngle;
				}
				else if (j==0){
					itgimg[pos_now] = itgimg[pos_up] + img[pos_now] * SolidAngle;
				}
				else{
					itgimg[pos_now] = itgimg[pos_left] + img[pos_now] * SolidAngle;
				}
			}
		}
	}

	// -- initial median cut algorithm
	vector<LightRegion> lightRegion;
	float energy_all = itgimg[height * width - 1];
	LightRegion lr(0, 0, height - 1, width - 1, energy_all);
	lightRegion.push_back(lr);

	// -- do median cut algorithm
	for (int times = 0; times < 8; times++){		// median split times
		
		int ss = lightRegion.size();
		Warning("Size: %i", ss);
		for (int kkk = 0; kkk < ss; kkk++){
			Warning("Energy: %f", lightRegion[kkk].Energy);
		}
		vector<LightRegion> output_region;

		for (int n = 0; n < ss; n++){
			//float halfenergy = lightRegion[n].Energy / 2;
			int posLeftUp = lightRegion[n].Start_i * width + lightRegion[n].Start_j;
			int posLeft = (lightRegion[n].Start_i + lightRegion[n].Height) * width + lightRegion[n].Start_j;
			int posUp = lightRegion[n].Start_i * width + lightRegion[n].Start_j + lightRegion[n].Width;
			int posEnd = (lightRegion[n].Start_i + lightRegion[n].Height) * width + lightRegion[n].Start_j + lightRegion[n].Width;
			
			float halfenergy = (itgimg[posEnd] - itgimg[posLeft] - itgimg[posUp] + itgimg[posLeftUp]) / 2;
				/*if (itgimg[posEnd] - itgimg[posLeft] - itgimg[posUp] + itgimg[posLeftUp] < halfenergy){
					output_region.push_back(lightRegion[n]);
					}*/

			float worh = ((float)lightRegion[n].Width) / ((float)lightRegion[n].Height);
			Warning("worh: %i %i %i %i", lightRegion[n].Start_i, lightRegion[n].Start_j, lightRegion[n].Height, lightRegion[n].Width);
			if (worh > 1){
				// split by width
				// split medical cut for each region in lightRegion.
				for (int j = 0; j <= lightRegion[n].Width; j++){
					int nowpos = (lightRegion[n].Start_i + lightRegion[n].Height) * width + lightRegion[n].Start_j + j;
					float energynow = itgimg[nowpos] - itgimg[posLeft] - itgimg[posUp] + itgimg[posLeftUp];
					if (energynow >= halfenergy){
						LightRegion lr1(lightRegion[n].Start_i, lightRegion[n].Start_j, lightRegion[n].Height, j, energynow);
						LightRegion lr2(lightRegion[n].Start_i, lightRegion[n].Start_j + j, lightRegion[n].Height, lightRegion[n].Width - j, halfenergy * 2 - energynow);
						output_region.push_back(lr1);
						output_region.push_back(lr2);
						break;
					}
				}
			}
			else{
				// split by height
				// split medical cut for each region in lightRegion.
				for (int i = 0; i <= lightRegion[n].Height; i++){
					int nowpos = (lightRegion[n].Start_i + i) * width + lightRegion[n].Start_j + lightRegion[n].Width;
					float energynow = itgimg[nowpos] - itgimg[posLeft] - itgimg[posUp] + itgimg[posLeftUp];
					if (energynow >= halfenergy){
						LightRegion lr1(lightRegion[n].Start_i, lightRegion[n].Start_j, i, lightRegion[n].Width, energynow);
						LightRegion lr2(lightRegion[n].Start_i + i, lightRegion[n].Start_j, lightRegion[n].Height - (i), lightRegion[n].Width, halfenergy * 2 - energynow);
						output_region.push_back(lr1);
						output_region.push_back(lr2);
						break;
					}
				}
			}
		}

		lightRegion.clear();
		lightRegion = output_region;

	}

	// need: lightRegion
	// get: lightInfo
	int ss = lightRegion.size();
	float pdf = 1.f / ss;
	for (int n = 0; n < ss; n++){

		RGBSpectrum spectrum = RGBSpectrum(0.0f);
		for (int i = 0; i <= lightRegion[n].Height; i++){
			for (int J = 0; J <= lightRegion[n].Width; J++){
				int index = (lightRegion[n].Start_i + i)*width + lightRegion[n].Start_j + J;
				spectrum += texels[index];
			}
		}
		//float pdf = lightRegion[n].Energy / energy_all;
		float u_unit = 1.f / (width - 1);
		float v_unit = 1.f / (height - 1);
		lightInfo.push_back(LightInfo((float)(lightRegion[n].Start_i + lightRegion[n].Height / 2) * v_unit,
			(float)(lightRegion[n].Start_j + lightRegion[n].Width / 2) * u_unit, pdf, spectrum));

		Warning("Center: %f %f", (float)(lightRegion[n].Start_i + lightRegion[n].Height / 2) * v_unit, (float)(lightRegion[n].Start_j + lightRegion[n].Width / 2) * u_unit);

	}
	delete[] texels;

    // Compute sampling distributions for rows and columns of image
    distribution = new Distribution2D(img, width, height);

	/* -- delete all local data -- */
	delete[] img;
}



Spectrum EnvironmentLight::Power(const Scene *scene) const {
    Point worldCenter;
    float worldRadius;
    scene->WorldBound().BoundingSphere(&worldCenter, &worldRadius);
    return M_PI * worldRadius * worldRadius *
        Spectrum(radianceMap->Lookup(.5f, .5f, .5f), SPECTRUM_ILLUMINANT);
}


Spectrum EnvironmentLight::Le(const RayDifferential &r) const {
    Vector wh = Normalize(WorldToLight(r.d));
    float s = SphericalPhi(wh) * INV_TWOPI;
    float t = SphericalTheta(wh) * INV_PI;
    return Spectrum(radianceMap->Lookup(s, t), SPECTRUM_ILLUMINANT);
}


void EnvironmentLight::SHProject(const Point &p, float pEpsilon,
        int lmax, const Scene *scene, bool computeLightVis,
        float time, RNG &rng, Spectrum *coeffs) const {
    // Project _InfiniteAreaLight_ to SH using Monte Carlo if visibility needed
    if (computeLightVis) {
        Light::SHProject(p, pEpsilon, lmax, scene, computeLightVis,
                         time, rng, coeffs);
        return;
    }
    for (int i = 0; i < SHTerms(lmax); ++i)
        coeffs[i] = 0.f;
    int ntheta = radianceMap->Height(), nphi = radianceMap->Width();
    if (min(ntheta, nphi) > 50) {
        // Project _InfiniteAreaLight_ to SH from lat-long representation

        // Precompute $\theta$ and $\phi$ values for lat-long map projection
        float *buf = new float[2*ntheta + 2*nphi];
        float *bufp = buf;
        float *sintheta = bufp;  bufp += ntheta;
        float *costheta = bufp;  bufp += ntheta;
        float *sinphi = bufp;    bufp += nphi;
        float *cosphi = bufp;
        for (int theta = 0; theta < ntheta; ++theta) {
            sintheta[theta] = sinf((theta + .5f)/ntheta * M_PI);
            costheta[theta] = cosf((theta + .5f)/ntheta * M_PI);
        }
        for (int phi = 0; phi < nphi; ++phi) {
            sinphi[phi] = sinf((phi + .5f)/nphi * 2.f * M_PI);
            cosphi[phi] = cosf((phi + .5f)/nphi * 2.f * M_PI);
        }
        float *Ylm = ALLOCA(float, SHTerms(lmax));
        for (int theta = 0; theta < ntheta; ++theta) {
            for (int phi = 0; phi < nphi; ++phi) {
                // Add _InfiniteAreaLight_ texel's contribution to SH coefficients
                Vector w = Vector(sintheta[theta] * cosphi[phi],
                                  sintheta[theta] * sinphi[phi],
                                  costheta[theta]);
                w = Normalize(LightToWorld(w));
                Spectrum Le = Spectrum(radianceMap->Texel(0, phi, theta),
                                       SPECTRUM_ILLUMINANT);
                SHEvaluate(w, lmax, Ylm);
                for (int i = 0; i < SHTerms(lmax); ++i)
                    coeffs[i] += Le * Ylm[i] * sintheta[theta] *
                        (M_PI / ntheta) * (2.f * M_PI / nphi);
            }
        }

        // Free memory used for lat-long theta and phi values
        delete[] buf;
    }
    else {
        // Project _InfiniteAreaLight_ to SH from cube map sampling
        SHProjectCube(InfiniteAreaCube(this, scene, time, computeLightVis,
                                       pEpsilon),
                      p, 200, lmax, coeffs);
    }
}

/*
Here need to create:
1. *visibility, *wi -> by environment map position
2. Ls				-> calculate RGBSpectrum by envirtnment map
3. *pdf				-> float calculate by environment map
*/
Spectrum EnvironmentLight::Sample_L(const Point &p, float pEpsilon,
        const LightSample &ls, float time, Vector *wi, float *pdf,
        VisibilityTester *visibility) const {

    PBRT_INFINITE_LIGHT_STARTED_SAMPLE();
    // Find $(u,v)$ sample coordinates in infinite light texture
	int pos = Floor2Int(ls.uComponent * lightInfo.size());

	// Convert infinite light sample point to direction
	float theta = lightInfo[pos].Pos_y * M_PI;
	float phi = lightInfo[pos].Pos_x * 2.f * M_PI;
    float costheta = cosf(theta), sintheta = sinf(theta);
    float sinphi = sinf(phi), cosphi = cosf(phi);
    *wi = LightToWorld(Vector(sintheta * cosphi, sintheta * sinphi, costheta));

    // Compute PDF for sampled infinite light direction
    *pdf = lightInfo[pos].Pdf;

    // Return radiance value for infinite light direction
    visibility->SetRay(p, pEpsilon, *wi, time);
    Spectrum Ls = Spectrum(lightInfo[pos].Spectrum, SPECTRUM_ILLUMINANT);
    PBRT_INFINITE_LIGHT_FINISHED_SAMPLE();
    return Ls;
}


float EnvironmentLight::Pdf(const Point &, const Vector &w) const {
    PBRT_INFINITE_LIGHT_STARTED_PDF();
    Vector wi = WorldToLight(w);
    float theta = SphericalTheta(wi), phi = SphericalPhi(wi);
    float sintheta = sinf(theta);
    if (sintheta == 0.f) return 0.f;
    float p = distribution->Pdf(phi * INV_TWOPI, theta * INV_PI) /
           (2.f * M_PI * M_PI * sintheta);
    PBRT_INFINITE_LIGHT_FINISHED_PDF();
    return p;
}


Spectrum EnvironmentLight::Sample_L(const Scene *scene,
        const LightSample &ls, float u1, float u2, float time,
        Ray *ray, Normal *Ns, float *pdf) const {
    PBRT_INFINITE_LIGHT_STARTED_SAMPLE();
    // Compute direction for infinite light sample ray

    // Find $(u,v)$ sample coordinates in infinite light texture
	// - get the light vector in the world space
    float uv[2], mapPdf;
    distribution->SampleContinuous(ls.uPos[0], ls.uPos[1], uv, &mapPdf);
    if (mapPdf == 0.f) return Spectrum(0.f);

    float theta = uv[1] * M_PI, phi = uv[0] * 2.f * M_PI;
    float costheta = cosf(theta), sintheta = sinf(theta);
    float sinphi = sinf(phi), cosphi = cosf(phi);
    Vector d = -LightToWorld(Vector(sintheta * cosphi, sintheta * sinphi,
                                    costheta));
    *Ns = (Normal)d;

    // Compute origin for infinite light sample ray
    Point worldCenter;
    float worldRadius;
    scene->WorldBound().BoundingSphere(&worldCenter, &worldRadius);	// get the scene size.
    Vector v1, v2;
    CoordinateSystem(-d, &v1, &v2);	// create the coordinate system.
    float d1, d2;
    ConcentricSampleDisk(u1, u2, &d1, &d2);
    Point Pdisk = worldCenter + worldRadius * (d1 * v1 + d2 * v2);
    *ray = Ray(Pdisk + worldRadius * -d, d, 0., INFINITY, time);	// get the sample ray

    // Compute _InfiniteAreaLight_ ray PDF
    float directionPdf = mapPdf / (2.f * M_PI * M_PI * sintheta);	// the direction's contribution
    float areaPdf = 1.f / (M_PI * worldRadius * worldRadius);		// the contribution per-area
    *pdf = directionPdf * areaPdf;
    if (sintheta == 0.f) *pdf = 0.f;
    Spectrum Ls = (radianceMap->Lookup(uv[0], uv[1]), SPECTRUM_ILLUMINANT);
    PBRT_INFINITE_LIGHT_FINISHED_SAMPLE();
    return Ls;
}

// CREATE METHOD
EnvironmentLight *CreateEnvironmentLight(const Transform &light2world,
	const ParamSet &paramSet) {
	Spectrum L = paramSet.FindOneSpectrum("L", Spectrum(1.0));
	Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
	string texmap = paramSet.FindOneFilename("mapname", "");
	int nSamples = paramSet.FindOneInt("nsamples", 1);
	if (PbrtOptions.quickRender) nSamples = max(1, nSamples / 4);
	return new EnvironmentLight(light2world, L * sc, nSamples, texmap);
}