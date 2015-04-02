
#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_CAMERAS_REALISTIC_H
#define PBRT_CAMERAS_REALISTIC_H

#include "camera.h"
#include "paramset.h"
#include "film.h"

struct lensData
{
	bool plane;
	float centerz;
	float radius, radiusSquard;
	float nratio, nratioSquared;
	float apradius, apradiusSquared;
};

// RealisticCamera Declarations
class RealisticCamera : public Camera {
public:
	// - RealisticCamera Public Methods
	RealisticCamera(const AnimatedTransform &cam2world,
						float hither, float yon, float sopen,
						float sclose, float filmdistance, float aperture_diameter, string specfilename,
						float filmdiag, Film *film);
	float GenerateRay(const CameraSample &sample, Ray *) const;
  
private:
	// - RealisticCamera private Methods
	float GetLensData(string specfile, float filmdistance,
		vector<lensData> &lens);	// return total length
	//void GetTransformData(Film *f, float totallenth, float filmdialog, 
	//	float hither, float yon, Transform &rastertocamera);

	Point RasterToCamera(Point &p) const;

	// - RealisticCamera private data
	float cliphiter;
	float clipyon;
	float shutteropen;
	float shutterclose;
	Film * film;

	float filmdistance;
	float totallength;

	// all lens
	int lensnum;
	vector<lensData> lens;

	// transform matrix.
	float xCamFilmRes;
	float yCamFilmRes;
	//Transform RasterToCamera;

};


RealisticCamera *CreateRealisticCamera(const ParamSet &params,
        const AnimatedTransform &cam2world, Film *film);


#endif	// PBRT_CAMERAS_REALISTIC_H