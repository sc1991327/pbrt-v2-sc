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
    float lensRadius, lensRadiusSquare;			// 1
	float apertureRadius, apertureRadiusSquare;	// 4
	float refractiveIndex;						// 3
    float axisPos;								// 2
    
    float distToFilm;
	float centerInCameraAxis;
};

class RealisticCamera : public Camera {
public:
	RealisticCamera(const AnimatedTransform &cam2world,
		float hither, float yon, float sopen,
		float sclose, float filmdistance, float aperture_diameter, string specfilename,
		float filmdiag, Film *film);
    float GenerateRay(const CameraSample &sample, Ray *) const;
    Point RasterToCamera(Point &p) const;
    
private:
	// initial method in
	float filmDiagonal;
	float filmDistance;
	Film* film;

	// for transform
	float xCamFilmRes, yCamFilmRes;
	float filmPlaneZInCameraAxis;

	int numLens;
    std::vector<lensData> lens;

};

RealisticCamera* CreateRealisticCamera(const ParamSet &params,
	const AnimatedTransform &cam2world, Film *film);

#endif /* defined(__pbrt__realistic__) */
