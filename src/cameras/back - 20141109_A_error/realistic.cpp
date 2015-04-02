
#include "stdafx.h"
#include "cameras/realistic.h"

#include <sstream>
#include <fstream>

// for split string
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

RealisticCamera::RealisticCamera(const AnimatedTransform &cam2world,
				 float hither, float yon, 
				 float sopen, float sclose, 
				 float fdis, float ad, string specfilename,
				 float fdag, Film *f)
	: Camera(cam2world, sopen, sclose, f), // pbrt-v2 does not specify hither and yon
	cliphiter(hither),
	clipyon(yon),
	shutteropen(sopen),
	shutterclose(sclose),
	film(f)
{
	filmdistance = fdis;	// the distance from film to the nearest lens

	// initial lens of data
	totallength = GetLensData(specfilename, filmdistance, lens);
	lensnum = lens.size();
	
	// for perspective
	GetTransformData(f, totallength, fdag, hither, yon,
		CameraToScreen, ScreenToRaster, RasterToCamera, RasterToScreen);

	Warning("totallenth: %f", totallength);
	Warning("lensnum: %i", lensnum);
	for (int i = 0; i < lensnum; i++){
		Warning("%f %f %f %f", lens[i].centerz, lens[i].radius, lens[i].nratio, lens[i].apradius);
	}

}

float RealisticCamera::GenerateRay(const CameraSample &sample, Ray *ray) const {

	// STEP 1 - generate first ray

	// use sample->imageX and sample->imageY to get raster-space coordinates of the sample point on the film.
	Point Pras(sample.imageX, sample.imageY, 0);
	Point Pcamera;
	RasterToCamera(Pras, &Pcamera);

	// use sample->lensU and sample->lensV to get a sample position on the lens
	float lensU, lensV;
	ConcentricSampleDisk(sample.lensU, sample.lensV, &lensU, &lensV);
	lensU *= lens[lensnum - 1].apradius;
	lensV *= lens[lensnum - 1].apradius;

	float pfz = lens[lensnum - 1].centerz + sqrtf(lens[lensnum - 1].radiusSquard - lensU * lensU - lensV * lensV);
	Point Pfocus = Point(lensU, lensV, pfz);
	Vector Rdir = Normalize(Pfocus - Pcamera);
	ray->o = Pcamera;
	ray->d = Rdir;
	ray->time = Lerp(sample.time, shutteropen, shutterclose);

	// STEP 2 - ray through the lens
	Point Plens;
	for (int i = 0; i < lensnum; i++) {

		lensData ls = lens[i];

		if ( ls.plane ){
			float t = (ls.centerz - ray->o.z) / ray->d.z;
			float x = ray->o.x + t * ray->d.x;
			float y = ray->o.y + t * ray->d.y;

			if (x * x + y * y > ls.apradiusSquared) 
				return 0.f;
		}
		else
		{
			// Find the intersection point
			Vector oc = ray->o - Point(0.f, 0.f, ls.centerz);
			float l1 = oc.LengthSquared();
			float rz = Dot(oc, ray->d);
			float l2 = rz *rz - l1 + ls.radiusSquard;
			if (l2 < 0.f)
				return 0.f;
			float t = (ls.radius > 0.f) ? (-rz + sqrtf(l2)) : (-rz - sqrtf(l2));
			
			Plens = ray->o + t * ray->d;
			if (Plens.x*Plens.x + Plens.y*Plens.y > ls.apradiusSquared) 
				return 0.f;

			// calculate the refraction 
			Vector N = (ls.radius > 0.f) ? Normalize(Point(0.f, 0.f, ls.centerz) - Plens)
				: Normalize(Plens - Point(0.f, 0.f, ls.centerz));
			Vector I = ray->d;
			float c1, c2;
			c1 = -Dot(I, N);
			c2 = ls.nratioSquared - 1.f + c1 * c1;
			if (c2 < 0.f) 	
				return 0.f;
			c2 = c1 - sqrtf(c2);
			Vector T = (I + c2 * N) / ls.nratio;

			ray->o = Plens;
			ray->d = Normalize(T);
		}
	}

	ray->mint = cliphiter;
	ray->maxt = (clipyon - cliphiter) / ray->d.z;

	// STEP 3 - output the ray and weight
	CameraToWorld(*ray, ray);
	
	float weight = Dot(Normalize(Plens - Pcamera), Vector(0, 0, 1));
	weight *= weight / totallength;
	weight *= weight * (lens[lensnum - 1].apradiusSquared * M_PI);
	return weight;

}

void RealisticCamera::GetTransformData(Film *f, float totallenth, 
	float filmdialog, float hither, float yon, 
	Transform &cameratoscreen, Transform &screentoraster, 
	Transform &rastertocamera, Transform &rastertoscreen){

	// Use filmdiag to calculate the sensor plane (in the real size)
	float diag = sqrtf(float(film->xResolution * film->xResolution + film->yResolution * film->yResolution));
	float rx = film->xResolution / (2 * diag);
	float ry = film->yResolution / (2 * diag);
	float screen[4];
	screen[0] = float(-filmdialog * rx);
	screen[1] = -screen[0];
	screen[2] = float(-filmdialog * ry);
	screen[3] = -screen[2];

	// - cameratoscreen
	cameratoscreen = Scale(1.f, 1.f, 1.f / (yon - hither)) * Translate(Vector(0.f, 0.f, totallenth));

	// - screentoraster
	screentoraster = Scale(float(f->xResolution), float(f->yResolution), 1.f) *
		Scale(1.f / (screen[1] - screen[0]), 1.f / (screen[2] - screen[3]), 1.f) *
		Translate(Vector(-screen[0], -screen[3], 0.f));

	// - rastertoscreen
	rastertoscreen = Inverse(screentoraster);

	// - rastertocamera
	rastertocamera = Inverse(cameratoscreen) * rastertoscreen;
}

float RealisticCamera::GetLensData(string specfile, float filmdistance,
	vector<lensData> &lens){

	vector<float> radius;	// lens radius
	vector<float> thick;	// near surface distance
	vector<float> nd;		// lens refraction
	vector<float> ap;		// lens diameter

	// open file and read line by line
	int num = 0;
	std::ifstream infile(specfile);
	std::string line;
	while (std::getline(infile, line))
	{
		std::vector<std::string> temp,elems;
		split(line, ' ', temp);

		if (temp[0] != "#"){

			split(line, '\t', elems);
			
			radius.push_back(std::stof(elems[0]));
			thick.push_back(std::stof(elems[1]));
			nd.push_back(std::stof(elems[2]));
			ap.push_back(std::stof(elems[3]));
			num++;
		}
	}

	// initial totallength
	thick[num - 1] = filmdistance;
	float totallength = 0;
	for (int i = 0; i < thick.size(); i++){
		totallength += thick[i];
	}

	// show data
	/*Warning("num: %i", num);
	Warning("radius thick nd ap");
	for (int i = 0; i < num; i++){
		Warning("%f %f %f %f", radius[i], thick[i], nd[i], ap[i]);
	}*/

	// initial lensData
	float total = -totallength;
	for (int i = num - 1; i >= 0; i--) {
		lensData ls;
		// radius, radiusSquard
		ls.radius = radius[i];
		ls.radiusSquard = ls.radius * ls.radius;
		// centerz
		// Note: lens position represented in camera space now, the camera direction: (0,0,-1)
		total += thick[i];
		ls.centerz = total - radius[i];
		// nratio, nratioSquared
		if (nd[i] > 0){
			ls.plane = false;
			ls.nratio = (i > 0 && nd[i - 1] != 0.f) ? (nd[i] / nd[i - 1]) : nd[i];
		}
		else{
			ls.plane = true;
			ls.nratio = 1.f;
		}
		ls.nratioSquared = ls.nratio * ls.nratio;
		// apradius, apradiusSquared
		ls.apradius = ap[i] * 0.5;
		ls.apradiusSquared = ls.apradius * ls.apradius;

		lens.push_back(ls);
	}

	radius.clear();
	thick.clear();
	nd.clear();
	ap.clear();

	return totallength;
}

RealisticCamera *CreateRealisticCamera(const ParamSet &params,
        const AnimatedTransform &cam2world, Film *film) {
	// Extract common camera parameters from \use{ParamSet}
	float hither = params.FindOneFloat("hither", -1);				
	float yon = params.FindOneFloat("yon", -1);						
	float shutteropen = params.FindOneFloat("shutteropen", -1);		
	float shutterclose = params.FindOneFloat("shutterclose", -1);	

	// Realistic camera-specific parameters
	string specfile = params.FindOneString("specfile", "");			
	float filmdistance = params.FindOneFloat("filmdistance", 70.0); 
 	float fstop = params.FindOneFloat("aperture_diameter", 1.0);
	float filmdiag = params.FindOneFloat("filmdiag", 35.0);			

	Assert(hither != -1 && yon != -1 && shutteropen != -1 &&
		shutterclose != -1 && filmdistance!= -1);
	if (specfile == "") {
	    Severe( "No lens spec file supplied!\n" );
	}
	return new RealisticCamera(cam2world, hither, yon,
				   shutteropen, shutterclose, filmdistance, fstop, 
				   specfile, filmdiag, film);
}
