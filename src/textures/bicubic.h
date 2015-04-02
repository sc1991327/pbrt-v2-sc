
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

#ifndef PBRT_TEXTURES_BICUBIC_H
#define PBRT_TEXTURES_BICUBIC_H

// textures/bilerp.h*
#include "pbrt.h"
#include "texture.h"
#include "paramset.h"

// BicubicTexture Declarations
template <typename T> class BicubicTexture : public Texture<T> {
public:
    // BicubicTexture Public Methods
	BicubicTexture(TextureMapping2D *m, 
		const T &t00, const T &t01, const T &t02, const T &t03,
		const T &t10, const T &t11, const T &t12, const T &t13,
		const T &t20, const T &t21, const T &t22, const T &t23,
		const T &t30, const T &t31, const T &t32, const T &t33)
        : mapping(m), 
		v00(t00), v01(t01), v02(t02), v03(t03), 
		v10(t10), v11(t11), v12(t12), v13(t13), 
		v20(t20), v21(t21), v22(t22), v23(t23), 
		v30(t30), v31(t31), v32(t32), v33(t33){
    }
	~BicubicTexture() {
        delete mapping;
    }
    T Evaluate(const DifferentialGeometry &dg) const {
        float s, t, dsdx, dtdx, dsdy, dtdy;
        mapping->Map(dg, &s, &t, &dsdx, &dtdx, &dsdy, &dtdy);
        return v00 + 
			v10 * s + v01 * t + 
			v20 * s * s + v11 * s * t + v02 * t * t +
			v21 * s * s * t + v12 * s * t * t + 
			v22 * s * s * t * t + v30 * s * s * s + v03 * t * t * t + 
			v31 * s * s * s * t + v13 * t * t * t * s +
			v32 * s * s * s * t * t + v23 * s * s * t * t * t + 
			v33 * s * s * s * t * t * t;
    }
private:
    // BicubicTexture Private Data
    TextureMapping2D *mapping;
    T v00, v01, v02, v03;
	T v10, v11, v12, v13;
	T v20, v21, v22, v23;
	T v30, v31, v32, v33;
};


BicubicTexture<float> *CreateBicubicFloatTexture(const Transform &tex2world,
        const TextureParams &tp);
BicubicTexture<Spectrum> *CreateBicubicSpectrumTexture(const Transform &tex2world,
        const TextureParams &tp);

#endif // PBRT_TEXTURES_BICUBIC_H
