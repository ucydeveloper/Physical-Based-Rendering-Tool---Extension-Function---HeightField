
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

#ifndef PBRT_SHAPES_HEIGHTFIELD2_H
#define PBRT_SHAPES_HEIGHTFIELD2_H

// shapes/heightfield.h*
#include "shape.h"


//class Heightfield2 : public Shape {
//public:
//    // Heightfield Public Methods
//    Heightfield2(const Transform *o2w, const Transform *w2o, bool ro, int nu, int nv, const float *zs);
//    ~Heightfield2();
//    bool CanIntersect() const;
//    void Refine(vector<Reference<Shape> > &refined) const;
//    BBox ObjectBound() const;
//private:
//    // Heightfield Private Data
//    float *z;
//    int nx, ny;
//};



// Heightfield Declarations
class Heightfield2 : public Shape {
public:
	// Heightfield Public Methods
	Heightfield2(const Transform *o2w,const Transform *w2o, bool ro, int nu, int nv, const float *zs);
	~Heightfield2();
	bool CanIntersect() const;
	void Refine(vector<Reference<Shape> > &refined) const;
	BBox ObjectBound() const;
	bool Intersect(const Ray &ray, float *tHit, float *rayEpsilon, DifferentialGeometry *dg) const;
	bool IntersectP(const Ray &ray) const;
	bool RayTriangleIntersection(/*Output*/ DifferentialGeometry * dg, float * tHit, \
			/*Input*/const Ray &ray,const Point * P,  int index0, int index1, int index2 , const float * uvs ) const;
	bool RayTriangleIntersectionP(/*Input*/const Ray &ray,const Point * P,  int index0, int index1, int index2 , const float * uvs ) const;
	void GetShadingGeometry(const Transform &obj2world,
			const DifferentialGeometry &dg,
			DifferentialGeometry *dgShading) const;
	void initNormals();
	void getMinMaxZ_FromRay_XY(const Ray &r,float *rayEpsilon, int blockNoX, int blockNoY, float &flMinZ, float &flMaxZ) const;
private:
	// Heightfield Data
	float *z;
	float *TileMaxZ;
	float *TileMinZ;
	Vector * m_pNormal;
	
	Point *P;
	float *uvs;
	int *m_verts ;
	
	float stepX;
	float stepY;
	int nx, ny;
	int ntris;
	int nverts;
};



Heightfield2 *CreateHeightfield2Shape(const Transform *o2w, const Transform *w2o,
        bool reverseOrientation, const ParamSet &params);

#endif // PBRT_SHAPES_HEIGHTFIELD2_H
