
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


// shapes/heightfield2.cpp*
#include "stdafx.h"
#include "shapes/heightfield2.h"
#include "shapes/trianglemesh.h"
#include "paramset.h"
#define RAY_EPSILON 0

 //Heightfield Method Definitions
Heightfield2::Heightfield2(const Transform *o2w, const Transform *w2o,
        bool ro, int x, int y, const float *zs)
    : Shape(o2w, w2o, ro) {
	nx = x;
	ny = y;
	z = new float[nx*ny];				//(nx * ny) Z Values, now it is in z now!
	m_pNormal = new Vector[nx*ny];

	//20. Input
	ntris = 2*(nx-1)*(ny-1);	
	nverts = nx*ny;
	stepX = 1.0 / (nx-1);		//4x4 means 0.33 0.66  1
	stepY = 1.0 / (ny-1); 

	//Tile No. is just ntris/2 = 2*(nx-1)*(ny-1)/2 = (nx-1)*(ny-1)
	TileMaxZ = new float [(nx-1)*(ny-1)];   //TileMaxZ [y * (nx-1) + x]  x,y: 0--nx-2
	TileMinZ = new float [(nx-1)*(ny-1)];	 //TileMinZ [y * (nx-1) + x]  x,y: 0--nx-2

	P = new Point[nx*ny];	
	uvs = new float[2*nx*ny];
	m_verts = new int[3*ntris];	//This is index, verts[0..3*ntris] = [0..nx*ny];

	
	memcpy(z, zs, nx*ny*sizeof(float));
	initNormals();
}

Heightfield2::~Heightfield2() {
	delete[] z;
	delete[] m_pNormal;

	delete[] TileMaxZ;
	delete[] TileMinZ;

	delete [] m_verts;
	delete [] P;
	delete [] uvs;	
}

void Heightfield2::initNormals() 
{		
	//10 Compute heightfield vertex positions
	Vector * m_pAllSurfaceNormal;
	m_pAllSurfaceNormal = new Vector [ntris];								//6*(nx-1)*(ny-1) > nx* ny

	int x, y;
	int pos = 0;
	for(y = 0; y < ny; ++y) {
		for (x = 0; x < nx; ++x) {
			P[pos].x = uvs[2*pos]   = (float)x / (float)(nx-1);
			P[pos].y = uvs[2*pos+1] = (float)y / (float)(ny-1);
			P[pos].z = z[pos];
			assert(pos == y * nx + x);		//[ y * nx + x]
			m_pNormal[pos] = Vector(0,0,0);
			++pos;
			
		}
	}
	//20 Build the Face's Normal Information.	
	int * vp = m_verts; 
	int index0, index1, index2, index3; 
	Vector p0, p1, p2, p3;
	int tri_counter = 0; 
	for(y = 0; y < ny-1; ++y) {
		for (x = 0; x < nx-1; ++x) {
	#define VERT(x,y) ((x)+(y)*nx)
			*vp++ = index0 = VERT(x, y);
			*vp++ = index1 = VERT(x+1, y);
			*vp++ = index2 = VERT(x+1, y+1);
				
				
			p0 = (Vector) P[index0];
			p1 = (Vector) P[index1];
			p2 = (Vector) P[index2];
			m_pAllSurfaceNormal[tri_counter] = Normalize(Cross ( (p1 - p2), (p0 - p2) ) );
			m_pNormal[index0] += m_pAllSurfaceNormal[tri_counter];
			m_pNormal[index1] += m_pAllSurfaceNormal[tri_counter];
			m_pNormal[index2] += m_pAllSurfaceNormal[tri_counter];

			tri_counter++;

			*vp++ = index0 = VERT(x, y);
			*vp++ = index1 = VERT(x+1, y+1);
			*vp++ = index2 = VERT(x, y+1);

			/*tri_no = 2 * block_no + 1;*/
			p0 = (Vector) P[index0];
			p1 = (Vector) P[index1];
			p2 = (Vector) P[index2];
			m_pAllSurfaceNormal[tri_counter] = Normalize(Cross ( (p1 - p2), (p0 - p2) ) );//©Ç©Çªº
			
			m_pNormal[index0] += m_pAllSurfaceNormal[tri_counter];
			m_pNormal[index1] += m_pAllSurfaceNormal[tri_counter];
			m_pNormal[index2] += m_pAllSurfaceNormal[tri_counter];

			tri_counter++;

			index0 = VERT(x, y);
			index1 = VERT(x+1, y);
			index2 = VERT(x+1, y+1);
			index3 = VERT(x, y+1);
			p0 = (Vector) P[index0];
			p1 = (Vector) P[index1];
			p2 = (Vector) P[index2];
			p3 = (Vector) P[index3];

			int tile_counter = tri_counter/2 - 1 ;
			TileMaxZ[tile_counter] = max(max(p0.z, p3.z), max(p2.z, p1.z) );
			TileMinZ[tile_counter] = min(min(p0.z, p3.z), min(p2.z, p1.z) );
			assert(tile_counter >= 0 && tile_counter < ( (nx-1)*(ny-1) ) );

		}
	#undef VERT
	}
	//30 For each vertex, normalize the Normal 
	pos = 0;
	for (y = 0; y < ny; ++y) {
		for (x = 0; x < nx; ++x) {
			m_pNormal[pos] = Normalize(m_pNormal[pos]);
			pos++;
		}	
	}	
	
	delete [] m_pAllSurfaceNormal;
	return ;
}

BBox Heightfield2::ObjectBound() const {
	float minz = z[0], maxz = z[0];
	for (int i = 1; i < nx*ny; ++i) {
		if (z[i] < minz) minz = z[i];
		if (z[i] > maxz) maxz = z[i];
	}
	return BBox(Point(0,0,minz), Point(1,1,maxz));
}

bool Heightfield2::CanIntersect() const {
	return true;
}

void Heightfield2::GetShadingGeometry(const Transform &obj2world,
			const DifferentialGeometry &dg,
			DifferentialGeometry *dgShading) const
{
	* dgShading = dg;
	return ;
}

bool Heightfield2::RayTriangleIntersection(/*Output*/ DifferentialGeometry * dg, float * tHit, \
			/*Input*/const Ray &ray,const Point * P,  int index0, int index1, int index2, const float * uvs  ) const
{
		const Point &p1 = P[index0];
		const Point &p2 = P[index1];
		const Point &p3 = P[index2];		
		
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
		

		// Fill in _DifferentialGeometry_ from triangle hit
		// Compute triangle partial derivatives
		Vector dpdu, dpdv;
		Normal dndu, dndv;
		float _uvs[3][2];
		_uvs[0][0] = uvs[2*index0 ];	//+0 u
		_uvs[0][1] = uvs[2*index0+1];	//+1 v
		_uvs[1][0] = uvs[2*index1];
		_uvs[1][1] = uvs[2*index1+1];
		_uvs[2][0] = uvs[2*index2];
		_uvs[2][1] = uvs[2*index2+1];

		// Interpolate $(u,v)$ triangle parametric coordinates
		float b0 = 1 - b1 - b2;
		float tu = b0*_uvs[0][0] + b1*_uvs[1][0] + b2*_uvs[2][0];
		float tv = b0*_uvs[0][1] + b1*_uvs[1][1] + b2*_uvs[2][1];
		
		//Now=========== I will manipulate the normal
		Vector avg_normal = m_pNormal[index0] * b0 + m_pNormal[index1] * b1 + m_pNormal[index2] * b2;
		//Vector temp_tangent(1,0,0);
		//Vector temp_surface(1,0,0);

		avg_normal = Normalize(avg_normal);
		//temp_surface = Normalize(Cross(avg_normal, temp_tangent) ); //now surface is |_ avg_normal
		//temp_tangent = Normalize(Cross(temp_surface, avg_normal) );		

		//dpdu = temp_tangent;
		//dpdv = temp_surface;
		// Compute deltas for triangle partial derivatives
		float du1 = _uvs[0][0] - _uvs[2][0];
		float du2 = _uvs[1][0] - _uvs[2][0];
		float dv1 = _uvs[0][1] - _uvs[2][1];
		float dv2 = _uvs[1][1] - _uvs[2][1];
		Vector dp1 = p1 - p3, dp2 = p2 - p3;
		Vector dn1 = m_pNormal[index0] - m_pNormal[index2], dn2 = m_pNormal[index1] - m_pNormal[index2];

		float determinant = du1 * dv2 - dv1 * du2;
		if (determinant == 0.f) {
		// Handle zero determinant for triangle partial derivative matrix
			CoordinateSystem(Normalize(Cross(e2, e1)), &dpdu, &dpdv);
			dndu = Normal(dpdu);
			dndv = Normal(dpdv);
		}
		else {
			float invdet = 1.f / determinant;
			dpdu = ( dv2 * dp1 - dv1 * dp2) * invdet;
			dpdv = (-du2 * dp1 + du1 * dp2) * invdet;
			dndu = Normal( ( dv2 * dn1 - dv1 * dn2) * invdet );
			dndv = Normal( (-du2 * dn1 + du1 * dn2) * invdet );
		}
		
		//Now================
		*dg = DifferentialGeometry((*ObjectToWorld)(ray(t)), 
								   (*ObjectToWorld)(dpdu), (*ObjectToWorld)(dpdv),
								   (*ObjectToWorld)(dndu), (*ObjectToWorld)(dndv),
								   tu, tv, this);
		*tHit = t;
		
		return true;	
}


bool Heightfield2::RayTriangleIntersectionP(/*Input*/const Ray &ray,const Point * P,  int index0, int index1, int index2, const float * uvs  ) const
{
		const Point &p1 = P[index0];
		const Point &p2 = P[index1];
		const Point &p3 = P[index2];
		
		
		//30.40
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
	
		return true;	
}

int sign(float input)
{
	if (input > 0) return 1;
	else if (input < 0) return -1; 
	else return 0;
}

void Heightfield2::getMinMaxZ_FromRay_XY(const Ray &ray, float *rayEpsilon, int blockNoX, int blockNoY, float &flMinZ, float &flMaxZ) const
{

	float x1 = blockNoX * stepX;
	float y1 = blockNoY * stepY;
	
	float x2 = (blockNoX+1) * stepX ;		//This can be the total edge.
	float y2 = (blockNoY+1) * stepY ;		//This can be the total edge.
	
	float tx1,tx2;
	float raydx = ray.d.x;
	if ( raydx  == 0 ) {	
		//raydx = (*rayEpsilon);
		tx1 = tx2 = 0;
	} 
	else{
		tx1 = (x1 - ray.o.x) / raydx;
		tx2 = (x2 - ray.o.x) / raydx;	
	}
	//float tx1 = (x1 - ray.o.x) / raydx;
	//float tx2 = (x2 - ray.o.x) / raydx;	

	float raydy = ray.d.y;
	float ty1, ty2;
	if ( raydy  == 0 ) {	
		//raydy = (*rayEpsilon);
		ty1 = ty2 = 0;
	}
	else{
		ty1 = (y1 - ray.o.y) / raydy;
		ty2 = (y2 - ray.o.y) / raydy;
	}
	//float ty1 = (y1 - ray.o.y) / raydy;
	//float ty2 = (y2 - ray.o.y) / raydy;
	
	//Get the middle two from tx1, tx2, ty1, ty2;
	float max1, max2, minbtmax12, min1, min2,maxbtmin12,z1,z2;
	if( tx1!=0 && tx2!=0 && ty1!=0 && ty2!=0 ){
		if (tx1 > ty1) {
			max1 = tx1; 
			min1 = ty1; 
		} else {
			max1 = ty1;
			min1 = tx1;
		}

		if (tx2 > ty2) {
			max2 = tx2; 
			min2 = ty2;
		} else {
			max2 = ty2;
			min2 = tx2;
		}

		if (max1 > max2) {
			minbtmax12 = max2;
		} else {
			minbtmax12 = max1;
		}

		if (min1 > min2) {
			maxbtmin12 = min1;
		} else {
			maxbtmin12 = min2;
		}

		//Calculte from minbtmax12, maxbtmin12 -> MinZ and MaxZ
		z1 = ray.o.z + minbtmax12 * ray.d.z ;
		z2 = ray.o.z + maxbtmin12 * ray.d.z ;
	}
	else if( tx1==0 && tx2==0 ){
		if(ty1>ty2){
			z1 = ray.o.z + ty2* ray.d.z ;
			z2 = ray.o.z + ty1* ray.d.z ;
		}
		else{
			z1 = ray.o.z + ty1* ray.d.z ;
			z2 = ray.o.z + ty2* ray.d.z ;
		}
	}
	else{
		if(tx1>tx2){
			z1 = ray.o.z + tx2* ray.d.z ;
			z2 = ray.o.z + tx1* ray.d.z ;
		}
		else{
			z1 = ray.o.z + tx1* ray.d.z ;
			z2 = ray.o.z + tx2* ray.d.z ;
		}
	}

	//Return the value
	flMinZ = min(z1,z2);
	flMaxZ = max(z1,z2);
	return;
}

bool Heightfield2::Intersect(const Ray &r, float *tHit,
		float *rayEpsilon, DifferentialGeometry *dg) const {
	printf("ray:%8.4f %8.4f %8.4f\n",r.d.x, r.d.y, r.d.z );
	//10. Transform _Ray_ to object space
	Ray ray;
	(*WorldToObject)(r, &ray);
	//25 Firstly test the Bounding Box with the Ray
	BBox BBoxHeightField;
	BBoxHeightField = this->ObjectBound();
	float t0= 0, t1 =0; //assert(t1 >= t0)
	bool bBBoxIntesected = BBoxHeightField.IntersectP(/*Input*/ray, /*output*/ &t0, &t1);
	
	assert(t1 >= t0);	
	
	bool bTriangleIntersected = false;
	bool bIntesectVoxel = false; //Default is no intersection, false!

	//30 Ray Triangle Intersection.

	if(bBBoxIntesected) {		
		//a) Firstly get all the info about the Grid. 		
		float raydx = ray.d.x;
		float raydy = ray.d.y;		
		float tDeltaX ,tDeltaY;
		//if(raydx==0)
		//	tDeltaX = 0;
		//else
			tDeltaX = stepX / raydx;	// stepX / cos
		//if(raydy==0)
		//	tDeltaY = 0;
		//else
			tDeltaY = stepY / raydy;  // stepY / sin
		//b) Step by step, march down all the necessary triangles.
		//b.1  t0 -> O+t0*D  to get the first block no.
		Point FirstHitPoint = ray.o + ray.d * (t0);
	
		int blockNoX = ( int )(  FirstHitPoint.x / stepX );	//RAY_EPSILON is make sure not fall to negative part
		int blockNoY = ( int )(  FirstHitPoint.y / stepY );
	
		if (blockNoX < 0) blockNoX = 0; 
		if (blockNoY < 0) blockNoY = 0; 
		if (blockNoX >= (nx-1) ) blockNoX = (nx-2); 
		if (blockNoY >= (ny-1) ) blockNoY = (ny-2); 
		//b.1.5

		t1 = t1+(*rayEpsilon);
		Point FinalHitPoint = ray.o + ray.d * (t1);
		
		int FinalblockNoX = int (  FinalHitPoint.x / stepX );	//RAY_EPSILON is make sure not fall to negative part
		int FinalblockNoY = int (  FinalHitPoint.y / stepY );
		
		if (FinalblockNoX < 0) FinalblockNoX = 0; 
		if (FinalblockNoY < 0) FinalblockNoY = 0; 
		if (FinalblockNoX >= (nx-1) ) FinalblockNoX = (nx-2); 
		if (FinalblockNoY >= (ny-1) ) FinalblockNoY = (ny-2); 

		//b.2 build up the small bounding box to initialize tMaxX and tMaxY (A lot of possibility)		
		
		float x1 = blockNoX * stepX;
		float y1 = blockNoY * stepY;
		
		float x2 = (blockNoX+1) * stepX ;		//This can be the total edge.
		float y2 = (blockNoY+1) * stepY ;		//This can be the total edge.  
		float tMaxX, tMaxY;		//we have to march down from t0 -> t1 to find out tMaxX and tMaxY
		//b.2.1 ray <-> (x1,0,0) (x2,0,0) => tMaxX, t = (x - Ox) / Dx
		//...
		//float tx1,tx2;
		if ( ray.d.x  == 0 ) {
			//The ray doesn't change at X, then only one Cell intersected by ZMin and ZMax at that cell		
			ray.d.x = (*rayEpsilon);
			//tx1 = 0;
			//tx2 = 0;
		} 
		//else{
		//	tx1 = (x1 - ray.o.x) / ray.d.x; 
		//	tx2 = (x2 - ray.o.x) / ray.d.x;
		//}
		float tx1 = (x1 - ray.o.x) / ray.d.x; 
		float tx2 = (x2 - ray.o.x) / ray.d.x;
		t0 = t0 + (*rayEpsilon);
		
		
		tMaxX = max(tx1, tx2);

		//b.2.2 ray <-> (0,y1,0) (0,y2,0) => tMaxY, t = (y - Oy) / Dy
		//...
		//float ty1,ty2;
		if ( ray.d.y  == 0 ) {
			//The ray doesn't change at X, then only one Cell intersected by ZMin and ZMax at that cell		
			ray.d.y = (*rayEpsilon);
			//ty1 = 0;
			//ty2 = 0;
		}
		//else{
		//	ty1 = (y1 - ray.o.y) / ray.d.y;
		//	ty2 = (y2 - ray.o.y) / ray.d.y;
		//}
		float ty1 = (y1 - ray.o.y) / ray.d.y;
		float ty2 = (y2 - ray.o.y) / ray.d.y;

		tMaxY = max(ty1,ty2);

		//b.3  step from t0 to t1.
		//Now we need to care about the sign now!
		int X = blockNoX;
		int Y = blockNoY;		

		int index0, index1, index2;
		//static long int ray_counter = 0;
		//ray_counter ++;
		static long int tri_loop_counter = 0;
		do {
			//By calling RayTriangleIntersection, we intersect this ray with Voxel.			
			
			//X, Y -> i (# of ntris)
			if (X >= (nx -1) ) break;
			if (Y >= (ny -1) ) break;
			if (X < 0) break;
			if (Y < 0) break;
			assert( X < (nx -1) && X >= 0 ); 
			assert( Y < (ny -1) && Y >= 0 ); 
			
			
			int block_no = (Y ) * (nx-1) + (X) ; 
			int tri_no1 = 2 * block_no + 0;

			//Only if the Z is possible then everything is possible
			float flMinZ , flMaxZ ;
			getMinMaxZ_FromRay_XY(ray, rayEpsilon, X, Y, flMinZ, flMaxZ);
			bool bOverflowZ  = (flMinZ > ( TileMaxZ[block_no] + RAY_EPSILON));
			bool bUnderflowZ = (flMaxZ < ( TileMinZ[block_no] - RAY_EPSILON ));
			if ( (!bOverflowZ) && (!bUnderflowZ) ) {
				//tri_loop_counter++;

				index0 = m_verts[ tri_no1 * 3 + 0];
				index1 = m_verts[ tri_no1 * 3 + 1];
				index2 = m_verts[ tri_no1 * 3 + 2];			
				bIntesectVoxel = this->RayTriangleIntersection(/*Output*/ dg, tHit, 
					/*Input*/ray, P, index0, index1, index2, uvs  );
				if ( bIntesectVoxel){ 				
					tri_loop_counter++;
					printf("tri_loop_counter:%d\n",tri_loop_counter);
					//printf("yes\n");
					break;} //Jump out of while


				int tri_no2 = 2 * block_no + 1;
				index0 = m_verts[ tri_no2 * 3 + 0];
				index1 = m_verts[ tri_no2 * 3 + 1];
				index2 = m_verts[ tri_no2 * 3 + 2];		
				bIntesectVoxel = this->RayTriangleIntersection(/*Output*/ dg, tHit, 
					/*Input*/ray, P, index0, index1, index2, uvs  );
				
				
				if ( bIntesectVoxel){ 				
					tri_loop_counter++;
					printf("tri_loop_counter:%d\n",tri_loop_counter);
					//printf("yes\n");
					break;}  //Jump out of while

				if (X == FinalblockNoX && Y == FinalblockNoY) break;
				
			}
			//b.1.5 (cont)			

			if ( ((tMaxX < tMaxY)/* && (tMaxX!=0)*/) /*|| tMaxY==0 */) {
				int signX = sign(ray.d.x);
				tMaxX = tMaxX +  abs(tDeltaX);
				X = X + signX * 1;			
			} else if( ((tMaxY < tMaxX)/* && (tMaxY!=0)*/)/* || tMaxX==0 */) {
				int signY = sign(ray.d.y);
				tMaxY = tMaxY +  abs(tDeltaY);
				Y = Y + signY * 1;			
			}

			
		}while( true ); //(tMaxX <=t1) || (tMaxY <=t1) ) ;      //otherwise, means the tMaxX and tMaxY both >= t1.	
		
	} 
	
	//50.EXIT: The only one	
	return bIntesectVoxel; //Only if all are false, then return false;		
}

bool Heightfield2::IntersectP(const Ray &r) const {
	float rayEpsilon = RAY_EPSILON;
	//10. Transform _Ray_ to object space
	Ray ray;
	(*WorldToObject)(r, &ray);	
	
	//25 Firstly test the Bounding Box with the Ray
	BBox BBoxHeightField;
	BBoxHeightField = this->ObjectBound();
	float t0= 0, t1 =0; //assert(t1 >= t0)
	bool bBBoxIntesected = true;
	bBBoxIntesected =  BBoxHeightField.IntersectP(/*Input*/ray, /*output*/ &t0, &t1);
	
	assert(t1 >= t0);	
	
	bool bTriangleIntersected = false;
	bool bIntesectVoxel = false; //Default is no intersection, false!

	//30 Ray Triangle Intersection.
	if(bBBoxIntesected == true) {		
		//a) Firstly get all the info about the Grid. 		
		
		float raydx = ray.d.x;
		float raydy = ray.d.y;		

		float tDeltaX = stepX / raydx;	// stepX / cos
		float tDeltaY = stepY / raydy;  // stepY / sin	
		//b) Step by step, march down all the necessary triangles.
		//b.1  t0 -> O+t0*D  to get the first block no.
		Point FirstHitPoint = ray.o + ray.d * (t0);
	
		int blockNoX = int (  FirstHitPoint.x / stepX );	//RAY_EPSILON is make sure not fall to negative part
		int blockNoY = int (  FirstHitPoint.y / stepY );
	
		if (blockNoX < 0) blockNoX = 0; 
		if (blockNoY < 0) blockNoY = 0; 
		if (blockNoX >= (nx-1) ) blockNoX = (nx-2); 
		if (blockNoY >= (ny-1) ) blockNoY = (ny-2); 
		//b.1.5

		t1 = t1+RAY_EPSILON;
		Point FinalHitPoint = ray.o + ray.d * (t1);
		
		int FinalblockNoX = int (  FinalHitPoint.x / stepX );	//RAY_EPSILON is make sure not fall to negative part
		int FinalblockNoY = int (  FinalHitPoint.y / stepY );
		
		if (FinalblockNoX < 0) FinalblockNoX = 0; 
		if (FinalblockNoY < 0) FinalblockNoY = 0; 
		if (FinalblockNoX >= (nx-1) ) FinalblockNoX = (nx-2); 
		if (FinalblockNoY >= (ny-1) ) FinalblockNoY = (ny-2); 

		//b.2 build up the small bounding box to initialize tMaxX and tMaxY (A lot of possibility)		
		
		float x1 = blockNoX * stepX;
		float y1 = blockNoY * stepY;
		
		float x2 = (blockNoX+1) * stepX ;		//This can be the total edge.
		float y2 = (blockNoY+1) * stepY ;		//This can be the total edge.
		float tMaxX, tMaxY;		//we have to march down from t0 -> t1 to find out tMaxX and tMaxY
		//b.2.1 ray <-> (x1,0,0) (x2,0,0) => tMaxX, t = (x - Ox) / Dx
		//...
		if ( ray.d.x  == 0 ) {
			//The ray doesn't change at X, then only one Cell intersected by ZMin and ZMax at that cell		
			ray.d.x = RAY_EPSILON;
		} 
		
		float tx1 = (x1 - ray.o.x) / ray.d.x;
		float tx2 = (x2 - ray.o.x) / ray.d.x;
		
		//t0 = t0 + RAY_EPSILON;
		
		
		tMaxX = max(tx1, tx2);

		//b.2.2 ray <-> (0,y1,0) (0,y2,0) => tMaxY, t = (y - Oy) / Dy
		//...
		if ( ray.d.y  == 0 ) {
			//The ray doesn't change at X, then only one Cell intersected by ZMin and ZMax at that cell		
			ray.d.y = RAY_EPSILON;
		}
		float ty1 = (y1 - ray.o.y) / ray.d.y;
		float ty2 = (y2 - ray.o.y) / ray.d.y;
		
		tMaxY = max(ty1,ty2);

		//b.3  step from t0 to t1.
		//Now we need to care about the sign now!
		int X = blockNoX;
		int Y = blockNoY;		

		int index0, index1, index2;
		//static long int ray_counter = 0;
		//ray_counter ++;
		//static long tri_loop_counter = 0;
		do {
			//By calling RayTriangleIntersection, we intersect this ray with Voxel.			
			//printf("intersectP : tri_loop_counter:%d\n",tri_loop_counter);
			//X, Y -> i (# of ntris)
			if (X >= (nx -1) ) break;
			if (Y >= (ny -1) ) break;
			if (X < 0) break;
			if (Y < 0) break;
			assert( X < (nx -1) && X >= 0 ); 
			assert( Y < (ny -1) && Y >= 0 ); 
			
			
			int block_no = (Y ) * (nx-1) + (X) ; 
			int tri_no1 = 2 * block_no + 0;

			//Only if the Z is possible then everything is possible
			float flMinZ , flMaxZ ;
			getMinMaxZ_FromRay_XY(ray, &rayEpsilon, X, Y, flMinZ, flMaxZ);

			//if( ( flMinZ<0.7 && flMinZ>-0.6 ) || ( flMaxZ<0.7 && flMaxZ>-0.6) )
			//printf("flMinZ:%f flMaxZ:%f\n",flMinZ,flMaxZ);
			
			bool bOverflowZ  = (flMinZ > ( TileMaxZ[block_no] + RAY_EPSILON));
			bool bUnderflowZ = (flMaxZ < ( TileMinZ[block_no] - RAY_EPSILON ));
			if ( (!bOverflowZ) && (!bUnderflowZ) ) {
				//tri_loop_counter++;
				//printf("tri_loop_counter:%d\n",tri_loop_counter);
				index0 = m_verts[ tri_no1 * 3 + 0];
				index1 = m_verts[ tri_no1 * 3 + 1];
				index2 = m_verts[ tri_no1 * 3 + 2];			
				bIntesectVoxel = this->RayTriangleIntersectionP( 
					/*Input*/ray, P, index0, index1, index2, uvs  );
				if ( bIntesectVoxel == true ) {
					//tri_loop_counter++;
					//printf("tri_loop_counter:%d\n",tri_loop_counter);
					//printf("yes\n");
					break;} //Jump out of while


				int tri_no2 = 2 * block_no + 1;
				index0 = m_verts[ tri_no2 * 3 + 0];
				index1 = m_verts[ tri_no2 * 3 + 1];
				index2 = m_verts[ tri_no2 * 3 + 2];		
				bIntesectVoxel = this->RayTriangleIntersectionP(
					/*Input*/ray, P, index0, index1, index2, uvs  );
				
				
				if ( bIntesectVoxel == true ) {
					//tri_loop_counter++;
					//printf("tri_loop_counter:%d\n",tri_loop_counter);
					//printf("yes\n");
					break;}  //Jump out of while

				if (X == FinalblockNoX && Y == FinalblockNoY) break;
				
			}
			//b.1.5 (cont)			

			if (tMaxX < tMaxY ) {
				int signX = sign(ray.d.x);
				tMaxX = tMaxX +  abs(tDeltaX);
				X = X + signX * 1;			
			} else {
				int signY = sign(ray.d.y);
				tMaxY = tMaxY +  abs(tDeltaY);
				Y = Y + signY * 1;			
			}

			
		}while( true ); //(tMaxX <=t1) || (tMaxY <=t1) ) ;      //otherwise, means the tMaxX and tMaxY both >= t1.	
		//printf("break the while loop\n");
	} 
	
	//50.EXIT: The only one	
	return bIntesectVoxel; //Only if all are false, then return false;		

}

void Heightfield2::Refine(vector<Reference<Shape> > &refined) const {
	int ntris = 2*(nx-1)*(ny-1);
	refined.reserve(ntris);
	int *verts = new int[3*ntris];
	Point *P = new Point[nx*ny];
	float *uvs = new float[2*nx*ny];
	int nverts = nx*ny;
	int x, y;
	// Compute heightfield vertex positions
	int pos = 0;
	for (y = 0; y < ny; ++y) {
		for (x = 0; x < nx; ++x) {
			P[pos].x = uvs[2*pos]   = (float)x / (float)(nx-1);
			P[pos].y = uvs[2*pos+1] = (float)y / (float)(ny-1);
			P[pos].z = z[pos];
			++pos;
		}
	}
	
	// Fill in heightfield vertex offset array
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
	ParamSet paramSet;
	paramSet.AddInt("indices", verts, 3*ntris);
	paramSet.AddFloat("uv", uvs, 2 * nverts);
	paramSet.AddPoint("P", P, nverts);
	
	refined.push_back(CreateTriangleMeshShape(ObjectToWorld, WorldToObject, ReverseOrientation, paramSet));
	delete[] P;
	delete[] uvs;
	delete[] verts;
}


Heightfield2 *CreateHeightfield2Shape(const Transform *o2w, const Transform *w2o,
        bool reverseOrientation, const ParamSet &params) {
    int nu = params.FindOneInt("nu", -1);
    int nv = params.FindOneInt("nv", -1);
    int nitems;
    const float *Pz = params.FindFloat("Pz", &nitems);
    Assert(nitems == nu*nv);
    Assert(nu != -1 && nv != -1 && Pz != NULL);
    return new Heightfield2(o2w, w2o, reverseOrientation, nu, nv, Pz);
}
