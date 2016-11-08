#ifndef _SPLINE_H
#define _SPLINE_H
#include <math.h>
#include "Point.h"
//#include "stdafx.h"
#define DIV_FACTOR 4.0 //adjust this factor to adjust the curve smoothness
class Curve
{
public:
	float  Ax,Ay;
	float  Bx,By;
	float  Cx,Cy;
	int    Ndiv;

	Curve(float ax,float ay,float bx,float by,float cx,float cy,int ndiv) 
	{
		Ax = ax;
		Ay = ay;
		Bx = bx;
		By = by;
		Cx = cx;
		Cy = cy;
		Ndiv = ndiv;
	}

	Curve(float ax,float ay,float bx,float by,float cx,float cy)
	{
		Ax = ax; 
		Ay = ay;
		Bx = bx; 
		By = by;
		Cx = cx; 
		Cy = cy;
		Ndiv = (int)(max(abs((int)Ax),abs((int)Ay))/DIV_FACTOR);
	}
	Curve() 
	{	
	};

	void PutCurve(float ax,float ay,float bx,float by,float cx,float cy) 
	{
		Ax = ax;
		Ay = ay;
		Bx = bx; 
		By = by;
		Cx = cx; 
		Cy = cy;
		Ndiv = (int)(max(abs((int)Ax),abs((int)Ay))/DIV_FACTOR);
	}

	//void draw(HDC hdc,float x,float y) 
	//{
	//	int OrigX,OrigY,NewX,NewY;
	//	float  t,f,g,h;
	//	if (Ndiv==0)
	//		Ndiv=1;

	//	OrigX = (int)x; 
	//	OrigY= (int)y;
	//	for(int i=1; i<=Ndiv ; i++)
	//	{
	//		t = 1.0f / (float)Ndiv * (float)i;
	//		f = t*t*(3.0f-2.0f*t);
	//		g = t*(t-1.0f)*(t-1.0f);
	//		h = t*t*(t-1.0f);
	//		NewX = (int)(x + Ax*f + Bx*g + Cx*h);
	//		NewY = (int)(y + Ay*f + By*g + Cy*h);
	//		MoveToEx(hdc, OrigX, OrigY, NULL);
	//		LineTo(hdc, NewX, NewY);
	//		OrigX = NewX;  
	//		OrigY=NewY;
	//	}
	//}
	int GetCount()
	{
		if (Ndiv==0)
			Ndiv=1;
		int PointCount = 1;

		for(int i=1; i<=Ndiv ; i++)
		{
			PointCount++;
		}
		return PointCount;
	}
	void GetCurve(float x,float y, Point points[], int& PointCount)
	{
		int X,Y;
		float  t,f,g,h;
		if (Ndiv==0)
			Ndiv=1;

		X = (int)x; 
		Y= (int)y;
		points[PointCount].x = X;
		points[PointCount].y = Y;
		PointCount++;

		for(int i=1; i<=Ndiv ; i++)
		{
			t = 1.0f / (float)Ndiv * (float)i;
			f = t*t*(3.0f-2.0f*t);
			g = t*(t-1.0f)*(t-1.0f);
			h = t*t*(t-1.0f);
			X = (int)(x + Ax*f + Bx*g + Cx*h);
			Y = (int)(y + Ay*f + By*g + Cy*h);
			points[PointCount].x = X;
			points[PointCount].y = Y;
			PointCount++;
		}
	}
  
};

class Spline 
{

public:
	float* Px;
	float* Py;
	float* Ax;
	float* Ay;
	float* Bx;
	float* By;
	float* Cx;
	float* Cy;
	float*  k;
	float*  Mat[3];

	int  NP;

	// constructor
	Spline(Point pt[], int np)
	{
		NP = np;
		Px = new float[NP];
		Py = new float[NP];
		Ax = new float[NP];
		Ay = new float[NP];
		Bx = new float[NP];
		By = new float[NP];
		Cx = new float[NP];
		Cy = new float[NP];
		k = new float[NP];
		Mat[0] = new float[NP];
		Mat[1] = new float[NP];
		Mat[2] = new float[NP];

		for(int i=0;i<NP ;i++) 
		{
			Px[i] = (float)pt[i].x;  
			Py[i] = (float)pt[i].y;
		}

	}

	Spline(float px[] , float py[] , int np) 
	{
		NP = np;
		Px = new float[NP];
		Py = new float[NP];
		Ax = new float[NP];
		Ay = new float[NP];
		Bx = new float[NP];
		By = new float[NP];
		Cx = new float[NP];
		Cy = new float[NP];
		k = new float[NP];
		Mat[0] = new float[NP];
		Mat[1] = new float[NP];
		Mat[2] = new float[NP];

		for(int i=0;i<NP ;i++) 
		{
			Px[i] = px[i];  
			Py[i] = py[i];
		}

	}
	
	~Spline()
	{
		delete[] Px;
		delete[] Py;
		delete[] Ax;
		delete[] Ay;
		delete[] Bx;
		delete[] By;
		delete[] Cx;
		delete[] Cy;
		delete[] k;
		delete[] Mat[0];
		delete[] Mat[1];
		delete[] Mat[2];
	}

	void Generate() 
	{
		float AMag , AMagOld;
    	// vector A
		int i;
		for(i= 0 ; i<=NP-2 ; i++ ) 
		{
			Ax[i] = Px[i+1] - Px[i];
			Ay[i] = Py[i+1] - Py[i];
		}
		// k
		AMagOld = (float)sqrt(Ax[0]*Ax[0] + Ay[0]*Ay[0]);
		for(i=0 ; i<=NP-3 ; i++) 
		{
			AMag = (float)sqrt(Ax[i+1]*Ax[i+1] + Ay[i+1]*Ay[i+1]);
			k[i] = AMagOld / AMag;
			AMagOld = AMag;
		}
		k[NP-2] = 1.0f;

		// Matrix
		for(i=1; i<=NP-2;i++) 
		{
			Mat[0][i] = 1.0f;
			Mat[1][i] = 2.0f*k[i-1]*(1.0f + k[i-1]);
			Mat[2][i] = k[i-1]*k[i-1]*k[i];
		}
		Mat[1][0] = 2.0f;
		Mat[2][0] = k[0];
		Mat[0][NP-1] = 1.0f;
		Mat[1][NP-1] = 2.0f*k[NP-2];

		// 
		for(i=1; i<=NP-2;i++) 
		{
			Bx[i] = 3.0f*(Ax[i-1] + k[i-1]*k[i-1]*Ax[i]);
			By[i] = 3.0f*(Ay[i-1] + k[i-1]*k[i-1]*Ay[i]);
		}
		Bx[0] = 3.0f*Ax[0];
		By[0] = 3.0f*Ay[0];
		Bx[NP-1] = 3.0f*Ax[NP-2];
		By[NP-1] = 3.0f*Ay[NP-2];

		//
		MatrixSolve(Bx);
		MatrixSolve(By);

		for(i=0 ; i<=NP-2 ; i++ ) 
		{
			Cx[i] = k[i]*Bx[i+1];
			Cy[i] = k[i]*By[i+1];
		}
	}

	void MatrixSolve(float B[]) 
	{
		float* Work = new float[NP];
		float* WorkB = new float[NP];
		int i;
		for(i=0;i<=NP-1;i++) 
		{
			Work[i] = B[i] / Mat[1][i];
			WorkB[i] = Work[i];
		}

		for(int j=0 ; j<10 ; j++) 
		{ ///  need convergence judge
			Work[0] = (B[0] - Mat[2][0]*WorkB[1])/Mat[1][0];
			for(int i=1; i<NP-1 ; i++ ) 
			{
				Work[i] = (B[i]-Mat[0][i]*WorkB[i-1]-Mat[2][i]*WorkB[i+1])
							/Mat[1][i];
			}
			Work[NP-1] = (B[NP-1] - Mat[0][NP-1]*WorkB[NP-2])/Mat[1][NP-1];

			for(i=0 ; i<=NP-1 ; i++ ) 
			{
				WorkB[i] = Work[i];
			}
		}
		for(i=0 ; i<=NP-1 ; i++ ) 
		{
			B[i] = Work[i];
		}
		delete[] Work;
		delete[] WorkB;
	}

	//void draw(HDC hdc) 
	//{
	//	Curve c;
	//	for(int i=0; i<NP-1 ; i++) 
	//	{
	//		c.PutCurve(Ax[i],Ay[i],Bx[i],By[i],Cx[i],Cy[i]);
	//		c.draw(hdc,Px[i],Py[i]);
	//	}
	//	
	//}
	int GetCurveCount()
	{
		Curve c;
		int count = 0;
		for(int i=0; i<NP-1 ; i++) 
		{
			c.PutCurve(Ax[i],Ay[i],Bx[i],By[i],Cx[i],Cy[i]);
			count += c.GetCount();
		}
		return count;
	}
	void GetCurve(Point points[], int& PointCount)
	{
		Curve c;
		for(int i=0; i<NP-1 ; i++) 
		{
			c.PutCurve(Ax[i],Ay[i],Bx[i],By[i],Cx[i],Cy[i]);
			c.GetCurve(Px[i],Py[i], points, PointCount);
		}
	}
  //////////// closed cubic spline ////////////////////
	void GenClosed() 
	{
		float AMag , AMagOld , AMag0;
        // vector A
		int i;
		for(i= 0 ; i<=NP-2 ; i++ ) 
		{
			Ax[i] = Px[i+1] - Px[i];
			Ay[i] = Py[i+1] - Py[i];
		}
		Ax[NP-1] = Px[0] - Px[NP-1];
		Ay[NP-1] = Py[0] - Py[NP-1];

		// k
		AMag0 = AMagOld = (float)sqrt(Ax[0]*Ax[0] + Ay[0]*Ay[0]);
		for(i=0 ; i<=NP-2 ; i++) 
		{
			AMag = (float)sqrt(Ax[i+1]*Ax[i+1] + Ay[i+1]*Ay[i+1]);
			k[i] = AMagOld / AMag;
			AMagOld = AMag;
		}
		k[NP-1]=AMagOld/AMag0; 

		// Matrix
		for(i=1; i<=NP-1;i++) 
		{
			Mat[0][i] = 1.0f;
			Mat[1][1] = 2.0f*k[i-1]*(1.0f + k[i-1]);
			Mat[2][i] = k[i-1]*k[i-1]*k[i];
		}
		Mat[0][0] = 1.0f;
		Mat[1][0] = 2.0f*k[NP-1]*(1.0f + k[NP-1]);
		Mat[2][0] = k[NP-1]*k[NP-1]*k[0];

		// 
		for(i=1; i<=NP-1;i++) 
		{
			Bx[i] = 3.0f*(Ax[i-1] + k[i-1]*k[i-1]*Ax[i]);
			By[i] = 3.0f*(Ay[i-1] + k[i-1]*k[i-1]*Ay[i]);
		}
		Bx[0] = 3.0f*(Ax[NP-1] + k[NP-1]*k[NP-1]*Ax[0]);
		By[0] = 3.0f*(Ay[NP-1] + k[NP-1]*k[NP-1]*Ay[0]);

		//
		MatrixSolveEX(Bx);
		MatrixSolveEX(By);

		for(i=0 ; i<=NP-2 ; i++ ) 
		{
			Cx[i] = k[i]*Bx[i+1];
			Cy[i] = k[i]*By[i+1];
		}
		Cx[NP-1] = k[NP-1]*Bx[0];
		Cy[NP-1] = k[NP-1]*By[0];
	}

  ///// tridiagonal matrix + elements of [0][0], [N-1][N-1] //// 
	void MatrixSolveEX(float B[]) 
	{
		int i;
		float* Work = new float[NP];
		float* WorkB = new float[NP];

		for(i=0;i<=NP-1;i++) 
		{
			Work[i] = B[i] / Mat[1][i];
			WorkB[i] = Work[i];
		}

		for(int j=0 ; j<10 ; j++) 
		{  // need judge of convergence
			Work[0] = (B[0]-Mat[0][0]*WorkB[NP-1]-Mat[2][0]*WorkB[1])
					/Mat[1][0];
			for(int i=1; i<NP-1 ; i++ ) 
			{
				Work[i] = (B[i]-Mat[0][i]*WorkB[i-1]-Mat[2][i]*WorkB[i+1])
							/Mat[1][i];
			}
			Work[NP-1] = (B[NP-1]-Mat[0][NP-1]*WorkB[NP-2]-Mat[2][NP-1]*WorkB[0])
							/Mat[1][NP-1];

			for(i=0 ; i<=NP-1 ; i++ ) 
			{
				WorkB[i] = Work[i];
			}
		}

		for(i=0 ; i<=NP-1 ; i++ ) 
		{
			B[i] = Work[i];
		}
	}

	//void drawClosed(HDC hdc) 
	//{
	//	Curve c;
	//	for(int i=0; i<NP ; i++) 
	//	{
	//		c.PutCurve(Ax[i],Ay[i],Bx[i],By[i],Cx[i],Cy[i]);
	//		c.draw(hdc ,Px[i],Py[i]);
	//	}
	//}


};

#endif
