#include <iostream>
#include <math.h>
#include <algorithm>
#include <iterator>
#include <fstream>

using namespace std;

void print_array(float arr[])
{
	for (size_t i = 0; i < 3; i++)
	{
		cout << arr[i] << " ";
	}
	cout << endl;
}

int size(float arr[])
{
	return sizeof(arr[0]);
}

void subtract(float A[], float B[], float C[])
{
	for (size_t i = 0; i < size(C); i++)
	{
		C[i] = A[i] - B[i];
	}
}

float magnitude(float vec[])
{
	float mag = 0.0;
	for (size_t i = 0; i < 3; i++)
	{
		mag += pow(vec[i], 2);
	}
	return sqrt(mag);
}

void normalize(float vec[], float normVec[])
{
	float vecMag = magnitude(vec);
	for (size_t i = 0; i < 3; i++)
	{
		normVec[i] = vec[i] / vecMag;
	}
}

float dot(float V1[], float V2[])
{
	float res = 0.0;
	for (size_t i = 0; i < 3; i++)
	{
		res += V1[i] * V2[i];
	}
	return res;
}

/* compute vector U */
void vectorU(float v1[], float v2[], float u[])
{

	float v2Xv1[3] = {0.0, 0.0, 0.0};
	// cross multiplication
	v2Xv1[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
	v2Xv1[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
	v2Xv1[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);

	normalize(v2Xv1, u);
} /* EOF */

/* compute vector N */
void vectorN(float v1[], float N[])
{
	normalize(v1, N);
} /* EOF */

/* compute vector V */
void vectorV(float v1[], float v2[], float v[])
{
	// cross multiplication
	v[0] = ((v1[1] * v2[2]) - (v1[2] * v2[1]));
	v[1] = ((v1[2] * v2[0]) - (v1[0] * v2[2]));
	v[2] = ((v1[0] * v2[1]) - (v1[1] * v2[0]));
} /* EOF */

/* compute Mcw */
void computeMcw(float VRP[], float VPN[], float VUP[])
{
	/* calculate u */
	float u[3] = {0.0, 0.0, 0.0};
	vectorU(VUP, VPN, u);

	/* calculate n */
	float n[3] = {0.0, 0.0, 0.0};
	vectorN(VPN, n);

	/* calculate v */
	float v[3] = {0.0, 0.0, 0.0};
	vectorV(n, u, v);

	float R[4][4] = {u[0], v[0], n[0], 0,
									 u[1], v[1], n[1], 0,
									 u[2], v[2], n[2], 0,
									 0, 0, 0, 1};

	float T[4][4] = {1, 0, 0, VRP[0],
									 0, 1, 0, VRP[1],
									 0, 0, 1, VRP[2],
									 0, 0, 0, 1};

	// initialize the matrix
	for (int i = 0; i < sizeof(Mcw) / sizeof(Mcw[0]); i++)
	{
		for (int j = 0; j < sizeof(Mcw) / sizeof(Mcw[0]); j++)
		{
			Mcw[i][j] = 0;
		}
	}

	// perform matrix multiplication
	for (int i = 0; i < sizeof(Mcw) / sizeof(Mcw[0]); i++)
	{
		for (int j = 0; j < sizeof(Mcw) / sizeof(Mcw[0]); j++)
		{
			for (int k = 0; k < sizeof(Mcw) / sizeof(Mcw[0]); ++k)
			{
				Mcw[i][j] += T[i][k] * R[k][j];
			}
		}
	}
} /* EOF */

/* RAY CONSTRUCTION FUNCTION */
/* Input:
	(i, j): pixel index
	P0: Camera Origin
	V0: Ray Direction
*/
/* Output:
	Calculated value of P0 and V0
*/
void ray_construction(int i, int j, float p0[], float v0[])
{

	/* Step1: */
	/* Map (j,i) from screen coord to (xc, yc) in the cam coord */
	float xc, yc;
	xc = (xmax - xmin) * j / (ROWS - 1) + xmin;
	yc = (ymax - ymin) * i / (COLS - 1) + ymin;

	/* Step2: */
	/* Transform Origin of the camera P0, from camera coordinates */
	/* To world camera. */
	/* NOTE: After transformation, P0 should be equal to VRP */
	float p04[4] = {0.0, 0.0, 0.0, 1.0},
				p04p[4] = {0.0, 0.0, 0.0, 1.0},
				p0p[3] = {0.0, 0.0, 0.0};

	/* P0' = Mcw X P0 */
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			p04p[i] += Mcw[i][j] * p04[j];
		}
	}

	/* drop the homogenous coord */
	/* convert from [0, 0, 0, 1] to [0, 0, 0] */
	for (int i = 0; i < 3; i++)
	{
		p0[i] = p04p[i];
	}

	/* Step3: */
	/* Transform (xc, yc, focal) from camera to world coordinates */
	float p1_4[4] = {xc, yc, focal, 1.0},
				p1_4p[4] = {0.0, 0.0, 0.0, 1.0},
				p1p[3] = {0.0, 0.0, 0.0};

	/* P1' = Mcw X P1 */
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			p1_4p[i] += Mcw[i][j] * p1_4[j];
		}
	}

	/* drop the homogenous coord */
	/* convert from [0, 0, 0, 1] to [0, 0, 0] */
	for (int i = 0; i < 3; i++)
	{
		p1p[i] = p1_4p[i];
	}

	/* calculate v0 */
	for (int i = 0; i < 3; i++)
	{
		v0[i] = p1p[i] - p0[i];
		// v0[i] = p0[i] - p1p[i];
	}

	/* normalize V0 to unit length */
	normalize(v0, v0);
	// print_array(v0);
} /* EOF */
