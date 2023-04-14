#include "mymodel.h"
#include "functions.h"
#include <math.h>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <fstream>
#include <string.h>

using namespace std;

/* RAY_SPHERE_INTERSECTION FUNCTION */
/* Input:
  P0 is the ray origin
  V0 is the ray direction
  S is the sphere object
*/
/* Ouput:
  t0: intersection point
*/
float ray_sphere_intersection(float P0[], float V0[], SPHERE S)
{
  /* P0 is the ray origin */
  /* V0 is the ray direction */
  // print_array(V0);
  /* let L be the vector from ray origin to the center of the sphere */
  /* L = P0 - S->center */

  /* Store the value of center of sphere in an array */
  float centerSphere[3] = {S.x, S.y, S.z};

  float L[3] = {0.0, 0.0, 0.0};
  float d = 0.0; /* Distance d */

  /* L = CenterOfSphere - P0 */
  for (int i = 0; i < 3; i++)
  {
    L[i] = centerSphere[i] - P0[i];
  }

  /* calculate the dot product of
  dot(L , ray direction)
  */
  float Tca = 0.0;
  for (int i = 0; i < 3; i++)
  {
    Tca += L[i] * V0[i];
  }

  /* point is behind the surface */
  if (Tca < 0)
    return -1.0;

  /* calculate L square */
  float LenL = magnitude(L);
  float L_sq = LenL * LenL;

  /* calculate Tca square */
  d = dot(L, L) - (Tca * Tca);
  sqrt(d);

  /* Ray passes outside of the surface */
  if (d < 0)
  {
    return -1;
  }
  /* Intersection takes place */
  else if (d <= S.radius)
  {
    float Thc = sqrt(pow(S.radius, 2) - pow(d, 2));
    float t0 = Tca - Thc;
    float t1 = Tca + Thc;

    /* t0 should be smaller than t1 */
    if (t0 > t1)
      std::swap(t0, t1);

    /* if t0 is less than 0, make it equal to 0 */
    /* yet t0 is less than 0, then no intersection takes place */
    if (t0 < 0)
    {
      t0 = t1;
      if (t0 < 0)
        return -1;
    }
    return t0;
  }
  return -1;
} /* EOF */

/* Ray-Object Intersection */
/* Input:
  ray – P0, V0
*/
/* Output:
  the nearest intersection point P[3] if found, along with
  N[3], the surface normal at that point, and
  kd, the diffuse reflection coefficient of the surface.
*/
/* Note: In a general system, the objects should be stored a list
  structure. A loop will scan through each object in the list. The
  nearest intersection point is found. In our case, we will have only
  two hard-coded objects: a sphere and a polygon.
  So, this part is “hard-coded” for now.
*/
bool ray_object_intersection(float P0[], float V0[], float P[], float N[], float *kd)
{

  /* Store the value of center of sphere */
  float centerSphere[3] = {obj1.x, obj1.y, obj1.z};
  /* N1, N2: Normal vector of Sphere and Surface resp. */
  float N1[3] = {0.0, 0.0, 0.0}, N2[3] = {0.0, 0.0, 0.0};

  /* t1: intersection value of the point on the Sphere */
  float t1 = 0;
  t1 = ray_sphere_intersection(P0, V0, obj1);

  /* If not point found */
  if (t1 < 0)
    return false;

  /* if no point found on Polygon */
  /* return values for sphere */
  else
  {
    /* Calculate the intersection point P */
    /*  P = P0 + t1*V0  */
    for (size_t i = 0; i < 3; i++)
    {
      P[i] = P0[i] + (t1 * V0[i]);
      N[i] = P[i] - centerSphere[i];
    }

    normalize(N, N);

    /* Assign kd = sphere's kd */
    *kd = obj1.kd;
    return 1;
  }
  return false;
} /* EOF */

/* Shading */
/* Input:
  P[3] – point position
    N[3] – surface normal at that point
    kd – diffuse reflection coefficient of the surface
*/
/*  Output: C – shading value */

float shading(float P[], float N[], float kd)
{
  /* L = LPR - P */
  float L[3] = {0.0, 0.0, 0.0};
  subtract(LPR, P, L);

  /* normalize L to unit length */
  normalize(L, L);

  /* dot product */
  /* N dot L */
  // normalize(N, N);
  float N_dot_L = dot(N, L);

  // if ((int)N_dot_L > 0)
  // cout << N_dot_L << endl;

  /* Shading value */
  // where (N * L) is dot-product
  if (N_dot_L < 0)
    return 0;
  else
    return Ip * kd * N_dot_L;

} /* EOF */

/* RAY TRACING FUNCTION */
/* Input:
  P0: Origin of the ray
  V0: Direction vector
*/
/* Output:
  C: Shading value, if the ray hits the surface
  NULL: if ray miss to hit any surface
*/
float ray_tracing(float P0[], float V0[])
{

  /* P: Intersection point */
  /* N: Normal vector of the surface */
  /* kd: Diffusion reflection coefficient of the object */
  float P[3] = {0.0, 0.0, 0.0}, N[3] = {0.0, 0.0, 0.0}, kd;

  /* If any intersection point is found,*/
  /* ray_object_intersection function would return : */
  /* Intersection point on the surface, Normal vector of the surface, and kd */
  bool found = ray_object_intersection(P0, V0, P, N, &kd);
  // cout << found << endl;

  // print_array(P);

  /* C: stores the shading value */
  float C = 0.0;

  /* If the intersection point is found, compute shading */
  /* Else return NULL */
  if (found)
  {
    /* Calculate the shading at the given point */
    /* Assign the value in C */
    C = shading(P, N, kd);
    // if (C > 0)
    // cout << C << endl;
    return C;
  }
  else
    return -1;
} /* EOF */

/* MAIN FUNCTION */
int main()
{

  /* initialize image buffer */
  for (int i = 0; i < ROWS; i++)
  {
    for (int j = 0; j < COLS; j++)
    {
      img[i][j] = 255;
    }
  }

  /* Calculate the Mcw based on the camera parameters*/
  /* This function call would update the Mcw matrix in the header file */
  computeMcw(VRP, VPN, VUP);

  /* declare and initialize the required variable */
  /* P0: Origin of the ray */
  /* V0: Direction of the ray */
  /* C: Stores the shading value */

  float P0[3] = {0.0, 0.0, 0.0}, C = 0.0;

  // for (int i = 20; i < 21; i++)
  for (int i = 0; i < ROWS; i++)
  {
    float V0[3] = {0.0, 0.0, 0.0};
    // for (int j = 20; j < 21; j++)
    for (int j = 0; j < COLS; j++)
    {

      /* Construct the ray V, starting from the CenterOfProjection P0*/
      /* And passing through the pixel (i, j) */
      ray_construction(i, j, P0, V0);
      // print_array(V0);

      /* if ray intersects, then add it to image buffer */
      C = ray_tracing(P0, V0);
      // cout << C << endl;
      if (C >= 0)
        img[i][j] = (unsigned char)C; /* save the shading value into the image buffer */
    }
  }

  /* Write the data from the image buffer */
  /* into the binary output file */
  FILE *outputFile;
  outputFile = fopen("./image.raw", "wb");
  float x = sizeof(img);
  fwrite(&img, sizeof(char), ROWS * COLS, outputFile);
  fclose(outputFile);

  return 1;
} /* EOF */
