#include "Transformations.h"

#include <iostream>

using namespace std;

void SetIdentity(smatrix& m)
{
	m.SetSize(4);
	for (int i = 0; i < GetSize(m); i++)
	{
		for (int j = 0; j < GetSize(m); j++)
			m(i,j) = (i == j) ? 1 : 0;
	}
}
void SetIdentity(vectorm& v)
{
	v.SetSize(4);
	for (int i = 0; i < GetSize(v); i++)
	{
		v[i] = (i == GetSize(v) - 1) ? 1 : 0;
	}
}

void SetVector(vectorm& v, double tx, double ty, double tz)
{
	SetIdentity(v);
	v[0] = tx;
	v[1] = ty;
	v[2] = tz;
}

void SetTranslationMatrix(smatrix& m, double tx, double ty, double tz)
{
	SetIdentity(m);
	m(0, GetSize(m) - 1) = tx;
	m(1, GetSize(m) - 1) = ty;
	m(2, GetSize(m) - 1) = tz;
}

void SetScalationMatrix(smatrix& m, double sx, double sy, double sz)
{
	SetIdentity(m);
	m(0,0) = sx;
	m(1,1) = sy;
	m(2,2) = sz;
}

void SetRotationXMatrix(smatrix& m, double angle)
{
	SetIdentity(m);
	m(1,1) = cos(angle);
	m(1,2) = -sin(angle);
	m(2,1) = sin(angle);
	m(2,2) = cos(angle);
}
void SetRotationYMatrix(smatrix& m, double angle)
{
	SetIdentity(m);
	m(0,0) = cos(angle);
	m(0,2) = sin(angle);
	m(2,0) = -sin(angle);
	m(2,2) = cos(angle);
}
void SetRotationZMatrix(smatrix& m, double angle)
{
	SetIdentity(m);
	m(0,0) = cos(angle);
	m(0,1) = -sin(angle);
	m(1,0) = sin(angle);
	m(1,1) = cos(angle);
}