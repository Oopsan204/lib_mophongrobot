#include "NumMethod.h"

#include <iostream>
using namespace std;


int Gauss_Jordan(const smatrix& a, const vectorm& f, vectorm& x)
{
	smatrix A(a);
	vectorm F(f);
	vectorm iix(GetSize(A));
	x.SetSize(GetSize(A));

	for (int i = 0; i < GetSize(A); i++)
	{//khu cac bien
		double s = 0;
		int ir = 0;
		double p = A(i,0);
		for (int j = 1; j < GetSize(A); j++)
			//Chon tru chinh trong hang thu i
		{
			s = A(i,j);
			if (fabs(s) > fabs(p)) { p = s; ir = j; }
		}

		if (p == 0.0)//ma tran A suy bien
			return -1;
		else
		{
			iix[i] = ir;//ghi vi tri pivot cua hang
			for (int k = 0; k < GetSize(A); k++)
			{
				if (k != i)//khu bien thu ir trong cac hang thu k
				{
					s = A(k, ir) / A(i,ir);
					for (int j = 0; j < GetSize(A); j++)
					{
						if (j == ir) A(k,j) = 0;
						else
						{
							A(k,j) -= s * A(i,j);
						}
					}
					F[k] -= s * F[i];
				} //khu xong bien thu ir trong cac hang thu k
			}
		}
	}//khu xong
//tinh nghiem
	for (int i = 0; i < GetSize(A); i++) x[(int)iix[i]] = F[i] / A(i,(int)iix[i]);
	return 0;
}

//////////////////////////////////////////////////////////////////////////

int InverseByGaussJordan(const smatrix& m1, smatrix& m2)
{
	m2 = m1;
	for (int i = 0; i < GetSize(m1); i++)
	{
		vectorm f(GetSize(m1)), x(GetSize(m1));
		for (int k = 0; k < GetSize(m1); k++) f[k] = 0.0;
		f[i] = 1.0;

		if (Gauss_Jordan(m1, f, x)) return -1;
		for (int j = 0; j < GetSize(m1); j++) m2(j,i) = x[j];
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////

int Newton_Raphson(vectorm& x, functionNR& func, const double& eps, const int& Max)
{
	smatrix F;
	vectorm f(x), dx(x);

	for (int i = 0; i < Max; i++)
	{
		func(F, f, x);
		int err = Gauss_Jordan(F, f, dx);
		if (err != 0) return err;
		int inconvr = 0;
		for (int j = 0; j < GetSize(x); j++)
		{
			if (fabs(dx[j]) >= eps) inconvr = 1;
			x[j] -= dx[j];
		}
		if (!inconvr) return 0;
	}
	return -1;
}