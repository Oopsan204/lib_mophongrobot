#pragma once
#include "Matrix.h"
class CLink
{
public:
	//kich thuoc hang so
	double theta, alpha, a, d;
	//cac bien khop
	double qtheta, qalpha, qa, qd;
	CLink();
	CLink(const CLink& link);
	CLink& operator=(const CLink& link);
	void SetConstants(const double thetain, const double alphain, const double ain, const double din);
	void SetParameters(const double qthetain, const double qalphain, const double ain, const double din);
	smatrix GetDHmatrix();
};