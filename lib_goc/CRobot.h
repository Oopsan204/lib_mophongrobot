#pragma once
#include "../matrix class/Matrix.h"
#include "CLink.h"
#include <vector>
#define NUMBER_LINKS 4 // bang so khau cua robot(tinh ca khau de)
class CRobot
{
	public: 
		CLink links[NUMBER_LINKS];
		double ltool; //chieu dai dao cua dao

		vector <double> pEx, pEy, pEz; //toa do diem cuoi (vitri)
		vector <double> q[NUMBER_LINKS]; 
		//q[0][j] la bien khop gan voi khau0,=0
		//q[1][j] la bien khop gan voi khau1=d1,
		//q[2][j] la bien khop gan voi khau2=theta2,
		//q[3][j] la bien khop gan voi khau3=theta3,

		vector<double> pExv, pEyv, pEzv; //van toc diem cuoi
		vector <double> qv[NUMBER_LINKS];
		vector <double> pExa, pEya, pEza; //gia toc diem cuoi
		vector <double> qa[NUMBER_LINKS];


		int idCurrentPoint; // so diem hien tai dang tinh tren quy dao

		CRobot();
		void init();
		void SolvingForwardKinematics();
		void SetNumberPoint(const int& n);
		void clear();
		void SetVariables();
		void SetEndPoint();
		void SolvingInverseKinematics_Position();
		void SolvingInverseKinematics_Velocity();
		void SolvingInverseKinematics_Acceleration();
};

