#pragma once
#include "Matrix.h"
#include "CLink.h"
#include <vector>
#define NUMBER_LINKS 4  // Giữ nguyên: 1 khâu đế + 3 khâu chuyển động // bang so khau cua robot(tinh ca khau de)
class CRobot
{
	public: 
		CLink links[NUMBER_LINKS];
		double ltool; //chieu dai dao cua dao
		double m_dtimer; // thoi gian moi buoc tinh toan tren quy dao

		
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

		// Hàm khởi tạo robot
		CRobot();
		// Hàm khởi tạo các thông số của robot
		void init();
		// Hàm giải bài toán động học thuận
		void SolvingForwardKinematics();
		// Hàm thiết lập số điểm trên quỹ đạo
		void SetNumberPoint(const int& n);
		// Hàm xóa dữ liệu của robot
		void clear();
		// Hàm thiết lập các biến khớp
		void SetVariables();
		// Hàm thiết lập vị trí điểm cuối
		void SetEndPoint();
		// Hàm giải bài toán động học ngược về vị trí
		void SolvingInverseKinematics_Position();
		// Hàm giải bài toán động học ngược về vận tốc
		void SolvingInverseKinematics_Velocity();
		// Hàm giải bài toán động học ngược về gia tốc
		void SolvingInverseKinematics_Acceleration();
};

