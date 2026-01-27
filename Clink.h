#pragma once
#include "Matrix.h"
class CLink
{
public:
	//kich thuoc hang so
	double theta, alpha, a, d;
	//cac bien khop
	double qtheta, qalpha, qa, qd;
	// Hàm khởi tạo mặc định
	CLink();
	// Hàm khởi tạo sao chép
	CLink(const CLink& link);
	// Toán tử gán
	CLink& operator=(const CLink& link);
	// Hàm thiết lập các hằng số DH (Denavit-Hartenberg)
	void SetConstants(const double thetain, const double alphain, const double ain, const double din);
	// Hàm thiết lập các tham số biến của khớp
	void SetParameters(const double qthetain, const double qalphain, const double ain, const double din);
	// Hàm lấy ma trận biến đổi Denavit-Hartenberg
	smatrix GetDHmatrix();
};