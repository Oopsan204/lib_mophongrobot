#include "CLink.h"
#include <stdio.h>
#include "Matrix.h"
#include <math.h>
// Hàm khởi tạo mặc định cho lớp CLink.
// Đặt tất cả các tham số DH và biến khớp về 0.
CLink::CLink() 
{
		SetConstants(0.0, 0.0, 0.0, 0.0);
		SetParameters(0.0, 0.0, 0.0, 0.0);
}
// Hàm khởi tạo sao chép.
// Sao chép các giá trị từ một đối tượng CLink khác.
CLink::CLink(const CLink& link)
{
	SetConstants(link.theta, link.alpha, link.a, link.d);
	SetParameters(link.qtheta, link.qalpha, link.qa, link.qd);
}
// Toán tử gán.
// Gán giá trị từ một đối tượng CLink khác, tránh tự gán.
CLink& CLink::operator=(const CLink& link)
{
	if (this != &link)
	{
		SetConstants(link.theta, link.alpha, link.a, link.d);
		SetParameters(link.qtheta, link.qalpha, link.qa, link.qd);
	}
	return *this;
}
// Thiết lập các hằng số Denavit-Hartenberg (DH) cho một khâu robot.
void CLink::SetConstants(const double thetain, const double alphain, const double ain, const double din)
{
	theta = thetain;
	alpha = alphain;
	a = ain;
	d = din;
}
// Thiết lập các tham số biến của khớp.
void CLink::SetParameters(const double qthetain, const double qalphain, const double ain, const double din)
{
	qtheta = qthetain;
	qalpha = qalphain;
	qa = ain;
	qd = din;
}
// Tính toán và trả về ma trận biến đổi Denavit-Hartenberg (DH) cho khâu.
// Ma trận này biểu diễn vị trí và hướng của hệ tọa độ của khâu so với hệ tọa độ của khâu trước đó.
smatrix CLink::GetDHmatrix()
{
	smatrix DH_TABLE(4); 
	double th = this->theta + this->qtheta;
	double al = this->alpha + this->qalpha;
	double a_val = this->a + this->qa;
	double d_val = this->d + this->qd;

	double ct = cos(th);
	double st = sin(th);
	double ca = cos(al);
	double sa = sin(al);
	DH_TABLE(0, 0) = ct;		DH_TABLE(0, 1) = -st * ca;		DH_TABLE(0, 2) = st * sa;		DH_TABLE(0, 3) = a_val * ct;
	DH_TABLE(1, 0) = st;		DH_TABLE(1, 1) = ct * ca;		DH_TABLE(1, 2) = -ct * sa;		DH_TABLE(1, 3) = a_val * st;
	DH_TABLE(2, 0) = 0.0;		DH_TABLE(2, 1) = sa;			DH_TABLE(2, 2) = ca;			DH_TABLE(2, 3) = d_val;
	DH_TABLE(3, 0) = 0.0;		DH_TABLE(3, 1) = 0.0;			DH_TABLE(3, 2) = 0.0;			DH_TABLE(3, 3) = 1.0;
	return DH_TABLE;

}



