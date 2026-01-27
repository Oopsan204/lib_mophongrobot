#include "CRobot.h"
#include "NumMethod.h"
CRobot* pRobot = NULL;
// Hàm khởi tạo của lớp CRobot.
// Gọi hàm init() để thiết lập các thông số ban đầu và gán con trỏ toàn cục pRobot.
CRobot::CRobot()
{
	init();
	pRobot = this;
}

// Hàm khởi tạo các thông số của robot.
// Thiết lập các tham số DH (chiều dài khâu) và chiều dài của dụng cụ.
void CRobot::init()
{
	//Khoi tao cac tham so DH mac dinh cho robot 4 bieu do
	links[2].a = 300.0;
	links[3].a = 200.0;
	// tong chieu dai 500
	ltool = 10.0; //chieu dai dao
}
// Giải bài toán động học thuận.
// Tính toán vị trí (x, y, z) của điểm cuối (end-effector) từ các giá trị biến khớp đã cho (q).
void CRobot::SolvingForwardKinematics()
{
	smatrix A0n, Ai;
	vectorm pE0, pEn(4);
	pEn[0] = 0;
	pEn[1] = 0;
	pEn[2] = -ltool;
	pEn[3] = 1;

	for (int j = 0; j < q[0].size(); j++)
	{
		links[0].SetParameters(0.0, 0.0, 0.0, 0.0); // fixed base
		links[1].SetParameters(0.0, 0.0, 0.0, q[1][j]); // prismatic joint
		links[2].SetParameters(q[2][j], 0.0, 0.0, 0.0); // revolute joint
		links[3].SetParameters(q[3][j], 0.0, 0.0, 0.0); // revolute joint

		A0n = links[0].GetDHmatrix();
		for (int i = 1; i < NUMBER_LINKS; i++)
		{
			Ai = links[i].GetDHmatrix();
			A0n = A0n * Ai;
		}
		pE0 = A0n * pEn;
		pEx[j] = pE0[0];
		pEy[j] = pE0[1];
		pEz[j] = pE0[2];
	}

	std::cout << "End SolvingForwardKinematics" << std::endl;
	std::cout << "Number of points: " << pEx.size() << std::endl;
	std::cout << "First point: (" << pEx[0] << ", " << pEy[0] << ", " << pEz[0] << ")" << std::endl;
	std::cout << "Last point: (" << pEx[pEx.size() - 1] << ", " << pEy[pEy.size() - 1] << ", " << pEz[pEz.size() - 1] << ")" << std::endl;

}

// Thiết lập số lượng điểm trên quỹ đạo.
// Cấp phát lại bộ nhớ cho tất cả các vector lưu trữ dữ liệu quỹ đạo (vị trí, vận tốc, gia tốc của điểm cuối và biến khớp).
void CRobot::SetNumberPoint(const int& n)
{
	clear();
	pEx.resize(n);
	pEy.resize(n);
	pEz.resize(n);
	pExv.resize(n);
	pEyv.resize(n);
	pEzv.resize(n);
	pExa.resize(n);
	pEya.resize(n);
	pEza.resize(n);

	for (int i = 0; i < NUMBER_LINKS; i++)
	{
		q[i].resize(n);
		qv[i].resize(n);
		qa[i].resize(n);

	}
}
// Xóa tất cả dữ liệu quỹ đạo đã được lưu trữ.
void CRobot::clear()
{
	pEx.clear();
	pEy.clear();
	pEz.clear();
	pExv.clear();
	pEyv.clear();
	pEzv.clear();
	pExa.clear();
	pEya.clear();
	pEza.clear();

	for (int i = 0; i < NUMBER_LINKS; i++)
	{
		q[i].clear();
		qv[i].clear();
		qa[i].clear();
	}
}
// Thiết lập một quỹ đạo mẫu trong không gian khớp.
// Tạo ra một chuỗi các giá trị biến khớp (q) theo một quy luật định trước (nội suy tuyến tính).
void CRobot::SetVariables()
{
	// TODO: Add your implementation code here.
	int nPoint = 101;
	SetNumberPoint(nPoint);

	double deg2rad = PI / 180.0; // Hằng số đổi độ sang radian

	for (int j = 0; j < nPoint; j++)
	{

		// Biến t chạy từ 0.0 đến 1.0 (tỉ lệ phần trăm hành trình)
		double t = (double)j / (nPoint - 1);

		q[0][j] = 0.0;
		// di chuyen tu 100 den 200 tinh tienh bang mm
		q[1][j] = 100 + t * (200 - 100);
		// tu 0 den 180 do
		q[2][j] = (0 + t * (90 - 0)) * deg2rad;
		// tu -90 do den 90 do
		q[3][j] = (-90 + t * (90 - (-90))) * deg2rad;
	}
}

// Hàm tính toán ma trận Jacobian (F) và vector lỗi (f) cho phương pháp Newton-Raphson.
// Đây là hàm được truyền vào giải thuật tìm nghiệm.
void funcJacobian(smatrix& F, vectorm& f, const vectorm& x)
{
	F.SetSize(GetSize(x));
	f.SetSize(GetSize(x));

	double c2 = cos(x[1]);
	double s2 = sin(x[1]);
	double c23 = cos(x[1] + x[2]);
	double s23 = sin(x[1] + x[2]);
	double a2 = pRobot->links[2].a;
	double a3 = pRobot->links[3].a;

	F(0, 0) = 0;
	F(0, 1) = -a2 * s2 - a3 * s23;
	F(0, 2) = -a3 * s23;

	F(1, 0) = 0;
	F(1, 1) = a2 * c2 + a3 * c23;
	F(1, 2) = a3 * c23;

	F(2, 0) = 1;
	F(2, 1) = 0;
	F(2, 2) = 0;

	f(0) = a2 * c2 + a3 * c23 - pRobot->pEx[pRobot->idCurrentPoint];
	f(1) = a2 * s2 + a3 * s23 - pRobot->pEy[pRobot->idCurrentPoint];
	f(2) = x[0] - pRobot->ltool - pRobot->pEz[pRobot->idCurrentPoint];

}
// Thiết lập một quỹ đạo mẫu trong không gian Descartes (không gian làm việc).
// Cụ thể là một quỹ đạo xoắn ốc.
void CRobot::SetEndPoint()
{

	int numSegments = 100;
	SetNumberPoint(numSegments + 1);
	double x0 = 250, y0 = 250, r = 100; // day vao tam duong lam viec
	double dAngle = 2 * PI / numSegments;
	for (int j = 0; j < numSegments + 1; j++)
	{
		// van toc diem cuoi
		pEx[j] = x0 + r * sin(j * dAngle);
		pEy[j] = y0 + r * cos(j * dAngle);
		pEz[j] = 100 + j * 100 / numSegments;

		// vam toc diem cuoi
		pExv[j] = r * cos(j * dAngle) * dAngle;
		pEyv[j] = -r * sin(j * dAngle) * dAngle;
		pEzv[j] = 100 / numSegments;

		// gia toc diem cuoi
		pExa[j] = -r * sin(j * dAngle) * dAngle * dAngle;
		pEya[j] = -r * cos(j * dAngle) * dAngle * dAngle;
		pEza[j] = 0;
	}
}

// Giải bài toán động học ngược cho vị trí.
// Tìm các giá trị biến khớp (q) để điểm cuối đạt được vị trí (pEx, pEy, pEz) mong muốn.
// Sử dụng phương pháp lặp Newton-Raphson.
void CRobot::SolvingInverseKinematics_Position()
{
	// TODO: Add your implementation code here.
	vectorm x(NUMBER_LINKS - 1);
	x(0) = 100;
	x(1) = PI / 4;
	x(2) = PI / 6;
	double eps = 0.0001; //epsilon
	int maxloop = 500;
	for (int j = 0; j < pEx.size(); j++)
	{
		idCurrentPoint = j;
		if (Newton_Raphson(x, funcJacobian, eps, maxloop) == 0)
		{
			q[0][j] = 0.0;
			for (int i = 1; i < NUMBER_LINKS; i++)
			{
				q[i][j] = x(i - 1);
			}
		}

	}
}
// Giải bài toán động học ngược cho vận tốc.
// Tính toán vận tốc các khớp (qv) từ vận tốc của điểm cuối (pExv, pEyv, pEzv) và vị trí khớp hiện tại (q).
// Dựa trên mối quan hệ: p_dot = J(q) * q_dot, suy ra q_dot = J(q)^-1 * p_dot.
void CRobot::SolvingInverseKinematics_Velocity()
{
	// TODO: Add your implementation code here.
	int numEq = NUMBER_LINKS - 1; // number of equations
	smatrix A(numEq);
	vectorm b(numEq);
	vectorm x(numEq);


	for (int j = 0; j < pExv.size(); j++)
	{
		double c2 = cos(q[2][j]);
		double s2 = sin(q[2][j]);
		double c23 = cos(q[2][j] + q[3][j]);
		double s23 = sin(q[2][j] + q[3][j]);
		double a2 = links[2].a;
		double a3 = links[3].a;
		A(0, 0) = 0;
		A(0, 1) = -a2 * s2 - a3 * s23;
		A(0, 2) = -a3 * s23;
		A(1, 0) = 0;
		A(1, 1) = a2 * c2 + a3 * c23;
		A(1, 2) = a3 * c23;
		A(2, 0) = 1;
		A(2, 1) = 0;
		A(2, 2) = 0;

		b(0) = pExv[j];
		b(1) = pEyv[j];
		b(2) = pEzv[j];

		Gauss_Jordan(A, b, x);
		qv[0][j] = 0.0;
		qv[1][j] = x[0]; //vd1
		qv[2][j] = x[1]; //vtheta1
		qv[3][j] = x[2]; //vtheta2
	}
}
// Giải bài toán động học ngược cho gia tốc.
// Tính toán gia tốc các khớp (qa) từ gia tốc điểm cuối (pExa, pEya, pEza), vị trí và vận tốc khớp.
// Dựa trên mối quan hệ: p_ddot = J_dot * q_dot + J * q_ddot.
void CRobot::SolvingInverseKinematics_Acceleration()
{
	// TODO: Add your implementation code here.
	int numEq = NUMBER_LINKS - 1; // number of equations
	smatrix A(numEq);
	vectorm b(numEq);
	vectorm x(numEq);

	for (int j = 0; j < pExv.size(); j++)
	{

		double c2 = cos(q[2][j]);
		double s2 = sin(q[2][j]);
		double c23 = cos(q[2][j] + q[3][j]);
		double s23 = sin(q[2][j] + q[3][j]);
		double a2 = links[2].a;
		double a3 = links[3].a;
		A(0, 0) = 0;
		A(0, 1) = -a2 * s2 - a3 * s23;
		A(0, 2) = -a3 * s23;
		A(1, 0) = 0;
		A(1, 1) = a2 * c2 + a3 * c23;
		A(1, 2) = a3 * c23;
		A(2, 0) = 1;
		A(2, 1) = 0;
		A(2, 2) = 0;

		//binh + q
		b(0) = pExa[j] + a2 * c2 * pow(qv[2][j], 2) + a3 * c23 * pow(qv[2][j] + qv[3][j], 2);
		b(1) = pEya[j] + a2 * s2 * pow(qv[2][j], 2) + a3 * s23 * pow(qv[2][j] + qv[3][j], 2);
		b(2) = pEza[j];

		Gauss_Jordan(A, b, x);

		qa[0][j] = 0.0;
		qa[1][j] = x[0]; //ad1
		qa[2][j] = x[1]; //atheta1
		qa[3][j] = x[2]; //atheta2
	}
}
