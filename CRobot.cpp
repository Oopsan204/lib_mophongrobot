#include "CRobot.h"
#include "NumMethod.h"
CRobot* pRobot = NULL;
// Hàm khởi tạo của lớp CRobot.
// Gọi hàm init() để thiết lập các thông số ban đầu và gán con trỏ toàn cục pRobot.
CRobot::CRobot()
{
	idCurrentPoint = 0; // Khởi tạo idCurrentPoint
	m_dtimer = 0.1; // Khởi tạo thời gian mỗi bước (100ms = 0.1s)
	init();
	SetVariables();
	SolvingForwardKinematics();
	pRobot = this;
}

// Hàm khởi tạo các thông số của robot.
// Thiết lập các tham số DH (chiều dài khâu) và chiều dài của dụng cụ.
void CRobot::init()
{
	double L1 = 481.0; // chieu dai khau 1
	double L2 = 560.0; // chieu dai khau 2
	double L3 = 659.0; // chieu dai khau 3


	//Khoi tao cac tham so DH mac dinh cho robot 3 khau
	// Khau 0: khau de (fixed base)
	links[0].SetConstants(0.0, 0.0, 0.0, 0.0);
	// khau 1 : khop quay q1
	links[1].SetConstants(0.0,PI/2, 0.0, L1);
	// khau 2 : khop quay q2
	links[2].SetConstants(0.0, 0.0, L2, 0.0);
	// khau 3 : khop quay q3
	links[3].SetConstants(0.0, 0.0, L3, 0.0);
	
	ltool = 0.0; //chieu dai dao
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
		links[1].SetParameters(q[1][j], 0.0, 0.0, 0.0); // revolute joint
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
		// q1 :quay tu -170 den 170 do
		q[1][j] = (-170 + t * (170 - (-170))) * deg2rad;
		// q2 : tu +135 do den -105 do
		q[2][j] = (135 + t * (-105 - 135)) * deg2rad;
		// q3 : tu 62 do den -205 do
		q[3][j] = (62 + t * (-205 - 62)) * deg2rad;
	}
}

// Hàm tính toán ma trận Jacobian (F) và vector lỗi (f) cho phương pháp Newton-Raphson.
// Đây là hàm được truyền vào giải thuật tìm nghiệm.
void funcJacobian(smatrix& F, vectorm& f, const vectorm& x)
{
	F.SetSize(GetSize(x));
	f.SetSize(GetSize(x));


	// x[0] = q1, x[1] = q2, x[2] = q3
	double q1 = x[0];
	double q2 = x[1];
	double q3 = x[2];

	double c1 = cos(q1);
	double s1 = sin(q1);
	double c2 = cos(q2);
	double s2 = sin(q2);
	double c23 = cos(q2 + q3);
	double s23 = sin(q2 + q3);

	double L1 = pRobot->links[1].d; // chieu dai khau 1
	double L2 = pRobot->links[2].a; // chieu dai khau 2
	double L3 = pRobot->links[3].a; // chieu dai khau 3

	// Ma trận Jacobian F = ∂EE/∂q
    // Dòng 1: ∂(px)/∂q

	F(0,0) = -s1 * (L3*c23 +L2*c2); // ∂(px)/∂q1
	F(0,1) = c1 * (-L3* s23 - L2 * s2); // ∂(px)/∂q2
	F(0,2)= -c1 * L3 * s23; // ∂(px)/∂q3

	// Dòng 2: ∂(py)/∂q
	F(1,0)= c1 * (L3 * c23 + L2 * c2); // ∂(py)/∂q1
	F(1,1)= s1 * (-L3 * s23 - L2 * s2); // ∂(py)/∂q2
	F(1,2)= -s1 * L3 * s23; // ∂(py)/∂q3

	// Dòng 3: ∂(pz)/∂q
	F(2,0)= 0; // ∂(pz)/∂q1
	F(2,1)= -L3 * c23 - L2 * c2; // ∂(pz)/∂q2
	F(2,2)= -L3 * c23; // ∂(pz)/∂q3

	// Vector lỗi f = EE_current - EE_target
	//tính Forward kinematics tai q hien tai
	double px_fk = c1 * (L3*c23 + L2*c2);
    double py_fk = s1 * (L3*c23 + L2*c2);
    double pz_fk = L1 - L3*s23 - L2*s2 - pRobot->ltool;

    f(0) = px_fk - pRobot->pEx[pRobot->idCurrentPoint];
    f(1) = py_fk - pRobot->pEy[pRobot->idCurrentPoint];
    f(2) = pz_fk - pRobot->pEz[pRobot->idCurrentPoint];

}
// Thiết lập một quỹ đạo mẫu trong không gian Descartes (không gian làm việc).
// Cụ thể là một quỹ đạo xoắn ốc.
void CRobot::SetEndPoint()
{

	int numSegments = 100;
	SetNumberPoint(numSegments + 1);
	double x0 = 600, y0 = 0, r = 300; // day vao tam duong lam viec
	double z_base = 400; // độ cao trung bình
	double dAngle = 2 * PI / numSegments;
	for (int j = 0; j < numSegments + 1; j++)
	{
		// van toc diem cuoi
		pEx[j] = x0 + r * sin(j * dAngle);
		pEy[j] = y0 + r * cos(j * dAngle);
		pEz[j] = z_base + 100 *sin(2*j*dAngle); // dao dong len xuong

		// vam toc diem cuoi
		pExv[j] = -r * cos(j * dAngle) * dAngle;
		pEyv[j] = r * sin(j * dAngle) * dAngle;
		pEzv[j] = 200 * cos(2 * j * dAngle) * dAngle;

		// gia toc diem cuoi
		pExa[j] = -r * sin(j * dAngle) * dAngle * dAngle;
		pEya[j] = -r * cos(j * dAngle) * dAngle * dAngle;
		pEza[j] = -400 * sin(2 * j * dAngle) * dAngle * dAngle;
	}
}

// Giải bài toán động học ngược cho vị trí.
// Tìm các giá trị biến khớp (q) để điểm cuối đạt được vị trí (pEx, pEy, pEz) mong muốn.
// Sử dụng phương pháp lặp Newton-Raphson.
void CRobot::SolvingInverseKinematics_Position()
{
	// TODO: Add your implementation code here.
	vectorm x(NUMBER_LINKS - 1);
	x(0) = 0.0;
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
        double q1 = q[1][j];
        double q2 = q[2][j];
        double q3 = q[3][j];
        
        double c1 = cos(q1);
        double s1 = sin(q1);
        double c2 = cos(q2);
        double s2 = sin(q2);
        double c23 = cos(q2 + q3);
        double s23 = sin(q2 + q3);
        
        double L2 = links[2].a;
        double L3 = links[3].a;

        // Ma trận Jacobian A = J(q)
        A(0, 0) = -s1 * (L3*c23 + L2*c2);
        A(0, 1) = c1 * (-L3*s23 - L2*s2);
        A(0, 2) = -c1 * L3*s23;
        
        A(1, 0) = c1 * (L3*c23 + L2*c2);
        A(1, 1) = s1 * (-L3*s23 - L2*s2);
        A(1, 2) = -s1 * L3*s23;
        
        A(2, 0) = 0;
        A(2, 1) = -L3*c23 - L2*c2;
        A(2, 2) = -L3*c23;

        b(0) = pExv[j];
        b(1) = pEyv[j];
        b(2) = pEzv[j];

        Gauss_Jordan(A, b, x);
        
        qv[0][j] = 0.0;
        qv[1][j] = x[0]; // vQ1
        qv[2][j] = x[1]; // vQ2
        qv[3][j] = x[2]; // vQ3
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

        double q1 = q[1][j];
        double q2 = q[2][j];
        double q3 = q[3][j];
        
        double c1 = cos(q1);
        double s1 = sin(q1);
        double c2 = cos(q2);
        double s2 = sin(q2);
        double c23 = cos(q2 + q3);
        double s23 = sin(q2 + q3);
        
        double L2 = links[2].a;
        double L3 = links[3].a;

        // Ma trận Jacobian (giống velocity)
        A(0, 0) = -s1 * (L3*c23 + L2*c2);
        A(0, 1) = c1 * (-L3*s23 - L2*s2);
        A(0, 2) = -c1 * L3*s23;
        
        A(1, 0) = c1 * (L3*c23 + L2*c2);
        A(1, 1) = s1 * (-L3*s23 - L2*s2);
        A(1, 2) = -s1 * L3*s23;
        
        A(2, 0) = 0;
        A(2, 1) = -L3*c23 - L2*c2;
        A(2, 2) = -L3*c23;

        // Tính J_dot * q_dot (phần phi tuyến)
        double qd1 = qv[1][j];
        double qd2 = qv[2][j];
        double qd3 = qv[3][j];
        
        // ∂²px/(∂q∂t) * q_dot
        double Jdot_qd_x = -c1*(L3*c23 + L2*c2)*qd1*qd1 
                           + s1*(L3*s23 + L2*s2)*qd1*qd2 
                           + s1*L3*s23*qd1*qd3
                           - c1*(L3*c23 + L2*c2)*(qd2*qd2)
                           - c1*L3*c23*(qd3*qd3)
                           - 2*c1*L3*c23*qd2*qd3;
        
        // ∂²py/(∂q∂t) * q_dot
        double Jdot_qd_y = -s1*(L3*c23 + L2*c2)*qd1*qd1 
                           - c1*(L3*s23 + L2*s2)*qd1*qd2 
                           - c1*L3*s23*qd1*qd3
                           - s1*(L3*c23 + L2*c2)*(qd2*qd2)
                           - s1*L3*c23*(qd3*qd3)
                           - 2*s1*L3*c23*qd2*qd3;
        
        // ∂²pz/(∂q∂t) * q_dot
        double Jdot_qd_z = (L3*s23 + L2*s2)*(qd2*qd2)
                           + L3*s23*(qd3*qd3)
                           + 2*L3*s23*qd2*qd3;

        b(0) = pExa[j] - Jdot_qd_x;
        b(1) = pEya[j] - Jdot_qd_y;
        b(2) = pEza[j] - Jdot_qd_z;

        Gauss_Jordan(A, b, x);

        qa[0][j] = 0.0;
        qa[1][j] = x[0]; // aQ1
        qa[2][j] = x[1]; // aQ2
        qa[3][j] = x[2]; // aQ3
	}
}
