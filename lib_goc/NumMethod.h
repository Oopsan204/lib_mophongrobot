#pragma once

#include  "Matrix.h"

// Giải hệ phương trình đại số tuyến tính bằng phương pháp Gauss-Jordan.
// Sử dụng phương pháp xoay vòng trong hàng (hoán vị cột).
// a: ma trận hệ số.
// f: vector vế phải.
// x: vector chứa kết quả nghiệm.
// Hàm trả về 0 nếu thành công, ngược lại trả về -1.
int Gauss_Jordan(const smatrix& a, const vectorm& f, vectorm& x);


//////////////////////////////////////////////////////////////////////////

// Tính ma trận nghịch đảo bằng phương pháp Gauss-Jordan.
// m1: ma trận đầu vào.
// m2: ma trận đầu ra (nghịch đảo của m1).
// Hàm trả về 0 nếu thành công, -1 nếu không thành công.
int InverseByGaussJordan(const smatrix& m1, smatrix& m2);


//////////////////////////////////////////////////////////////////////////

// Định nghĩa một kiểu con trỏ hàm cho phương pháp Newton-Raphson.
// Hàm này sẽ tính toán ma trận Jacobian và vector hàm f(x).
typedef void functionNR(smatrix& a, vectorm& c, const vectorm& x);

// Giải hệ phương trình phi tuyến bằng phương pháp Newton-Raphson.
// x: vector chứa xấp xỉ ban đầu và sẽ chứa kết quả cuối cùng.
// func: hàm người dùng định nghĩa để tính ma trận Jacobian (a) và vector hàm (c) tại điểm x.
// eps: độ chính xác mong muốn.
// Max: số lần lặp tối đa cho phép.
// Hàm trả về 0 nếu hội tụ thành công, ngược lại trả về -1.
int Newton_Raphson(vectorm& x, functionNR& func,const double& eps = 1e-10, const int& Max = 100);

/*
Ví dụ về cách định nghĩa hàm 'func' cho Newton_Raphson:

void myFunc(smatrix & a, vectorm & c, const vectorm & x)
{
	// Tính ma trận Jacobian a(i,j) = df[i]/dx[j]
	a(0,0)=5.0; a(0,1)=6.0; a(0,2)=3.0;
	a(1,0)=1.0; a(1,1)=1.0; a(1,2)=1.0;
	a(2,0)=2.0; a(2,1)=7.0; a(2,2)=0.5;

	// Tính vector hàm c[i] = f[i](x)
	c[0]=(5.0*x[0]+6.0*x[1]+3.0*x[2]-26.0);
	c[1]=(x[0]+x[1]+x[2]-6.0);
	c[2]=(2.0*x[0]+7.0*x[1]+0.5*x[2]-17.5);
}
*/