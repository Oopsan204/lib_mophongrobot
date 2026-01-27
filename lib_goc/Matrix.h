// matrix.h: interface for the matrix class.
#pragma once

#include <iostream>
using namespace std;

#ifndef PI
	#define PI 3.1415926535897
#endif
//===========================================================================
//===========================================================================
// LỚP MA TRẬN CƠ SỞ (CLASS BASE MATRIX)
//===========================================================================
//===========================================================================
class matrix
{
protected:
   int sizecol/*số cột*/,sizerow/*số hàng*/;
   double **data; // Con trỏ đôi để lưu trữ dữ liệu ma trận
//===========================================================================
public:
// constructors/destructor
   // Hàm khởi tạo mặc định
   matrix();
   // Hàm khởi tạo với số hàng và số cột cho trước
   matrix(const int & m,const int & n);
   // Hàm khởi tạo sao chép
   matrix(const matrix & matrix1);
   // Hàm hủy
   ~matrix();
//===========================================================================
// Các toán tử với ma trận khác
   // Cộng hai ma trận
   friend matrix  operator+(const matrix & matrix1,const matrix & matrix2);
   // Trừ hai ma trận
   friend matrix  operator-(const matrix & matrix1,const matrix & matrix2);
   // Nhân hai ma trận
   friend  matrix  operator*(const matrix & matrix1,const matrix & matrix2);
//===========================================================================
// Các toán tử gán với một số vô hướng
   // Cộng ma trận với một số
   matrix operator+=(const double & add);
   // Trừ ma trận cho một số
   matrix operator-=(const double & minus);
   // Nhân ma trận với một số
   matrix operator*=(const double & multiply);
   // Chia ma trận cho một số
   matrix operator/=(const double & devide);
//===========================================================================
// Các toán tử với một số vô hướng (dạng friend)
   // Cộng một số với ma trận
   friend matrix  operator+(const double & add,const matrix& matrix1);
   // Cộng ma trận với một số
   friend matrix  operator+(const matrix& matrix1,const double & add);
   // Trừ một số cho ma trận
   friend matrix  operator-(const double & minus,const matrix& matrix1);
   // Trừ ma trận cho một số
   friend matrix  operator-(const matrix& matrix1,const double & minus);
   // Nhân ma trận với một số
   friend matrix  operator*(const matrix& matrix1,const double & multiply);
   // Nhân một số với ma trận
   friend matrix  operator*(const double & multiply,const matrix& matrix1);
   // Chia ma trận cho một số
   matrix operator/(const double & devide)const;
//===========================================================================
// Các toán tử gán với ma trận khác
    // Cộng dồn ma trận
    matrix operator+=(const matrix & matrix1);
    // Trừ dồn ma trận
    matrix operator-=(const matrix & matrix1);

//===========================================================================
// Toán tử tăng/giảm (hậu tố)
   // Tăng tất cả phần tử lên 1 (hậu tố)
   matrix operator++(int);
   // Giảm tất cả phần tử đi 1 (hậu tố)
   matrix operator--(int);

// Toán tử tăng/giảm (tiền tố)
   // Tăng tất cả phần tử lên 1 (tiền tố)
   matrix& operator++();
   // Giảm tất cả phần tử đi 1 (tiền tố)
   matrix& operator--();
//===========================================================================
   // Toán tử chuyển vị (dạng friend)
   friend matrix  operator!(const matrix & matrix1);
//===========================================================================
// Truy cập phần tử
   // Lấy giá trị phần tử (hằng)
   double const & operator()(const int &i,const int &j)const;
   // Lấy/Gán giá trị phần tử
   double & operator()(const int &i,const int &j);
//===========================================================================
// Nhập/Xuất
   // In ma trận ra màn hình (dạng friend)
   friend void  print(const matrix& matrix1);
   // Toán tử xuất ra stream
   friend ostream& operator<<(ostream & co,const matrix & matrix2);
   // Toán tử nhập từ stream
   friend istream& operator>>(istream & ci,matrix & matrix2);
//===========================================================================
// Phép gán và so sánh
   // Toán tử gán
   matrix & operator=(const matrix&matrix1);
   // Toán tử so sánh bằng
   friend int  operator==(const matrix&matrix1,const matrix&matrix2);
//===========================================================================
// Lấy kích thước
   // Lấy số cột
   friend int  GetCol(const matrix&matrix1);
   // Lấy số hàng
   friend int  GetRow(const matrix&matrix1);
//===========================================================================
// Toán tử đổi dấu
   matrix operator-(const matrix&matrix1);
//===========================================================================
// Tính các chuẩn của ma trận
   // Tính chuẩn cột (chuẩn 1)
   friend double  StandardCollum(const matrix & m1);
   // Tính chuẩn hàng (chuẩn vô cùng)
   friend double  StandardRow(const matrix & m1);
   // Tính chuẩn Euclide (chuẩn Frobenius)
   friend double  StandardEuclide(const matrix & m1);
//===========================================================================
// Các hàm tiện ích khác
   // Thiết lập giá trị một phần tử
   friend void  SetMember(matrix & mt,const int& i,const int& j, const double & d);
	// Tính chuẩn cực đại (giá trị tuyệt đối lớn nhất của phần tử)
	friend double  StandardMax(const matrix & m);
	// Hoán vị hai cột
	friend void  SwapCol(matrix & m,const int & i,const int&j);
	// Hoán vị hai hàng
	friend void  SwapRow(matrix & m,const int & i,const int&j);
	// Tìm vị trí phần tử lớn nhất trong cột
	friend int  GetMaxInCol(const matrix &m,const int& i);
	// Tìm vị trí phần tử lớn nhất trong hàng
	friend int  GetMaxInRow(const matrix &m,const int& i);
	// Tìm vị trí phần tử có giá trị tuyệt đối lớn nhất trong cột
	friend int  GetAbsMaxInCol(const matrix &m,const int& i);
	// Tìm vị trí phần tử có giá trị tuyệt đối lớn nhất trong hàng
	friend int  GetAbsMaxInRow(const matrix &m,const int& i);
	// Thay đổi kích thước ma trận
	void SetSize(const int & m,const int & n);
   // Hàm chuyển vị
   friend matrix  Transpose(const matrix & matrix1);
};
//***************************************************************************
//***************************************************************************
// LỚP MA TRẬN VUÔNG (CLASS SQUARE MATRIX)
//***************************************************************************
//***************************************************************************
class  smatrix : public matrix
{
private: int size; // Kích thước (số hàng = số cột)
public:
// constructors/destructor
   // Hàm khởi tạo mặc định
   smatrix();
   // Hàm hủy
   ~smatrix();
   // Hàm khởi tạo với kích thước cho trước
   smatrix(const int & m);
   // Hàm khởi tạo sao chép từ ma trận vuông khác
   smatrix(const smatrix & smatrix1);
   // Hàm khởi tạo từ một ma trận thường (nếu là ma trận vuông)
   smatrix(const matrix & ma1);
// Phân tích và tính toán
   // Phân tích LU. boolRes=0 nếu thành công.
   friend matrix  LU(const smatrix & m1,int & boolRes);
   // Tính định thức. boolRes=0 nếu thành công.
   friend double  det(const smatrix & m1,int& boolRes);
   // Tính ma trận nghịch đảo. boolRes=0 nếu thành công.
   friend smatrix  inverse(const smatrix & m1,int & boolRes);
// Kích thước và thiết lập
   // Lấy kích thước ma trận vuông
   friend int  GetSize(const smatrix & m1);
   // Thay đổi kích thước ma trận vuông
   void SetSize(const int & m);
	// Chuyển ma trận thành ma trận đơn vị
   friend void	 SetToUnit(smatrix & m);
// Chuẩn hóa
   // Chuẩn hóa theo cột (độ dài mỗi vector cột = 1)
   friend smatrix  NormalizeByCol(const smatrix & m);
   // Chuẩn hóa theo hàng (độ dài mỗi vector hàng = 1)
   friend smatrix  NormalizeByRow(const smatrix & m);
};

//***************************************************************************
//***************************************************************************
// LỚP VECTOR (Kế thừa từ ma trận) => VECTORM
//***************************************************************************
//***************************************************************************

class  vectorm : public matrix
{
protected: int size; // Kích thước (số phần tử)
public:
// constructors/destructor
	// Hàm khởi tạo mặc định
	vectorm();
	// Hàm khởi tạo sao chép
	vectorm(const vectorm& v);
	// Hàm khởi tạo vector 2D
	vectorm(const double& x, const double& y);
	// Hàm khởi tạo vector 3D
	vectorm(const double& x, const double& y, const double& z);
	// Hàm khởi tạo với kích thước cho trước
	vectorm(const int& m);
	// Hàm hủy
	~vectorm();
// Kích thước
	// Thay đổi kích thước vector
	void SetSize(const int& m);
// Tính độ dài và chuẩn
	// Tính độ dài (module) của vector
	friend double  module(const vectorm& a);
	friend double  length(const vectorm& a);
	// Tính tổng các giá trị tuyệt đối (chuẩn 1)
	friend double  StandardVal(const vectorm& a);
	// Tính chuẩn Euclide (độ dài, chuẩn 2)
	friend double  StandardEuclide(const vectorm& a);
	// Tính chuẩn cực đại (phần tử lớn nhất, chuẩn vô cùng)
	friend double  StandardMax(const vectorm& a);
// Chuẩn hóa
	// Chuẩn hóa vector (chia cho phần tử có giá trị tuyệt đối lớn nhất)
	friend vectorm  standard(const vectorm& a);
	// Chuẩn hóa Euclide (đưa về vector đơn vị có độ dài = 1)
	friend vectorm  normalize(const vectorm& a);
// Chuyển đổi
	// Khởi tạo vector từ ma trận (một hàng hoặc một cột)
	vectorm(const matrix& ma1);
// Truy cập phần tử
	// Lấy giá trị phần tử (hằng)
	double const& operator()(const int& i)const;
	// Lấy/Gán giá trị phần tử
	double& operator()(const int& i);
	// Lấy/Gán giá trị phần tử (dạng mảng)
	double& operator[](const int& i);
	double const& operator[](const int& i)const;
// Các phép toán vector
	// Tích vô hướng (dot product)
	friend double  dot(const vectorm& matrix1, const vectorm& matrix2);
	// Tích có hướng (cross product)
	friend vectorm  cross(const vectorm& vectorm1, const vectorm& vectorm2);
	// Tính góc giữa hai vector (radian)
	friend double  angle(const vectorm& vt1, const vectorm& vt2);
// Thiết lập và gán
	// Gán giá trị cho một phần tử
	void SetData(const int& i, const double& d);
	// Lấy kích thước vector
	friend int  GetSize(const vectorm& vt);
	// Toán tử gán
	const vectorm& operator=(const vectorm& v);
};
