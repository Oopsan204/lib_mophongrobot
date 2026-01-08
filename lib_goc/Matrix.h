// matrix.h: interface for the matrix class.
#pragma once

#include <iostream>
using namespace std;

#ifndef PI
	#define PI 3.1415926535897
#endif
//===========================================================================
//===========================================================================
//CLASS BASE MATRIX
//===========================================================================
//===========================================================================
class matrix
{
protected:
   int sizecol/*size collum*/,sizerow/*size row*/;
   double **data;
//===========================================================================
public:
//1
   matrix();
//2
   matrix(const int & m,const int & n);
//3
   matrix(const matrix & matrix1);
//4
   ~matrix();
//===========================================================================
//5
   friend matrix  operator+(const matrix & matrix1,const matrix & matrix2);
   //matrix1+matrix2
//6
   friend matrix  operator-(const matrix & matrix1,const matrix & matrix2);
   //matrix1-matrix2
//7
   friend  matrix  operator*(const matrix & matrix1,const matrix & matrix2);
   //matrix1*matrix2
//===========================================================================
//8
   matrix operator+=(const double & add);//matrix+=add
//9
   matrix operator-=(const double & minus);//matrix-=minus
//10
   matrix operator*=(const double & multiply);//matrix*=multiply
//11
   matrix operator/=(const double & devide);//matrix/=devide
//===========================================================================
//12 a
   //matrix operator+(const double & add)const;//matrix=matrix+add
   //b
   friend matrix  operator+(const double & add,const matrix& matrix1);
   //matrix=add+matrix
   //c
   friend matrix  operator+(const matrix& matrix1,const double & add);
   //matrix=matrix+add
//13 a
  // matrix operator-(const double & minus)const;//matrix=matrix-minus
   //b
   friend matrix  operator-(const double & minus,const matrix& matrix1);
   //matrix=minus-matrix
   //c
   friend matrix  operator-(const matrix& matrix1,const double & minus);
   //matrix=matrix-minus
//14 a
   //matrix operator*(const double & multiply)const;//matrix=matrix*multiply
   //b
   friend matrix  operator*(const matrix& matrix1,const double & multiply);
   //matrix=matrix1*multiply
   //c
   friend matrix  operator*(const double & multiply,const matrix& matrix1);
   //matrix=multiply*matrix1
//15
   matrix operator/(const double & devide)const;//matrix=matrix/devide
//===========================================================================
    //matrix operator+(const matrix & matrix1);//matrix=matrix+matrix1
    //matrix operator-(const matrix & matrix1);//matrix=matrix-matrix1
//16
    matrix operator+=(const matrix & matrix1);//matrix+=matrix1
//17
    matrix operator-=(const matrix & matrix1);//matrix-=matrix1

//===========================================================================
//18
   matrix operator++(int);//matrix++
//19
   matrix operator--(int);//matrix--

//20
   matrix& operator++();//++matrix
//21
   matrix& operator--();//--matrix
//===========================================================================
//22
   friend matrix  operator!(const matrix & matrix1);//chuyen vi
//===========================================================================
   //int&  operator[](int &i)operator[](int &j);//lay phan tu
//23
   double const & operator()(const int &i,const int &j)const;//lay phan tu
   double & operator()(const int &i,const int &j);//lay phan tu
//===========================================================================
//24
   friend void  print(const matrix& matrix1);
   //in mot matrix ra man hinh
//25
   friend ostream& operator<<(ostream & co,const matrix & matrix2);
   //in mot matrix ra man hinh
//26
   friend istream& operator>>(istream & ci,matrix & matrix2);
   //Nhap du lieu cho ma tran
//===========================================================================
//27
//sua chua du lieu ma tran
//   void change();
//===========================================================================
//28
//gan ma tran
   matrix & operator=(const matrix&matrix1);
//29
   //so sanh ma tran
   friend int  operator==(const matrix&matrix1,const matrix&matrix2);
//===========================================================================
   //phep lay kich thuoc matrix
//30
   friend int  GetCol(const matrix&matrix1);//=sizecol
//31
   friend int  GetRow(const matrix&matrix1);//=sizerow
//32
//   matrix operator-();
   matrix operator-(const matrix&matrix1);
//===========================================================================
//Tinh cac chuan cua matran
//33
   friend double  StandardCollum(const matrix & m1);
   //tinh chuan cot
//34
   friend double  StandardRow(const matrix & m1);
   //tinh chuan hang
//35
   friend double  StandardEuclide(const matrix & m1);
   //tinh chuan Euclide
//36
   friend void  SetMember(matrix & mt,const int& i,const int& j, const double & d);//=sizerow
//37
//   void setdata(const int& i,const int& j,const double& d);
//38
	friend double  StandardMax(const matrix & m);
	//Tinh chuan cuc dai
//39
	friend void  SwapCol(matrix & m,const int & i,const int&j);
	//Hoan vi 2 cot
//40
	friend void  SwapRow(matrix & m,const int & i,const int&j);
	//Hoan vi 2 hang
//41
	friend int  GetMaxInCol(const matrix &m,const int& i);
	//lay vi tri phan tu lon nhat trong cot thu i
//42
	friend int  GetMaxInRow(const matrix &m,const int& i);
	//lay vi tri phan tu lon nhat trong hang thu i
//43
	friend int  GetAbsMaxInCol(const matrix &m,const int& i);
	//lay vi tri phan tu lon nhat trong cot thu i
//44
	friend int  GetAbsMaxInRow(const matrix &m,const int& i);
	//lay vi tri phan tu lon nhat trong hang thu i
//45
	void SetSize(const int & m,const int & n);
	//Dat kich thuoc ma tran
//46
   friend matrix  Transpose(const matrix & matrix1);//chuyen vi
};
//***************************************************************************
//***************************************************************************
//CLASS SQUARE MATRIX
//***************************************************************************
//***************************************************************************
class  smatrix : public matrix
{
private: int size;
public:
//1
   smatrix();
   ~smatrix();
//2
   smatrix(const int & m);
//3
   smatrix(const smatrix & smatrix1);
//4
   smatrix(const matrix & ma1);
//5--phep phan tich LU------------------
   friend matrix  LU(const smatrix & m1,int & boolRes);
   //neu thanh cong ham cho boolRes=0 nguoc lai cho boolRes=-1
   //Cho ma tran U la ma tran tam giac tren (tren duong cheo chinh)
   //Cho ma tran L la ma tran tam giac duoi (duoi duong cheo chinh) voi cac phan tu tren duong cheo chinh = 1
//6--phep tinh dinh thuc-----------------
   friend double  det(const smatrix & m1,int& boolRes);
   //neu thanh cong ham cho bool=0 nguoc lai cho bool=-1
//7--phep tinh matran nghich dao---------
   friend smatrix  inverse(const smatrix & m1,int & boolRes);
	 //neu thanh cong ham cho bool=0 nguoc lai cho bool=-1
//8--phep lay kich thuoc ma tran
   friend int  GetSize(const smatrix & m1);
//9
   void SetSize(const int & m);
	//Dat kich thuoc ma tran
//10
   friend void	 SetToUnit(smatrix & m);
//11
   friend smatrix  NormalizeByCol(const smatrix & m);//chuan hoa cho cac cot : co do dai =1
   friend smatrix  NormalizeByRow(const smatrix & m);//chuan hoa cho cac hang : co do dai =1
};

//***************************************************************************
//***************************************************************************
//CLASS VECTOR FROM MATRIX => VECTORM
//***************************************************************************
//***************************************************************************

class  vectorm : public matrix
{
protected: int size;//double *data;
public:
	//1
	vectorm();
	//2
	vectorm(const vectorm& v);
	vectorm(const double& x, const double& y);
	vectorm(const double& x, const double& y, const double& z);
	//3
	vectorm(const int& m);
	void SetSize(const int& m);
	//4
	~vectorm();
	//5
	friend double  module(const vectorm& a); /*Tinh do dai vectorm*/
	friend double  length(const vectorm& a); /*Tinh do dai vectorm*/
	//6
	friend double  StandardVal(const vectorm& a);
	/*Tinh chuan tuyet doi*/
//7
	friend double  StandardEuclide(const vectorm& a);
	/*Tinh chuan Euclide*/
//8
	friend double  StandardMax(const vectorm& a);
	/*Tinh chuan cuc dai*/
//9
	friend vectorm  standard(const vectorm& a);
	//chuan hoa vectorm theo thanh phan co tri tuyet doi lon nhat
 //10
	friend vectorm  normalize(const vectorm& a);   //chuan hoa theo Euclide//
	//tra ve vectorm co do dai = 1 don vi
 //11
	vectorm(const matrix& ma1);
	//khoi tao vectorm tu mot matrix dac biet : chi co 1 hang hoac 1 cot
 //12
	double const& operator()(const int& i)const;
	double& operator()(const int& i);
	//lay phan tu cua vectorm
	double& operator[](const int& i);
	double const& operator[](const int& i)const;
	//13
	friend double  dot(const vectorm& matrix1, const vectorm& matrix2);
	//dot product or scalar product
	//lay tich vo huong cua 2 vectorm (!vectorm1)*vectorm2
 //14
	friend vectorm  cross(const vectorm& vectorm1, const vectorm& vectorm2);
	//cross product
	//lay tich co huong cua 2 vectorm (!vectorm1)*vectorm2
 //-----------
 //15
	friend double  angle(const vectorm& vt1, const vectorm& vt2);
	//cho goc giua 2 vectorm do bang radian
 //17
	void SetData(const int& i, const double& d);
	//19
	friend int  GetSize(const vectorm& vt);
	//28
	const vectorm& operator=(const vectorm& v);
};
