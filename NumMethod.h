#pragma once

#include "Matrix.h"

int Gauss_Jordan(const smatrix& a, const vectorm& f, vectorm& x);
//Giai he ft dai so tuyen tinh bang phuong phap Gauss_Jordan
//Tru xoay trong hang (hoan vi cot)
//a : matran he so
//f : vector ve phai
//x : vector chua ket qua
//Neu thanh cong ham tra lai 0 nguoc lai ham cho -1

//////////////////////////////////////////////////////////////////////////

int InverseByGaussJordan(const smatrix& m1, smatrix& m2);
//Tinh nghich dao cua ma tran bang cach giai he phuong trinh dai so tuyen tinh
//voi phuong phap GaussJordan
//m1 : la ma tran ban dau
//m2 : la ma tran nghich dao cua m1
//ham tra lai 0 neu thanh cong
// "   "   " -1 neu khong thanh cong

//////////////////////////////////////////////////////////////////////////

typedef void functionNR(smatrix& a, vectorm& c, const vectorm& x);

int Newton_Raphson(vectorm& x, functionNR& func,const double& eps = 1e-10, const int& Max = 100);
//Giai he phuong trinh phi tuyen bang phuong Newton_Raphson
//x : vector chua sap xi dau va se chua ket qua ra
//eps:do chinh xac can dat
//Max:so lan lap toi da cho phep
//Ham tra lai 0 neu thanh cong; nguoc lai ham cho -1
// Buoc 1 : F(x)*X=-f(x)
//Chu y : de su dung ham truoc het phai tao ham Func(Smatrix & A,vector & c,vector & x);
/*
void Func(smatrix & a,vectorm & c,const vectorm & x)
{
	a(0,0)=5.0; a(0,1)=6.0; a(0,2)=3.0;
	a(1,0)=1.0; a(1,1)=1.0; a(1,2)=1.0;
	a(2,0)=2.0; a(2,1)=7.0; a(2,2)=0.5;
	c[0]=(5.0*x[0]+6.0*x[1]+3.0*x[2]-26.0);
	c[1]=(x[0]+x[1]+x[2]-6.0);
	c[2]=(2.0*x[0]+7.0*x[1]+0.5*x[2]-17.5);
//Ket qua nghiem cua phuong trinh nay la :
// x[0]=1;
// x[1]=2;
// x[3]=3;
}
*/
//	  ham se tao matran Jacobi dat vao  matran a,
//	  vector c chua vector f(x)
//	  vector x chua X (la vector dau vao)