// matrix.cpp: implementation of the matrix class.
#include "../lib/Matrix.h"
#include <stdio.h>
#include <conio.h>
#include <process.h>
#include <math.h>
#include <iomanip>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
//1
matrix::matrix()
{
  sizerow=sizecol=0;data=NULL;
}
//---------------------------------------------------------------------------
//2
matrix::matrix(const int & m,const int & n)
{
   sizerow=m;sizecol=n;
   data=NULL;
   data=new double *[m];
   int i,j;
   for (i=0;i<m;i++)
	  data[i]=new double [n];
   for (i=0;i<m;i++)
	  for (j=0;j<n;j++)
		data[i][j]=0.0;
}
//---------------------------------------------------------------------------
//3
matrix::matrix(const matrix & matrix1)
{
   sizecol=matrix1.sizecol;sizerow=matrix1.sizerow;
   int i,j;
   data=NULL;
   data=new double *[sizerow];
   for (i=0;i<sizerow;i++)
	  data[i]=new double [sizecol];
   for (i=0;i<sizerow;i++)
      for (j=0;j<sizecol;j++)
	data[i][j]=matrix1.data[i][j];
}
//---------------------------------------------------------------------------
//4
matrix::~matrix()
{
   if (data==NULL)
	{
	sizerow=sizecol=0;
	return;
	}
   for (int i=0;i<sizerow;i++)
   {
	delete []data[i];
	data[i]=NULL;
   }
   delete []data;
   data=NULL;
   sizerow=sizecol=0;
}

//5----------------------------------------------------------------------------
matrix operator +(const matrix &matrix1,const matrix &matrix2)
{
	int i,j;
	matrix matrixtg(matrix1.sizerow,matrix2.sizecol);
	for(i=0;i<matrix1.sizerow;i++)
		for(j=0;j<matrix2.sizecol;j++)
		     matrixtg.data[i][j]=matrix1.data[i][j]+matrix2.data[i][j];
	return matrixtg;
}
//6----------------------------------------------------------------------------
matrix operator -(const matrix &matrix1,const matrix &matrix2)
{
	int i,j;
	matrix matrixtg(matrix1.sizerow,matrix2.sizecol);
	for(i=0;i<matrix1.sizerow;i++)
		for(j=0;j<matrix2.sizecol;j++)
		     matrixtg.data[i][j]=matrix1.data[i][j]-matrix2.data[i][j];
	return matrixtg;
  }
//---------------------------------------------------------------------------
//7
matrix operator*(const matrix & matrix1,const matrix & matrix2)//matrix=matrix1*matrix2
  {
  int i,j,k;
   int row=matrix1.sizerow;
   matrix matrixmul(row,matrix2.sizecol);
   if (matrix1.sizecol!=matrix2.sizerow) return matrixmul;
   for (i=0;i<row;i++)
	  for (j=0;j<matrix2.sizecol;j++)
	{
	matrixmul.data[i][j]=0.0;
	for (k=0;k<matrix1.sizecol;k++)
		 matrixmul.data[i][j]+=matrix1.data[i][k]*matrix2.data[k][j];
	}
   return matrixmul;
  }
//8----------------------------------------------------------------------------
matrix matrix::operator+=(const double &add)
{
	int i,j;
	for(i=0;i<sizerow;i++)
		for(j=0;j<sizecol;j++)
			data[i][j]+=add;
	return *this;
}
//9----------------------------------------------------------------------------
matrix matrix:: operator-=(const double &minus)
{
	int i,j;
	for(i=0;i<sizerow;i++)
		for(j=0;j<sizecol;j++)
		    data[i][j]-=minus;
	return *this;
}

//---------------------------------------------------------------------------
//10
matrix matrix::operator*=(const double &multiply)//matrix*=multiply
  {
  int i,j;
  for (i=0;i<sizerow;i++)
    for (j=0;j<sizecol;j++)
       data[i][j]*=multiply;

  return *this;
  }
//---------------------------------------------------------------------------
//11
matrix matrix::operator/=(const double &devide)//matrix/=devide
  {
  if (devide==0) return *this;
  int i,j;
  for (i=0;i<sizerow;i++)
    for (j=0;j<sizecol;j++)
       data[i][j]/=devide;

  return *this;
  }
//12----------------------------------------------------------------------------
/*matrix matrix::operator+(const double &add)const
{
	int i,j;
	for(i=0;i<sizerow;i++)
		for(j=0;j<sizecol;j++)
			data[i][j]=data[i][j]+add;
	return *this;
}
*/
//===================
matrix operator+(const double &add,const matrix &matrix1)
{
   matrix matrixt(matrix1);
   int i,j;
   for (i=0;i<matrix1.sizerow;i++)
      for (j=0;j<matrix1.sizecol;j++)
	  matrixt.data[i][j]=matrix1.data[i][j]+add;

  return matrixt;//*this;
  }
//===================

matrix operator+(const matrix &matrix1,const double &add)
{
   matrix matrixt(matrix1);
   int i,j;
   for (i=0;i<GetRow(matrix1);i++)
	  for (j=0;j<GetCol(matrix1);j++)
	  matrixt.data[i][j]= matrix1(i,j)+add;

  return matrixt;
  }

//13----------------------------------------------------------------------------
/*matrix matrix:: operator-(const double &minus)const
{
	int i,j;
	for(i=0;i<sizerow;i++)
		for(j=0;j<sizecol;j++)
			data[i][j]=data[i][j]-minus;
	return *this;
}
*/
//===================
matrix operator-(const double& minus, const matrix& matrix1)
{
	matrix matrixt(matrix1);
	int i, j;
	//sizerow=matrix1.sizerow;sizecol=matrix1.sizecol;
	for (i = 0; i < matrix1.sizerow; i++)
		for (j = 0; j < matrix1.sizecol; j++)
			matrixt.data[i][j] = minus - matrix1.data[i][j];
	return matrixt;//*this;
}

matrix operator-(const matrix &matrix1,const double &minus)
{
   matrix matrixt(matrix1);
   int i,j;
   //sizerow=matrix1.sizerow;sizecol=matrix1.sizecol;
   for (i=0;i<matrix1.sizerow;i++)
      for (j=0;j<matrix1.sizecol;j++)
	  matrixt.data[i][j]=matrix1.data[i][j]-minus;
  return matrixt;//*this;
}

//---------------------------------------------------------------------------
//14
/*matrix matrix::operator*(const double & multiply)const//matrix=matrix*multiply
  {
   int i,j;
   for (i=0;i<sizerow;i++)
	for (j=0;j<sizecol;j++)
       data[i][j]=data[i][j]*multiply;

  return *this;
  }
*/
//===================

matrix operator*(const matrix& matrix1,const double & multiply)//matrix=matrix1*multiply
  {
   matrix matrixt(matrix1);
   int i,j;
   //sizerow=matrix1.sizerow;sizecol=matrix1.sizecol;
   for (i=0;i<GetRow(matrix1);i++)
	for (j=0;j<GetCol(matrix1);j++)
       matrixt.data[i][j]=matrix1(i,j)*multiply;
  return matrixt;//*this;
  }

//===================
matrix operator*(const double & multiply,const matrix& matrix1)//matrix=multiply*matrix1
  {
   matrix matrixt(matrix1);
   int i,j;
   //sizerow=matrix1.sizerow;sizecol=matrix1.sizecol;
   for (i=0;i<matrix1.sizerow;i++)
    for (j=0;j<matrix1.sizecol;j++)
       matrixt.data[i][j]=matrix1.data[i][j]*multiply;
  return matrixt;//*this;
  }

//---------------------------------------------------------------------------
//15
matrix matrix::operator/(const double & devide)const//matrix=matrix/devide
  {
  if (devide==0) return *this;
  int i,j;
   for (i=0;i<sizerow;i++)
    for (j=0;j<sizecol;j++)
	   data[i][j]=data[i][j]/devide;

  return *this;
  }
//---------------------------------------------------------------------------
//16
matrix matrix::operator+=(const matrix & matrix1)//matrix+=matrix1
  {
  if ((matrix1.sizerow!=sizerow)||(matrix1.sizecol!=sizecol)) return *this;
  int i,j;
  for (i=0;i<sizerow;i++)
	for (j=0;j<sizecol;j++)
	   data[i][j]+=matrix1.data[i][j];
  return *this;
  }
//---------------------------------------------------------------------------
//17
matrix matrix::operator-=(const matrix & matrix1)//matrix-=matrix1
  {
  if ((matrix1.sizerow!=sizerow)||(matrix1.sizecol!=sizecol))return *this;
  int i,j;
  for (i=0;i<sizerow;i++)
	for (j=0;j<sizecol;j++)
	   data[i][j]-=matrix1.data[i][j];
  return *this;
  }
//---------------------------------------------------------------------------
//18----------------------------------------------------------------------------
matrix matrix:: operator++(int)
{
	int i,j;
	for(i=0;i<sizerow;i++)
		for(j=0;j<sizecol;j++)
		    data[i][j]++;
	return *this;
}
//19----------------------------------------------------------------------------
matrix matrix:: operator--(int)
{
	int i,j;
	for(i=0;i<sizerow;i++)
		for(j=0;j<sizecol;j++)
		    data[i][j]--;
	return *this;
}

//20----------------------------------------------------------------------------
matrix& matrix:: operator++()
{
	int i,j;
	for(i=0;i<sizerow;i++)
		for(j=0;j<sizecol;j++)
			++data[i][j];
	return *this;
}
//21----------------------------------------------------------------------------
matrix& matrix:: operator--()
{
	int i,j;
	for(i=0;i<sizerow;i++)
		for(j=0;j<sizecol;j++)
			--data[i][j];
	return *this;
}
//----------------------------------------------------------------------------
//22
  matrix operator!(const matrix & matrix1)//chuyen vi
	 {
	 matrix matrix2(matrix1);
	 int i,j;
	 matrix matrixtg(matrix1.sizecol,matrix1.sizerow);
	 //sizerow=matrix2.sizecol;sizecol=matrix2.sizerow;
	 for (i=0;i<matrixtg.sizerow;i++)
	for (j=0;j<matrixtg.sizecol;j++)
	  matrixtg.data[i][j]=matrix2.data[j][i];
	  //data[i][j]=matrix2.data[j][i];
	 return matrixtg;
     }


//---------------------------------------------------------------------------
//23
   double const & matrix::operator()(const int &i,const int &j)const//lay phan tu
   {
   return data[i][j];
   }
   double & matrix::operator()(const int &i,const int &j)//lay phan tu
   {
   return data[i][j];
   }
//---------------------------------------------------------------------------
//24
void print(const matrix & matrix1)
{
	if (matrix1.data!=NULL)
	{
	int i,j;
	if ((matrix1.sizerow<10)&&(matrix1.sizecol<7))
	{
	//   printf("\n___________Print matrix______________");
	   printf("\nRow \\ Collum \n");
	   printf("   ");//////////////////////////////////////////////
	   for (j=0;j<matrix1.sizecol;j++)
		   {
		   _cprintf("%12i",j+1);
		   }
	   printf("\n");
	   for (i=0;i<matrix1.sizerow;i++)
		 {
		  _cprintf("%3i",i+1);
		  for (j=0;j<matrix1.sizecol;j++)
		{
		_cprintf("%12.6lf",matrix1.data[i][j]);
		if (j==matrix1.sizecol-1)
			{_cprintf("\n");
			 printf("\n");
			}
		}
	  }
	}
	else
	{
	   for (i=0;i<matrix1.sizerow;i++)
		  for (j=0;j<matrix1.sizecol;j++)
		{
		printf("\n");
		_cprintf("  Member:[");
		_cprintf("%i",i+1);
		_cprintf(",");
		_cprintf("%i",j+1);
		_cprintf("] = ");
		_cprintf(" %30lf",matrix1.data[i][j]);
		if (((i+1)*matrix1.sizecol+(1+j))%22==0)
			 {
			  printf("\n");
			  _cprintf("_____ Press any key to continue ....");
			  _getch();
			 }
		   }

	}
	//printf("\n");
	//cprintf("====>>-End of matrix or vector-------------------------------------------");
	//printf("\n");
	}
}
//---------------------------------------------------------------------------
//25
ostream& operator<<(ostream & co,const matrix & matrix2)
{
	/*
	int i,j;
	for (i=0;i<matrix2.sizerow;i++)
	   for (j=0;j<matrix2.sizecol;j++)
	 {
	 co<<"\n  Member :["<<i+1<<","<<j+1<<"] = "<<matrix2.data[i][j];
	 if (((i+1)*matrix2.sizecol+j+1)%22==0)
		  {
		   co<<"\n   ---Press any key to continue ....";
		   _getch();
		  }
	  }
	co<<"\n";
	return co;
	*/
	int i, j;
	for (i = 0; i < matrix2.sizerow; i++)
	{
		for (j = 0; j < matrix2.sizecol; j++)
		{
			co << setiosflags(ios::showpoint) << setw(13) << setprecision(5) << matrix2.data[i][j] << "\t";
		}
		co << "\n";
	}
	return co;
}
//---------------------------------------------------------------------------
//26
istream& operator>>(istream & ci,matrix & matrix2)
{
   int i,j;
   cout<<endl;
   for (i=0;i<matrix2.sizerow;i++)
	  for (j=0;j<matrix2.sizecol;j++)
	{
	 cout<<"  Item: ("<<i+1<<","<<j+1<<") = ";
	 ci>>matrix2.data[i][j];
	}
   return ci;
}
//---------------------------------------------------------------------------
   //sua chua du lieu ma tran
//27
//   void change();


//---------------------------------------------------------------------------
//gan ma tran
//28
matrix & matrix::operator=(const matrix & matrix1)
{
  int i,j;
  if ((sizecol!=matrix1.sizecol)||(sizerow!=matrix1.sizerow))
  {
  if (data!=NULL)
	{
	for (i=0;i<sizerow;i++)
	{
	  delete []data[i];
	  data[i]=NULL;
	}
	delete []data;
	}
   data=NULL;
   sizerow=matrix1.sizerow;sizecol=matrix1.sizecol;
   data=new double *[sizerow];
   for (i=0;i<sizerow;i++)
	  data[i]=new double [sizecol];
  }
   for (i=0;i<matrix1.sizerow;i++)
	 for (j=0;j<matrix1.sizecol;j++)
	   data[i][j]=matrix1.data[i][j];

  return *this;
}

//---------------------------------------------------------------------------
//so sanh ma tran
//29
int operator==(const matrix&matrix1,const matrix&matrix2)
  {
  int boolean;
  if ((matrix1.sizecol==matrix2.sizecol)&&(matrix1.sizerow==matrix2.sizerow))
	{
	int i,j;
	for  (i=0;i<matrix1.sizerow;i++)
	  for (j=0;j<matrix1.sizerow;j++)
	if (matrix1.data[i][j]==matrix2.data[i][j])
		boolean=1;
		else
		{
		boolean=0;

		return boolean;
		}
	 return boolean;
	 }
   return boolean=0;
   }
//---------------------------------------------------------------------------
//30
int GetCol(const matrix& matrix1)
  {
  return matrix1.sizecol;
  }
//31
int GetRow(const matrix& matrix1)
  {
  return matrix1.sizerow;
  }
//---------------------------------------------------------------------------
//32
matrix matrix::operator-(const matrix&matrix1)
     {
     sizerow=matrix1.sizerow;sizecol=matrix1.sizecol;
     int i,j;
	 for (i=0;i<sizerow;i++)
	 for (j=0;j<sizecol;j++)
	   data[i][j]-=matrix1.data[i][j];
     return *this;
	 }

//---------------------------------------------------------------------------
//33
double StandardCollum(const matrix & m1)
   {
    double max=0,mid;
    int i,j;
    for (j=0;j<m1.sizecol;j++)
	  {
	  mid=0;
	  for (i=0;i<m1.sizerow;i++)  mid+=fabs(m1.data[i][j]);
	  if (mid>max) max=mid;
	  }
    return max;
   }
//34
double StandardRow(const matrix & m1)
   {
	double max=0,mid;
	int i,j;
    for (i=0;i<m1.sizerow;i++)
	  {
	  mid=0;
	  for (j=0;j<m1.sizecol;j++)  mid+=fabs(m1.data[i][j]);
	  if (mid>max) max=mid;
	  }
    return max;
   }
//35
double StandardEuclide(const matrix & m1)
   {
    double num=0;
    int i,j;
	for (i=0;i<m1.sizerow;i++)
	  for (j=0;j<m1.sizecol;j++)
	  num+=pow(m1.data[i][j],2);
    num=sqrt(num);
	return num;
   }
//36
void SetMember(matrix & mt,const int& i,const int& j, const double & d)
  {
  if ((i<mt.sizerow)&&(j<mt.sizecol)&&(i>=0)&&(j>=0))
	mt.data[i][j]=d;
  }
//37
/*
void matrix::setdata(const int& i,const int& j,const double& d)
  {
  if ((i<0)||(i>=sizerow)||(j<0)||(j>=sizecol))
     {
     printf(" It is not a member of matrix");
	 getch();exit(1);
	 }
  data[i][j]=d;
  }
*/
//38
double StandardMax(const matrix & m)
//Tinh chuan cuc dai
{
	double max = m(0,0);
	for (int i=0;i<GetRow(m);i++)
		for (int j=0;j<GetCol(m);j++)
			if (m(i,j)>max) max=m(i,j);
	return max;
}
//39
void SwapCol(matrix & m,const int & i,const int&j)
//Hoan vi 2 cot
{
	double temp;
	for (int index=0;index<GetRow(m);index++)
	{
		temp=m(index,i);
		m(index,i)=m(index,j);
		m(index,j)=temp;
	}
}
//40
void SwapRow(matrix & m,const int & i,const int&j)
//Hoan vi 2 hang
{
	double temp;
	for (int index=0;index<GetCol(m);index++)
	{
		temp=m(i,index);
		m(i,index)=m(j,index);
		m(j,index)=temp;
	}
}
//41
int GetMaxInCol(const matrix &m,const int& i)
//lay vi tri phan tu lon nhat trong cot thu i
{
	int index=0;
	for (int j=0;j<GetRow(m);j++)
		if (m(j,i)>m(index,i)) index=j;
	return index;
}
//42
int GetMaxInRow(const matrix &m,const int& i)
//lay vi tri phan tu lon nhat trong hang thu i
{
	int index=0;
	for (int j=0;j<GetCol(m);j++)
		if (m(i,j)>m(i,index)) index=j;
	return index;
}

//43
int GetAbsMaxInCol(const matrix &m,const int& i)
//lay vi tri phan tu lon nhat trong cot thu i
{
	int index=0;
	for (int j=0;j<GetRow(m);j++)
		if (fabs(m(j,i))>fabs(m(index,i))) index=j;
	return index;
}

//44
int GetAbsMaxInRow(const matrix &m,const int& i)
//lay vi tri phan tu lon nhat trong hang thu i
{
	int index=0;
	for (int j=0;j<GetCol(m);j++)
		if (fabs(m(i,j))>fabs(m(i,index))) index=j;
	return index;
}

//45
void matrix::SetSize(const int & m,const int & n)
//Dat kich thuoc ma tran
{
   int i,j;
   if (data!=NULL)
   {
   for (i=0;i<sizerow;i++)
   {
	  delete []data[i];
	  data[i]=NULL;
   }
   delete []data;data=NULL;
   }
   sizerow=m;sizecol=n;
   data=new double *[sizerow];
   for (i=0;i<sizerow;i++)
	  data[i]=new double [sizecol];
   for (i=0;i<sizerow;i++)
	 for (j=0;j<sizecol;j++)
	   data[i][j]=0.0;
}

//46
  matrix Transpose(const matrix & matrix1)//chuyen vi
  {
	 return !matrix1;
  }
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//CLASS SMATRIX
//===========================================================================
//1
smatrix::smatrix()
  {
  size=0;
  sizerow=sizecol=0;
  data=NULL;
  }
smatrix::~smatrix()
{
	matrix::~matrix();
	size=0;
	sizerow=sizecol=0;
	data=NULL;
}
//---------------------------------------------------------------------------
//2
smatrix::smatrix(const int & m)
   {
   //size=0;
   size=sizerow=sizecol=m;
   data=new double *[m];
   int i,j;
   for (i=0;i<m;i++)
	  data[i]=new double [m];
   for (i=0;i<m;i++)
	  for (j=0;j<m;j++)
	data[i][j]=0.0;
   }
//---------------------------------------------------------------------------
//3
smatrix::smatrix(const smatrix & smatrix1)
   {
   //size=smatrix1.size;
   size=sizerow=sizecol=smatrix1.sizerow;
   int i,j;
   data=new double *[size];
   for (i=0;i<size;i++)
      data[i]=new double [size];
   for (i=0;i<size;i++)
      for (j=0;j<size;j++)
	data[i][j]=smatrix1.data[i][j];
   }
//---------------------------------------------------------------------------
//4
smatrix::smatrix(const matrix & ma1)
 {
   //size=smatrix1.size;
	if (GetCol(ma1)==GetRow(ma1))
	   size=sizerow=sizecol=GetRow(ma1);
	else
	{
		size = sizerow = sizecol = min(GetRow(ma1), GetRow(ma1));
	}
   int i,j;
   data=new double *[size];
   for (i=0;i<size;i++)
      data[i]=new double [size];
   for (i=0;i<size;i++)
      for (j=0;j<size;j++)
	data[i][j]=ma1(i,j);
}

smatrix NormalizeByCol(const smatrix & m)
{
	int n=GetSize(m);
	smatrix out(m);
	vectorm x(n);
	for (int i=0;i<n;i++)
	{
		for (int j=0;j<n;j++) x[j]=out(j,i);
		x=normalize(x);
		for (int j=0;j<n;j++) out(j,i)=x[j];
	}
	return out;
}

smatrix NormalizeByRow(const smatrix & m)
{
	int n=GetSize(m);
	smatrix out(m);
	vectorm x(n);
	for (int i=0;i<n;i++)
	{
		for (int j=0;j<n;j++) x[j]=out(i,j);
		x=normalize(x);
		for (int j=0;j<n;j++) out(i,j)=x[j];
	}
	return out;
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//5
matrix LU(const smatrix & m1,int & boolRes)
{
//neu thanh cong ham cho boolRes=0 nguoc lai cho boolRes=-1
	boolRes = 0;
smatrix m(m1.size+1);
int i,t;//,size=m.sizerow;

for (i=0;i<m1.size;i++)
  for (t=0;t<m1.size;t++)
	m.data[i][t]=m1.data[i][t];

for (i=0;i<m1.size;i++)
 {
  if (m.data[i][i])
	 for (int j=i+1;j<m1.size;j++)
	   {
	   m.data[j][i]=m.data[j][i]/m.data[i][i];
	   for (int k=i+1;k<m1.size;k++)
	  m.data[j][k]=m.data[j][k]-m.data[j][i]*m.data[i][k];
	   }
  else
  {
   int j=i;
   while ((!m.data[i][j])&&(j<m1.size)) j++;

   if (!m.data[i][j])
	  {
	  for (i=0;i<m1.size+2;i++)
	  delete []m.data[i];
	  delete []m.data;
	  m.data=NULL;
	  m.sizerow=0;m.sizecol=0;
	  //cout<<"\n Matran suy bien";
	  boolRes =-1;
	  return matrix();
	  }
   else
	 {
	m.data[i][m1.size]=j+1;
	//m.data[i][m1.size]=i+1;
	//m.data[i][m1.size+1]=j+1;

	double temp;
	for (t=0;t<m1.size;t++)
	{
	  temp=m.data[t][i];
	  m.data[t][i]=m.data[t][j];
	  m.data[t][j]=temp;
	}
	 i--;
	 }
   }
}
delete []m.data[m1.size];delete []m.data[m1.size+1];
m.sizerow=m1.size;m.sizecol=m1.size+2;
boolRes =0;
return m;
}
//---------------------------------------------------------------------------
//6--phep tinh dinh thuc-----------------
double det(const smatrix & m1,int & boolRes)
   {
   //neu thanh cong ham cho boolRes=0 nguoc lai cho boolRes=-1
   matrix m;
   boolRes = 0;
   m=LU(m1, boolRes);
   if (boolRes) return 0.0;
   double det=1;
   for (int i=0;i<m1.size;i++)
	 det*=m(i,i);
   for (int i=0;i<m1.size;i++)
	  if (m(i,m1.size)!=0)
	  det=-det;
   return det;
   }
//---------------------------------------------------------------------------
//7--phep tinh matran nghich dao---------
smatrix inverse(const smatrix & m1,int & boolRes)
  {
  //neu thanh cong ham cho boolRes=0 nguoc lai cho boolRes=-1
	boolRes = 0;
  matrix a(LU(m1, boolRes));
  if (boolRes)  return smatrix();
  double *y;
  y=new double[m1.size];
  int k,i,j,b;
  for (i=0;i<m1.size;i++) y[i]=0.0;
  smatrix x(m1);
  for (k=0;k<m1.size;k++)
    {
      for (i=0;i<m1.size;i++)
       {
	  if (i==k) b=1; else b=0;
	  y[i]=b;
	  for (j=0;j<i;j++)
	    y[i]-= a(i,j)*y[j];
       }
      for (i=m1.size-1;i>=0;i--)
       {
	  x.data[i][k]=y[i];
	  for (j=m1.size-1;j>i;j--)
		   x.data[i][k]-=a(i,j)*x.data[j][k];
	  x.data[i][k]/=a(i,i);
       }
    }

  delete []y;

  for (i=0;i<m1.size;i++)
	 if (a(i,m1.size)!=0)
	{
	int temp=0;
	for (j=0;j<m1.size;j++)
		  {
		   temp=(int)x.data[j][(int)a(i,m1.size)-1];
		   x.data[j][(int)a(i,m1.size)-1]=x.data[j][(int)a(i,m1.size+1)-1];
		   x.data[j][(int)a(i,m1.size+1)-1]=temp;
		  }
	}
   return x;
   }
//8--phep lay kich thuoc ma tran
int GetSize(const smatrix & m1)
 {
	return m1.size;
 }
//9
void smatrix::SetSize(const int & m)
//Dat kich thuoc ma tran
{
   matrix::SetSize(m,m);
   size=m;
}

//10
void	SetToUnit(smatrix & m)
{
	for (int i=0;i<GetSize(m);i++)
	{
		m(i,i)=1;
		for (int j=i+1;j<GetSize(m);j++) m(i,j)=m(j,i)=0;
	}
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction of vectorm
//////////////////////////////////////////////////////////////////////

//1-------------------------------------------------------------------------------
vectorm::vectorm()
{
	size = 0; sizerow = 0; sizecol = 0;
	data = NULL;
}
//2-------------------------------------------------------------------------------
vectorm::vectorm(const vectorm& v)
{
	size = v.size; sizerow = v.size; sizecol = 1;
	data = new double* [size];
	for (int i = 0; i < size; i++)  data[i] = new double[1];
	for (int i = 0; i < size; i++)  data[i][0] = v.data[i][0];
}
vectorm::vectorm(const double& x, const double& y)
{
	this->SetSize(2);
	this->data[0][0] = x;
	this->data[1][0] = y;
}
vectorm::vectorm(const double& x, const double& y, const double& z)
{
	this->SetSize(3);
	this->data[0][0] = x;
	this->data[1][0] = y;
	this->data[2][0] = z;
}
//3-------------------------------------------------------------------------------
vectorm::vectorm(const int& m)
{
	size = m; sizerow = m; sizecol = 1;
	data = new double* [size];
	for (int i = 0; i < size; i++)  data[i] = new double[1];
	for (int i = 0; i < size; i++)  data[i][0] = 0.0;
}
void vectorm::SetSize(const int& m)
{
	matrix::SetSize(m, 1);
	size = m;
}
//4-------------------------------------------------------------------------------
vectorm::~vectorm()
{
	matrix::~matrix();
	size = 0;
}
//5-------------------------------------------------------------------------------
double module(const vectorm& a)
{
	double t = 0.0;
	for (int i = 0; i < GetRow(a); i++)
		t += (a(i) * a(i));
	return sqrt(t);
}

double length(const vectorm& a)
{
	double t = 0.0;
	for (int i = 0; i < GetRow(a); i++)
		t += (a(i) * a(i));
	return sqrt(t);
}

//6-------------------------------------------------------------------------------
double StandardVal(const vectorm& a)
{
	double x = 0.0;
	for (int i = 0; i < GetRow(a); i++)
		x += fabs(a(i));
	return x;
}
//7--------------------------------------------------------------------------
double StandardEuclide(const vectorm& a)
{
	double x = 0.0;
	for (int i = 0; i < GetRow(a); i++)
		x += (a(i) * a(i));
	x = sqrt(x);
	return x;
}
//8--------------------------------------------------------------------------
double StandardMax(const vectorm& a)
{
	double max = a(0);
	for (int i = 0; i < GetRow(a) - 1; i++)
		if (fabs(a(i)) < fabs(a(i + 1)))
			max = fabs(a(i + 1));
		else
			max = fabs(a(i));
	return max;
}
//9--------------------------------------------------------------------------
vectorm standard(const vectorm& a)
{
	vectorm tg(a);
	double t = StandardMax(a);
	for (int i = 0; i < GetRow(a); i++)
		tg.data[i][0] /= t;
	return tg;
}
//10-----------------------------------------------------------------------------
vectorm normalize(const vectorm& a)
{
	vectorm tg(a);
	double t = module(tg);
	for (int i = 0; i < a.sizerow; i++)
		tg.data[i][0] /= t;
	return tg;
}
//11------------------------------------------------------------------------------
vectorm::vectorm(const matrix& ma1)
{
	if (GetCol(ma1) == 1)
	{
		size = sizerow = GetRow(ma1); sizecol = 1;
		int i;
		data = new double* [size];
		for (i = 0; i < size; i++) data[i] = new double[1];
		for (i = 0; i < size; i++) data[i][0] = ma1(i, 0);
	}
	else
		if (GetRow(ma1) == 1)
		{
			size = sizecol = GetCol(ma1); sizerow = 1;
			int i;
			data = new double* [1];
			data[0] = new double[size];
			for (i = 0; i < size; i++) data[0][i] = ma1(0, i);
		}
		else
		{
			if ((GetCol(ma1) > 1) && (GetRow(ma1) > 1))
			{
				size = sizecol = GetCol(ma1); sizerow = 1;
				int i;
				data = new double* [1];
				data[0] = new double[size];
				for (i = 0; i < size; i++) data[0][i] = ma1(0, i);
			}
			else
			{
				size = 0;
				sizerow = sizecol = 0;
				data = NULL;
			}
			//printf("It isn't a vectorm !!!__Press any key to continue...");getch();exit(1);
		}
}
//12------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
double& vectorm::operator()(const int& i)//lay phan tu
{
	if ((i < 0) || (i >= size))
	{
		//printf(" It is not a member of vectorm");getch();exit(1);
		return data[0][0];
	}
	return data[i][0];
}
double const& vectorm::operator()(const int& i)const//lay phan tu
{
	if ((i < 0) || (i >= size))
	{
		//printf(" It is not a member of vectorm");getch();exit(1);
		return data[0][0];
	}
	return data[i][0];
}

double& vectorm::operator[](const int& i)//lay phan tu
{
	if ((i < 0) || (i >= size))
	{
		//printf(" It is not a member of vectorm");getch();exit(1);
		return data[0][0];
	}
	return data[i][0];
}

double const& vectorm::operator[](const int& i)const//lay phan tu
{
	if ((i < 0) || (i >= size))
	{
		//printf(" It is not a member of vectorm");getch();exit(1);
		return data[0][0];
	}
	return data[i][0];
}


//13------------------------------------------------------------------------
//--------------------------------------------------------------------------
double dot(const vectorm& matrix1, const vectorm& matrix2)
//double=matrix1*matrix2
{
	if (GetRow(matrix1) != GetRow(matrix2)) return 0;
	double kq = 0.0;
	int i;
	for (i = 0; i < GetRow(matrix1); i++)
		kq += matrix1(i) * matrix2(i);
	return kq;
}
//14------------------------------------------------------------------------
//--------------------------------------------------------------------------
vectorm cross(const vectorm& vectorm1, const vectorm& vectorm2)
//vectorm = vectorm1 x vectorm2
{
	//if ((vectorm1.size != vectorm2.size)||(vectorm1.size<3))
	//  {
	//  printf("Error on size vectorm !");getch();exit(1);
	//  }
	int n = min(vectorm1.size, vectorm2.size);
	int i, j, k, p, q;
	j = 1; k = 2; p = n - 1; q = n - 2;
	vectorm vtcross(n);
	if (n >= 3)
		for (i = 0; i < n; i++)
		{
			vtcross.data[i][0] = vectorm1(j) * vectorm2(k) - vectorm1(p) * vectorm2(q);
			j++; k++; p++; q++;
			if (j == n) j = 0; if (k == n) k = 0; if (p == n) p = 0; if (q == n) q = 0;
		}
	return vtcross;
}
//15------------------------------------------------------------------------
//--------------------------------------------------------------------------
double angle(const vectorm& vt1, const vectorm& vt2)
{
	double goc;
	goc = dot(normalize(vt1), normalize(vt2));
	goc = acos(goc);
	return goc;
}
//--------------------------------------------------------------------------------
//17

void vectorm::SetData(const int& i, const double& d)
{
	if ((i < 0) || (i >= sizerow))
	{
		//printf(" It is not a member of matrix");getch();exit(1);
		return;
	}
	data[i][0] = d;
}
//=====================================================================
//19
int GetSize(const vectorm& vt)
{
	return vt.size;
}
//=====================================================================
//28
const vectorm& vectorm::operator=(const vectorm& v)
{
	if (size != v.size)
	{
		if (data != NULL)
			for (int i = 0; i < size; i++)  delete[]data[i];
		delete[]data;
		data = NULL;
		size = v.size; sizerow = v.size; sizecol = 1;

		data = new double* [size];
		for (int i = 0; i < size; i++)  data[i] = new double[1];
	}
	for (int i = 0; i < size; i++)  data[i][0] = v.data[i][0];
	return *this;
}
