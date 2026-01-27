#include "Transformations.h"

#include <iostream>

using namespace std;
// Thiết lập một ma trận vuông 4x4 thành ma trận đơn vị.
void SetIdentity(smatrix& m)
{
	m.SetSize(4);
	for (int i = 0; i < GetSize(m); i++)
	{
		for (int j = 0; j < GetSize(m); j++)
			m(i,j) = (i == j) ? 1 : 0;
	}
}
// Thiết lập một vector 4 chiều cho tọa độ đồng nhất.
// Thành phần cuối cùng (w) được đặt là 1, các thành phần khác là 0. Biểu diễn cho một điểm ở gốc tọa độ.
void SetIdentity(vectorm& v)
{
	v.SetSize(4);
	for (int i = 0; i < GetSize(v); i++)
	{
		v[i] = (i == GetSize(v) - 1) ? 1 : 0;
	}
}
// Thiết lập các thành phần (x, y, z) của một vector vị trí 4 chiều trong tọa độ đồng nhất.
void SetVector(vectorm& v, double tx, double ty, double tz)
{
	SetIdentity(v);
	v[0] = tx;
	v[1] = ty;
	v[2] = tz;
}
// Tạo ma trận biến đổi tịnh tiến 4x4.
void SetTranslationMatrix(smatrix& m, double tx, double ty, double tz)
{
	SetIdentity(m);
	m(0, GetSize(m) - 1) = tx;
	m(1, GetSize(m) - 1) = ty;
	m(2, GetSize(m) - 1) = tz;
}
// Tạo ma trận biến đổi tỉ lệ 4x4.
void SetScalationMatrix(smatrix& m, double sx, double sy, double sz)
{
	SetIdentity(m);
	m(0,0) = sx;
	m(1,1) = sy;
	m(2,2) = sz;
}
// Tạo ma trận quay 4x4 quanh trục X một góc `angle` (radian).
void SetRotationXMatrix(smatrix& m, double angle)
{
	SetIdentity(m);
	m(1,1) = cos(angle);
	m(1,2) = -sin(angle);
	m(2,1) = sin(angle);
	m(2,2) = cos(angle);
}
// Tạo ma trận quay 4x4 quanh trục Y một góc `angle` (radian).
void SetRotationYMatrix(smatrix& m, double angle)
{
	SetIdentity(m);
	m(0,0) = cos(angle);
	m(0,2) = sin(angle);
	m(2,0) = -sin(angle);
	m(2,2) = cos(angle);
}
// Tạo ma trận quay 4x4 quanh trục Z một góc `angle` (radian).
void SetRotationZMatrix(smatrix& m, double angle)
{
	SetIdentity(m);
	m(0,0) = cos(angle);
	m(0,1) = -sin(angle);
	m(1,0) = sin(angle);
	m(1,1) = cos(angle);
}