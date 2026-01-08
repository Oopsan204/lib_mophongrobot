#include "Transformations.h"

#include <iostream>

using namespace std;

/**
 * Khởi tạo ma trận đơn vị (Identity matrix) 4x4
 * Ma trận đơn vị có 1 trên đường chéo chính, 0 ở các vị trí khác
 * Khi nhân với ma trận/vector khác, giữ nguyên giá trị (I * A = A)
 */
void SetIdentity(smatrix& m)
{
	m.SetSize(4);
	for (int i = 0; i < GetSize(m); i++)
	{
		for (int j = 0; j < GetSize(m); j++)
			m(i,j) = (i == j) ? 1 : 0;  // 1 nếu i==j (đường chéo), 0 nếu khác
	}
}
/**
 * Khới tạo vector đơn vị (Identity vector) dạng đồng nhất
 * Tạo vector [0, 0, 0, 1]^T - biểu diễn gốc tọa độ trong hệ đồng nhất
 */
void SetIdentity(vectorm& v)
{
	v.SetSize(4);
	for (int i = 0; i < GetSize(v); i++)
	{
		v[i] = (i == GetSize(v) - 1) ? 1 : 0;  // Phần tử cuối = 1, các phần tử khác = 0
	}
}

/**
 * Tạo vector vị trí 3D trong tọa độ đồng nhất
 * Biến đổi điểm (x, y, z) thành vector đồng nhất [x, y, z, 1]^T
 * Phần tử thứ 4 = 1 cho phép thực hiện phép tịnh tiến bằng nhân ma trận
 */
void SetVector(vectorm& v, double tx, double ty, double tz)
{
	SetIdentity(v);
	v[0] = tx;  // Tọa độ X
	v[1] = ty;  // Tọa độ Y
	v[2] = tz;  // Tọa độ Z
	// v[3] = 1 (đã được đặt bởi SetIdentity)
}

/**
 * Tạo ma trận tịnh tiến (Translation matrix)
 * Dịch chuyển một điểm theo vector (tx, ty, tz)
 * 
 * Công thức: P' = T * P
 * Trong đó: P' = [x+tx, y+ty, z+tz, 1]^T
 * 
 * Ma trận:
 * | 1  0  0  tx |
 * | 0  1  0  ty |
 * | 0  0  1  tz |
 * | 0  0  0   1 |
 */
void SetTranslationMatrix(smatrix& m, double tx, double ty, double tz)
{
	SetIdentity(m);  // Bắt đầu với ma trận đơn vị
	m(0, GetSize(m) - 1) = tx;  // Cột cuối, hàng 0: dịch chuyển X
	m(1, GetSize(m) - 1) = ty;  // Cột cuối, hàng 1: dịch chuyển Y
	m(2, GetSize(m) - 1) = tz;  // Cột cuối, hàng 2: dịch chuyển Z
}

/**
 * Tạo ma trận tỷ lệ (Scaling matrix)
 * Phóng to/thu nhỏ điểm theo các hệ số sx, sy, sz
 * 
 * Công thức: P' = S * P
 * Trong đó: P' = [x*sx, y*sy, z*sz, 1]^T
 * 
 * Ma trận:
 * | sx  0   0  0 |
 * | 0  sy   0  0 |
 * | 0   0  sz  0 |
 * | 0   0   0  1 |
 */
void SetScalationMatrix(smatrix& m, double sx, double sy, double sz)
{
	SetIdentity(m);  // Bắt đầu với ma trận đơn vị
	m(0,0) = sx;  // Hệ số tỷ lệ theo X
	m(1,1) = sy;  // Hệ số tỷ lệ theo Y
	m(2,2) = sz;  // Hệ số tỷ lệ theo Z
}

/**
 * Tạo ma trận quay quanh trục X (Rotation about X-axis)
 * Quay điểm quanh trục X một góc θ (ngược chiều kim đồng hồ)
 * 
 * Trục X không thay đổi, Y và Z quay quanh X
 * 
 * Ma trận:
 * | 1     0         0      0 |
 * | 0  cos(θ)  -sin(θ)    0 |
 * | 0  sin(θ)   cos(θ)    0 |
 * | 0     0         0      1 |
 */
void SetRotationXMatrix(smatrix& m, double angle)
{
	SetIdentity(m);
	m(1,1) = cos(angle);   // Thành phần cos trong mặt phẳng YZ
	m(1,2) = -sin(angle);  // Thành phần -sin
	m(2,1) = sin(angle);   // Thành phần sin
	m(2,2) = cos(angle);   // Thành phần cos
}

/**
 * Tạo ma trận quay quanh trục Y (Rotation about Y-axis)
 * Quay điểm quanh trục Y một góc θ (ngược chiều kim đồng hồ)
 * 
 * Trục Y không thay đổi, X và Z quay quanh Y
 * 
 * Ma trận:
 * |  cos(θ)  0  sin(θ)  0 |
 * |     0     1     0     0 |
 * | -sin(θ)  0  cos(θ)  0 |
 * |     0     0     0     1 |
 */
void SetRotationYMatrix(smatrix& m, double angle)
{
	SetIdentity(m);
	m(0,0) = cos(angle);   // Thành phần cos trong mặt phẳng XZ
	m(0,2) = sin(angle);   // Thành phần sin (dấu dương)
	m(2,0) = -sin(angle);  // Thành phần -sin
	m(2,2) = cos(angle);   // Thành phần cos
}

/**
 * Tạo ma trận quay quanh trục Z (Rotation about Z-axis)
 * Quay điểm quanh trục Z một góc θ (ngược chiều kim đồng hồ)
 * 
 * Trục Z không thay đổi, X và Y quay quanh Z
 * 
 * Ma trận:
 * | cos(θ)  -sin(θ)  0  0 |
 * | sin(θ)   cos(θ)  0  0 |
 * |    0        0      1  0 |
 * |    0        0      0  1 |
 */
void SetRotationZMatrix(smatrix& m, double angle)
{
	SetIdentity(m);
	m(0,0) = cos(angle);   // Thành phần cos trong mặt phẳng XY
	m(0,1) = -sin(angle);  // Thành phần -sin
	m(1,0) = sin(angle);   // Thành phần sin
	m(1,1) = cos(angle);   // Thành phần cos
}