#include "NumMethod.h"

#include <iostream>
using namespace std;

// Giải hệ phương trình đại số tuyến tính A*x = f bằng phương pháp khử Gauss-Jordan.
// a: ma trận hệ số A.
// f: vector vế phải f.
// x: vector nghiệm x (kết quả trả về).
// Trả về 0 nếu thành công, -1 nếu ma trận suy biến.
int Gauss_Jordan(const smatrix& a, const vectorm& f, vectorm& x)
{
	smatrix A(a);
	vectorm F(f);
	vectorm iix(GetSize(A)); // Vector để lưu chỉ số cột của phần tử trụ
	x.SetSize(GetSize(A));

	for (int i = 0; i < GetSize(A); i++)
	{// Bắt đầu quá trình khử cho từng hàng
		double s = 0;
		int ir = 0; // Chỉ số cột của phần tử trụ
		double p = A(i,0);
		// Chọn phần tử trụ (pivot) trên hàng i: là phần tử có giá trị tuyệt đối lớn nhất
		for (int j = 1; j < GetSize(A); j++)
		{
			s = A(i,j);
			if (fabs(s) > fabs(p)) { p = s; ir = j; }
		}

		if (p == 0.0) // Nếu phần tử trụ bằng 0, ma trận suy biến
			return -1;
		else
		{
			iix[i] = ir; // Ghi lại vị trí cột của phần tử trụ
			// Khử các biến ở các hàng khác
			for (int k = 0; k < GetSize(A); k++)
			{
				if (k != i) // Bỏ qua hàng hiện tại (hàng i)
				{
					// Tính hệ số nhân s
					s = A(k, ir) / A(i,ir);
					// Cập nhật lại hàng k
					for (int j = 0; j < GetSize(A); j++)
					{
						if (j == ir) A(k,j) = 0; // Đặt phần tử ở cột trụ về 0
						else
						{
							A(k,j) -= s * A(i,j);
						}
					}
					// Cập nhật vector vế phải F
					F[k] -= s * F[i];
				} 
			}
		}
	}// Kết thúc quá trình khử

	// Tính nghiệm từ ma trận đã khử (dạng chéo)
	for (int i = 0; i < GetSize(A); i++) x[(int)iix[i]] = F[i] / A(i,(int)iix[i]);
	return 0;
}

//////////////////////////////////////////////////////////////////////////

// Tính ma trận nghịch đảo m2 của ma trận m1 bằng phương pháp Gauss-Jordan.
// Giải hệ A*X = I, với X là ma trận nghịch đảo cần tìm, I là ma trận đơn vị.
int InverseByGaussJordan(const smatrix& m1, smatrix& m2)
{
	m2 = m1;
	// Lặp qua từng cột của ma trận đơn vị
	for (int i = 0; i < GetSize(m1); i++)
	{
		vectorm f(GetSize(m1)), x(GetSize(m1));
		// Tạo vector vế phải f là cột thứ i của ma trận đơn vị
		for (int k = 0; k < GetSize(m1); k++) f[k] = 0.0;
		f[i] = 1.0;
		// Giải hệ m1*x = f để tìm cột thứ i của ma trận nghịch đảo
		if (Gauss_Jordan(m1, f, x)) return -1; // Nếu giải không thành công
		// Gán vector nghiệm x vào cột thứ i của ma trận kết quả m2
		for (int j = 0; j < GetSize(m1); j++) m2(j,i) = x[j];
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////

// Giải hệ phương trình phi tuyến bằng phương pháp lặp Newton-Raphson.
// x: vector nghiệm (xấp xỉ ban đầu và kết quả trả về).
// func: con trỏ đến hàm tính ma trận Jacobian F và vector f(x).
// eps: sai số cho phép.
// Max: số lần lặp tối đa.
// Trả về 0 nếu thành công, -1 nếu không hội tụ hoặc có lỗi.
int Newton_Raphson(vectorm& x, functionNR& func, const double& eps, const int& Max)
{
	smatrix F; // Ma trận Jacobian
	vectorm f(x), dx(x); // vector hàm f(x) và vector số gia dx

	for (int i = 0; i < Max; i++)
	{
		// 1. Tính ma trận Jacobian F và vector hàm f tại điểm x hiện tại
		func(F, f, x);
		// 2. Giải hệ phương trình tuyến tính F * dx = f để tìm dx
		int err = Gauss_Jordan(F, f, dx);
		if (err != 0) return err; // Lỗi nếu không giải được hệ
		
		int inconvr = 0; // Biến kiểm tra điều kiện hội tụ
		// 3. Cập nhật nghiệm: x_new = x_old - dx
		for (int j = 0; j < GetSize(x); j++)
		{
			// Nếu có bất kỳ số gia nào lớn hơn sai số, quá trình lặp chưa hội tụ
			if (fabs(dx[j]) >= eps) inconvr = 1;
			x[j] -= dx[j];
		}
		// Nếu tất cả các số gia đều nhỏ hơn sai số, nghiệm đã hội tụ
		if (!inconvr) return 0;
	}
	// Nếu vượt quá số lần lặp tối đa mà chưa hội tụ
	return -1;
}