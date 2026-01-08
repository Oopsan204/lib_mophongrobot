#pragma once
#include "Matrix.h"

/**
 * Lớp Clink - Đại diện cho một khớp (link) của robot
 * Sử dụng phương pháp Denavit-Hartenberg (DH) để mô tả khớp
 * 
 * Mỗi khớp được định nghĩa bởi 4 tham số DH:
 * - theta (θ): góc quay quanh trục Z
 * - alpha (α): góc xoắn quanh trục X 
 * - a: khoảng cách dịch chuyển dọc theo trục X
 * - d: khoảng cách dịch chuyển dọc theo trục Z
 */
class Clink
{
public:
	// ==================== THAM SỐ DH CỐ ĐỊNH ====================
	// Các tham số này là hằng số trong cấu trúc robot
	double theta;  // Góc quay cố định quanh trục Z (radian)
	double alpha;  // Góc xoắn cố định quanh trục X (radian)
	double a;      // Chiều dài khớp dọc theo trục X (mm hoặc đơn vị chiều dài)
	double d;      // Độ lệch dọc theo trục Z (mm hoặc đơn vị chiều dài)

	// ==================== BIẾN KHỚP ====================
	// Các biến này thay đổi khi robot chuyển động
	double qtheta; // Biến góc quay (cho khớp quay - revolute joint)
	double qalpha; // Biến góc xoắn (thường = 0)
	double qa;     // Biến chiều dài a (cho khớp trượt - prismatic joint)
	double qd;     // Biến độ lệch d (cho khớp trượt - prismatic joint)
	
	/**
	 * Constructor - Khởi tạo khớp với giá trị mặc định = 0
	 */
	Clink();
	
	/**
	 * Copy constructor - Sao chép một khớp từ khớp khác
	 * @param link: Khớp nguồn cần sao chép
	 */
	Clink(const Clink &link);
	
	/**
	 * Toán tử gán - Gán giá trị từ khớp khác
	 * @param link: Khớp nguồn
	 * @return: Tham chiếu đến khớp hiện tại
	 */
	Clink& operator=(const Clink &link);
	
	/**
	 * Đặt các tham số DH cố định của khớp
	 * @param ThetaIn: Giá trị theta cố định
	 * @param AlphaIn: Giá trị alpha cố định
	 * @param AIn: Giá trị a cố định (chiều dài khớp)
	 * @param DIn: Giá trị d cố định (độ lệch)
	 */
	void SetConstants(const double ThetaIn, const double AlphaIn, const double AIn, const double DIn);
	
	/**
	 * Đặt các biến khớp (joint variables) tại thời điểm hiện tại
	 * @param qThetaIn: Biến góc quay
	 * @param qAlphaIn: Biến góc xoắn (thường = 0)
	 * @param qAIn: Biến chiều dài a
	 * @param qDIn: Biến độ lệch d
	 */
	void SetParameters(const double qThetaIn, const double qAlphaIn, const double qAIn, const double qDIn);
	
	/**
	 * Tính và trả về ma trận biến đổi DH (4x4) của khớp
	 * Ma trận này biểu diễn phép biến đổi từ hệ tọa độ khớp i-1 sang khớp i
	 * 
	 * Công thức ma trận DH chuẩn:
	 * [cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a*cos(θ)]
	 * [sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a*sin(θ)]
	 * [  0         sin(α)         cos(α)          d     ]
	 * [  0           0              0             1     ]
	 * 
	 * @return: Ma trận DH 4x4 (smatrix)
	 */
	smatrix GetDHmatrix();

};

