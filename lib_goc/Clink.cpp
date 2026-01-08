#include "Clink.h"
 
/**
 * Constructor - Khởi tạo khớp với tất cả tham số = 0
 * Đây là trạng thái mặc định của khớp khi chưa được cấu hình
 */
Clink::Clink()
{
    // Khởi tạo các tham số DH cố định = 0
    SetConstants(0, 0, 0, 0);
    // Khởi tạo các biến khớp = 0
    SetParameters(0, 0, 0, 0);
}

/**
 * Copy Constructor - Sao chép toàn bộ thông tin từ khớp khác
 * @param link: Khớp nguồn cần sao chép
 */
Clink::Clink(const Clink &link)
{
    // Sao chép các tham số DH cố định
    SetConstants(link.theta, link.alpha, link.a, link.d);
    // Sao chép các biến khớp
    SetParameters(link.qtheta, link.qalpha, link.qa, link.qd);
}

/**
 * Toán tử gán (Assignment operator)
 * Gán giá trị từ khớp khác vào khớp hiện tại
 * @param link: Khớp nguồn
 * @return: Tham chiếu đến khớp hiện tại
 */
Clink& Clink::operator=(const Clink &link)
{
    // Kiểm tra tự gán (self-assignment)
    if (this != &link)
    {
        // Sao chép các tham số DH cố định
        SetConstants(link.theta, link.alpha, link.a, link.d);
        // Sao chép các biến khớp
        SetParameters(link.qtheta, link.qalpha, link.qa, link.qd);
    }
    return *this;
}

/**
 * Đặt các tham số DH cố định của khớp
 * Các tham số này được xác định bởi cấu trúc cơ khí của robot và không thay đổi
 * 
 * @param ThetaIn: Góc quay cố định quanh trục Z (radian)
 * @param AlphaIn: Góc xoắn cố định quanh trục X (radian)  
 * @param AIn: Chiều dài khớp dọc theo trục X (mm)
 * @param DIn: Độ lệch dọc theo trục Z (mm)
 */
void Clink::SetConstants(const double ThetaIn, const double AlphaIn, const double AIn, const double DIn)
{
    theta = ThetaIn; 
    alpha = AlphaIn;
    a = AIn;
    d = DIn;
}

/**
 * Đặt các biến khớp tại thời điểm hiện tại
 * Các biến này thay đổi khi robot chuyển động
 * - Đối với khớp quay (revolute): qtheta thay đổi, các biến khác = 0
 * - Đối với khớp trượt (prismatic): qa hoặc qd thay đổi, các biến khác = 0
 * 
 * @param qThetaIn: Biến góc quay (radian)
 * @param qAlphaIn: Biến góc xoắn (thường = 0)
 * @param qAIn: Biến chiều dài a (mm)
 * @param qDIn: Biến độ lệch d (mm)
 */
void Clink::SetParameters(const double qThetaIn, const double qAlphaIn, const double qAIn, const double qDIn)
{
    qtheta = qThetaIn;
    qalpha = qAlphaIn;
    qa = qAIn;
    qd = qDIn;
}

/**
 * Tính toán ma trận biến đổi Denavit-Hartenberg (DH)
 * Ma trận này mô tả phép biến đổi tọa độ từ khớp i-1 sang khớp i
 * 
 * Công thức ma trận DH chuẩn (Standard DH Convention):
 * 
 *     | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a*cos(θ) |
 * T = | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a*sin(θ) |
 *     |   0         sin(α)         cos(α)          d     |
 *     |   0           0              0             1     |
 * 
 * Ý nghĩa các thành phần:
 * - Góc θ (theta): Quay quanh trục Z_{i-1}
 * - Dịch chuyển d: Dịch chuyển dọc theo trục Z_{i-1}
 * - Dịch chuyển a: Dịch chuyển dọc theo trục X_i
 * - Góc α (alpha): Quay quanh trục X_i
 * 
 * @return: Ma trận biến đổi đồng nhất 4x4 (homogeneous transformation matrix)
 */
smatrix Clink::GetDHmatrix()
{
    // Tính các tham số thực tế = tham số cố định + biến khớp
    double theta = this->theta + qtheta;  // Tổng góc quay
    double alpha = this->alpha + qalpha;  // Tổng góc xoắn
    double a = this->a + qa;              // Tổng chiều dài
    double d = this->d + qd;              // Tổng độ lệch

    // Tạo ma trận DH 4x4
    smatrix DH(4);
    
    // Hàng 1: Thành phần quay và tịnh tiến theo trục X
    DH(0, 0) = cos(theta);                      // R11: cos(θ)
    DH(0, 1) = -sin(theta) * cos(alpha);        // R12: -sin(θ)cos(α)
    DH(0, 2) = sin(theta) * sin(alpha);         // R13: sin(θ)sin(α)
    DH(0, 3) = a * cos(theta);                  // Px: a*cos(θ)
    
    // Hàng 2: Thành phần quay và tịnh tiến theo trục Y
    DH(1, 0) = sin(theta);                      // R21: sin(θ)
    DH(1, 1) = cos(theta) * cos(alpha);         // R22: cos(θ)cos(α)
    DH(1, 2) = -cos(theta) * sin(alpha);        // R23: -cos(θ)sin(α)
    DH(1, 3) = a * sin(theta);                  // Py: a*sin(θ)
    
    // Hàng 3: Thành phần quay và tịnh tiến theo trục Z
    DH(2, 0) = 0;                               // R31: 0
    DH(2, 1) = sin(alpha);                      // R32: sin(α)
    DH(2, 2) = cos(alpha);                      // R33: cos(α)
    DH(2, 3) = d;                               // Pz: d
    
    // Hàng 4: Hàng đồng nhất (homogeneous row)
    DH(3, 0) = 0;                               // 0
    DH(3, 1) = 0;                               // 0
    DH(3, 2) = 0;                               // 0
    DH(3, 3) = 1;                               // 1

    return DH;
}
