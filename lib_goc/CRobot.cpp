#include "../lib/CRobot.h"
#include "../lib/NumMethod.h"
#include "Matrix.h"
#include <iostream>
#include <cmath>
#include <cstddef>

CRobot* CRobot::p_robot = NULL;

CRobot::CRobot()
{
    initRobot();
    p_robot = this;
}

/**
 * Khởi tạo các tham số Denavit-Hartenberg cho robot
 * 
 * Cấu hình theo bảng DH:
 * ------------------------------------------------
 * | Khớp | a    | α    | d    | θ (biến)    |
 * ------------------------------------------------
 * | 0→1 | 0    | π/2  | L1   | Q1           |
 * | 1→2 | L2   | 0    | 0    | Q2           |
 * | 2→3 | L3   | 0    | 0    | Q3           |
 * ------------------------------------------------
 * 
 * Giá trị chiều dài:
 * - L1 = 481 (chiều cao khớp đếu tiên)
 * - L2 = 560 (chiều dài khớp thứ 2)
 * - L3 = 659 (chiều dài khớp thứ 3)
 */
void CRobot::initRobot()
{
    // Initialize robot links with DH parameters according to DH table
    // Định nghĩa các chiều dài khớp
    double L1 = 481;  // Chiều dài d của khớp 1
    double L2 = 560;  // Chiều dài a của khớp 2
    double L3 = 659;  // Chiều dài a của khớp 3

    // Link 0 (0→1): a=0, alpha=π/2, d=L1, theta=Q1 (biến)
    links[0].SetConstants(0, PI/2, L1, 0);
    
    // Link 1 (1→2): a=L2, alpha=0, d=0, theta=Q2 (biến)
    links[1].SetConstants(L2, 0, 0, 0);
    
    // Link 2 (2→3): a=L3, alpha=0, d=0, theta=Q3 (biến)
    links[2].SetConstants(L3, 0, 0, 0);
    
    // Link 3: không sử dụng trong bảng DH này
    links[3].SetConstants(0, 0, 0, 0);
    
    LTool = 0;  // Chiều dài công cụ đầu robot

}


double CRobot::ClampJointAngle(int jointIndex, double angle)
{
    double min_angle, max_angle;
    switch (jointIndex) {
        case 0:
            min_angle = Q1_MIN * PI / 180.0;
            max_angle = Q1_MAX * PI / 180.0;
            break;
        case 1:
            min_angle = Q2_MIN * PI / 180.0;
            max_angle = Q2_MAX * PI / 180.0;
            break;
        case 2:
            min_angle = Q3_MIN * PI / 180.0;
            max_angle = Q3_MAX * PI / 180.0;
            break;
        default:
            return angle;
    }
    if (angle < min_angle) {
        return min_angle;
    }
    if (angle > max_angle) {
        return max_angle;
    }
    return angle;
}

/**
 * Giải bài toán Động học Thuận (Forward Kinematics)
 * 
 * MỤC ĐÍCH:
 * Tính toán vị trí end-effector (x, y, z) dựa trên các giá trị khớp đã cho
 * 
 * THUẬT TOÁN:
 * 1. Đặt vị trí end-effector trong hệ tọa độ local: pEn = [0, 0, -LTool, 1]^T
 * 2. Với mỗi điểm thời gian j:
 *    a. Đặt biến khớp cho các link dựa trên q[i][j]
 *    b. Tính ma trận biến đổi tích lũy: A0n = A0 * A1 * A2 * A3 * ... * An
 *    c. Biến đổi vị trí end-effector về hệ tọa độ gốc: pEx0 = A0n * pEn
 *    d. Lưu tọa độ (x, y, z) vào các mảng pEx, pEy, pEz
 * 3. In kết quả ra màn hình
 * 
 * KẾT QUẢ:
 * Các mảng pEx, pEy, pEz chứa tọa độ end-effector tại mỗi điểm thời gian
 */
void CRobot::solvingForwardKinematics()
{
// Khai báo các ma trận cần thiết
smatrix A0n,Ai;           // A0n: Ma trận biến đổi tích lũy từ gốc đến khớp n
                          // Ai: Ma trận biến đổi từ khớp i-1 đến khớp i
vectorm pEx0,pEn(4);      // pEn: Vị trí end-effector trong hệ tọa độ local
                          // pEx0: Vị trí end-effector trong hệ tọa độ gốc

// Định nghĩa vị trí end-effector trong hệ tọa độ local
// End-effector nằm dưới trục Z một khoảng bằng LTool
pEn[0]=0;        // X = 0 (nằm trên trục Z)
pEn[1]=0;        // Y = 0 (nằm trên trục Z)
pEn[2]=-LTool;   // Z = -LTool (chiều dài công cụ hướng xuống)
pEn[3]=1;        // Thành phần đồng nhất

// Vòng lặp qua tất cả các điểm thời gian trong quỹ đạo
for(int j=0;j<q[0].size();j++)
{
// ==== BƯỚC 1: ĐẶT BIẾN KHỚP CHO TỪ NG LINK ====
// Đặt các biến khớp theo bảng DH (theta là biến)
// Set joint variables according to DH table
// Link 0 (0→1): theta = Q1 (variable)
links[0].SetParameters(q[0][j], 0, 0, 0);

// Link 1 (1→2): theta = Q2 (variable)
links[1].SetParameters(q[1][j], 0, 0, 0);

// Link 2 (2→3): theta = Q3 (variable)
links[2].SetParameters(q[2][j], 0, 0, 0);

// Link 3: not used but kept for compatibility
links[3].SetParameters(0, 0, 0, 0);

// ==== BƯỚC 2: TÍNH MA TRẬN BIẾN ĐỔI TÍCH LŨY ====
// Lấy ma trận DH của link đầu tiên (link 0)
A0n = links[0].GetDHmatrix();

// Nhân dồn các ma trận DH của các link tiếp theo
// A0n = A0 * A1 * A2 * ... * An
for (int i=1;i<NUMBER_LINKS;i++)
{
    Ai=links[i].GetDHmatrix();  // Lấy ma trận DH của link i
    A0n=A0n*Ai;                 // Nhân tích lũy: A0n = A0n * Ai

}

// in các ma trận A0, A1, A2, A0n ra màn hình

std::cout << "\nTransformation matrix A0n at point " << j << ":\n";
std::cout << A0n << std::endl;




// ==== BƯỚC 3: BIẾN ĐỔI VỊ TRÍ END-EFFECTOR ====
// Biến đổi vị trí end-effector từ hệ local sang hệ gốc
pEx0=A0n*pEn;  // pEx0 = A0n * pEn = [x, y, z, 1]^T

// Lưu tọa độ (x, y, z) vào mảng
pEx[j]=pEx0[0];  // Tọa độ X
pEy[j]=pEx0[1];  // Tọa độ Y
pEz[j]=pEx0[2];  // Tọa độ Z

}  // Kết thúc vòng lặp qua các điểm


// ==== BƯỚC 4: HIỂN THỊ KẾT QUẢ ====
// In các mảng tọa độ ra màn hình
std::cout<<"\nPosition of end-effector: "<<std::endl;
std::cout<<"px= ";
for(size_t i = 0; i < pEx.size(); i++) {
    std::cout << pEx[i] << " ";
}
std::cout << std::endl;
std::cout<<"py= ";
for(size_t i = 0; i < pEy.size(); i++) {
    std::cout << pEy[i] << " ";
}
std::cout << std::endl;
std::cout<<"pz= ";
for(size_t i = 0; i < pEz.size(); i++) {
    std::cout << pEz[i] << " ";
}
std::cout << std::endl;

}  // Kết thúc hàm solvingForwardKinematics

/**
 * Thiết lập số lượng điểm cho quỹ đạo
 * Cấp phát bộ nhớ cho các mảng lưu tọa độ và biến khớp
 * 
 * @param n: Số lượng điểm trên quỹ đạo
 */
void CRobot::SetNumPoint(const int& n)
{
    // Cấp phát bộ nhớ cho mảng tọa độ end-effector
    pEx.resize(n);  // Mảng tọa độ X
    pEy.resize(n);  // Mảng tọa độ Y
    pEz.resize(n);  // Mảng tọa độ Z
    pExv.resize(n);  // Mảng vận tốc X
    pEyv.resize(n);  // Mảng vận tốc Y
    pEzv.resize(n);  // Mảng vận tốc Z
    pExa.resize(n);  // Mảng gia tốc X
    pEya.resize(n);  // Mảng gia tốc Y
    pEza.resize(n);  // Mảng gia tốc Z
    
    // Cấp phát bộ nhớ cho mảng biến khớp của từng link
    for (int i = 0; i < NUMBER_LINKS; i++) {
        q[i].resize(n);  // Mỗi link có n giá trị khớp theo thời gian
        qv[i].resize(n); // Mỗi link có n giá trị vận tốc khớp theo thời gian
        qa[i].resize(n); // Mỗi link có n giá trị gia tốc khớp theo thời gian
    }
}

/**
 * Xóa toàn bộ dữ liệu đã lưu
 * Giải phóng bộ nhớ của các mảng tọa độ và biến khớp
 */
void CRobot::Clear()
{
    // Xóa mảng tọa độ end-effector
    pEx.clear();
    pEy.clear();
    pEz.clear();
    pExv.clear();
    pEyv.clear();
    pEzv.clear();
    pExa.clear();
    pEya.clear();
    pEza.clear();
    
    // Xóa mảng biến khớp của từng link
    for (int i = 0; i < NUMBER_LINKS; i++) {
        q[i].clear();
        qv[i].clear();
        qa[i].clear();
    }
}

/**
 * Thiết lập các biến khớp cho toàn bộ quỹ đạo
 * 
 * Tạo quỹ đạo với 101 điểm, các khớp thay đổi tuyến tính:
 * - Q1 (khớp 0): Từ -150° đến 150° (trong giới hạn -205° đến 205°)
 * - Q2 (khớp 1): Từ 10° đến 45° (trong giới hạn -163° đến 163°)
 * - Q3 (khớp 2): Từ 0° đến -30° (trong giới hạn -105° đến 105°)
 */
void CRobot::SetVariable()
{
    // Set joint variables for trajectory with 101 points
    int NumberPoints=101;
    SetNumPoint(NumberPoints);
    
    for (int j=0; j<NumberPoints; j++) {
        // Q1: Joint 1 rotates from -170 to +170 (-170° to 170°)
        double q1 = -170 * PI/180 + j * 340 * PI/180 / (NumberPoints-1);
        q[0][j] = ClampJointAngle(0, q1); // Giới hạn trong phạm vi [-205°, 205°]
        
        // Q2: Joint 2 rotates from +135 to -105 (10° to 45°)
        double q2 = 135 * PI/180 + j * (-240) * PI/180 / (NumberPoints-1);
        q[1][j] = ClampJointAngle(1, q2); // Giới hạn trong phạm vi [-163°, 163°]
        
        // Q3: Joint 3 rotates from +62 to -205 (0° to -30°)
        double q3 = 62 * PI/180 + j * (-267) * PI/180 / (NumberPoints-1);
        q[2][j] = ClampJointAngle(2, q3); // Giới hạn trong phạm vi [-105°, 105°]
        
        // q[3]: not used in DH table but kept for compatibility
        q[3][j] = 0;
    }
}



void CRobot::SetEndPoint()
{
    // Phương án tối ưu: Hình tròn trong mặt phẳng YZ phù hợp với giới hạn góc
    int numSegments = 100;
    SetNumPoint(numSegments + 1);  // Cấp phát cho 101 điểm (0 đến 100)
    dAngle = 2 * PI / numSegments;
    // Tham số hình tròn - Điều chỉnh để phù hợp với workspace và giới hạn góc
    double x0 = 800;    // Tâm X (gần hơn với tầm với tối đa)
    double y0 = 0;      // Tâm Y (trục chính phía trước robot)
    double z0 = 300;    // Tâm Z (độ cao thấp hơn, an toàn hơn)
    double r = 100;     // Bán kính nhỏ hơn (100mm thay vì 150mm)

for (int j = 0; j < numSegments + 1; j++)
{
  // van toc diem cuoi
  pEx[j] = x0 + r * sin(j * dAngle);
  pEy[j] = y0 + r * cos(j * dAngle);
  pEz[j] = 100 + j * 100 / numSegments;

  // van toc diem cuoi
  pExv[j] = r * cos(j * dAngle) * dAngle;
  pEyv[j] = -r * sin(j * dAngle) * dAngle;
  pEzv[j] = 100 / numSegments;

  // gia toc diem cuoi
  pExa[j] = -r * sin(j * dAngle) * dAngle * dAngle;
  pEya[j] = -r * cos(j * dAngle) * dAngle * dAngle;
  pEza[j] = 0;
}
}

void CRobot::funcJacobi(smatrix & a,vectorm& c,const vectorm& x)
{
    a.SetSize(3);
    c.SetSize(3);

    // x = [Q1, Q2, Q3]^T
    double q1 = x(0);
    double q2 = x(1);
    double q3 = x(2);
    
    double c1 = cos(q1);
    double s1 = sin(q1);
    double c2 = cos(q2);
    double s2 = sin(q2);
    double c23 = cos(q2 + q3);
    double s23 = sin(q2 + q3);

    // Lấy chiều dài các khớp từ bảng DH
    double L1 = p_robot->links[0].d;  // Chiều cao khớp 1 (d của link 0) = 481
    double L2 = p_robot->links[1].a;  // Chiều dài khớp 2 (a của link 1) = 560
    double L3 = p_robot->links[2].a;  // Chiều dài khớp 3 (a của link 2) = 659
    
    // Ma trận Jacobian J:
    // [∂x/∂q1  ∂x/∂q2  ∂x/∂q3]
    // [∂y/∂q1  ∂y/∂q2  ∂y/∂q3]
    // [∂z/∂q1  ∂z/∂q2  ∂z/∂q3]
    
    // Hàng 1: ∂x/∂qi
    a(0,0) = -s1 * (L2*c2 + L3*c23);     // ∂x/∂q1
    a(0,1) = -c1 * (L2*s2 + L3*s23);     // ∂x/∂q2
    a(0,2) = -c1 * L3*s23;                // ∂x/∂q3

    // Hàng 2: ∂y/∂qi
    a(1,0) = c1 * (L2*c2 + L3*c23);      // ∂y/∂q1
    a(1,1) = -s1 * (L2*s2 + L3*s23);     // ∂y/∂q2
    a(1,2) = -s1 * L3*s23;                // ∂y/∂q3

    // Hàng 3: ∂z/∂qi
    a(2,0) = 0;                           // ∂z/∂q1
    a(2,1) = -L2*c2 - L3*c23;             // ∂z/∂q2
    a(2,2) = -L3*c23;                     // ∂z/∂q3

    // Vector sai số f(x) = [fx, fy, fz]^T
    // fx = x_current - x_desired
    // fy = y_current - y_desired  
    // fz = z_current - z_desired
    
    // Tính vị trí hiện tại của end-effector dựa trên các góc khớp
    double r = L2*c2 + L3*c23;            // Bán kính trong mặt phẳng XY
    double x_current = c1 * r;
    double y_current = s1 * r;
    double z_current = L1 - L2*s2 - L3*s23;  // Chiều cao (481 - ...)
    
    c(0) = x_current - p_robot->pEx[p_robot->idCurrentPoint];
    c(1) = y_current - p_robot->pEy[p_robot->idCurrentPoint];
    c(2) = z_current - p_robot->pEz[p_robot->idCurrentPoint];
}


void CRobot::SolvingInverKinematicPosition()
{
    // Giải bài toán động học nghịch bằng phương pháp Newton-Raphson
    // Tìm các góc khớp [Q1, Q2, Q3] cho mỗi điểm trên quỹ đạo
    
    vectorm x(3);  // Vector chứa 3 biến khớp [Q1, Q2, Q3]
    
    // Giá trị khởi tạo ban đầu (initial guess)
    // Dựa trên quỹ đạo hình tròn YZ tại x=800, y=0±100, z=300±100 (200-400mm)
    // Với L1=481, L2=560, L3=659, tầm với tối đa ≈ 1219mm
    x(0) = 0 * PI/180;       // Q1 = 0° 
    x(1) = -70 * PI/180;     // Q2 = -70° 
    x(2) = -60 * PI/180;     // Q3 = -60° 

    double eps = 1e-6;      // Độ chính xác
    int maxloop = 1000;     // Số vòng lặp tối đa
    
    std::cout << "\n Start solving inverse kinematics..." << std::endl;
    int success_count = 0;
    int limited_count = 0;  // Đếm số lần góc bị giới hạn
    
    for (int j = 0; j < (int)pEx.size(); j++) 
    {
        idCurrentPoint = j;
        
        // Giải hệ phương trình phi tuyến bằng Newton-Raphson
        if (Newton_Raphson(x, funcJacobi, eps, maxloop) == 0) 
        {
            // Lưu nghiệm vào các biến khớp
            q[0][j] = x(0);  // Q1
            q[1][j] = x(1);  // Q2
            q[2][j] = x(2);  // Q3
            q[3][j] = 0;           // Link 4 không sử dụng
            
            success_count++;
        }
        else
        {
            std::cout << "not converged  " << j << std::endl;
            // Giữ nguyên giá trị từ điểm trước hoặc set về 0
            if (j > 0) {
                q[0][j] = q[0][j-1];
                q[1][j] = q[1][j-1];
                q[2][j] = q[2][j-1];
                q[3][j] = 0;
            }
        }
    }
    
}


void CRobot::SolvinginverseKinemaics_Velocity()
{
int numEq = 3; // Số phương trình (3 cho robot 3 khớp)
smatrix A(numEq);   // Ma trận Jacobian
vectorm b(numEq);   // Vector sai số
vectorm x(numEq);  // Vector vận tốc khớp cần tìm
for (size_t j = 0; j < pExv.size();j++){

    double c1 = cos(q[0][j]);
    double s1 = sin(q[0][j]);
    double c2 = cos(q[1][j]);
    double s2 = sin(q[1][j]);
    double c23 = cos(q[1][j] + q[2][j]);
    double s23 = sin(q[1][j] + q[2][j]);
    double a2 = links[1].a;
    double a3 = links[2].a;

    A(0,0) = -s1*(a2*c2 + a3*c23);
    A(0,1) = -c1*(a2*s2 + a3*s23);
    A(0,2) = -c1*a3*s23;
    A(1,0) = c1*(a2*c2 + a3*c23);
    A(1,1) = -s1*(a2*s2 + a3*s23);
    A(1,2) = -s1*a3*s23;
    A(2,0) = 0;
    A(2,1) = -a2*c2 - a3*c23;
    A(2,2) = -a3*c23;

    b(0) = pExv[j];
    b(1) = pEyv[j];
    b(2) = pEzv[j];

    Gauss_Jordan(A, b, x);
    qv[0][j] =x(0);
    qv[1][j] =x(1); 
    qv[2][j] =x(2);
    qv[3][j] =x(3);

}

}

void CRobot::SolvinginverseKinemaics_Acceleration()
{
int numEq = 3; // Số phương trình (3 cho robot 3 khớp)
smatrix A(numEq);   // Ma trận Jacobian
vectorm b(numEq);   // Vector sai số
vectorm x(numEq);  // Vector vận tốc khớp cần tìm
for (size_t j = 0; j < pExa.size();j++){

    double c1 = cos(q[0][j]);
    double s1 = sin(q[0][j]);
    double c2 = cos(q[1][j]);
    double s2 = sin(q[1][j]);
    double c23 = cos(q[1][j] + q[2][j]);
    double s23 = sin(q[1][j] + q[2][j]);
    double a2 = links[1].a;
    double a3 = links[2].a;

    A(0,0) = -s1*(a2*c2 + a3*c23);
    A(0,1) = -c1*(a2*s2 + a3*s23);
    A(0,2) = -c1*a3*s23;
    A(1,0) = c1*(a2*c2 + a3*c23);
    A(1,1) = -s1*(a2*s2 + a3*s23);
    A(1,2) = -s1*a3*s23;
    A(2,0) = 0;
    A(2,1) = -a2*c2 - a3*c23;
    A(2,2) = -a3*c23;

    double qv1 = qv[0][j];
    double qv2 = qv[1][j];
    double qv3 = qv[2][j];

    b(0) = pExa[j] + a2 * c2 * pow(qv[2][j], 2) + a3 * c23 * (pow(qv[2][j], 2) + pow(qv[3][j], 2));
    b(1) = pEya[j] + a2 * s2 * pow(qv[2][j], 2) + a3 * s23 * (pow(qv[2][j], 2) + pow(qv[3][j], 2));
    b(2) = pEza[j];

    Gauss_Jordan(A, b, x);
    qa[0][j] =x(0);
    qa[1][j] =x(1); 
    qa[2][j] =x(2);
    qa[3][j] =x(3); 
}
}