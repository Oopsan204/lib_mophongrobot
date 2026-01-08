#pragma once
#include "Clink.h"
#include <vector>
#include <fstream>
#include <string>

// Định nghĩa số lượng khớp (links) của robot
#define NUMBER_LINKS 4

// Giới hạn góc quay của các khớp (theo độ, sẽ chuyển sang radian khi sử dụng)
#define Q1_MIN -205.0  // Giới hạn dưới khớp 1 (độ)
#define Q1_MAX  205.0  // Giới hạn trên khớp 1 (độ)
#define Q2_MIN -163.0  // Giới hạn dưới khớp 2 (độ)
#define Q2_MAX  163.0  // Giới hạn trên khớp 2 (độ)
#define Q3_MIN -105.0  // Giới hạn dưới khớp 3 (độ)
#define Q3_MAX  105.0  // Giới hạn trên khớp 3 (độ)

/**
 * Lớp CRobot - Đại diện cho một cánh tay robot
 * Robot bao gồm 4 khớp (links) và một công cụ (tool) ở đầu
 */
class CRobot
{
    public:
    // Mảng chứa 4 khớp (links) của robot
    Clink links[NUMBER_LINKS];
    
    // Chiều dài của công cụ gắn ở đầu robot (end-effector)
    double LTool; // length of tool
    
    // Vector lưu trữ tọa độ x, y, z của đầu robot (end-effector) theo thời gian
    vector<double> pEx,pEy,pEz; // position of end-effector


    vector<double> pExv,pEyv,pEzv; // velocity of end-effector
    vector<double>qv[NUMBER_LINKS]; // velocity of joint variables
    vector<double> pExa,pEya,pEza; // acceleration of end-effector
    vector<double>qa[NUMBER_LINKS]; // acceleration of joint variables
    // Mảng vector lưu trữ các biến khớp (joint variables) của từng link theo thời gian
    vector<double> q[NUMBER_LINKS]; // joint variables

    int idCurrentPoint; // Chỉ số điểm hiện tại trong quỹ đạo

    double dAngle;

    /**
     * Constructor - Hàm khởi tạo robot
     * Tự động gọi initRobot() để khởi tạo các thông số ban đầu
     */
    CRobot();
    
    /**
     * Khởi tạo các thông số ban đầu của robot
     * Thiết lập các tham số DH (Denavit-Hartenberg) cho từng khớp
     */
    void initRobot();
    
    /**
     * Giải bài toán động học thuận (Forward Kinematics)
     * Tính toán vị trí của đầu robot dựa trên các góc khớp đã cho
     * Sử dụng ma trận biến đổi DH để tính toán
     */
    void solvingForwardKinematics();
    
    /**
     * Thiết lập số lượng điểm cần tính toán cho quỹ đạo
     * @param n: số lượng điểm trên quỹ đạo
     */
    void SetNumPoint(const int& n);
    
    /**
     * Xóa tất cả dữ liệu đã lưu trữ
     * Làm sạch các vector chứa vị trí và biến khớp
     */
    void Clear();
    
    /**
     * Thiết lập các giá trị biến khớp cho toàn bộ quỹ đạo
     * Định nghĩa chuyển động của từng khớp theo thời gian
     */
    void SetVariable();
    
/**
        * Thiết lập điểm cuối cho quỹ đạo
        * Tạo quỹ đạo hình tròn trong không gian 3D
        * @param numSegments: Số đoạn để chia quỹ đạo (mặc định 100)
*/
    void SetEndPoint();
/**
* Giải bài toán động học ngược (Inverse Kinematics)
 * Tính toán các biến khớp dựa trên vị trí đầu robot mong muốn
 *@param x: vectorm chứa các biến khớp ban đầu và kết quả
 *@param funcJacobi: con trỏ hàm tính Jacobian
 *@param eps: sai số chấp nhận được
 *@param maxloop: số vòng lặp tối đa
*/
    void SolvingInverKinematicPosition();
    
    /**
     * Kiểm tra và giới hạn góc quay của khớp trong phạm vi cho phép
     * @param jointIndex: Chỉ số khớp (0, 1, 2)
     * @param angle: Góc cần kiểm tra (radian)
     * @return: Góc sau khi giới hạn (radian)
        */

    static CRobot* p_robot;
    void SolvinginverseKinemaics_Velocity();
    void SolvinginverseKinemaics_Acceleration();
    double ClampJointAngle(int jointIndex, double angle);
    static void funcJacobi(smatrix& a, vectorm& c, const vectorm& x);


};



