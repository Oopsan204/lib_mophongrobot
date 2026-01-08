#pragma once
#include "Matrix.h"

/**
 * Module Transformations - Cung cấp các hàm biến đổi hình học 3D
 * Sử dụng ma trận đồng nhất 4x4 (homogeneous transformation matrices)
 * để biểu diễn các phép biến đổi trong không gian 3D
 */

// ==================== HÀM KHỞI TẠO MA TRẬN/VECTOR ====================

/**
 * Tạo ma trận đơn vị 4x4 (Identity matrix)
 * Ma trận đơn vị không làm thay đổi điểm khi nhân
 * @param m: Ma trận cần đặt thành đơn vị
 */
void SetIdentity(smatrix& m);

/**
 * Tạo vector đồng nhất đơn vị [0, 0, 0, 1]^T
 * @param v: Vector cần đặt thành đơn vị
 */
void SetIdentity(vectorm& v);

/**
 * Tạo vector vị trí 3D dạng đồng nhất [x, y, z, 1]^T
 * @param v: Vector đích
 * @param tx: Tọa độ x
 * @param ty: Tọa độ y
 * @param tz: Tọa độ z
 */
void SetVector(vectorm& v, double tx, double ty, double tz);

// ==================== MA TRẬN TỊNH TIẾN ====================

/**
 * Tạo ma trận tịnh tiến (Translation matrix)
 * Dịch chuyển điểm theo vector (tx, ty, tz)
 * 
 * Ma trận dạng:
 * | 1  0  0  tx |
 * | 0  1  0  ty |
 * | 0  0  1  tz |
 * | 0  0  0   1 |
 * 
 * @param m: Ma trận đích
 * @param tx: Dịch chuyển theo trục X
 * @param ty: Dịch chuyển theo trục Y
 * @param tz: Dịch chuyển theo trục Z
 */
void SetTranslationMatrix(smatrix& m, double tx, double ty, double tz);

// ==================== MA TRẬN TỶ LỆ ====================

/**
 * Tạo ma trận tỷ lệ (Scaling matrix)
 * Phóng to/thu nhỏ theo các hệ số sx, sy, sz
 * 
 * Ma trận dạng:
 * | sx  0   0  0 |
 * | 0  sy   0  0 |
 * | 0   0  sz  0 |
 * | 0   0   0  1 |
 * 
 * @param m: Ma trận đích
 * @param sx: Hệ số tỷ lệ theo trục X
 * @param sy: Hệ số tỷ lệ theo trục Y
 * @param sz: Hệ số tỷ lệ theo trục Z
 */
void SetScalationMatrix(smatrix& m, double sx, double sy, double sz);

// ==================== MA TRẬN QUAY ====================

/**
 * Tạo ma trận quay quanh trục X (Rotation about X-axis)
 * Quay ngược chiều kim đồng hồ khi nhìn từ đầu trục X về gốc
 * 
 * Ma trận dạng:
 * | 1     0         0      0 |
 * | 0  cos(θ)  -sin(θ)    0 |
 * | 0  sin(θ)   cos(θ)    0 |
 * | 0     0         0      1 |
 * 
 * @param m: Ma trận đích
 * @param angle: Góc quay (radian)
 */
void SetRotationXMatrix(smatrix& m, double angle);

/**
 * Tạo ma trận quay quanh trục Y (Rotation about Y-axis)
 * Quay ngược chiều kim đồng hồ khi nhìn từ đầu trục Y về gốc
 * 
 * Ma trận dạng:
 * |  cos(θ)  0  sin(θ)  0 |
 * |     0     1     0     0 |
 * | -sin(θ)  0  cos(θ)  0 |
 * |     0     0     0     1 |
 * 
 * @param m: Ma trận đích
 * @param angle: Góc quay (radian)
 */
void SetRotationYMatrix(smatrix& m, double angle);

/**
 * Tạo ma trận quay quanh trục Z (Rotation about Z-axis)
 * Quay ngược chiều kim đồng hồ khi nhìn từ đầu trục Z về gốc
 * 
 * Ma trận dạng:
 * | cos(θ)  -sin(θ)  0  0 |
 * | sin(θ)   cos(θ)  0  0 |
 * |    0        0      1  0 |
 * |    0        0      0  1 |
 * 
 * @param m: Ma trận đích
 * @param angle: Góc quay (radian)
 */
void SetRotationZMatrix(smatrix& m, double angle);