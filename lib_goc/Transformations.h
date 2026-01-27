#pragma once
#include "Matrix.h"

// Thiết lập một ma trận vuông thành ma trận đơn vị
void SetIdentity(smatrix& m);
// Thiết lập một vector thành vector đơn vị (có thể là vector 0 hoặc 1 tùy thuộc vào ngữ cảnh)
void SetIdentity(vectorm& v);

// Thiết lập các thành phần của một vector
void SetVector(vectorm& v, double tx, double ty, double tz);

// Tạo ma trận biến đổi tịnh tiến
void SetTranslationMatrix(smatrix& m, double tx, double ty, double tz);
// Tạo ma trận biến đổi tỉ lệ
void SetScalationMatrix(smatrix& m, double sx, double sy, double sz);

// Tạo ma trận quay quanh trục X
void SetRotationXMatrix(smatrix& m, double angle);
// Tạo ma trận quay quanh trục Y
void SetRotationYMatrix(smatrix& m, double angle);
// Tạo ma trận quay quanh trục Z
void SetRotationZMatrix(smatrix& m, double angle);