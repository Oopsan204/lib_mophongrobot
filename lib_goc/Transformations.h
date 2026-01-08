#pragma once
#include "../matrix class/Matrix.h"

void SetIdentity(smatrix& m);
void SetIdentity(vectorm& v);

void SetVector(vectorm& v, double tx, double ty, double tz);

void SetTranslationMatrix(smatrix& m, double tx, double ty, double tz);
void SetScalationMatrix(smatrix& m, double sx, double sy, double sz);

void SetRotationXMatrix(smatrix& m, double angle);
void SetRotationYMatrix(smatrix& m, double angle);
void SetRotationZMatrix(smatrix& m, double angle);