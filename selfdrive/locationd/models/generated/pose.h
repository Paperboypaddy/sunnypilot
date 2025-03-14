#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_7827726718003508843);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6277241276544534268);
void pose_H_mod_fun(double *state, double *out_3091072411791179619);
void pose_f_fun(double *state, double dt, double *out_1071594590870993149);
void pose_F_fun(double *state, double dt, double *out_1482318386001013498);
void pose_h_4(double *state, double *unused, double *out_1405613756174050193);
void pose_H_4(double *state, double *unused, double *out_2061445973247901116);
void pose_h_10(double *state, double *unused, double *out_7299504663005157793);
void pose_H_10(double *state, double *unused, double *out_212622638766991425);
void pose_h_13(double *state, double *unused, double *out_8700349808613667701);
void pose_H_13(double *state, double *unused, double *out_5273719798580233917);
void pose_h_14(double *state, double *unused, double *out_1187658249473560640);
void pose_H_14(double *state, double *unused, double *out_6024686829587385645);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}