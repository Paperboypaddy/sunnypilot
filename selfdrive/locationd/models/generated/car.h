#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4474920018550507185);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2872096011199435276);
void car_H_mod_fun(double *state, double *out_7026224005864505857);
void car_f_fun(double *state, double dt, double *out_5957602480347530017);
void car_F_fun(double *state, double dt, double *out_823148608051821538);
void car_h_25(double *state, double *unused, double *out_4948611342472557250);
void car_H_25(double *state, double *unused, double *out_2122532180679841017);
void car_h_24(double *state, double *unused, double *out_7882170571962217135);
void car_H_24(double *state, double *unused, double *out_1277535571896145618);
void car_h_30(double *state, double *unused, double *out_5223805404757063139);
void car_H_30(double *state, double *unused, double *out_395800777827407610);
void car_h_26(double *state, double *unused, double *out_3743877300182716179);
void car_H_26(double *state, double *unused, double *out_5864035499553897241);
void car_h_27(double *state, double *unused, double *out_7656520890721244418);
void car_H_27(double *state, double *unused, double *out_2619394849011350827);
void car_h_29(double *state, double *unused, double *out_3181851414887163514);
void car_H_29(double *state, double *unused, double *out_906032122141799794);
void car_h_28(double *state, double *unused, double *out_8942129508512501858);
void car_H_28(double *state, double *unused, double *out_4176366894927730780);
void car_h_31(double *state, double *unused, double *out_5001472049973254374);
void car_H_31(double *state, double *unused, double *out_2091886218802880589);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}