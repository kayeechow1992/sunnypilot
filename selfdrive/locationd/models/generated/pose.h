#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8557638590374089642);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2952464144299303284);
void pose_H_mod_fun(double *state, double *out_1165030831798606535);
void pose_f_fun(double *state, double dt, double *out_1265326236921530365);
void pose_F_fun(double *state, double dt, double *out_6710846957464201168);
void pose_h_4(double *state, double *unused, double *out_5156063247866132134);
void pose_H_4(double *state, double *unused, double *out_4936838041642514107);
void pose_h_10(double *state, double *unused, double *out_398640295991963377);
void pose_H_10(double *state, double *unused, double *out_3538087001133252611);
void pose_h_13(double *state, double *unused, double *out_4028504955015290804);
void pose_H_13(double *state, double *unused, double *out_1103082578339990083);
void pose_h_14(double *state, double *unused, double *out_4693736396154390042);
void pose_H_14(double *state, double *unused, double *out_1854049609347141811);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}