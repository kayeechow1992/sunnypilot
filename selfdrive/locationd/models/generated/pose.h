#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_5999678731626763007);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5109969285037859944);
void pose_H_mod_fun(double *state, double *out_5507197653198032450);
void pose_f_fun(double *state, double dt, double *out_8465735489954780143);
void pose_F_fun(double *state, double dt, double *out_7084358017667610949);
void pose_h_4(double *state, double *unused, double *out_5086288106009602117);
void pose_H_4(double *state, double *unused, double *out_354679268158951715);
void pose_h_10(double *state, double *unused, double *out_4884892022508388741);
void pose_H_10(double *state, double *unused, double *out_1066608819722372465);
void pose_h_13(double *state, double *unused, double *out_9106628773609145872);
void pose_H_13(double *state, double *unused, double *out_2857594557173381086);
void pose_h_14(double *state, double *unused, double *out_6735107942124736820);
void pose_H_14(double *state, double *unused, double *out_3437467700454324011);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}