#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6316962376878717030);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2486354416350675894);
void pose_H_mod_fun(double *state, double *out_5713404591469320854);
void pose_f_fun(double *state, double dt, double *out_5650402834277827452);
void pose_F_fun(double *state, double dt, double *out_715380118175320270);
void pose_h_4(double *state, double *unused, double *out_8448258150288935568);
void pose_H_4(double *state, double *unused, double *out_6467565593524033461);
void pose_h_10(double *state, double *unused, double *out_7723716202278938104);
void pose_H_10(double *state, double *unused, double *out_2203730435840599590);
void pose_h_13(double *state, double *unused, double *out_3557376465177495290);
void pose_H_13(double *state, double *unused, double *out_4368547271868817226);
void pose_h_14(double *state, double *unused, double *out_5615766574460706046);
void pose_H_14(double *state, double *unused, double *out_3384777161228661165);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}