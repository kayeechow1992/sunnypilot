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
void car_err_fun(double *nom_x, double *delta_x, double *out_7160966149438702742);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8286581338862958071);
void car_H_mod_fun(double *state, double *out_8091820366049054439);
void car_f_fun(double *state, double dt, double *out_3012454108615454130);
void car_F_fun(double *state, double dt, double *out_3489887080496387331);
void car_h_25(double *state, double *unused, double *out_3326705303044979689);
void car_H_25(double *state, double *unused, double *out_5451231882475832337);
void car_h_24(double *state, double *unused, double *out_1753629198841740935);
void car_H_24(double *state, double *unused, double *out_9013189039835951381);
void car_h_30(double *state, double *unused, double *out_3472225127265634660);
void car_H_30(double *state, double *unused, double *out_2932898923968583710);
void car_h_26(double *state, double *unused, double *out_8210371176772448590);
void car_H_26(double *state, double *unused, double *out_9192735201349888561);
void car_h_27(double *state, double *unused, double *out_8328378570705825334);
void car_H_27(double *state, double *unused, double *out_709304852784640493);
void car_h_29(double *state, double *unused, double *out_7170639512648241321);
void car_H_29(double *state, double *unused, double *out_2422667579654191526);
void car_h_28(double *state, double *unused, double *out_4892458287168401335);
void car_H_28(double *state, double *unused, double *out_7505066596723722100);
void car_h_31(double *state, double *unused, double *out_7356773513843899486);
void car_H_31(double *state, double *unused, double *out_8627800770126311579);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}