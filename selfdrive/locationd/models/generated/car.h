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
void car_err_fun(double *nom_x, double *delta_x, double *out_5314001146802841789);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6074024826246342691);
void car_H_mod_fun(double *state, double *out_4125613732498138525);
void car_f_fun(double *state, double dt, double *out_4632162160905515264);
void car_F_fun(double *state, double dt, double *out_6694718791291255514);
void car_h_25(double *state, double *unused, double *out_494806105813248704);
void car_H_25(double *state, double *unused, double *out_778078092686526315);
void car_h_24(double *state, double *unused, double *out_4632131779815812499);
void car_H_24(double *state, double *unused, double *out_7353649899278044416);
void car_h_30(double *state, double *unused, double *out_4178745339455625313);
void car_H_30(double *state, double *unused, double *out_7694768434178143070);
void car_h_26(double *state, double *unused, double *out_5122255192357287748);
void car_H_26(double *state, double *unused, double *out_2963425226187529909);
void car_h_27(double *state, double *unused, double *out_4506867161847596941);
void car_H_27(double *state, double *unused, double *out_1526024166257138666);
void car_h_29(double *state, double *unused, double *out_3256398094973075500);
void car_H_29(double *state, double *unused, double *out_3239386893126689699);
void car_h_28(double *state, double *unused, double *out_1902461841038899290);
void car_H_28(double *state, double *unused, double *out_1275756621561363448);
void car_h_31(double *state, double *unused, double *out_6650253301653908833);
void car_H_31(double *state, double *unused, double *out_808724054563486743);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}