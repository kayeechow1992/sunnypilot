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
void car_err_fun(double *nom_x, double *delta_x, double *out_60803319336079971);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1408274798934208939);
void car_H_mod_fun(double *state, double *out_1701774390690660890);
void car_f_fun(double *state, double dt, double *out_3524986914669110790);
void car_F_fun(double *state, double dt, double *out_5953381863819616117);
void car_h_25(double *state, double *unused, double *out_1287182306383378938);
void car_H_25(double *state, double *unused, double *out_5224755040680152567);
void car_h_24(double *state, double *unused, double *out_4041588140338162674);
void car_H_24(double *state, double *unused, double *out_8782680639482475703);
void car_h_30(double *state, double *unused, double *out_5483652919966971998);
void car_H_30(double *state, double *unused, double *out_7743087999187401194);
void car_h_26(double *state, double *unused, double *out_4166781689476582478);
void car_H_26(double *state, double *unused, double *out_1483251721806096343);
void car_h_27(double *state, double *unused, double *out_6288855574044224583);
void car_H_27(double *state, double *unused, double *out_8480062003338207205);
void car_h_29(double *state, double *unused, double *out_8296205023236825381);
void car_H_29(double *state, double *unused, double *out_8253319343501793378);
void car_h_28(double *state, double *unused, double *out_2693113597962616182);
void car_H_28(double *state, double *unused, double *out_3170920326432262804);
void car_h_31(double *state, double *unused, double *out_9050447563204051379);
void car_H_31(double *state, double *unused, double *out_5255401002557112995);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}