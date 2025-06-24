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
void car_err_fun(double *nom_x, double *delta_x, double *out_8940526391582811510);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2709810885817138075);
void car_H_mod_fun(double *state, double *out_508315998301077392);
void car_f_fun(double *state, double dt, double *out_8017446621876077330);
void car_F_fun(double *state, double dt, double *out_1265785675537677501);
void car_h_25(double *state, double *unused, double *out_1637349263115963356);
void car_H_25(double *state, double *unused, double *out_6526598309698716930);
void car_h_24(double *state, double *unused, double *out_330597390551984456);
void car_H_24(double *state, double *unused, double *out_715212678675939965);
void car_h_30(double *state, double *unused, double *out_2000100265138058994);
void car_H_30(double *state, double *unused, double *out_6655937256841957000);
void car_h_26(double *state, double *unused, double *out_6219896846765324034);
void car_H_26(double *state, double *unused, double *out_8178642445136778462);
void car_h_27(double *state, double *unused, double *out_4005380607964993479);
void car_H_27(double *state, double *unused, double *out_8830700568642381911);
void car_h_29(double *state, double *unused, double *out_7001773532798904639);
void car_H_29(double *state, double *unused, double *out_6145705912527564816);
void car_h_28(double *state, double *unused, double *out_1192828504695140096);
void car_H_28(double *state, double *unused, double *out_8580433023946606693);
void car_h_31(double *state, double *unused, double *out_1912543325400469245);
void car_H_31(double *state, double *unused, double *out_6495952347821756502);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}