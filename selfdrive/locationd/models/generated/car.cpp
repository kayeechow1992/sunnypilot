#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8940526391582811510) {
   out_8940526391582811510[0] = delta_x[0] + nom_x[0];
   out_8940526391582811510[1] = delta_x[1] + nom_x[1];
   out_8940526391582811510[2] = delta_x[2] + nom_x[2];
   out_8940526391582811510[3] = delta_x[3] + nom_x[3];
   out_8940526391582811510[4] = delta_x[4] + nom_x[4];
   out_8940526391582811510[5] = delta_x[5] + nom_x[5];
   out_8940526391582811510[6] = delta_x[6] + nom_x[6];
   out_8940526391582811510[7] = delta_x[7] + nom_x[7];
   out_8940526391582811510[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2709810885817138075) {
   out_2709810885817138075[0] = -nom_x[0] + true_x[0];
   out_2709810885817138075[1] = -nom_x[1] + true_x[1];
   out_2709810885817138075[2] = -nom_x[2] + true_x[2];
   out_2709810885817138075[3] = -nom_x[3] + true_x[3];
   out_2709810885817138075[4] = -nom_x[4] + true_x[4];
   out_2709810885817138075[5] = -nom_x[5] + true_x[5];
   out_2709810885817138075[6] = -nom_x[6] + true_x[6];
   out_2709810885817138075[7] = -nom_x[7] + true_x[7];
   out_2709810885817138075[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_508315998301077392) {
   out_508315998301077392[0] = 1.0;
   out_508315998301077392[1] = 0.0;
   out_508315998301077392[2] = 0.0;
   out_508315998301077392[3] = 0.0;
   out_508315998301077392[4] = 0.0;
   out_508315998301077392[5] = 0.0;
   out_508315998301077392[6] = 0.0;
   out_508315998301077392[7] = 0.0;
   out_508315998301077392[8] = 0.0;
   out_508315998301077392[9] = 0.0;
   out_508315998301077392[10] = 1.0;
   out_508315998301077392[11] = 0.0;
   out_508315998301077392[12] = 0.0;
   out_508315998301077392[13] = 0.0;
   out_508315998301077392[14] = 0.0;
   out_508315998301077392[15] = 0.0;
   out_508315998301077392[16] = 0.0;
   out_508315998301077392[17] = 0.0;
   out_508315998301077392[18] = 0.0;
   out_508315998301077392[19] = 0.0;
   out_508315998301077392[20] = 1.0;
   out_508315998301077392[21] = 0.0;
   out_508315998301077392[22] = 0.0;
   out_508315998301077392[23] = 0.0;
   out_508315998301077392[24] = 0.0;
   out_508315998301077392[25] = 0.0;
   out_508315998301077392[26] = 0.0;
   out_508315998301077392[27] = 0.0;
   out_508315998301077392[28] = 0.0;
   out_508315998301077392[29] = 0.0;
   out_508315998301077392[30] = 1.0;
   out_508315998301077392[31] = 0.0;
   out_508315998301077392[32] = 0.0;
   out_508315998301077392[33] = 0.0;
   out_508315998301077392[34] = 0.0;
   out_508315998301077392[35] = 0.0;
   out_508315998301077392[36] = 0.0;
   out_508315998301077392[37] = 0.0;
   out_508315998301077392[38] = 0.0;
   out_508315998301077392[39] = 0.0;
   out_508315998301077392[40] = 1.0;
   out_508315998301077392[41] = 0.0;
   out_508315998301077392[42] = 0.0;
   out_508315998301077392[43] = 0.0;
   out_508315998301077392[44] = 0.0;
   out_508315998301077392[45] = 0.0;
   out_508315998301077392[46] = 0.0;
   out_508315998301077392[47] = 0.0;
   out_508315998301077392[48] = 0.0;
   out_508315998301077392[49] = 0.0;
   out_508315998301077392[50] = 1.0;
   out_508315998301077392[51] = 0.0;
   out_508315998301077392[52] = 0.0;
   out_508315998301077392[53] = 0.0;
   out_508315998301077392[54] = 0.0;
   out_508315998301077392[55] = 0.0;
   out_508315998301077392[56] = 0.0;
   out_508315998301077392[57] = 0.0;
   out_508315998301077392[58] = 0.0;
   out_508315998301077392[59] = 0.0;
   out_508315998301077392[60] = 1.0;
   out_508315998301077392[61] = 0.0;
   out_508315998301077392[62] = 0.0;
   out_508315998301077392[63] = 0.0;
   out_508315998301077392[64] = 0.0;
   out_508315998301077392[65] = 0.0;
   out_508315998301077392[66] = 0.0;
   out_508315998301077392[67] = 0.0;
   out_508315998301077392[68] = 0.0;
   out_508315998301077392[69] = 0.0;
   out_508315998301077392[70] = 1.0;
   out_508315998301077392[71] = 0.0;
   out_508315998301077392[72] = 0.0;
   out_508315998301077392[73] = 0.0;
   out_508315998301077392[74] = 0.0;
   out_508315998301077392[75] = 0.0;
   out_508315998301077392[76] = 0.0;
   out_508315998301077392[77] = 0.0;
   out_508315998301077392[78] = 0.0;
   out_508315998301077392[79] = 0.0;
   out_508315998301077392[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8017446621876077330) {
   out_8017446621876077330[0] = state[0];
   out_8017446621876077330[1] = state[1];
   out_8017446621876077330[2] = state[2];
   out_8017446621876077330[3] = state[3];
   out_8017446621876077330[4] = state[4];
   out_8017446621876077330[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8017446621876077330[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8017446621876077330[7] = state[7];
   out_8017446621876077330[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1265785675537677501) {
   out_1265785675537677501[0] = 1;
   out_1265785675537677501[1] = 0;
   out_1265785675537677501[2] = 0;
   out_1265785675537677501[3] = 0;
   out_1265785675537677501[4] = 0;
   out_1265785675537677501[5] = 0;
   out_1265785675537677501[6] = 0;
   out_1265785675537677501[7] = 0;
   out_1265785675537677501[8] = 0;
   out_1265785675537677501[9] = 0;
   out_1265785675537677501[10] = 1;
   out_1265785675537677501[11] = 0;
   out_1265785675537677501[12] = 0;
   out_1265785675537677501[13] = 0;
   out_1265785675537677501[14] = 0;
   out_1265785675537677501[15] = 0;
   out_1265785675537677501[16] = 0;
   out_1265785675537677501[17] = 0;
   out_1265785675537677501[18] = 0;
   out_1265785675537677501[19] = 0;
   out_1265785675537677501[20] = 1;
   out_1265785675537677501[21] = 0;
   out_1265785675537677501[22] = 0;
   out_1265785675537677501[23] = 0;
   out_1265785675537677501[24] = 0;
   out_1265785675537677501[25] = 0;
   out_1265785675537677501[26] = 0;
   out_1265785675537677501[27] = 0;
   out_1265785675537677501[28] = 0;
   out_1265785675537677501[29] = 0;
   out_1265785675537677501[30] = 1;
   out_1265785675537677501[31] = 0;
   out_1265785675537677501[32] = 0;
   out_1265785675537677501[33] = 0;
   out_1265785675537677501[34] = 0;
   out_1265785675537677501[35] = 0;
   out_1265785675537677501[36] = 0;
   out_1265785675537677501[37] = 0;
   out_1265785675537677501[38] = 0;
   out_1265785675537677501[39] = 0;
   out_1265785675537677501[40] = 1;
   out_1265785675537677501[41] = 0;
   out_1265785675537677501[42] = 0;
   out_1265785675537677501[43] = 0;
   out_1265785675537677501[44] = 0;
   out_1265785675537677501[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1265785675537677501[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1265785675537677501[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1265785675537677501[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1265785675537677501[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1265785675537677501[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1265785675537677501[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1265785675537677501[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1265785675537677501[53] = -9.8000000000000007*dt;
   out_1265785675537677501[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1265785675537677501[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1265785675537677501[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1265785675537677501[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1265785675537677501[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1265785675537677501[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1265785675537677501[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1265785675537677501[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1265785675537677501[62] = 0;
   out_1265785675537677501[63] = 0;
   out_1265785675537677501[64] = 0;
   out_1265785675537677501[65] = 0;
   out_1265785675537677501[66] = 0;
   out_1265785675537677501[67] = 0;
   out_1265785675537677501[68] = 0;
   out_1265785675537677501[69] = 0;
   out_1265785675537677501[70] = 1;
   out_1265785675537677501[71] = 0;
   out_1265785675537677501[72] = 0;
   out_1265785675537677501[73] = 0;
   out_1265785675537677501[74] = 0;
   out_1265785675537677501[75] = 0;
   out_1265785675537677501[76] = 0;
   out_1265785675537677501[77] = 0;
   out_1265785675537677501[78] = 0;
   out_1265785675537677501[79] = 0;
   out_1265785675537677501[80] = 1;
}
void h_25(double *state, double *unused, double *out_1637349263115963356) {
   out_1637349263115963356[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6526598309698716930) {
   out_6526598309698716930[0] = 0;
   out_6526598309698716930[1] = 0;
   out_6526598309698716930[2] = 0;
   out_6526598309698716930[3] = 0;
   out_6526598309698716930[4] = 0;
   out_6526598309698716930[5] = 0;
   out_6526598309698716930[6] = 1;
   out_6526598309698716930[7] = 0;
   out_6526598309698716930[8] = 0;
}
void h_24(double *state, double *unused, double *out_330597390551984456) {
   out_330597390551984456[0] = state[4];
   out_330597390551984456[1] = state[5];
}
void H_24(double *state, double *unused, double *out_715212678675939965) {
   out_715212678675939965[0] = 0;
   out_715212678675939965[1] = 0;
   out_715212678675939965[2] = 0;
   out_715212678675939965[3] = 0;
   out_715212678675939965[4] = 1;
   out_715212678675939965[5] = 0;
   out_715212678675939965[6] = 0;
   out_715212678675939965[7] = 0;
   out_715212678675939965[8] = 0;
   out_715212678675939965[9] = 0;
   out_715212678675939965[10] = 0;
   out_715212678675939965[11] = 0;
   out_715212678675939965[12] = 0;
   out_715212678675939965[13] = 0;
   out_715212678675939965[14] = 1;
   out_715212678675939965[15] = 0;
   out_715212678675939965[16] = 0;
   out_715212678675939965[17] = 0;
}
void h_30(double *state, double *unused, double *out_2000100265138058994) {
   out_2000100265138058994[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6655937256841957000) {
   out_6655937256841957000[0] = 0;
   out_6655937256841957000[1] = 0;
   out_6655937256841957000[2] = 0;
   out_6655937256841957000[3] = 0;
   out_6655937256841957000[4] = 1;
   out_6655937256841957000[5] = 0;
   out_6655937256841957000[6] = 0;
   out_6655937256841957000[7] = 0;
   out_6655937256841957000[8] = 0;
}
void h_26(double *state, double *unused, double *out_6219896846765324034) {
   out_6219896846765324034[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8178642445136778462) {
   out_8178642445136778462[0] = 0;
   out_8178642445136778462[1] = 0;
   out_8178642445136778462[2] = 0;
   out_8178642445136778462[3] = 0;
   out_8178642445136778462[4] = 0;
   out_8178642445136778462[5] = 0;
   out_8178642445136778462[6] = 0;
   out_8178642445136778462[7] = 1;
   out_8178642445136778462[8] = 0;
}
void h_27(double *state, double *unused, double *out_4005380607964993479) {
   out_4005380607964993479[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8830700568642381911) {
   out_8830700568642381911[0] = 0;
   out_8830700568642381911[1] = 0;
   out_8830700568642381911[2] = 0;
   out_8830700568642381911[3] = 1;
   out_8830700568642381911[4] = 0;
   out_8830700568642381911[5] = 0;
   out_8830700568642381911[6] = 0;
   out_8830700568642381911[7] = 0;
   out_8830700568642381911[8] = 0;
}
void h_29(double *state, double *unused, double *out_7001773532798904639) {
   out_7001773532798904639[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6145705912527564816) {
   out_6145705912527564816[0] = 0;
   out_6145705912527564816[1] = 1;
   out_6145705912527564816[2] = 0;
   out_6145705912527564816[3] = 0;
   out_6145705912527564816[4] = 0;
   out_6145705912527564816[5] = 0;
   out_6145705912527564816[6] = 0;
   out_6145705912527564816[7] = 0;
   out_6145705912527564816[8] = 0;
}
void h_28(double *state, double *unused, double *out_1192828504695140096) {
   out_1192828504695140096[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8580433023946606693) {
   out_8580433023946606693[0] = 1;
   out_8580433023946606693[1] = 0;
   out_8580433023946606693[2] = 0;
   out_8580433023946606693[3] = 0;
   out_8580433023946606693[4] = 0;
   out_8580433023946606693[5] = 0;
   out_8580433023946606693[6] = 0;
   out_8580433023946606693[7] = 0;
   out_8580433023946606693[8] = 0;
}
void h_31(double *state, double *unused, double *out_1912543325400469245) {
   out_1912543325400469245[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6495952347821756502) {
   out_6495952347821756502[0] = 0;
   out_6495952347821756502[1] = 0;
   out_6495952347821756502[2] = 0;
   out_6495952347821756502[3] = 0;
   out_6495952347821756502[4] = 0;
   out_6495952347821756502[5] = 0;
   out_6495952347821756502[6] = 0;
   out_6495952347821756502[7] = 0;
   out_6495952347821756502[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_8940526391582811510) {
  err_fun(nom_x, delta_x, out_8940526391582811510);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2709810885817138075) {
  inv_err_fun(nom_x, true_x, out_2709810885817138075);
}
void car_H_mod_fun(double *state, double *out_508315998301077392) {
  H_mod_fun(state, out_508315998301077392);
}
void car_f_fun(double *state, double dt, double *out_8017446621876077330) {
  f_fun(state,  dt, out_8017446621876077330);
}
void car_F_fun(double *state, double dt, double *out_1265785675537677501) {
  F_fun(state,  dt, out_1265785675537677501);
}
void car_h_25(double *state, double *unused, double *out_1637349263115963356) {
  h_25(state, unused, out_1637349263115963356);
}
void car_H_25(double *state, double *unused, double *out_6526598309698716930) {
  H_25(state, unused, out_6526598309698716930);
}
void car_h_24(double *state, double *unused, double *out_330597390551984456) {
  h_24(state, unused, out_330597390551984456);
}
void car_H_24(double *state, double *unused, double *out_715212678675939965) {
  H_24(state, unused, out_715212678675939965);
}
void car_h_30(double *state, double *unused, double *out_2000100265138058994) {
  h_30(state, unused, out_2000100265138058994);
}
void car_H_30(double *state, double *unused, double *out_6655937256841957000) {
  H_30(state, unused, out_6655937256841957000);
}
void car_h_26(double *state, double *unused, double *out_6219896846765324034) {
  h_26(state, unused, out_6219896846765324034);
}
void car_H_26(double *state, double *unused, double *out_8178642445136778462) {
  H_26(state, unused, out_8178642445136778462);
}
void car_h_27(double *state, double *unused, double *out_4005380607964993479) {
  h_27(state, unused, out_4005380607964993479);
}
void car_H_27(double *state, double *unused, double *out_8830700568642381911) {
  H_27(state, unused, out_8830700568642381911);
}
void car_h_29(double *state, double *unused, double *out_7001773532798904639) {
  h_29(state, unused, out_7001773532798904639);
}
void car_H_29(double *state, double *unused, double *out_6145705912527564816) {
  H_29(state, unused, out_6145705912527564816);
}
void car_h_28(double *state, double *unused, double *out_1192828504695140096) {
  h_28(state, unused, out_1192828504695140096);
}
void car_H_28(double *state, double *unused, double *out_8580433023946606693) {
  H_28(state, unused, out_8580433023946606693);
}
void car_h_31(double *state, double *unused, double *out_1912543325400469245) {
  h_31(state, unused, out_1912543325400469245);
}
void car_H_31(double *state, double *unused, double *out_6495952347821756502) {
  H_31(state, unused, out_6495952347821756502);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
