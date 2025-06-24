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
void err_fun(double *nom_x, double *delta_x, double *out_60803319336079971) {
   out_60803319336079971[0] = delta_x[0] + nom_x[0];
   out_60803319336079971[1] = delta_x[1] + nom_x[1];
   out_60803319336079971[2] = delta_x[2] + nom_x[2];
   out_60803319336079971[3] = delta_x[3] + nom_x[3];
   out_60803319336079971[4] = delta_x[4] + nom_x[4];
   out_60803319336079971[5] = delta_x[5] + nom_x[5];
   out_60803319336079971[6] = delta_x[6] + nom_x[6];
   out_60803319336079971[7] = delta_x[7] + nom_x[7];
   out_60803319336079971[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1408274798934208939) {
   out_1408274798934208939[0] = -nom_x[0] + true_x[0];
   out_1408274798934208939[1] = -nom_x[1] + true_x[1];
   out_1408274798934208939[2] = -nom_x[2] + true_x[2];
   out_1408274798934208939[3] = -nom_x[3] + true_x[3];
   out_1408274798934208939[4] = -nom_x[4] + true_x[4];
   out_1408274798934208939[5] = -nom_x[5] + true_x[5];
   out_1408274798934208939[6] = -nom_x[6] + true_x[6];
   out_1408274798934208939[7] = -nom_x[7] + true_x[7];
   out_1408274798934208939[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1701774390690660890) {
   out_1701774390690660890[0] = 1.0;
   out_1701774390690660890[1] = 0.0;
   out_1701774390690660890[2] = 0.0;
   out_1701774390690660890[3] = 0.0;
   out_1701774390690660890[4] = 0.0;
   out_1701774390690660890[5] = 0.0;
   out_1701774390690660890[6] = 0.0;
   out_1701774390690660890[7] = 0.0;
   out_1701774390690660890[8] = 0.0;
   out_1701774390690660890[9] = 0.0;
   out_1701774390690660890[10] = 1.0;
   out_1701774390690660890[11] = 0.0;
   out_1701774390690660890[12] = 0.0;
   out_1701774390690660890[13] = 0.0;
   out_1701774390690660890[14] = 0.0;
   out_1701774390690660890[15] = 0.0;
   out_1701774390690660890[16] = 0.0;
   out_1701774390690660890[17] = 0.0;
   out_1701774390690660890[18] = 0.0;
   out_1701774390690660890[19] = 0.0;
   out_1701774390690660890[20] = 1.0;
   out_1701774390690660890[21] = 0.0;
   out_1701774390690660890[22] = 0.0;
   out_1701774390690660890[23] = 0.0;
   out_1701774390690660890[24] = 0.0;
   out_1701774390690660890[25] = 0.0;
   out_1701774390690660890[26] = 0.0;
   out_1701774390690660890[27] = 0.0;
   out_1701774390690660890[28] = 0.0;
   out_1701774390690660890[29] = 0.0;
   out_1701774390690660890[30] = 1.0;
   out_1701774390690660890[31] = 0.0;
   out_1701774390690660890[32] = 0.0;
   out_1701774390690660890[33] = 0.0;
   out_1701774390690660890[34] = 0.0;
   out_1701774390690660890[35] = 0.0;
   out_1701774390690660890[36] = 0.0;
   out_1701774390690660890[37] = 0.0;
   out_1701774390690660890[38] = 0.0;
   out_1701774390690660890[39] = 0.0;
   out_1701774390690660890[40] = 1.0;
   out_1701774390690660890[41] = 0.0;
   out_1701774390690660890[42] = 0.0;
   out_1701774390690660890[43] = 0.0;
   out_1701774390690660890[44] = 0.0;
   out_1701774390690660890[45] = 0.0;
   out_1701774390690660890[46] = 0.0;
   out_1701774390690660890[47] = 0.0;
   out_1701774390690660890[48] = 0.0;
   out_1701774390690660890[49] = 0.0;
   out_1701774390690660890[50] = 1.0;
   out_1701774390690660890[51] = 0.0;
   out_1701774390690660890[52] = 0.0;
   out_1701774390690660890[53] = 0.0;
   out_1701774390690660890[54] = 0.0;
   out_1701774390690660890[55] = 0.0;
   out_1701774390690660890[56] = 0.0;
   out_1701774390690660890[57] = 0.0;
   out_1701774390690660890[58] = 0.0;
   out_1701774390690660890[59] = 0.0;
   out_1701774390690660890[60] = 1.0;
   out_1701774390690660890[61] = 0.0;
   out_1701774390690660890[62] = 0.0;
   out_1701774390690660890[63] = 0.0;
   out_1701774390690660890[64] = 0.0;
   out_1701774390690660890[65] = 0.0;
   out_1701774390690660890[66] = 0.0;
   out_1701774390690660890[67] = 0.0;
   out_1701774390690660890[68] = 0.0;
   out_1701774390690660890[69] = 0.0;
   out_1701774390690660890[70] = 1.0;
   out_1701774390690660890[71] = 0.0;
   out_1701774390690660890[72] = 0.0;
   out_1701774390690660890[73] = 0.0;
   out_1701774390690660890[74] = 0.0;
   out_1701774390690660890[75] = 0.0;
   out_1701774390690660890[76] = 0.0;
   out_1701774390690660890[77] = 0.0;
   out_1701774390690660890[78] = 0.0;
   out_1701774390690660890[79] = 0.0;
   out_1701774390690660890[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3524986914669110790) {
   out_3524986914669110790[0] = state[0];
   out_3524986914669110790[1] = state[1];
   out_3524986914669110790[2] = state[2];
   out_3524986914669110790[3] = state[3];
   out_3524986914669110790[4] = state[4];
   out_3524986914669110790[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3524986914669110790[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3524986914669110790[7] = state[7];
   out_3524986914669110790[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5953381863819616117) {
   out_5953381863819616117[0] = 1;
   out_5953381863819616117[1] = 0;
   out_5953381863819616117[2] = 0;
   out_5953381863819616117[3] = 0;
   out_5953381863819616117[4] = 0;
   out_5953381863819616117[5] = 0;
   out_5953381863819616117[6] = 0;
   out_5953381863819616117[7] = 0;
   out_5953381863819616117[8] = 0;
   out_5953381863819616117[9] = 0;
   out_5953381863819616117[10] = 1;
   out_5953381863819616117[11] = 0;
   out_5953381863819616117[12] = 0;
   out_5953381863819616117[13] = 0;
   out_5953381863819616117[14] = 0;
   out_5953381863819616117[15] = 0;
   out_5953381863819616117[16] = 0;
   out_5953381863819616117[17] = 0;
   out_5953381863819616117[18] = 0;
   out_5953381863819616117[19] = 0;
   out_5953381863819616117[20] = 1;
   out_5953381863819616117[21] = 0;
   out_5953381863819616117[22] = 0;
   out_5953381863819616117[23] = 0;
   out_5953381863819616117[24] = 0;
   out_5953381863819616117[25] = 0;
   out_5953381863819616117[26] = 0;
   out_5953381863819616117[27] = 0;
   out_5953381863819616117[28] = 0;
   out_5953381863819616117[29] = 0;
   out_5953381863819616117[30] = 1;
   out_5953381863819616117[31] = 0;
   out_5953381863819616117[32] = 0;
   out_5953381863819616117[33] = 0;
   out_5953381863819616117[34] = 0;
   out_5953381863819616117[35] = 0;
   out_5953381863819616117[36] = 0;
   out_5953381863819616117[37] = 0;
   out_5953381863819616117[38] = 0;
   out_5953381863819616117[39] = 0;
   out_5953381863819616117[40] = 1;
   out_5953381863819616117[41] = 0;
   out_5953381863819616117[42] = 0;
   out_5953381863819616117[43] = 0;
   out_5953381863819616117[44] = 0;
   out_5953381863819616117[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5953381863819616117[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5953381863819616117[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5953381863819616117[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5953381863819616117[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5953381863819616117[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5953381863819616117[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5953381863819616117[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5953381863819616117[53] = -9.8000000000000007*dt;
   out_5953381863819616117[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5953381863819616117[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5953381863819616117[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5953381863819616117[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5953381863819616117[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5953381863819616117[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5953381863819616117[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5953381863819616117[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5953381863819616117[62] = 0;
   out_5953381863819616117[63] = 0;
   out_5953381863819616117[64] = 0;
   out_5953381863819616117[65] = 0;
   out_5953381863819616117[66] = 0;
   out_5953381863819616117[67] = 0;
   out_5953381863819616117[68] = 0;
   out_5953381863819616117[69] = 0;
   out_5953381863819616117[70] = 1;
   out_5953381863819616117[71] = 0;
   out_5953381863819616117[72] = 0;
   out_5953381863819616117[73] = 0;
   out_5953381863819616117[74] = 0;
   out_5953381863819616117[75] = 0;
   out_5953381863819616117[76] = 0;
   out_5953381863819616117[77] = 0;
   out_5953381863819616117[78] = 0;
   out_5953381863819616117[79] = 0;
   out_5953381863819616117[80] = 1;
}
void h_25(double *state, double *unused, double *out_1287182306383378938) {
   out_1287182306383378938[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5224755040680152567) {
   out_5224755040680152567[0] = 0;
   out_5224755040680152567[1] = 0;
   out_5224755040680152567[2] = 0;
   out_5224755040680152567[3] = 0;
   out_5224755040680152567[4] = 0;
   out_5224755040680152567[5] = 0;
   out_5224755040680152567[6] = 1;
   out_5224755040680152567[7] = 0;
   out_5224755040680152567[8] = 0;
}
void h_24(double *state, double *unused, double *out_4041588140338162674) {
   out_4041588140338162674[0] = state[4];
   out_4041588140338162674[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8782680639482475703) {
   out_8782680639482475703[0] = 0;
   out_8782680639482475703[1] = 0;
   out_8782680639482475703[2] = 0;
   out_8782680639482475703[3] = 0;
   out_8782680639482475703[4] = 1;
   out_8782680639482475703[5] = 0;
   out_8782680639482475703[6] = 0;
   out_8782680639482475703[7] = 0;
   out_8782680639482475703[8] = 0;
   out_8782680639482475703[9] = 0;
   out_8782680639482475703[10] = 0;
   out_8782680639482475703[11] = 0;
   out_8782680639482475703[12] = 0;
   out_8782680639482475703[13] = 0;
   out_8782680639482475703[14] = 1;
   out_8782680639482475703[15] = 0;
   out_8782680639482475703[16] = 0;
   out_8782680639482475703[17] = 0;
}
void h_30(double *state, double *unused, double *out_5483652919966971998) {
   out_5483652919966971998[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7743087999187401194) {
   out_7743087999187401194[0] = 0;
   out_7743087999187401194[1] = 0;
   out_7743087999187401194[2] = 0;
   out_7743087999187401194[3] = 0;
   out_7743087999187401194[4] = 1;
   out_7743087999187401194[5] = 0;
   out_7743087999187401194[6] = 0;
   out_7743087999187401194[7] = 0;
   out_7743087999187401194[8] = 0;
}
void h_26(double *state, double *unused, double *out_4166781689476582478) {
   out_4166781689476582478[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1483251721806096343) {
   out_1483251721806096343[0] = 0;
   out_1483251721806096343[1] = 0;
   out_1483251721806096343[2] = 0;
   out_1483251721806096343[3] = 0;
   out_1483251721806096343[4] = 0;
   out_1483251721806096343[5] = 0;
   out_1483251721806096343[6] = 0;
   out_1483251721806096343[7] = 1;
   out_1483251721806096343[8] = 0;
}
void h_27(double *state, double *unused, double *out_6288855574044224583) {
   out_6288855574044224583[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8480062003338207205) {
   out_8480062003338207205[0] = 0;
   out_8480062003338207205[1] = 0;
   out_8480062003338207205[2] = 0;
   out_8480062003338207205[3] = 1;
   out_8480062003338207205[4] = 0;
   out_8480062003338207205[5] = 0;
   out_8480062003338207205[6] = 0;
   out_8480062003338207205[7] = 0;
   out_8480062003338207205[8] = 0;
}
void h_29(double *state, double *unused, double *out_8296205023236825381) {
   out_8296205023236825381[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8253319343501793378) {
   out_8253319343501793378[0] = 0;
   out_8253319343501793378[1] = 1;
   out_8253319343501793378[2] = 0;
   out_8253319343501793378[3] = 0;
   out_8253319343501793378[4] = 0;
   out_8253319343501793378[5] = 0;
   out_8253319343501793378[6] = 0;
   out_8253319343501793378[7] = 0;
   out_8253319343501793378[8] = 0;
}
void h_28(double *state, double *unused, double *out_2693113597962616182) {
   out_2693113597962616182[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3170920326432262804) {
   out_3170920326432262804[0] = 1;
   out_3170920326432262804[1] = 0;
   out_3170920326432262804[2] = 0;
   out_3170920326432262804[3] = 0;
   out_3170920326432262804[4] = 0;
   out_3170920326432262804[5] = 0;
   out_3170920326432262804[6] = 0;
   out_3170920326432262804[7] = 0;
   out_3170920326432262804[8] = 0;
}
void h_31(double *state, double *unused, double *out_9050447563204051379) {
   out_9050447563204051379[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5255401002557112995) {
   out_5255401002557112995[0] = 0;
   out_5255401002557112995[1] = 0;
   out_5255401002557112995[2] = 0;
   out_5255401002557112995[3] = 0;
   out_5255401002557112995[4] = 0;
   out_5255401002557112995[5] = 0;
   out_5255401002557112995[6] = 0;
   out_5255401002557112995[7] = 0;
   out_5255401002557112995[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_60803319336079971) {
  err_fun(nom_x, delta_x, out_60803319336079971);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1408274798934208939) {
  inv_err_fun(nom_x, true_x, out_1408274798934208939);
}
void car_H_mod_fun(double *state, double *out_1701774390690660890) {
  H_mod_fun(state, out_1701774390690660890);
}
void car_f_fun(double *state, double dt, double *out_3524986914669110790) {
  f_fun(state,  dt, out_3524986914669110790);
}
void car_F_fun(double *state, double dt, double *out_5953381863819616117) {
  F_fun(state,  dt, out_5953381863819616117);
}
void car_h_25(double *state, double *unused, double *out_1287182306383378938) {
  h_25(state, unused, out_1287182306383378938);
}
void car_H_25(double *state, double *unused, double *out_5224755040680152567) {
  H_25(state, unused, out_5224755040680152567);
}
void car_h_24(double *state, double *unused, double *out_4041588140338162674) {
  h_24(state, unused, out_4041588140338162674);
}
void car_H_24(double *state, double *unused, double *out_8782680639482475703) {
  H_24(state, unused, out_8782680639482475703);
}
void car_h_30(double *state, double *unused, double *out_5483652919966971998) {
  h_30(state, unused, out_5483652919966971998);
}
void car_H_30(double *state, double *unused, double *out_7743087999187401194) {
  H_30(state, unused, out_7743087999187401194);
}
void car_h_26(double *state, double *unused, double *out_4166781689476582478) {
  h_26(state, unused, out_4166781689476582478);
}
void car_H_26(double *state, double *unused, double *out_1483251721806096343) {
  H_26(state, unused, out_1483251721806096343);
}
void car_h_27(double *state, double *unused, double *out_6288855574044224583) {
  h_27(state, unused, out_6288855574044224583);
}
void car_H_27(double *state, double *unused, double *out_8480062003338207205) {
  H_27(state, unused, out_8480062003338207205);
}
void car_h_29(double *state, double *unused, double *out_8296205023236825381) {
  h_29(state, unused, out_8296205023236825381);
}
void car_H_29(double *state, double *unused, double *out_8253319343501793378) {
  H_29(state, unused, out_8253319343501793378);
}
void car_h_28(double *state, double *unused, double *out_2693113597962616182) {
  h_28(state, unused, out_2693113597962616182);
}
void car_H_28(double *state, double *unused, double *out_3170920326432262804) {
  H_28(state, unused, out_3170920326432262804);
}
void car_h_31(double *state, double *unused, double *out_9050447563204051379) {
  h_31(state, unused, out_9050447563204051379);
}
void car_H_31(double *state, double *unused, double *out_5255401002557112995) {
  H_31(state, unused, out_5255401002557112995);
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
