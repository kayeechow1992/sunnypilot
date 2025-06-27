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
void err_fun(double *nom_x, double *delta_x, double *out_5314001146802841789) {
   out_5314001146802841789[0] = delta_x[0] + nom_x[0];
   out_5314001146802841789[1] = delta_x[1] + nom_x[1];
   out_5314001146802841789[2] = delta_x[2] + nom_x[2];
   out_5314001146802841789[3] = delta_x[3] + nom_x[3];
   out_5314001146802841789[4] = delta_x[4] + nom_x[4];
   out_5314001146802841789[5] = delta_x[5] + nom_x[5];
   out_5314001146802841789[6] = delta_x[6] + nom_x[6];
   out_5314001146802841789[7] = delta_x[7] + nom_x[7];
   out_5314001146802841789[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6074024826246342691) {
   out_6074024826246342691[0] = -nom_x[0] + true_x[0];
   out_6074024826246342691[1] = -nom_x[1] + true_x[1];
   out_6074024826246342691[2] = -nom_x[2] + true_x[2];
   out_6074024826246342691[3] = -nom_x[3] + true_x[3];
   out_6074024826246342691[4] = -nom_x[4] + true_x[4];
   out_6074024826246342691[5] = -nom_x[5] + true_x[5];
   out_6074024826246342691[6] = -nom_x[6] + true_x[6];
   out_6074024826246342691[7] = -nom_x[7] + true_x[7];
   out_6074024826246342691[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4125613732498138525) {
   out_4125613732498138525[0] = 1.0;
   out_4125613732498138525[1] = 0.0;
   out_4125613732498138525[2] = 0.0;
   out_4125613732498138525[3] = 0.0;
   out_4125613732498138525[4] = 0.0;
   out_4125613732498138525[5] = 0.0;
   out_4125613732498138525[6] = 0.0;
   out_4125613732498138525[7] = 0.0;
   out_4125613732498138525[8] = 0.0;
   out_4125613732498138525[9] = 0.0;
   out_4125613732498138525[10] = 1.0;
   out_4125613732498138525[11] = 0.0;
   out_4125613732498138525[12] = 0.0;
   out_4125613732498138525[13] = 0.0;
   out_4125613732498138525[14] = 0.0;
   out_4125613732498138525[15] = 0.0;
   out_4125613732498138525[16] = 0.0;
   out_4125613732498138525[17] = 0.0;
   out_4125613732498138525[18] = 0.0;
   out_4125613732498138525[19] = 0.0;
   out_4125613732498138525[20] = 1.0;
   out_4125613732498138525[21] = 0.0;
   out_4125613732498138525[22] = 0.0;
   out_4125613732498138525[23] = 0.0;
   out_4125613732498138525[24] = 0.0;
   out_4125613732498138525[25] = 0.0;
   out_4125613732498138525[26] = 0.0;
   out_4125613732498138525[27] = 0.0;
   out_4125613732498138525[28] = 0.0;
   out_4125613732498138525[29] = 0.0;
   out_4125613732498138525[30] = 1.0;
   out_4125613732498138525[31] = 0.0;
   out_4125613732498138525[32] = 0.0;
   out_4125613732498138525[33] = 0.0;
   out_4125613732498138525[34] = 0.0;
   out_4125613732498138525[35] = 0.0;
   out_4125613732498138525[36] = 0.0;
   out_4125613732498138525[37] = 0.0;
   out_4125613732498138525[38] = 0.0;
   out_4125613732498138525[39] = 0.0;
   out_4125613732498138525[40] = 1.0;
   out_4125613732498138525[41] = 0.0;
   out_4125613732498138525[42] = 0.0;
   out_4125613732498138525[43] = 0.0;
   out_4125613732498138525[44] = 0.0;
   out_4125613732498138525[45] = 0.0;
   out_4125613732498138525[46] = 0.0;
   out_4125613732498138525[47] = 0.0;
   out_4125613732498138525[48] = 0.0;
   out_4125613732498138525[49] = 0.0;
   out_4125613732498138525[50] = 1.0;
   out_4125613732498138525[51] = 0.0;
   out_4125613732498138525[52] = 0.0;
   out_4125613732498138525[53] = 0.0;
   out_4125613732498138525[54] = 0.0;
   out_4125613732498138525[55] = 0.0;
   out_4125613732498138525[56] = 0.0;
   out_4125613732498138525[57] = 0.0;
   out_4125613732498138525[58] = 0.0;
   out_4125613732498138525[59] = 0.0;
   out_4125613732498138525[60] = 1.0;
   out_4125613732498138525[61] = 0.0;
   out_4125613732498138525[62] = 0.0;
   out_4125613732498138525[63] = 0.0;
   out_4125613732498138525[64] = 0.0;
   out_4125613732498138525[65] = 0.0;
   out_4125613732498138525[66] = 0.0;
   out_4125613732498138525[67] = 0.0;
   out_4125613732498138525[68] = 0.0;
   out_4125613732498138525[69] = 0.0;
   out_4125613732498138525[70] = 1.0;
   out_4125613732498138525[71] = 0.0;
   out_4125613732498138525[72] = 0.0;
   out_4125613732498138525[73] = 0.0;
   out_4125613732498138525[74] = 0.0;
   out_4125613732498138525[75] = 0.0;
   out_4125613732498138525[76] = 0.0;
   out_4125613732498138525[77] = 0.0;
   out_4125613732498138525[78] = 0.0;
   out_4125613732498138525[79] = 0.0;
   out_4125613732498138525[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4632162160905515264) {
   out_4632162160905515264[0] = state[0];
   out_4632162160905515264[1] = state[1];
   out_4632162160905515264[2] = state[2];
   out_4632162160905515264[3] = state[3];
   out_4632162160905515264[4] = state[4];
   out_4632162160905515264[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4632162160905515264[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4632162160905515264[7] = state[7];
   out_4632162160905515264[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6694718791291255514) {
   out_6694718791291255514[0] = 1;
   out_6694718791291255514[1] = 0;
   out_6694718791291255514[2] = 0;
   out_6694718791291255514[3] = 0;
   out_6694718791291255514[4] = 0;
   out_6694718791291255514[5] = 0;
   out_6694718791291255514[6] = 0;
   out_6694718791291255514[7] = 0;
   out_6694718791291255514[8] = 0;
   out_6694718791291255514[9] = 0;
   out_6694718791291255514[10] = 1;
   out_6694718791291255514[11] = 0;
   out_6694718791291255514[12] = 0;
   out_6694718791291255514[13] = 0;
   out_6694718791291255514[14] = 0;
   out_6694718791291255514[15] = 0;
   out_6694718791291255514[16] = 0;
   out_6694718791291255514[17] = 0;
   out_6694718791291255514[18] = 0;
   out_6694718791291255514[19] = 0;
   out_6694718791291255514[20] = 1;
   out_6694718791291255514[21] = 0;
   out_6694718791291255514[22] = 0;
   out_6694718791291255514[23] = 0;
   out_6694718791291255514[24] = 0;
   out_6694718791291255514[25] = 0;
   out_6694718791291255514[26] = 0;
   out_6694718791291255514[27] = 0;
   out_6694718791291255514[28] = 0;
   out_6694718791291255514[29] = 0;
   out_6694718791291255514[30] = 1;
   out_6694718791291255514[31] = 0;
   out_6694718791291255514[32] = 0;
   out_6694718791291255514[33] = 0;
   out_6694718791291255514[34] = 0;
   out_6694718791291255514[35] = 0;
   out_6694718791291255514[36] = 0;
   out_6694718791291255514[37] = 0;
   out_6694718791291255514[38] = 0;
   out_6694718791291255514[39] = 0;
   out_6694718791291255514[40] = 1;
   out_6694718791291255514[41] = 0;
   out_6694718791291255514[42] = 0;
   out_6694718791291255514[43] = 0;
   out_6694718791291255514[44] = 0;
   out_6694718791291255514[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6694718791291255514[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6694718791291255514[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6694718791291255514[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6694718791291255514[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6694718791291255514[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6694718791291255514[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6694718791291255514[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6694718791291255514[53] = -9.8000000000000007*dt;
   out_6694718791291255514[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6694718791291255514[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6694718791291255514[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6694718791291255514[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6694718791291255514[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6694718791291255514[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6694718791291255514[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6694718791291255514[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6694718791291255514[62] = 0;
   out_6694718791291255514[63] = 0;
   out_6694718791291255514[64] = 0;
   out_6694718791291255514[65] = 0;
   out_6694718791291255514[66] = 0;
   out_6694718791291255514[67] = 0;
   out_6694718791291255514[68] = 0;
   out_6694718791291255514[69] = 0;
   out_6694718791291255514[70] = 1;
   out_6694718791291255514[71] = 0;
   out_6694718791291255514[72] = 0;
   out_6694718791291255514[73] = 0;
   out_6694718791291255514[74] = 0;
   out_6694718791291255514[75] = 0;
   out_6694718791291255514[76] = 0;
   out_6694718791291255514[77] = 0;
   out_6694718791291255514[78] = 0;
   out_6694718791291255514[79] = 0;
   out_6694718791291255514[80] = 1;
}
void h_25(double *state, double *unused, double *out_494806105813248704) {
   out_494806105813248704[0] = state[6];
}
void H_25(double *state, double *unused, double *out_778078092686526315) {
   out_778078092686526315[0] = 0;
   out_778078092686526315[1] = 0;
   out_778078092686526315[2] = 0;
   out_778078092686526315[3] = 0;
   out_778078092686526315[4] = 0;
   out_778078092686526315[5] = 0;
   out_778078092686526315[6] = 1;
   out_778078092686526315[7] = 0;
   out_778078092686526315[8] = 0;
}
void h_24(double *state, double *unused, double *out_4632131779815812499) {
   out_4632131779815812499[0] = state[4];
   out_4632131779815812499[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7353649899278044416) {
   out_7353649899278044416[0] = 0;
   out_7353649899278044416[1] = 0;
   out_7353649899278044416[2] = 0;
   out_7353649899278044416[3] = 0;
   out_7353649899278044416[4] = 1;
   out_7353649899278044416[5] = 0;
   out_7353649899278044416[6] = 0;
   out_7353649899278044416[7] = 0;
   out_7353649899278044416[8] = 0;
   out_7353649899278044416[9] = 0;
   out_7353649899278044416[10] = 0;
   out_7353649899278044416[11] = 0;
   out_7353649899278044416[12] = 0;
   out_7353649899278044416[13] = 0;
   out_7353649899278044416[14] = 1;
   out_7353649899278044416[15] = 0;
   out_7353649899278044416[16] = 0;
   out_7353649899278044416[17] = 0;
}
void h_30(double *state, double *unused, double *out_4178745339455625313) {
   out_4178745339455625313[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7694768434178143070) {
   out_7694768434178143070[0] = 0;
   out_7694768434178143070[1] = 0;
   out_7694768434178143070[2] = 0;
   out_7694768434178143070[3] = 0;
   out_7694768434178143070[4] = 1;
   out_7694768434178143070[5] = 0;
   out_7694768434178143070[6] = 0;
   out_7694768434178143070[7] = 0;
   out_7694768434178143070[8] = 0;
}
void h_26(double *state, double *unused, double *out_5122255192357287748) {
   out_5122255192357287748[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2963425226187529909) {
   out_2963425226187529909[0] = 0;
   out_2963425226187529909[1] = 0;
   out_2963425226187529909[2] = 0;
   out_2963425226187529909[3] = 0;
   out_2963425226187529909[4] = 0;
   out_2963425226187529909[5] = 0;
   out_2963425226187529909[6] = 0;
   out_2963425226187529909[7] = 1;
   out_2963425226187529909[8] = 0;
}
void h_27(double *state, double *unused, double *out_4506867161847596941) {
   out_4506867161847596941[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1526024166257138666) {
   out_1526024166257138666[0] = 0;
   out_1526024166257138666[1] = 0;
   out_1526024166257138666[2] = 0;
   out_1526024166257138666[3] = 1;
   out_1526024166257138666[4] = 0;
   out_1526024166257138666[5] = 0;
   out_1526024166257138666[6] = 0;
   out_1526024166257138666[7] = 0;
   out_1526024166257138666[8] = 0;
}
void h_29(double *state, double *unused, double *out_3256398094973075500) {
   out_3256398094973075500[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3239386893126689699) {
   out_3239386893126689699[0] = 0;
   out_3239386893126689699[1] = 1;
   out_3239386893126689699[2] = 0;
   out_3239386893126689699[3] = 0;
   out_3239386893126689699[4] = 0;
   out_3239386893126689699[5] = 0;
   out_3239386893126689699[6] = 0;
   out_3239386893126689699[7] = 0;
   out_3239386893126689699[8] = 0;
}
void h_28(double *state, double *unused, double *out_1902461841038899290) {
   out_1902461841038899290[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1275756621561363448) {
   out_1275756621561363448[0] = 1;
   out_1275756621561363448[1] = 0;
   out_1275756621561363448[2] = 0;
   out_1275756621561363448[3] = 0;
   out_1275756621561363448[4] = 0;
   out_1275756621561363448[5] = 0;
   out_1275756621561363448[6] = 0;
   out_1275756621561363448[7] = 0;
   out_1275756621561363448[8] = 0;
}
void h_31(double *state, double *unused, double *out_6650253301653908833) {
   out_6650253301653908833[0] = state[8];
}
void H_31(double *state, double *unused, double *out_808724054563486743) {
   out_808724054563486743[0] = 0;
   out_808724054563486743[1] = 0;
   out_808724054563486743[2] = 0;
   out_808724054563486743[3] = 0;
   out_808724054563486743[4] = 0;
   out_808724054563486743[5] = 0;
   out_808724054563486743[6] = 0;
   out_808724054563486743[7] = 0;
   out_808724054563486743[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5314001146802841789) {
  err_fun(nom_x, delta_x, out_5314001146802841789);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6074024826246342691) {
  inv_err_fun(nom_x, true_x, out_6074024826246342691);
}
void car_H_mod_fun(double *state, double *out_4125613732498138525) {
  H_mod_fun(state, out_4125613732498138525);
}
void car_f_fun(double *state, double dt, double *out_4632162160905515264) {
  f_fun(state,  dt, out_4632162160905515264);
}
void car_F_fun(double *state, double dt, double *out_6694718791291255514) {
  F_fun(state,  dt, out_6694718791291255514);
}
void car_h_25(double *state, double *unused, double *out_494806105813248704) {
  h_25(state, unused, out_494806105813248704);
}
void car_H_25(double *state, double *unused, double *out_778078092686526315) {
  H_25(state, unused, out_778078092686526315);
}
void car_h_24(double *state, double *unused, double *out_4632131779815812499) {
  h_24(state, unused, out_4632131779815812499);
}
void car_H_24(double *state, double *unused, double *out_7353649899278044416) {
  H_24(state, unused, out_7353649899278044416);
}
void car_h_30(double *state, double *unused, double *out_4178745339455625313) {
  h_30(state, unused, out_4178745339455625313);
}
void car_H_30(double *state, double *unused, double *out_7694768434178143070) {
  H_30(state, unused, out_7694768434178143070);
}
void car_h_26(double *state, double *unused, double *out_5122255192357287748) {
  h_26(state, unused, out_5122255192357287748);
}
void car_H_26(double *state, double *unused, double *out_2963425226187529909) {
  H_26(state, unused, out_2963425226187529909);
}
void car_h_27(double *state, double *unused, double *out_4506867161847596941) {
  h_27(state, unused, out_4506867161847596941);
}
void car_H_27(double *state, double *unused, double *out_1526024166257138666) {
  H_27(state, unused, out_1526024166257138666);
}
void car_h_29(double *state, double *unused, double *out_3256398094973075500) {
  h_29(state, unused, out_3256398094973075500);
}
void car_H_29(double *state, double *unused, double *out_3239386893126689699) {
  H_29(state, unused, out_3239386893126689699);
}
void car_h_28(double *state, double *unused, double *out_1902461841038899290) {
  h_28(state, unused, out_1902461841038899290);
}
void car_H_28(double *state, double *unused, double *out_1275756621561363448) {
  H_28(state, unused, out_1275756621561363448);
}
void car_h_31(double *state, double *unused, double *out_6650253301653908833) {
  h_31(state, unused, out_6650253301653908833);
}
void car_H_31(double *state, double *unused, double *out_808724054563486743) {
  H_31(state, unused, out_808724054563486743);
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
