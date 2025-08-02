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
void err_fun(double *nom_x, double *delta_x, double *out_7160966149438702742) {
   out_7160966149438702742[0] = delta_x[0] + nom_x[0];
   out_7160966149438702742[1] = delta_x[1] + nom_x[1];
   out_7160966149438702742[2] = delta_x[2] + nom_x[2];
   out_7160966149438702742[3] = delta_x[3] + nom_x[3];
   out_7160966149438702742[4] = delta_x[4] + nom_x[4];
   out_7160966149438702742[5] = delta_x[5] + nom_x[5];
   out_7160966149438702742[6] = delta_x[6] + nom_x[6];
   out_7160966149438702742[7] = delta_x[7] + nom_x[7];
   out_7160966149438702742[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8286581338862958071) {
   out_8286581338862958071[0] = -nom_x[0] + true_x[0];
   out_8286581338862958071[1] = -nom_x[1] + true_x[1];
   out_8286581338862958071[2] = -nom_x[2] + true_x[2];
   out_8286581338862958071[3] = -nom_x[3] + true_x[3];
   out_8286581338862958071[4] = -nom_x[4] + true_x[4];
   out_8286581338862958071[5] = -nom_x[5] + true_x[5];
   out_8286581338862958071[6] = -nom_x[6] + true_x[6];
   out_8286581338862958071[7] = -nom_x[7] + true_x[7];
   out_8286581338862958071[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8091820366049054439) {
   out_8091820366049054439[0] = 1.0;
   out_8091820366049054439[1] = 0.0;
   out_8091820366049054439[2] = 0.0;
   out_8091820366049054439[3] = 0.0;
   out_8091820366049054439[4] = 0.0;
   out_8091820366049054439[5] = 0.0;
   out_8091820366049054439[6] = 0.0;
   out_8091820366049054439[7] = 0.0;
   out_8091820366049054439[8] = 0.0;
   out_8091820366049054439[9] = 0.0;
   out_8091820366049054439[10] = 1.0;
   out_8091820366049054439[11] = 0.0;
   out_8091820366049054439[12] = 0.0;
   out_8091820366049054439[13] = 0.0;
   out_8091820366049054439[14] = 0.0;
   out_8091820366049054439[15] = 0.0;
   out_8091820366049054439[16] = 0.0;
   out_8091820366049054439[17] = 0.0;
   out_8091820366049054439[18] = 0.0;
   out_8091820366049054439[19] = 0.0;
   out_8091820366049054439[20] = 1.0;
   out_8091820366049054439[21] = 0.0;
   out_8091820366049054439[22] = 0.0;
   out_8091820366049054439[23] = 0.0;
   out_8091820366049054439[24] = 0.0;
   out_8091820366049054439[25] = 0.0;
   out_8091820366049054439[26] = 0.0;
   out_8091820366049054439[27] = 0.0;
   out_8091820366049054439[28] = 0.0;
   out_8091820366049054439[29] = 0.0;
   out_8091820366049054439[30] = 1.0;
   out_8091820366049054439[31] = 0.0;
   out_8091820366049054439[32] = 0.0;
   out_8091820366049054439[33] = 0.0;
   out_8091820366049054439[34] = 0.0;
   out_8091820366049054439[35] = 0.0;
   out_8091820366049054439[36] = 0.0;
   out_8091820366049054439[37] = 0.0;
   out_8091820366049054439[38] = 0.0;
   out_8091820366049054439[39] = 0.0;
   out_8091820366049054439[40] = 1.0;
   out_8091820366049054439[41] = 0.0;
   out_8091820366049054439[42] = 0.0;
   out_8091820366049054439[43] = 0.0;
   out_8091820366049054439[44] = 0.0;
   out_8091820366049054439[45] = 0.0;
   out_8091820366049054439[46] = 0.0;
   out_8091820366049054439[47] = 0.0;
   out_8091820366049054439[48] = 0.0;
   out_8091820366049054439[49] = 0.0;
   out_8091820366049054439[50] = 1.0;
   out_8091820366049054439[51] = 0.0;
   out_8091820366049054439[52] = 0.0;
   out_8091820366049054439[53] = 0.0;
   out_8091820366049054439[54] = 0.0;
   out_8091820366049054439[55] = 0.0;
   out_8091820366049054439[56] = 0.0;
   out_8091820366049054439[57] = 0.0;
   out_8091820366049054439[58] = 0.0;
   out_8091820366049054439[59] = 0.0;
   out_8091820366049054439[60] = 1.0;
   out_8091820366049054439[61] = 0.0;
   out_8091820366049054439[62] = 0.0;
   out_8091820366049054439[63] = 0.0;
   out_8091820366049054439[64] = 0.0;
   out_8091820366049054439[65] = 0.0;
   out_8091820366049054439[66] = 0.0;
   out_8091820366049054439[67] = 0.0;
   out_8091820366049054439[68] = 0.0;
   out_8091820366049054439[69] = 0.0;
   out_8091820366049054439[70] = 1.0;
   out_8091820366049054439[71] = 0.0;
   out_8091820366049054439[72] = 0.0;
   out_8091820366049054439[73] = 0.0;
   out_8091820366049054439[74] = 0.0;
   out_8091820366049054439[75] = 0.0;
   out_8091820366049054439[76] = 0.0;
   out_8091820366049054439[77] = 0.0;
   out_8091820366049054439[78] = 0.0;
   out_8091820366049054439[79] = 0.0;
   out_8091820366049054439[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3012454108615454130) {
   out_3012454108615454130[0] = state[0];
   out_3012454108615454130[1] = state[1];
   out_3012454108615454130[2] = state[2];
   out_3012454108615454130[3] = state[3];
   out_3012454108615454130[4] = state[4];
   out_3012454108615454130[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3012454108615454130[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3012454108615454130[7] = state[7];
   out_3012454108615454130[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3489887080496387331) {
   out_3489887080496387331[0] = 1;
   out_3489887080496387331[1] = 0;
   out_3489887080496387331[2] = 0;
   out_3489887080496387331[3] = 0;
   out_3489887080496387331[4] = 0;
   out_3489887080496387331[5] = 0;
   out_3489887080496387331[6] = 0;
   out_3489887080496387331[7] = 0;
   out_3489887080496387331[8] = 0;
   out_3489887080496387331[9] = 0;
   out_3489887080496387331[10] = 1;
   out_3489887080496387331[11] = 0;
   out_3489887080496387331[12] = 0;
   out_3489887080496387331[13] = 0;
   out_3489887080496387331[14] = 0;
   out_3489887080496387331[15] = 0;
   out_3489887080496387331[16] = 0;
   out_3489887080496387331[17] = 0;
   out_3489887080496387331[18] = 0;
   out_3489887080496387331[19] = 0;
   out_3489887080496387331[20] = 1;
   out_3489887080496387331[21] = 0;
   out_3489887080496387331[22] = 0;
   out_3489887080496387331[23] = 0;
   out_3489887080496387331[24] = 0;
   out_3489887080496387331[25] = 0;
   out_3489887080496387331[26] = 0;
   out_3489887080496387331[27] = 0;
   out_3489887080496387331[28] = 0;
   out_3489887080496387331[29] = 0;
   out_3489887080496387331[30] = 1;
   out_3489887080496387331[31] = 0;
   out_3489887080496387331[32] = 0;
   out_3489887080496387331[33] = 0;
   out_3489887080496387331[34] = 0;
   out_3489887080496387331[35] = 0;
   out_3489887080496387331[36] = 0;
   out_3489887080496387331[37] = 0;
   out_3489887080496387331[38] = 0;
   out_3489887080496387331[39] = 0;
   out_3489887080496387331[40] = 1;
   out_3489887080496387331[41] = 0;
   out_3489887080496387331[42] = 0;
   out_3489887080496387331[43] = 0;
   out_3489887080496387331[44] = 0;
   out_3489887080496387331[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3489887080496387331[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3489887080496387331[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3489887080496387331[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3489887080496387331[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3489887080496387331[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3489887080496387331[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3489887080496387331[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3489887080496387331[53] = -9.8000000000000007*dt;
   out_3489887080496387331[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3489887080496387331[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3489887080496387331[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3489887080496387331[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3489887080496387331[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3489887080496387331[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3489887080496387331[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3489887080496387331[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3489887080496387331[62] = 0;
   out_3489887080496387331[63] = 0;
   out_3489887080496387331[64] = 0;
   out_3489887080496387331[65] = 0;
   out_3489887080496387331[66] = 0;
   out_3489887080496387331[67] = 0;
   out_3489887080496387331[68] = 0;
   out_3489887080496387331[69] = 0;
   out_3489887080496387331[70] = 1;
   out_3489887080496387331[71] = 0;
   out_3489887080496387331[72] = 0;
   out_3489887080496387331[73] = 0;
   out_3489887080496387331[74] = 0;
   out_3489887080496387331[75] = 0;
   out_3489887080496387331[76] = 0;
   out_3489887080496387331[77] = 0;
   out_3489887080496387331[78] = 0;
   out_3489887080496387331[79] = 0;
   out_3489887080496387331[80] = 1;
}
void h_25(double *state, double *unused, double *out_3326705303044979689) {
   out_3326705303044979689[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5451231882475832337) {
   out_5451231882475832337[0] = 0;
   out_5451231882475832337[1] = 0;
   out_5451231882475832337[2] = 0;
   out_5451231882475832337[3] = 0;
   out_5451231882475832337[4] = 0;
   out_5451231882475832337[5] = 0;
   out_5451231882475832337[6] = 1;
   out_5451231882475832337[7] = 0;
   out_5451231882475832337[8] = 0;
}
void h_24(double *state, double *unused, double *out_1753629198841740935) {
   out_1753629198841740935[0] = state[4];
   out_1753629198841740935[1] = state[5];
}
void H_24(double *state, double *unused, double *out_9013189039835951381) {
   out_9013189039835951381[0] = 0;
   out_9013189039835951381[1] = 0;
   out_9013189039835951381[2] = 0;
   out_9013189039835951381[3] = 0;
   out_9013189039835951381[4] = 1;
   out_9013189039835951381[5] = 0;
   out_9013189039835951381[6] = 0;
   out_9013189039835951381[7] = 0;
   out_9013189039835951381[8] = 0;
   out_9013189039835951381[9] = 0;
   out_9013189039835951381[10] = 0;
   out_9013189039835951381[11] = 0;
   out_9013189039835951381[12] = 0;
   out_9013189039835951381[13] = 0;
   out_9013189039835951381[14] = 1;
   out_9013189039835951381[15] = 0;
   out_9013189039835951381[16] = 0;
   out_9013189039835951381[17] = 0;
}
void h_30(double *state, double *unused, double *out_3472225127265634660) {
   out_3472225127265634660[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2932898923968583710) {
   out_2932898923968583710[0] = 0;
   out_2932898923968583710[1] = 0;
   out_2932898923968583710[2] = 0;
   out_2932898923968583710[3] = 0;
   out_2932898923968583710[4] = 1;
   out_2932898923968583710[5] = 0;
   out_2932898923968583710[6] = 0;
   out_2932898923968583710[7] = 0;
   out_2932898923968583710[8] = 0;
}
void h_26(double *state, double *unused, double *out_8210371176772448590) {
   out_8210371176772448590[0] = state[7];
}
void H_26(double *state, double *unused, double *out_9192735201349888561) {
   out_9192735201349888561[0] = 0;
   out_9192735201349888561[1] = 0;
   out_9192735201349888561[2] = 0;
   out_9192735201349888561[3] = 0;
   out_9192735201349888561[4] = 0;
   out_9192735201349888561[5] = 0;
   out_9192735201349888561[6] = 0;
   out_9192735201349888561[7] = 1;
   out_9192735201349888561[8] = 0;
}
void h_27(double *state, double *unused, double *out_8328378570705825334) {
   out_8328378570705825334[0] = state[3];
}
void H_27(double *state, double *unused, double *out_709304852784640493) {
   out_709304852784640493[0] = 0;
   out_709304852784640493[1] = 0;
   out_709304852784640493[2] = 0;
   out_709304852784640493[3] = 1;
   out_709304852784640493[4] = 0;
   out_709304852784640493[5] = 0;
   out_709304852784640493[6] = 0;
   out_709304852784640493[7] = 0;
   out_709304852784640493[8] = 0;
}
void h_29(double *state, double *unused, double *out_7170639512648241321) {
   out_7170639512648241321[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2422667579654191526) {
   out_2422667579654191526[0] = 0;
   out_2422667579654191526[1] = 1;
   out_2422667579654191526[2] = 0;
   out_2422667579654191526[3] = 0;
   out_2422667579654191526[4] = 0;
   out_2422667579654191526[5] = 0;
   out_2422667579654191526[6] = 0;
   out_2422667579654191526[7] = 0;
   out_2422667579654191526[8] = 0;
}
void h_28(double *state, double *unused, double *out_4892458287168401335) {
   out_4892458287168401335[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7505066596723722100) {
   out_7505066596723722100[0] = 1;
   out_7505066596723722100[1] = 0;
   out_7505066596723722100[2] = 0;
   out_7505066596723722100[3] = 0;
   out_7505066596723722100[4] = 0;
   out_7505066596723722100[5] = 0;
   out_7505066596723722100[6] = 0;
   out_7505066596723722100[7] = 0;
   out_7505066596723722100[8] = 0;
}
void h_31(double *state, double *unused, double *out_7356773513843899486) {
   out_7356773513843899486[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8627800770126311579) {
   out_8627800770126311579[0] = 0;
   out_8627800770126311579[1] = 0;
   out_8627800770126311579[2] = 0;
   out_8627800770126311579[3] = 0;
   out_8627800770126311579[4] = 0;
   out_8627800770126311579[5] = 0;
   out_8627800770126311579[6] = 0;
   out_8627800770126311579[7] = 0;
   out_8627800770126311579[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7160966149438702742) {
  err_fun(nom_x, delta_x, out_7160966149438702742);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8286581338862958071) {
  inv_err_fun(nom_x, true_x, out_8286581338862958071);
}
void car_H_mod_fun(double *state, double *out_8091820366049054439) {
  H_mod_fun(state, out_8091820366049054439);
}
void car_f_fun(double *state, double dt, double *out_3012454108615454130) {
  f_fun(state,  dt, out_3012454108615454130);
}
void car_F_fun(double *state, double dt, double *out_3489887080496387331) {
  F_fun(state,  dt, out_3489887080496387331);
}
void car_h_25(double *state, double *unused, double *out_3326705303044979689) {
  h_25(state, unused, out_3326705303044979689);
}
void car_H_25(double *state, double *unused, double *out_5451231882475832337) {
  H_25(state, unused, out_5451231882475832337);
}
void car_h_24(double *state, double *unused, double *out_1753629198841740935) {
  h_24(state, unused, out_1753629198841740935);
}
void car_H_24(double *state, double *unused, double *out_9013189039835951381) {
  H_24(state, unused, out_9013189039835951381);
}
void car_h_30(double *state, double *unused, double *out_3472225127265634660) {
  h_30(state, unused, out_3472225127265634660);
}
void car_H_30(double *state, double *unused, double *out_2932898923968583710) {
  H_30(state, unused, out_2932898923968583710);
}
void car_h_26(double *state, double *unused, double *out_8210371176772448590) {
  h_26(state, unused, out_8210371176772448590);
}
void car_H_26(double *state, double *unused, double *out_9192735201349888561) {
  H_26(state, unused, out_9192735201349888561);
}
void car_h_27(double *state, double *unused, double *out_8328378570705825334) {
  h_27(state, unused, out_8328378570705825334);
}
void car_H_27(double *state, double *unused, double *out_709304852784640493) {
  H_27(state, unused, out_709304852784640493);
}
void car_h_29(double *state, double *unused, double *out_7170639512648241321) {
  h_29(state, unused, out_7170639512648241321);
}
void car_H_29(double *state, double *unused, double *out_2422667579654191526) {
  H_29(state, unused, out_2422667579654191526);
}
void car_h_28(double *state, double *unused, double *out_4892458287168401335) {
  h_28(state, unused, out_4892458287168401335);
}
void car_H_28(double *state, double *unused, double *out_7505066596723722100) {
  H_28(state, unused, out_7505066596723722100);
}
void car_h_31(double *state, double *unused, double *out_7356773513843899486) {
  h_31(state, unused, out_7356773513843899486);
}
void car_H_31(double *state, double *unused, double *out_8627800770126311579) {
  H_31(state, unused, out_8627800770126311579);
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
