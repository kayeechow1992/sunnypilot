#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5999678731626763007) {
   out_5999678731626763007[0] = delta_x[0] + nom_x[0];
   out_5999678731626763007[1] = delta_x[1] + nom_x[1];
   out_5999678731626763007[2] = delta_x[2] + nom_x[2];
   out_5999678731626763007[3] = delta_x[3] + nom_x[3];
   out_5999678731626763007[4] = delta_x[4] + nom_x[4];
   out_5999678731626763007[5] = delta_x[5] + nom_x[5];
   out_5999678731626763007[6] = delta_x[6] + nom_x[6];
   out_5999678731626763007[7] = delta_x[7] + nom_x[7];
   out_5999678731626763007[8] = delta_x[8] + nom_x[8];
   out_5999678731626763007[9] = delta_x[9] + nom_x[9];
   out_5999678731626763007[10] = delta_x[10] + nom_x[10];
   out_5999678731626763007[11] = delta_x[11] + nom_x[11];
   out_5999678731626763007[12] = delta_x[12] + nom_x[12];
   out_5999678731626763007[13] = delta_x[13] + nom_x[13];
   out_5999678731626763007[14] = delta_x[14] + nom_x[14];
   out_5999678731626763007[15] = delta_x[15] + nom_x[15];
   out_5999678731626763007[16] = delta_x[16] + nom_x[16];
   out_5999678731626763007[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5109969285037859944) {
   out_5109969285037859944[0] = -nom_x[0] + true_x[0];
   out_5109969285037859944[1] = -nom_x[1] + true_x[1];
   out_5109969285037859944[2] = -nom_x[2] + true_x[2];
   out_5109969285037859944[3] = -nom_x[3] + true_x[3];
   out_5109969285037859944[4] = -nom_x[4] + true_x[4];
   out_5109969285037859944[5] = -nom_x[5] + true_x[5];
   out_5109969285037859944[6] = -nom_x[6] + true_x[6];
   out_5109969285037859944[7] = -nom_x[7] + true_x[7];
   out_5109969285037859944[8] = -nom_x[8] + true_x[8];
   out_5109969285037859944[9] = -nom_x[9] + true_x[9];
   out_5109969285037859944[10] = -nom_x[10] + true_x[10];
   out_5109969285037859944[11] = -nom_x[11] + true_x[11];
   out_5109969285037859944[12] = -nom_x[12] + true_x[12];
   out_5109969285037859944[13] = -nom_x[13] + true_x[13];
   out_5109969285037859944[14] = -nom_x[14] + true_x[14];
   out_5109969285037859944[15] = -nom_x[15] + true_x[15];
   out_5109969285037859944[16] = -nom_x[16] + true_x[16];
   out_5109969285037859944[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_5507197653198032450) {
   out_5507197653198032450[0] = 1.0;
   out_5507197653198032450[1] = 0.0;
   out_5507197653198032450[2] = 0.0;
   out_5507197653198032450[3] = 0.0;
   out_5507197653198032450[4] = 0.0;
   out_5507197653198032450[5] = 0.0;
   out_5507197653198032450[6] = 0.0;
   out_5507197653198032450[7] = 0.0;
   out_5507197653198032450[8] = 0.0;
   out_5507197653198032450[9] = 0.0;
   out_5507197653198032450[10] = 0.0;
   out_5507197653198032450[11] = 0.0;
   out_5507197653198032450[12] = 0.0;
   out_5507197653198032450[13] = 0.0;
   out_5507197653198032450[14] = 0.0;
   out_5507197653198032450[15] = 0.0;
   out_5507197653198032450[16] = 0.0;
   out_5507197653198032450[17] = 0.0;
   out_5507197653198032450[18] = 0.0;
   out_5507197653198032450[19] = 1.0;
   out_5507197653198032450[20] = 0.0;
   out_5507197653198032450[21] = 0.0;
   out_5507197653198032450[22] = 0.0;
   out_5507197653198032450[23] = 0.0;
   out_5507197653198032450[24] = 0.0;
   out_5507197653198032450[25] = 0.0;
   out_5507197653198032450[26] = 0.0;
   out_5507197653198032450[27] = 0.0;
   out_5507197653198032450[28] = 0.0;
   out_5507197653198032450[29] = 0.0;
   out_5507197653198032450[30] = 0.0;
   out_5507197653198032450[31] = 0.0;
   out_5507197653198032450[32] = 0.0;
   out_5507197653198032450[33] = 0.0;
   out_5507197653198032450[34] = 0.0;
   out_5507197653198032450[35] = 0.0;
   out_5507197653198032450[36] = 0.0;
   out_5507197653198032450[37] = 0.0;
   out_5507197653198032450[38] = 1.0;
   out_5507197653198032450[39] = 0.0;
   out_5507197653198032450[40] = 0.0;
   out_5507197653198032450[41] = 0.0;
   out_5507197653198032450[42] = 0.0;
   out_5507197653198032450[43] = 0.0;
   out_5507197653198032450[44] = 0.0;
   out_5507197653198032450[45] = 0.0;
   out_5507197653198032450[46] = 0.0;
   out_5507197653198032450[47] = 0.0;
   out_5507197653198032450[48] = 0.0;
   out_5507197653198032450[49] = 0.0;
   out_5507197653198032450[50] = 0.0;
   out_5507197653198032450[51] = 0.0;
   out_5507197653198032450[52] = 0.0;
   out_5507197653198032450[53] = 0.0;
   out_5507197653198032450[54] = 0.0;
   out_5507197653198032450[55] = 0.0;
   out_5507197653198032450[56] = 0.0;
   out_5507197653198032450[57] = 1.0;
   out_5507197653198032450[58] = 0.0;
   out_5507197653198032450[59] = 0.0;
   out_5507197653198032450[60] = 0.0;
   out_5507197653198032450[61] = 0.0;
   out_5507197653198032450[62] = 0.0;
   out_5507197653198032450[63] = 0.0;
   out_5507197653198032450[64] = 0.0;
   out_5507197653198032450[65] = 0.0;
   out_5507197653198032450[66] = 0.0;
   out_5507197653198032450[67] = 0.0;
   out_5507197653198032450[68] = 0.0;
   out_5507197653198032450[69] = 0.0;
   out_5507197653198032450[70] = 0.0;
   out_5507197653198032450[71] = 0.0;
   out_5507197653198032450[72] = 0.0;
   out_5507197653198032450[73] = 0.0;
   out_5507197653198032450[74] = 0.0;
   out_5507197653198032450[75] = 0.0;
   out_5507197653198032450[76] = 1.0;
   out_5507197653198032450[77] = 0.0;
   out_5507197653198032450[78] = 0.0;
   out_5507197653198032450[79] = 0.0;
   out_5507197653198032450[80] = 0.0;
   out_5507197653198032450[81] = 0.0;
   out_5507197653198032450[82] = 0.0;
   out_5507197653198032450[83] = 0.0;
   out_5507197653198032450[84] = 0.0;
   out_5507197653198032450[85] = 0.0;
   out_5507197653198032450[86] = 0.0;
   out_5507197653198032450[87] = 0.0;
   out_5507197653198032450[88] = 0.0;
   out_5507197653198032450[89] = 0.0;
   out_5507197653198032450[90] = 0.0;
   out_5507197653198032450[91] = 0.0;
   out_5507197653198032450[92] = 0.0;
   out_5507197653198032450[93] = 0.0;
   out_5507197653198032450[94] = 0.0;
   out_5507197653198032450[95] = 1.0;
   out_5507197653198032450[96] = 0.0;
   out_5507197653198032450[97] = 0.0;
   out_5507197653198032450[98] = 0.0;
   out_5507197653198032450[99] = 0.0;
   out_5507197653198032450[100] = 0.0;
   out_5507197653198032450[101] = 0.0;
   out_5507197653198032450[102] = 0.0;
   out_5507197653198032450[103] = 0.0;
   out_5507197653198032450[104] = 0.0;
   out_5507197653198032450[105] = 0.0;
   out_5507197653198032450[106] = 0.0;
   out_5507197653198032450[107] = 0.0;
   out_5507197653198032450[108] = 0.0;
   out_5507197653198032450[109] = 0.0;
   out_5507197653198032450[110] = 0.0;
   out_5507197653198032450[111] = 0.0;
   out_5507197653198032450[112] = 0.0;
   out_5507197653198032450[113] = 0.0;
   out_5507197653198032450[114] = 1.0;
   out_5507197653198032450[115] = 0.0;
   out_5507197653198032450[116] = 0.0;
   out_5507197653198032450[117] = 0.0;
   out_5507197653198032450[118] = 0.0;
   out_5507197653198032450[119] = 0.0;
   out_5507197653198032450[120] = 0.0;
   out_5507197653198032450[121] = 0.0;
   out_5507197653198032450[122] = 0.0;
   out_5507197653198032450[123] = 0.0;
   out_5507197653198032450[124] = 0.0;
   out_5507197653198032450[125] = 0.0;
   out_5507197653198032450[126] = 0.0;
   out_5507197653198032450[127] = 0.0;
   out_5507197653198032450[128] = 0.0;
   out_5507197653198032450[129] = 0.0;
   out_5507197653198032450[130] = 0.0;
   out_5507197653198032450[131] = 0.0;
   out_5507197653198032450[132] = 0.0;
   out_5507197653198032450[133] = 1.0;
   out_5507197653198032450[134] = 0.0;
   out_5507197653198032450[135] = 0.0;
   out_5507197653198032450[136] = 0.0;
   out_5507197653198032450[137] = 0.0;
   out_5507197653198032450[138] = 0.0;
   out_5507197653198032450[139] = 0.0;
   out_5507197653198032450[140] = 0.0;
   out_5507197653198032450[141] = 0.0;
   out_5507197653198032450[142] = 0.0;
   out_5507197653198032450[143] = 0.0;
   out_5507197653198032450[144] = 0.0;
   out_5507197653198032450[145] = 0.0;
   out_5507197653198032450[146] = 0.0;
   out_5507197653198032450[147] = 0.0;
   out_5507197653198032450[148] = 0.0;
   out_5507197653198032450[149] = 0.0;
   out_5507197653198032450[150] = 0.0;
   out_5507197653198032450[151] = 0.0;
   out_5507197653198032450[152] = 1.0;
   out_5507197653198032450[153] = 0.0;
   out_5507197653198032450[154] = 0.0;
   out_5507197653198032450[155] = 0.0;
   out_5507197653198032450[156] = 0.0;
   out_5507197653198032450[157] = 0.0;
   out_5507197653198032450[158] = 0.0;
   out_5507197653198032450[159] = 0.0;
   out_5507197653198032450[160] = 0.0;
   out_5507197653198032450[161] = 0.0;
   out_5507197653198032450[162] = 0.0;
   out_5507197653198032450[163] = 0.0;
   out_5507197653198032450[164] = 0.0;
   out_5507197653198032450[165] = 0.0;
   out_5507197653198032450[166] = 0.0;
   out_5507197653198032450[167] = 0.0;
   out_5507197653198032450[168] = 0.0;
   out_5507197653198032450[169] = 0.0;
   out_5507197653198032450[170] = 0.0;
   out_5507197653198032450[171] = 1.0;
   out_5507197653198032450[172] = 0.0;
   out_5507197653198032450[173] = 0.0;
   out_5507197653198032450[174] = 0.0;
   out_5507197653198032450[175] = 0.0;
   out_5507197653198032450[176] = 0.0;
   out_5507197653198032450[177] = 0.0;
   out_5507197653198032450[178] = 0.0;
   out_5507197653198032450[179] = 0.0;
   out_5507197653198032450[180] = 0.0;
   out_5507197653198032450[181] = 0.0;
   out_5507197653198032450[182] = 0.0;
   out_5507197653198032450[183] = 0.0;
   out_5507197653198032450[184] = 0.0;
   out_5507197653198032450[185] = 0.0;
   out_5507197653198032450[186] = 0.0;
   out_5507197653198032450[187] = 0.0;
   out_5507197653198032450[188] = 0.0;
   out_5507197653198032450[189] = 0.0;
   out_5507197653198032450[190] = 1.0;
   out_5507197653198032450[191] = 0.0;
   out_5507197653198032450[192] = 0.0;
   out_5507197653198032450[193] = 0.0;
   out_5507197653198032450[194] = 0.0;
   out_5507197653198032450[195] = 0.0;
   out_5507197653198032450[196] = 0.0;
   out_5507197653198032450[197] = 0.0;
   out_5507197653198032450[198] = 0.0;
   out_5507197653198032450[199] = 0.0;
   out_5507197653198032450[200] = 0.0;
   out_5507197653198032450[201] = 0.0;
   out_5507197653198032450[202] = 0.0;
   out_5507197653198032450[203] = 0.0;
   out_5507197653198032450[204] = 0.0;
   out_5507197653198032450[205] = 0.0;
   out_5507197653198032450[206] = 0.0;
   out_5507197653198032450[207] = 0.0;
   out_5507197653198032450[208] = 0.0;
   out_5507197653198032450[209] = 1.0;
   out_5507197653198032450[210] = 0.0;
   out_5507197653198032450[211] = 0.0;
   out_5507197653198032450[212] = 0.0;
   out_5507197653198032450[213] = 0.0;
   out_5507197653198032450[214] = 0.0;
   out_5507197653198032450[215] = 0.0;
   out_5507197653198032450[216] = 0.0;
   out_5507197653198032450[217] = 0.0;
   out_5507197653198032450[218] = 0.0;
   out_5507197653198032450[219] = 0.0;
   out_5507197653198032450[220] = 0.0;
   out_5507197653198032450[221] = 0.0;
   out_5507197653198032450[222] = 0.0;
   out_5507197653198032450[223] = 0.0;
   out_5507197653198032450[224] = 0.0;
   out_5507197653198032450[225] = 0.0;
   out_5507197653198032450[226] = 0.0;
   out_5507197653198032450[227] = 0.0;
   out_5507197653198032450[228] = 1.0;
   out_5507197653198032450[229] = 0.0;
   out_5507197653198032450[230] = 0.0;
   out_5507197653198032450[231] = 0.0;
   out_5507197653198032450[232] = 0.0;
   out_5507197653198032450[233] = 0.0;
   out_5507197653198032450[234] = 0.0;
   out_5507197653198032450[235] = 0.0;
   out_5507197653198032450[236] = 0.0;
   out_5507197653198032450[237] = 0.0;
   out_5507197653198032450[238] = 0.0;
   out_5507197653198032450[239] = 0.0;
   out_5507197653198032450[240] = 0.0;
   out_5507197653198032450[241] = 0.0;
   out_5507197653198032450[242] = 0.0;
   out_5507197653198032450[243] = 0.0;
   out_5507197653198032450[244] = 0.0;
   out_5507197653198032450[245] = 0.0;
   out_5507197653198032450[246] = 0.0;
   out_5507197653198032450[247] = 1.0;
   out_5507197653198032450[248] = 0.0;
   out_5507197653198032450[249] = 0.0;
   out_5507197653198032450[250] = 0.0;
   out_5507197653198032450[251] = 0.0;
   out_5507197653198032450[252] = 0.0;
   out_5507197653198032450[253] = 0.0;
   out_5507197653198032450[254] = 0.0;
   out_5507197653198032450[255] = 0.0;
   out_5507197653198032450[256] = 0.0;
   out_5507197653198032450[257] = 0.0;
   out_5507197653198032450[258] = 0.0;
   out_5507197653198032450[259] = 0.0;
   out_5507197653198032450[260] = 0.0;
   out_5507197653198032450[261] = 0.0;
   out_5507197653198032450[262] = 0.0;
   out_5507197653198032450[263] = 0.0;
   out_5507197653198032450[264] = 0.0;
   out_5507197653198032450[265] = 0.0;
   out_5507197653198032450[266] = 1.0;
   out_5507197653198032450[267] = 0.0;
   out_5507197653198032450[268] = 0.0;
   out_5507197653198032450[269] = 0.0;
   out_5507197653198032450[270] = 0.0;
   out_5507197653198032450[271] = 0.0;
   out_5507197653198032450[272] = 0.0;
   out_5507197653198032450[273] = 0.0;
   out_5507197653198032450[274] = 0.0;
   out_5507197653198032450[275] = 0.0;
   out_5507197653198032450[276] = 0.0;
   out_5507197653198032450[277] = 0.0;
   out_5507197653198032450[278] = 0.0;
   out_5507197653198032450[279] = 0.0;
   out_5507197653198032450[280] = 0.0;
   out_5507197653198032450[281] = 0.0;
   out_5507197653198032450[282] = 0.0;
   out_5507197653198032450[283] = 0.0;
   out_5507197653198032450[284] = 0.0;
   out_5507197653198032450[285] = 1.0;
   out_5507197653198032450[286] = 0.0;
   out_5507197653198032450[287] = 0.0;
   out_5507197653198032450[288] = 0.0;
   out_5507197653198032450[289] = 0.0;
   out_5507197653198032450[290] = 0.0;
   out_5507197653198032450[291] = 0.0;
   out_5507197653198032450[292] = 0.0;
   out_5507197653198032450[293] = 0.0;
   out_5507197653198032450[294] = 0.0;
   out_5507197653198032450[295] = 0.0;
   out_5507197653198032450[296] = 0.0;
   out_5507197653198032450[297] = 0.0;
   out_5507197653198032450[298] = 0.0;
   out_5507197653198032450[299] = 0.0;
   out_5507197653198032450[300] = 0.0;
   out_5507197653198032450[301] = 0.0;
   out_5507197653198032450[302] = 0.0;
   out_5507197653198032450[303] = 0.0;
   out_5507197653198032450[304] = 1.0;
   out_5507197653198032450[305] = 0.0;
   out_5507197653198032450[306] = 0.0;
   out_5507197653198032450[307] = 0.0;
   out_5507197653198032450[308] = 0.0;
   out_5507197653198032450[309] = 0.0;
   out_5507197653198032450[310] = 0.0;
   out_5507197653198032450[311] = 0.0;
   out_5507197653198032450[312] = 0.0;
   out_5507197653198032450[313] = 0.0;
   out_5507197653198032450[314] = 0.0;
   out_5507197653198032450[315] = 0.0;
   out_5507197653198032450[316] = 0.0;
   out_5507197653198032450[317] = 0.0;
   out_5507197653198032450[318] = 0.0;
   out_5507197653198032450[319] = 0.0;
   out_5507197653198032450[320] = 0.0;
   out_5507197653198032450[321] = 0.0;
   out_5507197653198032450[322] = 0.0;
   out_5507197653198032450[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8465735489954780143) {
   out_8465735489954780143[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8465735489954780143[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8465735489954780143[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8465735489954780143[3] = dt*state[12] + state[3];
   out_8465735489954780143[4] = dt*state[13] + state[4];
   out_8465735489954780143[5] = dt*state[14] + state[5];
   out_8465735489954780143[6] = state[6];
   out_8465735489954780143[7] = state[7];
   out_8465735489954780143[8] = state[8];
   out_8465735489954780143[9] = state[9];
   out_8465735489954780143[10] = state[10];
   out_8465735489954780143[11] = state[11];
   out_8465735489954780143[12] = state[12];
   out_8465735489954780143[13] = state[13];
   out_8465735489954780143[14] = state[14];
   out_8465735489954780143[15] = state[15];
   out_8465735489954780143[16] = state[16];
   out_8465735489954780143[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7084358017667610949) {
   out_7084358017667610949[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7084358017667610949[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7084358017667610949[2] = 0;
   out_7084358017667610949[3] = 0;
   out_7084358017667610949[4] = 0;
   out_7084358017667610949[5] = 0;
   out_7084358017667610949[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7084358017667610949[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7084358017667610949[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7084358017667610949[9] = 0;
   out_7084358017667610949[10] = 0;
   out_7084358017667610949[11] = 0;
   out_7084358017667610949[12] = 0;
   out_7084358017667610949[13] = 0;
   out_7084358017667610949[14] = 0;
   out_7084358017667610949[15] = 0;
   out_7084358017667610949[16] = 0;
   out_7084358017667610949[17] = 0;
   out_7084358017667610949[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7084358017667610949[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7084358017667610949[20] = 0;
   out_7084358017667610949[21] = 0;
   out_7084358017667610949[22] = 0;
   out_7084358017667610949[23] = 0;
   out_7084358017667610949[24] = 0;
   out_7084358017667610949[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7084358017667610949[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7084358017667610949[27] = 0;
   out_7084358017667610949[28] = 0;
   out_7084358017667610949[29] = 0;
   out_7084358017667610949[30] = 0;
   out_7084358017667610949[31] = 0;
   out_7084358017667610949[32] = 0;
   out_7084358017667610949[33] = 0;
   out_7084358017667610949[34] = 0;
   out_7084358017667610949[35] = 0;
   out_7084358017667610949[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7084358017667610949[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7084358017667610949[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7084358017667610949[39] = 0;
   out_7084358017667610949[40] = 0;
   out_7084358017667610949[41] = 0;
   out_7084358017667610949[42] = 0;
   out_7084358017667610949[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7084358017667610949[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7084358017667610949[45] = 0;
   out_7084358017667610949[46] = 0;
   out_7084358017667610949[47] = 0;
   out_7084358017667610949[48] = 0;
   out_7084358017667610949[49] = 0;
   out_7084358017667610949[50] = 0;
   out_7084358017667610949[51] = 0;
   out_7084358017667610949[52] = 0;
   out_7084358017667610949[53] = 0;
   out_7084358017667610949[54] = 0;
   out_7084358017667610949[55] = 0;
   out_7084358017667610949[56] = 0;
   out_7084358017667610949[57] = 1;
   out_7084358017667610949[58] = 0;
   out_7084358017667610949[59] = 0;
   out_7084358017667610949[60] = 0;
   out_7084358017667610949[61] = 0;
   out_7084358017667610949[62] = 0;
   out_7084358017667610949[63] = 0;
   out_7084358017667610949[64] = 0;
   out_7084358017667610949[65] = 0;
   out_7084358017667610949[66] = dt;
   out_7084358017667610949[67] = 0;
   out_7084358017667610949[68] = 0;
   out_7084358017667610949[69] = 0;
   out_7084358017667610949[70] = 0;
   out_7084358017667610949[71] = 0;
   out_7084358017667610949[72] = 0;
   out_7084358017667610949[73] = 0;
   out_7084358017667610949[74] = 0;
   out_7084358017667610949[75] = 0;
   out_7084358017667610949[76] = 1;
   out_7084358017667610949[77] = 0;
   out_7084358017667610949[78] = 0;
   out_7084358017667610949[79] = 0;
   out_7084358017667610949[80] = 0;
   out_7084358017667610949[81] = 0;
   out_7084358017667610949[82] = 0;
   out_7084358017667610949[83] = 0;
   out_7084358017667610949[84] = 0;
   out_7084358017667610949[85] = dt;
   out_7084358017667610949[86] = 0;
   out_7084358017667610949[87] = 0;
   out_7084358017667610949[88] = 0;
   out_7084358017667610949[89] = 0;
   out_7084358017667610949[90] = 0;
   out_7084358017667610949[91] = 0;
   out_7084358017667610949[92] = 0;
   out_7084358017667610949[93] = 0;
   out_7084358017667610949[94] = 0;
   out_7084358017667610949[95] = 1;
   out_7084358017667610949[96] = 0;
   out_7084358017667610949[97] = 0;
   out_7084358017667610949[98] = 0;
   out_7084358017667610949[99] = 0;
   out_7084358017667610949[100] = 0;
   out_7084358017667610949[101] = 0;
   out_7084358017667610949[102] = 0;
   out_7084358017667610949[103] = 0;
   out_7084358017667610949[104] = dt;
   out_7084358017667610949[105] = 0;
   out_7084358017667610949[106] = 0;
   out_7084358017667610949[107] = 0;
   out_7084358017667610949[108] = 0;
   out_7084358017667610949[109] = 0;
   out_7084358017667610949[110] = 0;
   out_7084358017667610949[111] = 0;
   out_7084358017667610949[112] = 0;
   out_7084358017667610949[113] = 0;
   out_7084358017667610949[114] = 1;
   out_7084358017667610949[115] = 0;
   out_7084358017667610949[116] = 0;
   out_7084358017667610949[117] = 0;
   out_7084358017667610949[118] = 0;
   out_7084358017667610949[119] = 0;
   out_7084358017667610949[120] = 0;
   out_7084358017667610949[121] = 0;
   out_7084358017667610949[122] = 0;
   out_7084358017667610949[123] = 0;
   out_7084358017667610949[124] = 0;
   out_7084358017667610949[125] = 0;
   out_7084358017667610949[126] = 0;
   out_7084358017667610949[127] = 0;
   out_7084358017667610949[128] = 0;
   out_7084358017667610949[129] = 0;
   out_7084358017667610949[130] = 0;
   out_7084358017667610949[131] = 0;
   out_7084358017667610949[132] = 0;
   out_7084358017667610949[133] = 1;
   out_7084358017667610949[134] = 0;
   out_7084358017667610949[135] = 0;
   out_7084358017667610949[136] = 0;
   out_7084358017667610949[137] = 0;
   out_7084358017667610949[138] = 0;
   out_7084358017667610949[139] = 0;
   out_7084358017667610949[140] = 0;
   out_7084358017667610949[141] = 0;
   out_7084358017667610949[142] = 0;
   out_7084358017667610949[143] = 0;
   out_7084358017667610949[144] = 0;
   out_7084358017667610949[145] = 0;
   out_7084358017667610949[146] = 0;
   out_7084358017667610949[147] = 0;
   out_7084358017667610949[148] = 0;
   out_7084358017667610949[149] = 0;
   out_7084358017667610949[150] = 0;
   out_7084358017667610949[151] = 0;
   out_7084358017667610949[152] = 1;
   out_7084358017667610949[153] = 0;
   out_7084358017667610949[154] = 0;
   out_7084358017667610949[155] = 0;
   out_7084358017667610949[156] = 0;
   out_7084358017667610949[157] = 0;
   out_7084358017667610949[158] = 0;
   out_7084358017667610949[159] = 0;
   out_7084358017667610949[160] = 0;
   out_7084358017667610949[161] = 0;
   out_7084358017667610949[162] = 0;
   out_7084358017667610949[163] = 0;
   out_7084358017667610949[164] = 0;
   out_7084358017667610949[165] = 0;
   out_7084358017667610949[166] = 0;
   out_7084358017667610949[167] = 0;
   out_7084358017667610949[168] = 0;
   out_7084358017667610949[169] = 0;
   out_7084358017667610949[170] = 0;
   out_7084358017667610949[171] = 1;
   out_7084358017667610949[172] = 0;
   out_7084358017667610949[173] = 0;
   out_7084358017667610949[174] = 0;
   out_7084358017667610949[175] = 0;
   out_7084358017667610949[176] = 0;
   out_7084358017667610949[177] = 0;
   out_7084358017667610949[178] = 0;
   out_7084358017667610949[179] = 0;
   out_7084358017667610949[180] = 0;
   out_7084358017667610949[181] = 0;
   out_7084358017667610949[182] = 0;
   out_7084358017667610949[183] = 0;
   out_7084358017667610949[184] = 0;
   out_7084358017667610949[185] = 0;
   out_7084358017667610949[186] = 0;
   out_7084358017667610949[187] = 0;
   out_7084358017667610949[188] = 0;
   out_7084358017667610949[189] = 0;
   out_7084358017667610949[190] = 1;
   out_7084358017667610949[191] = 0;
   out_7084358017667610949[192] = 0;
   out_7084358017667610949[193] = 0;
   out_7084358017667610949[194] = 0;
   out_7084358017667610949[195] = 0;
   out_7084358017667610949[196] = 0;
   out_7084358017667610949[197] = 0;
   out_7084358017667610949[198] = 0;
   out_7084358017667610949[199] = 0;
   out_7084358017667610949[200] = 0;
   out_7084358017667610949[201] = 0;
   out_7084358017667610949[202] = 0;
   out_7084358017667610949[203] = 0;
   out_7084358017667610949[204] = 0;
   out_7084358017667610949[205] = 0;
   out_7084358017667610949[206] = 0;
   out_7084358017667610949[207] = 0;
   out_7084358017667610949[208] = 0;
   out_7084358017667610949[209] = 1;
   out_7084358017667610949[210] = 0;
   out_7084358017667610949[211] = 0;
   out_7084358017667610949[212] = 0;
   out_7084358017667610949[213] = 0;
   out_7084358017667610949[214] = 0;
   out_7084358017667610949[215] = 0;
   out_7084358017667610949[216] = 0;
   out_7084358017667610949[217] = 0;
   out_7084358017667610949[218] = 0;
   out_7084358017667610949[219] = 0;
   out_7084358017667610949[220] = 0;
   out_7084358017667610949[221] = 0;
   out_7084358017667610949[222] = 0;
   out_7084358017667610949[223] = 0;
   out_7084358017667610949[224] = 0;
   out_7084358017667610949[225] = 0;
   out_7084358017667610949[226] = 0;
   out_7084358017667610949[227] = 0;
   out_7084358017667610949[228] = 1;
   out_7084358017667610949[229] = 0;
   out_7084358017667610949[230] = 0;
   out_7084358017667610949[231] = 0;
   out_7084358017667610949[232] = 0;
   out_7084358017667610949[233] = 0;
   out_7084358017667610949[234] = 0;
   out_7084358017667610949[235] = 0;
   out_7084358017667610949[236] = 0;
   out_7084358017667610949[237] = 0;
   out_7084358017667610949[238] = 0;
   out_7084358017667610949[239] = 0;
   out_7084358017667610949[240] = 0;
   out_7084358017667610949[241] = 0;
   out_7084358017667610949[242] = 0;
   out_7084358017667610949[243] = 0;
   out_7084358017667610949[244] = 0;
   out_7084358017667610949[245] = 0;
   out_7084358017667610949[246] = 0;
   out_7084358017667610949[247] = 1;
   out_7084358017667610949[248] = 0;
   out_7084358017667610949[249] = 0;
   out_7084358017667610949[250] = 0;
   out_7084358017667610949[251] = 0;
   out_7084358017667610949[252] = 0;
   out_7084358017667610949[253] = 0;
   out_7084358017667610949[254] = 0;
   out_7084358017667610949[255] = 0;
   out_7084358017667610949[256] = 0;
   out_7084358017667610949[257] = 0;
   out_7084358017667610949[258] = 0;
   out_7084358017667610949[259] = 0;
   out_7084358017667610949[260] = 0;
   out_7084358017667610949[261] = 0;
   out_7084358017667610949[262] = 0;
   out_7084358017667610949[263] = 0;
   out_7084358017667610949[264] = 0;
   out_7084358017667610949[265] = 0;
   out_7084358017667610949[266] = 1;
   out_7084358017667610949[267] = 0;
   out_7084358017667610949[268] = 0;
   out_7084358017667610949[269] = 0;
   out_7084358017667610949[270] = 0;
   out_7084358017667610949[271] = 0;
   out_7084358017667610949[272] = 0;
   out_7084358017667610949[273] = 0;
   out_7084358017667610949[274] = 0;
   out_7084358017667610949[275] = 0;
   out_7084358017667610949[276] = 0;
   out_7084358017667610949[277] = 0;
   out_7084358017667610949[278] = 0;
   out_7084358017667610949[279] = 0;
   out_7084358017667610949[280] = 0;
   out_7084358017667610949[281] = 0;
   out_7084358017667610949[282] = 0;
   out_7084358017667610949[283] = 0;
   out_7084358017667610949[284] = 0;
   out_7084358017667610949[285] = 1;
   out_7084358017667610949[286] = 0;
   out_7084358017667610949[287] = 0;
   out_7084358017667610949[288] = 0;
   out_7084358017667610949[289] = 0;
   out_7084358017667610949[290] = 0;
   out_7084358017667610949[291] = 0;
   out_7084358017667610949[292] = 0;
   out_7084358017667610949[293] = 0;
   out_7084358017667610949[294] = 0;
   out_7084358017667610949[295] = 0;
   out_7084358017667610949[296] = 0;
   out_7084358017667610949[297] = 0;
   out_7084358017667610949[298] = 0;
   out_7084358017667610949[299] = 0;
   out_7084358017667610949[300] = 0;
   out_7084358017667610949[301] = 0;
   out_7084358017667610949[302] = 0;
   out_7084358017667610949[303] = 0;
   out_7084358017667610949[304] = 1;
   out_7084358017667610949[305] = 0;
   out_7084358017667610949[306] = 0;
   out_7084358017667610949[307] = 0;
   out_7084358017667610949[308] = 0;
   out_7084358017667610949[309] = 0;
   out_7084358017667610949[310] = 0;
   out_7084358017667610949[311] = 0;
   out_7084358017667610949[312] = 0;
   out_7084358017667610949[313] = 0;
   out_7084358017667610949[314] = 0;
   out_7084358017667610949[315] = 0;
   out_7084358017667610949[316] = 0;
   out_7084358017667610949[317] = 0;
   out_7084358017667610949[318] = 0;
   out_7084358017667610949[319] = 0;
   out_7084358017667610949[320] = 0;
   out_7084358017667610949[321] = 0;
   out_7084358017667610949[322] = 0;
   out_7084358017667610949[323] = 1;
}
void h_4(double *state, double *unused, double *out_5086288106009602117) {
   out_5086288106009602117[0] = state[6] + state[9];
   out_5086288106009602117[1] = state[7] + state[10];
   out_5086288106009602117[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_354679268158951715) {
   out_354679268158951715[0] = 0;
   out_354679268158951715[1] = 0;
   out_354679268158951715[2] = 0;
   out_354679268158951715[3] = 0;
   out_354679268158951715[4] = 0;
   out_354679268158951715[5] = 0;
   out_354679268158951715[6] = 1;
   out_354679268158951715[7] = 0;
   out_354679268158951715[8] = 0;
   out_354679268158951715[9] = 1;
   out_354679268158951715[10] = 0;
   out_354679268158951715[11] = 0;
   out_354679268158951715[12] = 0;
   out_354679268158951715[13] = 0;
   out_354679268158951715[14] = 0;
   out_354679268158951715[15] = 0;
   out_354679268158951715[16] = 0;
   out_354679268158951715[17] = 0;
   out_354679268158951715[18] = 0;
   out_354679268158951715[19] = 0;
   out_354679268158951715[20] = 0;
   out_354679268158951715[21] = 0;
   out_354679268158951715[22] = 0;
   out_354679268158951715[23] = 0;
   out_354679268158951715[24] = 0;
   out_354679268158951715[25] = 1;
   out_354679268158951715[26] = 0;
   out_354679268158951715[27] = 0;
   out_354679268158951715[28] = 1;
   out_354679268158951715[29] = 0;
   out_354679268158951715[30] = 0;
   out_354679268158951715[31] = 0;
   out_354679268158951715[32] = 0;
   out_354679268158951715[33] = 0;
   out_354679268158951715[34] = 0;
   out_354679268158951715[35] = 0;
   out_354679268158951715[36] = 0;
   out_354679268158951715[37] = 0;
   out_354679268158951715[38] = 0;
   out_354679268158951715[39] = 0;
   out_354679268158951715[40] = 0;
   out_354679268158951715[41] = 0;
   out_354679268158951715[42] = 0;
   out_354679268158951715[43] = 0;
   out_354679268158951715[44] = 1;
   out_354679268158951715[45] = 0;
   out_354679268158951715[46] = 0;
   out_354679268158951715[47] = 1;
   out_354679268158951715[48] = 0;
   out_354679268158951715[49] = 0;
   out_354679268158951715[50] = 0;
   out_354679268158951715[51] = 0;
   out_354679268158951715[52] = 0;
   out_354679268158951715[53] = 0;
}
void h_10(double *state, double *unused, double *out_4884892022508388741) {
   out_4884892022508388741[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_4884892022508388741[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_4884892022508388741[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_1066608819722372465) {
   out_1066608819722372465[0] = 0;
   out_1066608819722372465[1] = 9.8100000000000005*cos(state[1]);
   out_1066608819722372465[2] = 0;
   out_1066608819722372465[3] = 0;
   out_1066608819722372465[4] = -state[8];
   out_1066608819722372465[5] = state[7];
   out_1066608819722372465[6] = 0;
   out_1066608819722372465[7] = state[5];
   out_1066608819722372465[8] = -state[4];
   out_1066608819722372465[9] = 0;
   out_1066608819722372465[10] = 0;
   out_1066608819722372465[11] = 0;
   out_1066608819722372465[12] = 1;
   out_1066608819722372465[13] = 0;
   out_1066608819722372465[14] = 0;
   out_1066608819722372465[15] = 1;
   out_1066608819722372465[16] = 0;
   out_1066608819722372465[17] = 0;
   out_1066608819722372465[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_1066608819722372465[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_1066608819722372465[20] = 0;
   out_1066608819722372465[21] = state[8];
   out_1066608819722372465[22] = 0;
   out_1066608819722372465[23] = -state[6];
   out_1066608819722372465[24] = -state[5];
   out_1066608819722372465[25] = 0;
   out_1066608819722372465[26] = state[3];
   out_1066608819722372465[27] = 0;
   out_1066608819722372465[28] = 0;
   out_1066608819722372465[29] = 0;
   out_1066608819722372465[30] = 0;
   out_1066608819722372465[31] = 1;
   out_1066608819722372465[32] = 0;
   out_1066608819722372465[33] = 0;
   out_1066608819722372465[34] = 1;
   out_1066608819722372465[35] = 0;
   out_1066608819722372465[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_1066608819722372465[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_1066608819722372465[38] = 0;
   out_1066608819722372465[39] = -state[7];
   out_1066608819722372465[40] = state[6];
   out_1066608819722372465[41] = 0;
   out_1066608819722372465[42] = state[4];
   out_1066608819722372465[43] = -state[3];
   out_1066608819722372465[44] = 0;
   out_1066608819722372465[45] = 0;
   out_1066608819722372465[46] = 0;
   out_1066608819722372465[47] = 0;
   out_1066608819722372465[48] = 0;
   out_1066608819722372465[49] = 0;
   out_1066608819722372465[50] = 1;
   out_1066608819722372465[51] = 0;
   out_1066608819722372465[52] = 0;
   out_1066608819722372465[53] = 1;
}
void h_13(double *state, double *unused, double *out_9106628773609145872) {
   out_9106628773609145872[0] = state[3];
   out_9106628773609145872[1] = state[4];
   out_9106628773609145872[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2857594557173381086) {
   out_2857594557173381086[0] = 0;
   out_2857594557173381086[1] = 0;
   out_2857594557173381086[2] = 0;
   out_2857594557173381086[3] = 1;
   out_2857594557173381086[4] = 0;
   out_2857594557173381086[5] = 0;
   out_2857594557173381086[6] = 0;
   out_2857594557173381086[7] = 0;
   out_2857594557173381086[8] = 0;
   out_2857594557173381086[9] = 0;
   out_2857594557173381086[10] = 0;
   out_2857594557173381086[11] = 0;
   out_2857594557173381086[12] = 0;
   out_2857594557173381086[13] = 0;
   out_2857594557173381086[14] = 0;
   out_2857594557173381086[15] = 0;
   out_2857594557173381086[16] = 0;
   out_2857594557173381086[17] = 0;
   out_2857594557173381086[18] = 0;
   out_2857594557173381086[19] = 0;
   out_2857594557173381086[20] = 0;
   out_2857594557173381086[21] = 0;
   out_2857594557173381086[22] = 1;
   out_2857594557173381086[23] = 0;
   out_2857594557173381086[24] = 0;
   out_2857594557173381086[25] = 0;
   out_2857594557173381086[26] = 0;
   out_2857594557173381086[27] = 0;
   out_2857594557173381086[28] = 0;
   out_2857594557173381086[29] = 0;
   out_2857594557173381086[30] = 0;
   out_2857594557173381086[31] = 0;
   out_2857594557173381086[32] = 0;
   out_2857594557173381086[33] = 0;
   out_2857594557173381086[34] = 0;
   out_2857594557173381086[35] = 0;
   out_2857594557173381086[36] = 0;
   out_2857594557173381086[37] = 0;
   out_2857594557173381086[38] = 0;
   out_2857594557173381086[39] = 0;
   out_2857594557173381086[40] = 0;
   out_2857594557173381086[41] = 1;
   out_2857594557173381086[42] = 0;
   out_2857594557173381086[43] = 0;
   out_2857594557173381086[44] = 0;
   out_2857594557173381086[45] = 0;
   out_2857594557173381086[46] = 0;
   out_2857594557173381086[47] = 0;
   out_2857594557173381086[48] = 0;
   out_2857594557173381086[49] = 0;
   out_2857594557173381086[50] = 0;
   out_2857594557173381086[51] = 0;
   out_2857594557173381086[52] = 0;
   out_2857594557173381086[53] = 0;
}
void h_14(double *state, double *unused, double *out_6735107942124736820) {
   out_6735107942124736820[0] = state[6];
   out_6735107942124736820[1] = state[7];
   out_6735107942124736820[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3437467700454324011) {
   out_3437467700454324011[0] = 0;
   out_3437467700454324011[1] = 0;
   out_3437467700454324011[2] = 0;
   out_3437467700454324011[3] = 0;
   out_3437467700454324011[4] = 0;
   out_3437467700454324011[5] = 0;
   out_3437467700454324011[6] = 1;
   out_3437467700454324011[7] = 0;
   out_3437467700454324011[8] = 0;
   out_3437467700454324011[9] = 0;
   out_3437467700454324011[10] = 0;
   out_3437467700454324011[11] = 0;
   out_3437467700454324011[12] = 0;
   out_3437467700454324011[13] = 0;
   out_3437467700454324011[14] = 0;
   out_3437467700454324011[15] = 0;
   out_3437467700454324011[16] = 0;
   out_3437467700454324011[17] = 0;
   out_3437467700454324011[18] = 0;
   out_3437467700454324011[19] = 0;
   out_3437467700454324011[20] = 0;
   out_3437467700454324011[21] = 0;
   out_3437467700454324011[22] = 0;
   out_3437467700454324011[23] = 0;
   out_3437467700454324011[24] = 0;
   out_3437467700454324011[25] = 1;
   out_3437467700454324011[26] = 0;
   out_3437467700454324011[27] = 0;
   out_3437467700454324011[28] = 0;
   out_3437467700454324011[29] = 0;
   out_3437467700454324011[30] = 0;
   out_3437467700454324011[31] = 0;
   out_3437467700454324011[32] = 0;
   out_3437467700454324011[33] = 0;
   out_3437467700454324011[34] = 0;
   out_3437467700454324011[35] = 0;
   out_3437467700454324011[36] = 0;
   out_3437467700454324011[37] = 0;
   out_3437467700454324011[38] = 0;
   out_3437467700454324011[39] = 0;
   out_3437467700454324011[40] = 0;
   out_3437467700454324011[41] = 0;
   out_3437467700454324011[42] = 0;
   out_3437467700454324011[43] = 0;
   out_3437467700454324011[44] = 1;
   out_3437467700454324011[45] = 0;
   out_3437467700454324011[46] = 0;
   out_3437467700454324011[47] = 0;
   out_3437467700454324011[48] = 0;
   out_3437467700454324011[49] = 0;
   out_3437467700454324011[50] = 0;
   out_3437467700454324011[51] = 0;
   out_3437467700454324011[52] = 0;
   out_3437467700454324011[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_5999678731626763007) {
  err_fun(nom_x, delta_x, out_5999678731626763007);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5109969285037859944) {
  inv_err_fun(nom_x, true_x, out_5109969285037859944);
}
void pose_H_mod_fun(double *state, double *out_5507197653198032450) {
  H_mod_fun(state, out_5507197653198032450);
}
void pose_f_fun(double *state, double dt, double *out_8465735489954780143) {
  f_fun(state,  dt, out_8465735489954780143);
}
void pose_F_fun(double *state, double dt, double *out_7084358017667610949) {
  F_fun(state,  dt, out_7084358017667610949);
}
void pose_h_4(double *state, double *unused, double *out_5086288106009602117) {
  h_4(state, unused, out_5086288106009602117);
}
void pose_H_4(double *state, double *unused, double *out_354679268158951715) {
  H_4(state, unused, out_354679268158951715);
}
void pose_h_10(double *state, double *unused, double *out_4884892022508388741) {
  h_10(state, unused, out_4884892022508388741);
}
void pose_H_10(double *state, double *unused, double *out_1066608819722372465) {
  H_10(state, unused, out_1066608819722372465);
}
void pose_h_13(double *state, double *unused, double *out_9106628773609145872) {
  h_13(state, unused, out_9106628773609145872);
}
void pose_H_13(double *state, double *unused, double *out_2857594557173381086) {
  H_13(state, unused, out_2857594557173381086);
}
void pose_h_14(double *state, double *unused, double *out_6735107942124736820) {
  h_14(state, unused, out_6735107942124736820);
}
void pose_H_14(double *state, double *unused, double *out_3437467700454324011) {
  H_14(state, unused, out_3437467700454324011);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
