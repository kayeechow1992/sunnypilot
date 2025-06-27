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
void err_fun(double *nom_x, double *delta_x, double *out_8557638590374089642) {
   out_8557638590374089642[0] = delta_x[0] + nom_x[0];
   out_8557638590374089642[1] = delta_x[1] + nom_x[1];
   out_8557638590374089642[2] = delta_x[2] + nom_x[2];
   out_8557638590374089642[3] = delta_x[3] + nom_x[3];
   out_8557638590374089642[4] = delta_x[4] + nom_x[4];
   out_8557638590374089642[5] = delta_x[5] + nom_x[5];
   out_8557638590374089642[6] = delta_x[6] + nom_x[6];
   out_8557638590374089642[7] = delta_x[7] + nom_x[7];
   out_8557638590374089642[8] = delta_x[8] + nom_x[8];
   out_8557638590374089642[9] = delta_x[9] + nom_x[9];
   out_8557638590374089642[10] = delta_x[10] + nom_x[10];
   out_8557638590374089642[11] = delta_x[11] + nom_x[11];
   out_8557638590374089642[12] = delta_x[12] + nom_x[12];
   out_8557638590374089642[13] = delta_x[13] + nom_x[13];
   out_8557638590374089642[14] = delta_x[14] + nom_x[14];
   out_8557638590374089642[15] = delta_x[15] + nom_x[15];
   out_8557638590374089642[16] = delta_x[16] + nom_x[16];
   out_8557638590374089642[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2952464144299303284) {
   out_2952464144299303284[0] = -nom_x[0] + true_x[0];
   out_2952464144299303284[1] = -nom_x[1] + true_x[1];
   out_2952464144299303284[2] = -nom_x[2] + true_x[2];
   out_2952464144299303284[3] = -nom_x[3] + true_x[3];
   out_2952464144299303284[4] = -nom_x[4] + true_x[4];
   out_2952464144299303284[5] = -nom_x[5] + true_x[5];
   out_2952464144299303284[6] = -nom_x[6] + true_x[6];
   out_2952464144299303284[7] = -nom_x[7] + true_x[7];
   out_2952464144299303284[8] = -nom_x[8] + true_x[8];
   out_2952464144299303284[9] = -nom_x[9] + true_x[9];
   out_2952464144299303284[10] = -nom_x[10] + true_x[10];
   out_2952464144299303284[11] = -nom_x[11] + true_x[11];
   out_2952464144299303284[12] = -nom_x[12] + true_x[12];
   out_2952464144299303284[13] = -nom_x[13] + true_x[13];
   out_2952464144299303284[14] = -nom_x[14] + true_x[14];
   out_2952464144299303284[15] = -nom_x[15] + true_x[15];
   out_2952464144299303284[16] = -nom_x[16] + true_x[16];
   out_2952464144299303284[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_1165030831798606535) {
   out_1165030831798606535[0] = 1.0;
   out_1165030831798606535[1] = 0.0;
   out_1165030831798606535[2] = 0.0;
   out_1165030831798606535[3] = 0.0;
   out_1165030831798606535[4] = 0.0;
   out_1165030831798606535[5] = 0.0;
   out_1165030831798606535[6] = 0.0;
   out_1165030831798606535[7] = 0.0;
   out_1165030831798606535[8] = 0.0;
   out_1165030831798606535[9] = 0.0;
   out_1165030831798606535[10] = 0.0;
   out_1165030831798606535[11] = 0.0;
   out_1165030831798606535[12] = 0.0;
   out_1165030831798606535[13] = 0.0;
   out_1165030831798606535[14] = 0.0;
   out_1165030831798606535[15] = 0.0;
   out_1165030831798606535[16] = 0.0;
   out_1165030831798606535[17] = 0.0;
   out_1165030831798606535[18] = 0.0;
   out_1165030831798606535[19] = 1.0;
   out_1165030831798606535[20] = 0.0;
   out_1165030831798606535[21] = 0.0;
   out_1165030831798606535[22] = 0.0;
   out_1165030831798606535[23] = 0.0;
   out_1165030831798606535[24] = 0.0;
   out_1165030831798606535[25] = 0.0;
   out_1165030831798606535[26] = 0.0;
   out_1165030831798606535[27] = 0.0;
   out_1165030831798606535[28] = 0.0;
   out_1165030831798606535[29] = 0.0;
   out_1165030831798606535[30] = 0.0;
   out_1165030831798606535[31] = 0.0;
   out_1165030831798606535[32] = 0.0;
   out_1165030831798606535[33] = 0.0;
   out_1165030831798606535[34] = 0.0;
   out_1165030831798606535[35] = 0.0;
   out_1165030831798606535[36] = 0.0;
   out_1165030831798606535[37] = 0.0;
   out_1165030831798606535[38] = 1.0;
   out_1165030831798606535[39] = 0.0;
   out_1165030831798606535[40] = 0.0;
   out_1165030831798606535[41] = 0.0;
   out_1165030831798606535[42] = 0.0;
   out_1165030831798606535[43] = 0.0;
   out_1165030831798606535[44] = 0.0;
   out_1165030831798606535[45] = 0.0;
   out_1165030831798606535[46] = 0.0;
   out_1165030831798606535[47] = 0.0;
   out_1165030831798606535[48] = 0.0;
   out_1165030831798606535[49] = 0.0;
   out_1165030831798606535[50] = 0.0;
   out_1165030831798606535[51] = 0.0;
   out_1165030831798606535[52] = 0.0;
   out_1165030831798606535[53] = 0.0;
   out_1165030831798606535[54] = 0.0;
   out_1165030831798606535[55] = 0.0;
   out_1165030831798606535[56] = 0.0;
   out_1165030831798606535[57] = 1.0;
   out_1165030831798606535[58] = 0.0;
   out_1165030831798606535[59] = 0.0;
   out_1165030831798606535[60] = 0.0;
   out_1165030831798606535[61] = 0.0;
   out_1165030831798606535[62] = 0.0;
   out_1165030831798606535[63] = 0.0;
   out_1165030831798606535[64] = 0.0;
   out_1165030831798606535[65] = 0.0;
   out_1165030831798606535[66] = 0.0;
   out_1165030831798606535[67] = 0.0;
   out_1165030831798606535[68] = 0.0;
   out_1165030831798606535[69] = 0.0;
   out_1165030831798606535[70] = 0.0;
   out_1165030831798606535[71] = 0.0;
   out_1165030831798606535[72] = 0.0;
   out_1165030831798606535[73] = 0.0;
   out_1165030831798606535[74] = 0.0;
   out_1165030831798606535[75] = 0.0;
   out_1165030831798606535[76] = 1.0;
   out_1165030831798606535[77] = 0.0;
   out_1165030831798606535[78] = 0.0;
   out_1165030831798606535[79] = 0.0;
   out_1165030831798606535[80] = 0.0;
   out_1165030831798606535[81] = 0.0;
   out_1165030831798606535[82] = 0.0;
   out_1165030831798606535[83] = 0.0;
   out_1165030831798606535[84] = 0.0;
   out_1165030831798606535[85] = 0.0;
   out_1165030831798606535[86] = 0.0;
   out_1165030831798606535[87] = 0.0;
   out_1165030831798606535[88] = 0.0;
   out_1165030831798606535[89] = 0.0;
   out_1165030831798606535[90] = 0.0;
   out_1165030831798606535[91] = 0.0;
   out_1165030831798606535[92] = 0.0;
   out_1165030831798606535[93] = 0.0;
   out_1165030831798606535[94] = 0.0;
   out_1165030831798606535[95] = 1.0;
   out_1165030831798606535[96] = 0.0;
   out_1165030831798606535[97] = 0.0;
   out_1165030831798606535[98] = 0.0;
   out_1165030831798606535[99] = 0.0;
   out_1165030831798606535[100] = 0.0;
   out_1165030831798606535[101] = 0.0;
   out_1165030831798606535[102] = 0.0;
   out_1165030831798606535[103] = 0.0;
   out_1165030831798606535[104] = 0.0;
   out_1165030831798606535[105] = 0.0;
   out_1165030831798606535[106] = 0.0;
   out_1165030831798606535[107] = 0.0;
   out_1165030831798606535[108] = 0.0;
   out_1165030831798606535[109] = 0.0;
   out_1165030831798606535[110] = 0.0;
   out_1165030831798606535[111] = 0.0;
   out_1165030831798606535[112] = 0.0;
   out_1165030831798606535[113] = 0.0;
   out_1165030831798606535[114] = 1.0;
   out_1165030831798606535[115] = 0.0;
   out_1165030831798606535[116] = 0.0;
   out_1165030831798606535[117] = 0.0;
   out_1165030831798606535[118] = 0.0;
   out_1165030831798606535[119] = 0.0;
   out_1165030831798606535[120] = 0.0;
   out_1165030831798606535[121] = 0.0;
   out_1165030831798606535[122] = 0.0;
   out_1165030831798606535[123] = 0.0;
   out_1165030831798606535[124] = 0.0;
   out_1165030831798606535[125] = 0.0;
   out_1165030831798606535[126] = 0.0;
   out_1165030831798606535[127] = 0.0;
   out_1165030831798606535[128] = 0.0;
   out_1165030831798606535[129] = 0.0;
   out_1165030831798606535[130] = 0.0;
   out_1165030831798606535[131] = 0.0;
   out_1165030831798606535[132] = 0.0;
   out_1165030831798606535[133] = 1.0;
   out_1165030831798606535[134] = 0.0;
   out_1165030831798606535[135] = 0.0;
   out_1165030831798606535[136] = 0.0;
   out_1165030831798606535[137] = 0.0;
   out_1165030831798606535[138] = 0.0;
   out_1165030831798606535[139] = 0.0;
   out_1165030831798606535[140] = 0.0;
   out_1165030831798606535[141] = 0.0;
   out_1165030831798606535[142] = 0.0;
   out_1165030831798606535[143] = 0.0;
   out_1165030831798606535[144] = 0.0;
   out_1165030831798606535[145] = 0.0;
   out_1165030831798606535[146] = 0.0;
   out_1165030831798606535[147] = 0.0;
   out_1165030831798606535[148] = 0.0;
   out_1165030831798606535[149] = 0.0;
   out_1165030831798606535[150] = 0.0;
   out_1165030831798606535[151] = 0.0;
   out_1165030831798606535[152] = 1.0;
   out_1165030831798606535[153] = 0.0;
   out_1165030831798606535[154] = 0.0;
   out_1165030831798606535[155] = 0.0;
   out_1165030831798606535[156] = 0.0;
   out_1165030831798606535[157] = 0.0;
   out_1165030831798606535[158] = 0.0;
   out_1165030831798606535[159] = 0.0;
   out_1165030831798606535[160] = 0.0;
   out_1165030831798606535[161] = 0.0;
   out_1165030831798606535[162] = 0.0;
   out_1165030831798606535[163] = 0.0;
   out_1165030831798606535[164] = 0.0;
   out_1165030831798606535[165] = 0.0;
   out_1165030831798606535[166] = 0.0;
   out_1165030831798606535[167] = 0.0;
   out_1165030831798606535[168] = 0.0;
   out_1165030831798606535[169] = 0.0;
   out_1165030831798606535[170] = 0.0;
   out_1165030831798606535[171] = 1.0;
   out_1165030831798606535[172] = 0.0;
   out_1165030831798606535[173] = 0.0;
   out_1165030831798606535[174] = 0.0;
   out_1165030831798606535[175] = 0.0;
   out_1165030831798606535[176] = 0.0;
   out_1165030831798606535[177] = 0.0;
   out_1165030831798606535[178] = 0.0;
   out_1165030831798606535[179] = 0.0;
   out_1165030831798606535[180] = 0.0;
   out_1165030831798606535[181] = 0.0;
   out_1165030831798606535[182] = 0.0;
   out_1165030831798606535[183] = 0.0;
   out_1165030831798606535[184] = 0.0;
   out_1165030831798606535[185] = 0.0;
   out_1165030831798606535[186] = 0.0;
   out_1165030831798606535[187] = 0.0;
   out_1165030831798606535[188] = 0.0;
   out_1165030831798606535[189] = 0.0;
   out_1165030831798606535[190] = 1.0;
   out_1165030831798606535[191] = 0.0;
   out_1165030831798606535[192] = 0.0;
   out_1165030831798606535[193] = 0.0;
   out_1165030831798606535[194] = 0.0;
   out_1165030831798606535[195] = 0.0;
   out_1165030831798606535[196] = 0.0;
   out_1165030831798606535[197] = 0.0;
   out_1165030831798606535[198] = 0.0;
   out_1165030831798606535[199] = 0.0;
   out_1165030831798606535[200] = 0.0;
   out_1165030831798606535[201] = 0.0;
   out_1165030831798606535[202] = 0.0;
   out_1165030831798606535[203] = 0.0;
   out_1165030831798606535[204] = 0.0;
   out_1165030831798606535[205] = 0.0;
   out_1165030831798606535[206] = 0.0;
   out_1165030831798606535[207] = 0.0;
   out_1165030831798606535[208] = 0.0;
   out_1165030831798606535[209] = 1.0;
   out_1165030831798606535[210] = 0.0;
   out_1165030831798606535[211] = 0.0;
   out_1165030831798606535[212] = 0.0;
   out_1165030831798606535[213] = 0.0;
   out_1165030831798606535[214] = 0.0;
   out_1165030831798606535[215] = 0.0;
   out_1165030831798606535[216] = 0.0;
   out_1165030831798606535[217] = 0.0;
   out_1165030831798606535[218] = 0.0;
   out_1165030831798606535[219] = 0.0;
   out_1165030831798606535[220] = 0.0;
   out_1165030831798606535[221] = 0.0;
   out_1165030831798606535[222] = 0.0;
   out_1165030831798606535[223] = 0.0;
   out_1165030831798606535[224] = 0.0;
   out_1165030831798606535[225] = 0.0;
   out_1165030831798606535[226] = 0.0;
   out_1165030831798606535[227] = 0.0;
   out_1165030831798606535[228] = 1.0;
   out_1165030831798606535[229] = 0.0;
   out_1165030831798606535[230] = 0.0;
   out_1165030831798606535[231] = 0.0;
   out_1165030831798606535[232] = 0.0;
   out_1165030831798606535[233] = 0.0;
   out_1165030831798606535[234] = 0.0;
   out_1165030831798606535[235] = 0.0;
   out_1165030831798606535[236] = 0.0;
   out_1165030831798606535[237] = 0.0;
   out_1165030831798606535[238] = 0.0;
   out_1165030831798606535[239] = 0.0;
   out_1165030831798606535[240] = 0.0;
   out_1165030831798606535[241] = 0.0;
   out_1165030831798606535[242] = 0.0;
   out_1165030831798606535[243] = 0.0;
   out_1165030831798606535[244] = 0.0;
   out_1165030831798606535[245] = 0.0;
   out_1165030831798606535[246] = 0.0;
   out_1165030831798606535[247] = 1.0;
   out_1165030831798606535[248] = 0.0;
   out_1165030831798606535[249] = 0.0;
   out_1165030831798606535[250] = 0.0;
   out_1165030831798606535[251] = 0.0;
   out_1165030831798606535[252] = 0.0;
   out_1165030831798606535[253] = 0.0;
   out_1165030831798606535[254] = 0.0;
   out_1165030831798606535[255] = 0.0;
   out_1165030831798606535[256] = 0.0;
   out_1165030831798606535[257] = 0.0;
   out_1165030831798606535[258] = 0.0;
   out_1165030831798606535[259] = 0.0;
   out_1165030831798606535[260] = 0.0;
   out_1165030831798606535[261] = 0.0;
   out_1165030831798606535[262] = 0.0;
   out_1165030831798606535[263] = 0.0;
   out_1165030831798606535[264] = 0.0;
   out_1165030831798606535[265] = 0.0;
   out_1165030831798606535[266] = 1.0;
   out_1165030831798606535[267] = 0.0;
   out_1165030831798606535[268] = 0.0;
   out_1165030831798606535[269] = 0.0;
   out_1165030831798606535[270] = 0.0;
   out_1165030831798606535[271] = 0.0;
   out_1165030831798606535[272] = 0.0;
   out_1165030831798606535[273] = 0.0;
   out_1165030831798606535[274] = 0.0;
   out_1165030831798606535[275] = 0.0;
   out_1165030831798606535[276] = 0.0;
   out_1165030831798606535[277] = 0.0;
   out_1165030831798606535[278] = 0.0;
   out_1165030831798606535[279] = 0.0;
   out_1165030831798606535[280] = 0.0;
   out_1165030831798606535[281] = 0.0;
   out_1165030831798606535[282] = 0.0;
   out_1165030831798606535[283] = 0.0;
   out_1165030831798606535[284] = 0.0;
   out_1165030831798606535[285] = 1.0;
   out_1165030831798606535[286] = 0.0;
   out_1165030831798606535[287] = 0.0;
   out_1165030831798606535[288] = 0.0;
   out_1165030831798606535[289] = 0.0;
   out_1165030831798606535[290] = 0.0;
   out_1165030831798606535[291] = 0.0;
   out_1165030831798606535[292] = 0.0;
   out_1165030831798606535[293] = 0.0;
   out_1165030831798606535[294] = 0.0;
   out_1165030831798606535[295] = 0.0;
   out_1165030831798606535[296] = 0.0;
   out_1165030831798606535[297] = 0.0;
   out_1165030831798606535[298] = 0.0;
   out_1165030831798606535[299] = 0.0;
   out_1165030831798606535[300] = 0.0;
   out_1165030831798606535[301] = 0.0;
   out_1165030831798606535[302] = 0.0;
   out_1165030831798606535[303] = 0.0;
   out_1165030831798606535[304] = 1.0;
   out_1165030831798606535[305] = 0.0;
   out_1165030831798606535[306] = 0.0;
   out_1165030831798606535[307] = 0.0;
   out_1165030831798606535[308] = 0.0;
   out_1165030831798606535[309] = 0.0;
   out_1165030831798606535[310] = 0.0;
   out_1165030831798606535[311] = 0.0;
   out_1165030831798606535[312] = 0.0;
   out_1165030831798606535[313] = 0.0;
   out_1165030831798606535[314] = 0.0;
   out_1165030831798606535[315] = 0.0;
   out_1165030831798606535[316] = 0.0;
   out_1165030831798606535[317] = 0.0;
   out_1165030831798606535[318] = 0.0;
   out_1165030831798606535[319] = 0.0;
   out_1165030831798606535[320] = 0.0;
   out_1165030831798606535[321] = 0.0;
   out_1165030831798606535[322] = 0.0;
   out_1165030831798606535[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1265326236921530365) {
   out_1265326236921530365[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1265326236921530365[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1265326236921530365[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1265326236921530365[3] = dt*state[12] + state[3];
   out_1265326236921530365[4] = dt*state[13] + state[4];
   out_1265326236921530365[5] = dt*state[14] + state[5];
   out_1265326236921530365[6] = state[6];
   out_1265326236921530365[7] = state[7];
   out_1265326236921530365[8] = state[8];
   out_1265326236921530365[9] = state[9];
   out_1265326236921530365[10] = state[10];
   out_1265326236921530365[11] = state[11];
   out_1265326236921530365[12] = state[12];
   out_1265326236921530365[13] = state[13];
   out_1265326236921530365[14] = state[14];
   out_1265326236921530365[15] = state[15];
   out_1265326236921530365[16] = state[16];
   out_1265326236921530365[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6710846957464201168) {
   out_6710846957464201168[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6710846957464201168[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6710846957464201168[2] = 0;
   out_6710846957464201168[3] = 0;
   out_6710846957464201168[4] = 0;
   out_6710846957464201168[5] = 0;
   out_6710846957464201168[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6710846957464201168[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6710846957464201168[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6710846957464201168[9] = 0;
   out_6710846957464201168[10] = 0;
   out_6710846957464201168[11] = 0;
   out_6710846957464201168[12] = 0;
   out_6710846957464201168[13] = 0;
   out_6710846957464201168[14] = 0;
   out_6710846957464201168[15] = 0;
   out_6710846957464201168[16] = 0;
   out_6710846957464201168[17] = 0;
   out_6710846957464201168[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6710846957464201168[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6710846957464201168[20] = 0;
   out_6710846957464201168[21] = 0;
   out_6710846957464201168[22] = 0;
   out_6710846957464201168[23] = 0;
   out_6710846957464201168[24] = 0;
   out_6710846957464201168[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6710846957464201168[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6710846957464201168[27] = 0;
   out_6710846957464201168[28] = 0;
   out_6710846957464201168[29] = 0;
   out_6710846957464201168[30] = 0;
   out_6710846957464201168[31] = 0;
   out_6710846957464201168[32] = 0;
   out_6710846957464201168[33] = 0;
   out_6710846957464201168[34] = 0;
   out_6710846957464201168[35] = 0;
   out_6710846957464201168[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6710846957464201168[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6710846957464201168[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6710846957464201168[39] = 0;
   out_6710846957464201168[40] = 0;
   out_6710846957464201168[41] = 0;
   out_6710846957464201168[42] = 0;
   out_6710846957464201168[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6710846957464201168[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6710846957464201168[45] = 0;
   out_6710846957464201168[46] = 0;
   out_6710846957464201168[47] = 0;
   out_6710846957464201168[48] = 0;
   out_6710846957464201168[49] = 0;
   out_6710846957464201168[50] = 0;
   out_6710846957464201168[51] = 0;
   out_6710846957464201168[52] = 0;
   out_6710846957464201168[53] = 0;
   out_6710846957464201168[54] = 0;
   out_6710846957464201168[55] = 0;
   out_6710846957464201168[56] = 0;
   out_6710846957464201168[57] = 1;
   out_6710846957464201168[58] = 0;
   out_6710846957464201168[59] = 0;
   out_6710846957464201168[60] = 0;
   out_6710846957464201168[61] = 0;
   out_6710846957464201168[62] = 0;
   out_6710846957464201168[63] = 0;
   out_6710846957464201168[64] = 0;
   out_6710846957464201168[65] = 0;
   out_6710846957464201168[66] = dt;
   out_6710846957464201168[67] = 0;
   out_6710846957464201168[68] = 0;
   out_6710846957464201168[69] = 0;
   out_6710846957464201168[70] = 0;
   out_6710846957464201168[71] = 0;
   out_6710846957464201168[72] = 0;
   out_6710846957464201168[73] = 0;
   out_6710846957464201168[74] = 0;
   out_6710846957464201168[75] = 0;
   out_6710846957464201168[76] = 1;
   out_6710846957464201168[77] = 0;
   out_6710846957464201168[78] = 0;
   out_6710846957464201168[79] = 0;
   out_6710846957464201168[80] = 0;
   out_6710846957464201168[81] = 0;
   out_6710846957464201168[82] = 0;
   out_6710846957464201168[83] = 0;
   out_6710846957464201168[84] = 0;
   out_6710846957464201168[85] = dt;
   out_6710846957464201168[86] = 0;
   out_6710846957464201168[87] = 0;
   out_6710846957464201168[88] = 0;
   out_6710846957464201168[89] = 0;
   out_6710846957464201168[90] = 0;
   out_6710846957464201168[91] = 0;
   out_6710846957464201168[92] = 0;
   out_6710846957464201168[93] = 0;
   out_6710846957464201168[94] = 0;
   out_6710846957464201168[95] = 1;
   out_6710846957464201168[96] = 0;
   out_6710846957464201168[97] = 0;
   out_6710846957464201168[98] = 0;
   out_6710846957464201168[99] = 0;
   out_6710846957464201168[100] = 0;
   out_6710846957464201168[101] = 0;
   out_6710846957464201168[102] = 0;
   out_6710846957464201168[103] = 0;
   out_6710846957464201168[104] = dt;
   out_6710846957464201168[105] = 0;
   out_6710846957464201168[106] = 0;
   out_6710846957464201168[107] = 0;
   out_6710846957464201168[108] = 0;
   out_6710846957464201168[109] = 0;
   out_6710846957464201168[110] = 0;
   out_6710846957464201168[111] = 0;
   out_6710846957464201168[112] = 0;
   out_6710846957464201168[113] = 0;
   out_6710846957464201168[114] = 1;
   out_6710846957464201168[115] = 0;
   out_6710846957464201168[116] = 0;
   out_6710846957464201168[117] = 0;
   out_6710846957464201168[118] = 0;
   out_6710846957464201168[119] = 0;
   out_6710846957464201168[120] = 0;
   out_6710846957464201168[121] = 0;
   out_6710846957464201168[122] = 0;
   out_6710846957464201168[123] = 0;
   out_6710846957464201168[124] = 0;
   out_6710846957464201168[125] = 0;
   out_6710846957464201168[126] = 0;
   out_6710846957464201168[127] = 0;
   out_6710846957464201168[128] = 0;
   out_6710846957464201168[129] = 0;
   out_6710846957464201168[130] = 0;
   out_6710846957464201168[131] = 0;
   out_6710846957464201168[132] = 0;
   out_6710846957464201168[133] = 1;
   out_6710846957464201168[134] = 0;
   out_6710846957464201168[135] = 0;
   out_6710846957464201168[136] = 0;
   out_6710846957464201168[137] = 0;
   out_6710846957464201168[138] = 0;
   out_6710846957464201168[139] = 0;
   out_6710846957464201168[140] = 0;
   out_6710846957464201168[141] = 0;
   out_6710846957464201168[142] = 0;
   out_6710846957464201168[143] = 0;
   out_6710846957464201168[144] = 0;
   out_6710846957464201168[145] = 0;
   out_6710846957464201168[146] = 0;
   out_6710846957464201168[147] = 0;
   out_6710846957464201168[148] = 0;
   out_6710846957464201168[149] = 0;
   out_6710846957464201168[150] = 0;
   out_6710846957464201168[151] = 0;
   out_6710846957464201168[152] = 1;
   out_6710846957464201168[153] = 0;
   out_6710846957464201168[154] = 0;
   out_6710846957464201168[155] = 0;
   out_6710846957464201168[156] = 0;
   out_6710846957464201168[157] = 0;
   out_6710846957464201168[158] = 0;
   out_6710846957464201168[159] = 0;
   out_6710846957464201168[160] = 0;
   out_6710846957464201168[161] = 0;
   out_6710846957464201168[162] = 0;
   out_6710846957464201168[163] = 0;
   out_6710846957464201168[164] = 0;
   out_6710846957464201168[165] = 0;
   out_6710846957464201168[166] = 0;
   out_6710846957464201168[167] = 0;
   out_6710846957464201168[168] = 0;
   out_6710846957464201168[169] = 0;
   out_6710846957464201168[170] = 0;
   out_6710846957464201168[171] = 1;
   out_6710846957464201168[172] = 0;
   out_6710846957464201168[173] = 0;
   out_6710846957464201168[174] = 0;
   out_6710846957464201168[175] = 0;
   out_6710846957464201168[176] = 0;
   out_6710846957464201168[177] = 0;
   out_6710846957464201168[178] = 0;
   out_6710846957464201168[179] = 0;
   out_6710846957464201168[180] = 0;
   out_6710846957464201168[181] = 0;
   out_6710846957464201168[182] = 0;
   out_6710846957464201168[183] = 0;
   out_6710846957464201168[184] = 0;
   out_6710846957464201168[185] = 0;
   out_6710846957464201168[186] = 0;
   out_6710846957464201168[187] = 0;
   out_6710846957464201168[188] = 0;
   out_6710846957464201168[189] = 0;
   out_6710846957464201168[190] = 1;
   out_6710846957464201168[191] = 0;
   out_6710846957464201168[192] = 0;
   out_6710846957464201168[193] = 0;
   out_6710846957464201168[194] = 0;
   out_6710846957464201168[195] = 0;
   out_6710846957464201168[196] = 0;
   out_6710846957464201168[197] = 0;
   out_6710846957464201168[198] = 0;
   out_6710846957464201168[199] = 0;
   out_6710846957464201168[200] = 0;
   out_6710846957464201168[201] = 0;
   out_6710846957464201168[202] = 0;
   out_6710846957464201168[203] = 0;
   out_6710846957464201168[204] = 0;
   out_6710846957464201168[205] = 0;
   out_6710846957464201168[206] = 0;
   out_6710846957464201168[207] = 0;
   out_6710846957464201168[208] = 0;
   out_6710846957464201168[209] = 1;
   out_6710846957464201168[210] = 0;
   out_6710846957464201168[211] = 0;
   out_6710846957464201168[212] = 0;
   out_6710846957464201168[213] = 0;
   out_6710846957464201168[214] = 0;
   out_6710846957464201168[215] = 0;
   out_6710846957464201168[216] = 0;
   out_6710846957464201168[217] = 0;
   out_6710846957464201168[218] = 0;
   out_6710846957464201168[219] = 0;
   out_6710846957464201168[220] = 0;
   out_6710846957464201168[221] = 0;
   out_6710846957464201168[222] = 0;
   out_6710846957464201168[223] = 0;
   out_6710846957464201168[224] = 0;
   out_6710846957464201168[225] = 0;
   out_6710846957464201168[226] = 0;
   out_6710846957464201168[227] = 0;
   out_6710846957464201168[228] = 1;
   out_6710846957464201168[229] = 0;
   out_6710846957464201168[230] = 0;
   out_6710846957464201168[231] = 0;
   out_6710846957464201168[232] = 0;
   out_6710846957464201168[233] = 0;
   out_6710846957464201168[234] = 0;
   out_6710846957464201168[235] = 0;
   out_6710846957464201168[236] = 0;
   out_6710846957464201168[237] = 0;
   out_6710846957464201168[238] = 0;
   out_6710846957464201168[239] = 0;
   out_6710846957464201168[240] = 0;
   out_6710846957464201168[241] = 0;
   out_6710846957464201168[242] = 0;
   out_6710846957464201168[243] = 0;
   out_6710846957464201168[244] = 0;
   out_6710846957464201168[245] = 0;
   out_6710846957464201168[246] = 0;
   out_6710846957464201168[247] = 1;
   out_6710846957464201168[248] = 0;
   out_6710846957464201168[249] = 0;
   out_6710846957464201168[250] = 0;
   out_6710846957464201168[251] = 0;
   out_6710846957464201168[252] = 0;
   out_6710846957464201168[253] = 0;
   out_6710846957464201168[254] = 0;
   out_6710846957464201168[255] = 0;
   out_6710846957464201168[256] = 0;
   out_6710846957464201168[257] = 0;
   out_6710846957464201168[258] = 0;
   out_6710846957464201168[259] = 0;
   out_6710846957464201168[260] = 0;
   out_6710846957464201168[261] = 0;
   out_6710846957464201168[262] = 0;
   out_6710846957464201168[263] = 0;
   out_6710846957464201168[264] = 0;
   out_6710846957464201168[265] = 0;
   out_6710846957464201168[266] = 1;
   out_6710846957464201168[267] = 0;
   out_6710846957464201168[268] = 0;
   out_6710846957464201168[269] = 0;
   out_6710846957464201168[270] = 0;
   out_6710846957464201168[271] = 0;
   out_6710846957464201168[272] = 0;
   out_6710846957464201168[273] = 0;
   out_6710846957464201168[274] = 0;
   out_6710846957464201168[275] = 0;
   out_6710846957464201168[276] = 0;
   out_6710846957464201168[277] = 0;
   out_6710846957464201168[278] = 0;
   out_6710846957464201168[279] = 0;
   out_6710846957464201168[280] = 0;
   out_6710846957464201168[281] = 0;
   out_6710846957464201168[282] = 0;
   out_6710846957464201168[283] = 0;
   out_6710846957464201168[284] = 0;
   out_6710846957464201168[285] = 1;
   out_6710846957464201168[286] = 0;
   out_6710846957464201168[287] = 0;
   out_6710846957464201168[288] = 0;
   out_6710846957464201168[289] = 0;
   out_6710846957464201168[290] = 0;
   out_6710846957464201168[291] = 0;
   out_6710846957464201168[292] = 0;
   out_6710846957464201168[293] = 0;
   out_6710846957464201168[294] = 0;
   out_6710846957464201168[295] = 0;
   out_6710846957464201168[296] = 0;
   out_6710846957464201168[297] = 0;
   out_6710846957464201168[298] = 0;
   out_6710846957464201168[299] = 0;
   out_6710846957464201168[300] = 0;
   out_6710846957464201168[301] = 0;
   out_6710846957464201168[302] = 0;
   out_6710846957464201168[303] = 0;
   out_6710846957464201168[304] = 1;
   out_6710846957464201168[305] = 0;
   out_6710846957464201168[306] = 0;
   out_6710846957464201168[307] = 0;
   out_6710846957464201168[308] = 0;
   out_6710846957464201168[309] = 0;
   out_6710846957464201168[310] = 0;
   out_6710846957464201168[311] = 0;
   out_6710846957464201168[312] = 0;
   out_6710846957464201168[313] = 0;
   out_6710846957464201168[314] = 0;
   out_6710846957464201168[315] = 0;
   out_6710846957464201168[316] = 0;
   out_6710846957464201168[317] = 0;
   out_6710846957464201168[318] = 0;
   out_6710846957464201168[319] = 0;
   out_6710846957464201168[320] = 0;
   out_6710846957464201168[321] = 0;
   out_6710846957464201168[322] = 0;
   out_6710846957464201168[323] = 1;
}
void h_4(double *state, double *unused, double *out_5156063247866132134) {
   out_5156063247866132134[0] = state[6] + state[9];
   out_5156063247866132134[1] = state[7] + state[10];
   out_5156063247866132134[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_4936838041642514107) {
   out_4936838041642514107[0] = 0;
   out_4936838041642514107[1] = 0;
   out_4936838041642514107[2] = 0;
   out_4936838041642514107[3] = 0;
   out_4936838041642514107[4] = 0;
   out_4936838041642514107[5] = 0;
   out_4936838041642514107[6] = 1;
   out_4936838041642514107[7] = 0;
   out_4936838041642514107[8] = 0;
   out_4936838041642514107[9] = 1;
   out_4936838041642514107[10] = 0;
   out_4936838041642514107[11] = 0;
   out_4936838041642514107[12] = 0;
   out_4936838041642514107[13] = 0;
   out_4936838041642514107[14] = 0;
   out_4936838041642514107[15] = 0;
   out_4936838041642514107[16] = 0;
   out_4936838041642514107[17] = 0;
   out_4936838041642514107[18] = 0;
   out_4936838041642514107[19] = 0;
   out_4936838041642514107[20] = 0;
   out_4936838041642514107[21] = 0;
   out_4936838041642514107[22] = 0;
   out_4936838041642514107[23] = 0;
   out_4936838041642514107[24] = 0;
   out_4936838041642514107[25] = 1;
   out_4936838041642514107[26] = 0;
   out_4936838041642514107[27] = 0;
   out_4936838041642514107[28] = 1;
   out_4936838041642514107[29] = 0;
   out_4936838041642514107[30] = 0;
   out_4936838041642514107[31] = 0;
   out_4936838041642514107[32] = 0;
   out_4936838041642514107[33] = 0;
   out_4936838041642514107[34] = 0;
   out_4936838041642514107[35] = 0;
   out_4936838041642514107[36] = 0;
   out_4936838041642514107[37] = 0;
   out_4936838041642514107[38] = 0;
   out_4936838041642514107[39] = 0;
   out_4936838041642514107[40] = 0;
   out_4936838041642514107[41] = 0;
   out_4936838041642514107[42] = 0;
   out_4936838041642514107[43] = 0;
   out_4936838041642514107[44] = 1;
   out_4936838041642514107[45] = 0;
   out_4936838041642514107[46] = 0;
   out_4936838041642514107[47] = 1;
   out_4936838041642514107[48] = 0;
   out_4936838041642514107[49] = 0;
   out_4936838041642514107[50] = 0;
   out_4936838041642514107[51] = 0;
   out_4936838041642514107[52] = 0;
   out_4936838041642514107[53] = 0;
}
void h_10(double *state, double *unused, double *out_398640295991963377) {
   out_398640295991963377[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_398640295991963377[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_398640295991963377[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3538087001133252611) {
   out_3538087001133252611[0] = 0;
   out_3538087001133252611[1] = 9.8100000000000005*cos(state[1]);
   out_3538087001133252611[2] = 0;
   out_3538087001133252611[3] = 0;
   out_3538087001133252611[4] = -state[8];
   out_3538087001133252611[5] = state[7];
   out_3538087001133252611[6] = 0;
   out_3538087001133252611[7] = state[5];
   out_3538087001133252611[8] = -state[4];
   out_3538087001133252611[9] = 0;
   out_3538087001133252611[10] = 0;
   out_3538087001133252611[11] = 0;
   out_3538087001133252611[12] = 1;
   out_3538087001133252611[13] = 0;
   out_3538087001133252611[14] = 0;
   out_3538087001133252611[15] = 1;
   out_3538087001133252611[16] = 0;
   out_3538087001133252611[17] = 0;
   out_3538087001133252611[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3538087001133252611[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3538087001133252611[20] = 0;
   out_3538087001133252611[21] = state[8];
   out_3538087001133252611[22] = 0;
   out_3538087001133252611[23] = -state[6];
   out_3538087001133252611[24] = -state[5];
   out_3538087001133252611[25] = 0;
   out_3538087001133252611[26] = state[3];
   out_3538087001133252611[27] = 0;
   out_3538087001133252611[28] = 0;
   out_3538087001133252611[29] = 0;
   out_3538087001133252611[30] = 0;
   out_3538087001133252611[31] = 1;
   out_3538087001133252611[32] = 0;
   out_3538087001133252611[33] = 0;
   out_3538087001133252611[34] = 1;
   out_3538087001133252611[35] = 0;
   out_3538087001133252611[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3538087001133252611[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3538087001133252611[38] = 0;
   out_3538087001133252611[39] = -state[7];
   out_3538087001133252611[40] = state[6];
   out_3538087001133252611[41] = 0;
   out_3538087001133252611[42] = state[4];
   out_3538087001133252611[43] = -state[3];
   out_3538087001133252611[44] = 0;
   out_3538087001133252611[45] = 0;
   out_3538087001133252611[46] = 0;
   out_3538087001133252611[47] = 0;
   out_3538087001133252611[48] = 0;
   out_3538087001133252611[49] = 0;
   out_3538087001133252611[50] = 1;
   out_3538087001133252611[51] = 0;
   out_3538087001133252611[52] = 0;
   out_3538087001133252611[53] = 1;
}
void h_13(double *state, double *unused, double *out_4028504955015290804) {
   out_4028504955015290804[0] = state[3];
   out_4028504955015290804[1] = state[4];
   out_4028504955015290804[2] = state[5];
}
void H_13(double *state, double *unused, double *out_1103082578339990083) {
   out_1103082578339990083[0] = 0;
   out_1103082578339990083[1] = 0;
   out_1103082578339990083[2] = 0;
   out_1103082578339990083[3] = 1;
   out_1103082578339990083[4] = 0;
   out_1103082578339990083[5] = 0;
   out_1103082578339990083[6] = 0;
   out_1103082578339990083[7] = 0;
   out_1103082578339990083[8] = 0;
   out_1103082578339990083[9] = 0;
   out_1103082578339990083[10] = 0;
   out_1103082578339990083[11] = 0;
   out_1103082578339990083[12] = 0;
   out_1103082578339990083[13] = 0;
   out_1103082578339990083[14] = 0;
   out_1103082578339990083[15] = 0;
   out_1103082578339990083[16] = 0;
   out_1103082578339990083[17] = 0;
   out_1103082578339990083[18] = 0;
   out_1103082578339990083[19] = 0;
   out_1103082578339990083[20] = 0;
   out_1103082578339990083[21] = 0;
   out_1103082578339990083[22] = 1;
   out_1103082578339990083[23] = 0;
   out_1103082578339990083[24] = 0;
   out_1103082578339990083[25] = 0;
   out_1103082578339990083[26] = 0;
   out_1103082578339990083[27] = 0;
   out_1103082578339990083[28] = 0;
   out_1103082578339990083[29] = 0;
   out_1103082578339990083[30] = 0;
   out_1103082578339990083[31] = 0;
   out_1103082578339990083[32] = 0;
   out_1103082578339990083[33] = 0;
   out_1103082578339990083[34] = 0;
   out_1103082578339990083[35] = 0;
   out_1103082578339990083[36] = 0;
   out_1103082578339990083[37] = 0;
   out_1103082578339990083[38] = 0;
   out_1103082578339990083[39] = 0;
   out_1103082578339990083[40] = 0;
   out_1103082578339990083[41] = 1;
   out_1103082578339990083[42] = 0;
   out_1103082578339990083[43] = 0;
   out_1103082578339990083[44] = 0;
   out_1103082578339990083[45] = 0;
   out_1103082578339990083[46] = 0;
   out_1103082578339990083[47] = 0;
   out_1103082578339990083[48] = 0;
   out_1103082578339990083[49] = 0;
   out_1103082578339990083[50] = 0;
   out_1103082578339990083[51] = 0;
   out_1103082578339990083[52] = 0;
   out_1103082578339990083[53] = 0;
}
void h_14(double *state, double *unused, double *out_4693736396154390042) {
   out_4693736396154390042[0] = state[6];
   out_4693736396154390042[1] = state[7];
   out_4693736396154390042[2] = state[8];
}
void H_14(double *state, double *unused, double *out_1854049609347141811) {
   out_1854049609347141811[0] = 0;
   out_1854049609347141811[1] = 0;
   out_1854049609347141811[2] = 0;
   out_1854049609347141811[3] = 0;
   out_1854049609347141811[4] = 0;
   out_1854049609347141811[5] = 0;
   out_1854049609347141811[6] = 1;
   out_1854049609347141811[7] = 0;
   out_1854049609347141811[8] = 0;
   out_1854049609347141811[9] = 0;
   out_1854049609347141811[10] = 0;
   out_1854049609347141811[11] = 0;
   out_1854049609347141811[12] = 0;
   out_1854049609347141811[13] = 0;
   out_1854049609347141811[14] = 0;
   out_1854049609347141811[15] = 0;
   out_1854049609347141811[16] = 0;
   out_1854049609347141811[17] = 0;
   out_1854049609347141811[18] = 0;
   out_1854049609347141811[19] = 0;
   out_1854049609347141811[20] = 0;
   out_1854049609347141811[21] = 0;
   out_1854049609347141811[22] = 0;
   out_1854049609347141811[23] = 0;
   out_1854049609347141811[24] = 0;
   out_1854049609347141811[25] = 1;
   out_1854049609347141811[26] = 0;
   out_1854049609347141811[27] = 0;
   out_1854049609347141811[28] = 0;
   out_1854049609347141811[29] = 0;
   out_1854049609347141811[30] = 0;
   out_1854049609347141811[31] = 0;
   out_1854049609347141811[32] = 0;
   out_1854049609347141811[33] = 0;
   out_1854049609347141811[34] = 0;
   out_1854049609347141811[35] = 0;
   out_1854049609347141811[36] = 0;
   out_1854049609347141811[37] = 0;
   out_1854049609347141811[38] = 0;
   out_1854049609347141811[39] = 0;
   out_1854049609347141811[40] = 0;
   out_1854049609347141811[41] = 0;
   out_1854049609347141811[42] = 0;
   out_1854049609347141811[43] = 0;
   out_1854049609347141811[44] = 1;
   out_1854049609347141811[45] = 0;
   out_1854049609347141811[46] = 0;
   out_1854049609347141811[47] = 0;
   out_1854049609347141811[48] = 0;
   out_1854049609347141811[49] = 0;
   out_1854049609347141811[50] = 0;
   out_1854049609347141811[51] = 0;
   out_1854049609347141811[52] = 0;
   out_1854049609347141811[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_8557638590374089642) {
  err_fun(nom_x, delta_x, out_8557638590374089642);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2952464144299303284) {
  inv_err_fun(nom_x, true_x, out_2952464144299303284);
}
void pose_H_mod_fun(double *state, double *out_1165030831798606535) {
  H_mod_fun(state, out_1165030831798606535);
}
void pose_f_fun(double *state, double dt, double *out_1265326236921530365) {
  f_fun(state,  dt, out_1265326236921530365);
}
void pose_F_fun(double *state, double dt, double *out_6710846957464201168) {
  F_fun(state,  dt, out_6710846957464201168);
}
void pose_h_4(double *state, double *unused, double *out_5156063247866132134) {
  h_4(state, unused, out_5156063247866132134);
}
void pose_H_4(double *state, double *unused, double *out_4936838041642514107) {
  H_4(state, unused, out_4936838041642514107);
}
void pose_h_10(double *state, double *unused, double *out_398640295991963377) {
  h_10(state, unused, out_398640295991963377);
}
void pose_H_10(double *state, double *unused, double *out_3538087001133252611) {
  H_10(state, unused, out_3538087001133252611);
}
void pose_h_13(double *state, double *unused, double *out_4028504955015290804) {
  h_13(state, unused, out_4028504955015290804);
}
void pose_H_13(double *state, double *unused, double *out_1103082578339990083) {
  H_13(state, unused, out_1103082578339990083);
}
void pose_h_14(double *state, double *unused, double *out_4693736396154390042) {
  h_14(state, unused, out_4693736396154390042);
}
void pose_H_14(double *state, double *unused, double *out_1854049609347141811) {
  H_14(state, unused, out_1854049609347141811);
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
