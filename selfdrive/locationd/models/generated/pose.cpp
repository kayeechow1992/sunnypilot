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
void err_fun(double *nom_x, double *delta_x, double *out_6316962376878717030) {
   out_6316962376878717030[0] = delta_x[0] + nom_x[0];
   out_6316962376878717030[1] = delta_x[1] + nom_x[1];
   out_6316962376878717030[2] = delta_x[2] + nom_x[2];
   out_6316962376878717030[3] = delta_x[3] + nom_x[3];
   out_6316962376878717030[4] = delta_x[4] + nom_x[4];
   out_6316962376878717030[5] = delta_x[5] + nom_x[5];
   out_6316962376878717030[6] = delta_x[6] + nom_x[6];
   out_6316962376878717030[7] = delta_x[7] + nom_x[7];
   out_6316962376878717030[8] = delta_x[8] + nom_x[8];
   out_6316962376878717030[9] = delta_x[9] + nom_x[9];
   out_6316962376878717030[10] = delta_x[10] + nom_x[10];
   out_6316962376878717030[11] = delta_x[11] + nom_x[11];
   out_6316962376878717030[12] = delta_x[12] + nom_x[12];
   out_6316962376878717030[13] = delta_x[13] + nom_x[13];
   out_6316962376878717030[14] = delta_x[14] + nom_x[14];
   out_6316962376878717030[15] = delta_x[15] + nom_x[15];
   out_6316962376878717030[16] = delta_x[16] + nom_x[16];
   out_6316962376878717030[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2486354416350675894) {
   out_2486354416350675894[0] = -nom_x[0] + true_x[0];
   out_2486354416350675894[1] = -nom_x[1] + true_x[1];
   out_2486354416350675894[2] = -nom_x[2] + true_x[2];
   out_2486354416350675894[3] = -nom_x[3] + true_x[3];
   out_2486354416350675894[4] = -nom_x[4] + true_x[4];
   out_2486354416350675894[5] = -nom_x[5] + true_x[5];
   out_2486354416350675894[6] = -nom_x[6] + true_x[6];
   out_2486354416350675894[7] = -nom_x[7] + true_x[7];
   out_2486354416350675894[8] = -nom_x[8] + true_x[8];
   out_2486354416350675894[9] = -nom_x[9] + true_x[9];
   out_2486354416350675894[10] = -nom_x[10] + true_x[10];
   out_2486354416350675894[11] = -nom_x[11] + true_x[11];
   out_2486354416350675894[12] = -nom_x[12] + true_x[12];
   out_2486354416350675894[13] = -nom_x[13] + true_x[13];
   out_2486354416350675894[14] = -nom_x[14] + true_x[14];
   out_2486354416350675894[15] = -nom_x[15] + true_x[15];
   out_2486354416350675894[16] = -nom_x[16] + true_x[16];
   out_2486354416350675894[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_5713404591469320854) {
   out_5713404591469320854[0] = 1.0;
   out_5713404591469320854[1] = 0.0;
   out_5713404591469320854[2] = 0.0;
   out_5713404591469320854[3] = 0.0;
   out_5713404591469320854[4] = 0.0;
   out_5713404591469320854[5] = 0.0;
   out_5713404591469320854[6] = 0.0;
   out_5713404591469320854[7] = 0.0;
   out_5713404591469320854[8] = 0.0;
   out_5713404591469320854[9] = 0.0;
   out_5713404591469320854[10] = 0.0;
   out_5713404591469320854[11] = 0.0;
   out_5713404591469320854[12] = 0.0;
   out_5713404591469320854[13] = 0.0;
   out_5713404591469320854[14] = 0.0;
   out_5713404591469320854[15] = 0.0;
   out_5713404591469320854[16] = 0.0;
   out_5713404591469320854[17] = 0.0;
   out_5713404591469320854[18] = 0.0;
   out_5713404591469320854[19] = 1.0;
   out_5713404591469320854[20] = 0.0;
   out_5713404591469320854[21] = 0.0;
   out_5713404591469320854[22] = 0.0;
   out_5713404591469320854[23] = 0.0;
   out_5713404591469320854[24] = 0.0;
   out_5713404591469320854[25] = 0.0;
   out_5713404591469320854[26] = 0.0;
   out_5713404591469320854[27] = 0.0;
   out_5713404591469320854[28] = 0.0;
   out_5713404591469320854[29] = 0.0;
   out_5713404591469320854[30] = 0.0;
   out_5713404591469320854[31] = 0.0;
   out_5713404591469320854[32] = 0.0;
   out_5713404591469320854[33] = 0.0;
   out_5713404591469320854[34] = 0.0;
   out_5713404591469320854[35] = 0.0;
   out_5713404591469320854[36] = 0.0;
   out_5713404591469320854[37] = 0.0;
   out_5713404591469320854[38] = 1.0;
   out_5713404591469320854[39] = 0.0;
   out_5713404591469320854[40] = 0.0;
   out_5713404591469320854[41] = 0.0;
   out_5713404591469320854[42] = 0.0;
   out_5713404591469320854[43] = 0.0;
   out_5713404591469320854[44] = 0.0;
   out_5713404591469320854[45] = 0.0;
   out_5713404591469320854[46] = 0.0;
   out_5713404591469320854[47] = 0.0;
   out_5713404591469320854[48] = 0.0;
   out_5713404591469320854[49] = 0.0;
   out_5713404591469320854[50] = 0.0;
   out_5713404591469320854[51] = 0.0;
   out_5713404591469320854[52] = 0.0;
   out_5713404591469320854[53] = 0.0;
   out_5713404591469320854[54] = 0.0;
   out_5713404591469320854[55] = 0.0;
   out_5713404591469320854[56] = 0.0;
   out_5713404591469320854[57] = 1.0;
   out_5713404591469320854[58] = 0.0;
   out_5713404591469320854[59] = 0.0;
   out_5713404591469320854[60] = 0.0;
   out_5713404591469320854[61] = 0.0;
   out_5713404591469320854[62] = 0.0;
   out_5713404591469320854[63] = 0.0;
   out_5713404591469320854[64] = 0.0;
   out_5713404591469320854[65] = 0.0;
   out_5713404591469320854[66] = 0.0;
   out_5713404591469320854[67] = 0.0;
   out_5713404591469320854[68] = 0.0;
   out_5713404591469320854[69] = 0.0;
   out_5713404591469320854[70] = 0.0;
   out_5713404591469320854[71] = 0.0;
   out_5713404591469320854[72] = 0.0;
   out_5713404591469320854[73] = 0.0;
   out_5713404591469320854[74] = 0.0;
   out_5713404591469320854[75] = 0.0;
   out_5713404591469320854[76] = 1.0;
   out_5713404591469320854[77] = 0.0;
   out_5713404591469320854[78] = 0.0;
   out_5713404591469320854[79] = 0.0;
   out_5713404591469320854[80] = 0.0;
   out_5713404591469320854[81] = 0.0;
   out_5713404591469320854[82] = 0.0;
   out_5713404591469320854[83] = 0.0;
   out_5713404591469320854[84] = 0.0;
   out_5713404591469320854[85] = 0.0;
   out_5713404591469320854[86] = 0.0;
   out_5713404591469320854[87] = 0.0;
   out_5713404591469320854[88] = 0.0;
   out_5713404591469320854[89] = 0.0;
   out_5713404591469320854[90] = 0.0;
   out_5713404591469320854[91] = 0.0;
   out_5713404591469320854[92] = 0.0;
   out_5713404591469320854[93] = 0.0;
   out_5713404591469320854[94] = 0.0;
   out_5713404591469320854[95] = 1.0;
   out_5713404591469320854[96] = 0.0;
   out_5713404591469320854[97] = 0.0;
   out_5713404591469320854[98] = 0.0;
   out_5713404591469320854[99] = 0.0;
   out_5713404591469320854[100] = 0.0;
   out_5713404591469320854[101] = 0.0;
   out_5713404591469320854[102] = 0.0;
   out_5713404591469320854[103] = 0.0;
   out_5713404591469320854[104] = 0.0;
   out_5713404591469320854[105] = 0.0;
   out_5713404591469320854[106] = 0.0;
   out_5713404591469320854[107] = 0.0;
   out_5713404591469320854[108] = 0.0;
   out_5713404591469320854[109] = 0.0;
   out_5713404591469320854[110] = 0.0;
   out_5713404591469320854[111] = 0.0;
   out_5713404591469320854[112] = 0.0;
   out_5713404591469320854[113] = 0.0;
   out_5713404591469320854[114] = 1.0;
   out_5713404591469320854[115] = 0.0;
   out_5713404591469320854[116] = 0.0;
   out_5713404591469320854[117] = 0.0;
   out_5713404591469320854[118] = 0.0;
   out_5713404591469320854[119] = 0.0;
   out_5713404591469320854[120] = 0.0;
   out_5713404591469320854[121] = 0.0;
   out_5713404591469320854[122] = 0.0;
   out_5713404591469320854[123] = 0.0;
   out_5713404591469320854[124] = 0.0;
   out_5713404591469320854[125] = 0.0;
   out_5713404591469320854[126] = 0.0;
   out_5713404591469320854[127] = 0.0;
   out_5713404591469320854[128] = 0.0;
   out_5713404591469320854[129] = 0.0;
   out_5713404591469320854[130] = 0.0;
   out_5713404591469320854[131] = 0.0;
   out_5713404591469320854[132] = 0.0;
   out_5713404591469320854[133] = 1.0;
   out_5713404591469320854[134] = 0.0;
   out_5713404591469320854[135] = 0.0;
   out_5713404591469320854[136] = 0.0;
   out_5713404591469320854[137] = 0.0;
   out_5713404591469320854[138] = 0.0;
   out_5713404591469320854[139] = 0.0;
   out_5713404591469320854[140] = 0.0;
   out_5713404591469320854[141] = 0.0;
   out_5713404591469320854[142] = 0.0;
   out_5713404591469320854[143] = 0.0;
   out_5713404591469320854[144] = 0.0;
   out_5713404591469320854[145] = 0.0;
   out_5713404591469320854[146] = 0.0;
   out_5713404591469320854[147] = 0.0;
   out_5713404591469320854[148] = 0.0;
   out_5713404591469320854[149] = 0.0;
   out_5713404591469320854[150] = 0.0;
   out_5713404591469320854[151] = 0.0;
   out_5713404591469320854[152] = 1.0;
   out_5713404591469320854[153] = 0.0;
   out_5713404591469320854[154] = 0.0;
   out_5713404591469320854[155] = 0.0;
   out_5713404591469320854[156] = 0.0;
   out_5713404591469320854[157] = 0.0;
   out_5713404591469320854[158] = 0.0;
   out_5713404591469320854[159] = 0.0;
   out_5713404591469320854[160] = 0.0;
   out_5713404591469320854[161] = 0.0;
   out_5713404591469320854[162] = 0.0;
   out_5713404591469320854[163] = 0.0;
   out_5713404591469320854[164] = 0.0;
   out_5713404591469320854[165] = 0.0;
   out_5713404591469320854[166] = 0.0;
   out_5713404591469320854[167] = 0.0;
   out_5713404591469320854[168] = 0.0;
   out_5713404591469320854[169] = 0.0;
   out_5713404591469320854[170] = 0.0;
   out_5713404591469320854[171] = 1.0;
   out_5713404591469320854[172] = 0.0;
   out_5713404591469320854[173] = 0.0;
   out_5713404591469320854[174] = 0.0;
   out_5713404591469320854[175] = 0.0;
   out_5713404591469320854[176] = 0.0;
   out_5713404591469320854[177] = 0.0;
   out_5713404591469320854[178] = 0.0;
   out_5713404591469320854[179] = 0.0;
   out_5713404591469320854[180] = 0.0;
   out_5713404591469320854[181] = 0.0;
   out_5713404591469320854[182] = 0.0;
   out_5713404591469320854[183] = 0.0;
   out_5713404591469320854[184] = 0.0;
   out_5713404591469320854[185] = 0.0;
   out_5713404591469320854[186] = 0.0;
   out_5713404591469320854[187] = 0.0;
   out_5713404591469320854[188] = 0.0;
   out_5713404591469320854[189] = 0.0;
   out_5713404591469320854[190] = 1.0;
   out_5713404591469320854[191] = 0.0;
   out_5713404591469320854[192] = 0.0;
   out_5713404591469320854[193] = 0.0;
   out_5713404591469320854[194] = 0.0;
   out_5713404591469320854[195] = 0.0;
   out_5713404591469320854[196] = 0.0;
   out_5713404591469320854[197] = 0.0;
   out_5713404591469320854[198] = 0.0;
   out_5713404591469320854[199] = 0.0;
   out_5713404591469320854[200] = 0.0;
   out_5713404591469320854[201] = 0.0;
   out_5713404591469320854[202] = 0.0;
   out_5713404591469320854[203] = 0.0;
   out_5713404591469320854[204] = 0.0;
   out_5713404591469320854[205] = 0.0;
   out_5713404591469320854[206] = 0.0;
   out_5713404591469320854[207] = 0.0;
   out_5713404591469320854[208] = 0.0;
   out_5713404591469320854[209] = 1.0;
   out_5713404591469320854[210] = 0.0;
   out_5713404591469320854[211] = 0.0;
   out_5713404591469320854[212] = 0.0;
   out_5713404591469320854[213] = 0.0;
   out_5713404591469320854[214] = 0.0;
   out_5713404591469320854[215] = 0.0;
   out_5713404591469320854[216] = 0.0;
   out_5713404591469320854[217] = 0.0;
   out_5713404591469320854[218] = 0.0;
   out_5713404591469320854[219] = 0.0;
   out_5713404591469320854[220] = 0.0;
   out_5713404591469320854[221] = 0.0;
   out_5713404591469320854[222] = 0.0;
   out_5713404591469320854[223] = 0.0;
   out_5713404591469320854[224] = 0.0;
   out_5713404591469320854[225] = 0.0;
   out_5713404591469320854[226] = 0.0;
   out_5713404591469320854[227] = 0.0;
   out_5713404591469320854[228] = 1.0;
   out_5713404591469320854[229] = 0.0;
   out_5713404591469320854[230] = 0.0;
   out_5713404591469320854[231] = 0.0;
   out_5713404591469320854[232] = 0.0;
   out_5713404591469320854[233] = 0.0;
   out_5713404591469320854[234] = 0.0;
   out_5713404591469320854[235] = 0.0;
   out_5713404591469320854[236] = 0.0;
   out_5713404591469320854[237] = 0.0;
   out_5713404591469320854[238] = 0.0;
   out_5713404591469320854[239] = 0.0;
   out_5713404591469320854[240] = 0.0;
   out_5713404591469320854[241] = 0.0;
   out_5713404591469320854[242] = 0.0;
   out_5713404591469320854[243] = 0.0;
   out_5713404591469320854[244] = 0.0;
   out_5713404591469320854[245] = 0.0;
   out_5713404591469320854[246] = 0.0;
   out_5713404591469320854[247] = 1.0;
   out_5713404591469320854[248] = 0.0;
   out_5713404591469320854[249] = 0.0;
   out_5713404591469320854[250] = 0.0;
   out_5713404591469320854[251] = 0.0;
   out_5713404591469320854[252] = 0.0;
   out_5713404591469320854[253] = 0.0;
   out_5713404591469320854[254] = 0.0;
   out_5713404591469320854[255] = 0.0;
   out_5713404591469320854[256] = 0.0;
   out_5713404591469320854[257] = 0.0;
   out_5713404591469320854[258] = 0.0;
   out_5713404591469320854[259] = 0.0;
   out_5713404591469320854[260] = 0.0;
   out_5713404591469320854[261] = 0.0;
   out_5713404591469320854[262] = 0.0;
   out_5713404591469320854[263] = 0.0;
   out_5713404591469320854[264] = 0.0;
   out_5713404591469320854[265] = 0.0;
   out_5713404591469320854[266] = 1.0;
   out_5713404591469320854[267] = 0.0;
   out_5713404591469320854[268] = 0.0;
   out_5713404591469320854[269] = 0.0;
   out_5713404591469320854[270] = 0.0;
   out_5713404591469320854[271] = 0.0;
   out_5713404591469320854[272] = 0.0;
   out_5713404591469320854[273] = 0.0;
   out_5713404591469320854[274] = 0.0;
   out_5713404591469320854[275] = 0.0;
   out_5713404591469320854[276] = 0.0;
   out_5713404591469320854[277] = 0.0;
   out_5713404591469320854[278] = 0.0;
   out_5713404591469320854[279] = 0.0;
   out_5713404591469320854[280] = 0.0;
   out_5713404591469320854[281] = 0.0;
   out_5713404591469320854[282] = 0.0;
   out_5713404591469320854[283] = 0.0;
   out_5713404591469320854[284] = 0.0;
   out_5713404591469320854[285] = 1.0;
   out_5713404591469320854[286] = 0.0;
   out_5713404591469320854[287] = 0.0;
   out_5713404591469320854[288] = 0.0;
   out_5713404591469320854[289] = 0.0;
   out_5713404591469320854[290] = 0.0;
   out_5713404591469320854[291] = 0.0;
   out_5713404591469320854[292] = 0.0;
   out_5713404591469320854[293] = 0.0;
   out_5713404591469320854[294] = 0.0;
   out_5713404591469320854[295] = 0.0;
   out_5713404591469320854[296] = 0.0;
   out_5713404591469320854[297] = 0.0;
   out_5713404591469320854[298] = 0.0;
   out_5713404591469320854[299] = 0.0;
   out_5713404591469320854[300] = 0.0;
   out_5713404591469320854[301] = 0.0;
   out_5713404591469320854[302] = 0.0;
   out_5713404591469320854[303] = 0.0;
   out_5713404591469320854[304] = 1.0;
   out_5713404591469320854[305] = 0.0;
   out_5713404591469320854[306] = 0.0;
   out_5713404591469320854[307] = 0.0;
   out_5713404591469320854[308] = 0.0;
   out_5713404591469320854[309] = 0.0;
   out_5713404591469320854[310] = 0.0;
   out_5713404591469320854[311] = 0.0;
   out_5713404591469320854[312] = 0.0;
   out_5713404591469320854[313] = 0.0;
   out_5713404591469320854[314] = 0.0;
   out_5713404591469320854[315] = 0.0;
   out_5713404591469320854[316] = 0.0;
   out_5713404591469320854[317] = 0.0;
   out_5713404591469320854[318] = 0.0;
   out_5713404591469320854[319] = 0.0;
   out_5713404591469320854[320] = 0.0;
   out_5713404591469320854[321] = 0.0;
   out_5713404591469320854[322] = 0.0;
   out_5713404591469320854[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5650402834277827452) {
   out_5650402834277827452[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5650402834277827452[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5650402834277827452[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5650402834277827452[3] = dt*state[12] + state[3];
   out_5650402834277827452[4] = dt*state[13] + state[4];
   out_5650402834277827452[5] = dt*state[14] + state[5];
   out_5650402834277827452[6] = state[6];
   out_5650402834277827452[7] = state[7];
   out_5650402834277827452[8] = state[8];
   out_5650402834277827452[9] = state[9];
   out_5650402834277827452[10] = state[10];
   out_5650402834277827452[11] = state[11];
   out_5650402834277827452[12] = state[12];
   out_5650402834277827452[13] = state[13];
   out_5650402834277827452[14] = state[14];
   out_5650402834277827452[15] = state[15];
   out_5650402834277827452[16] = state[16];
   out_5650402834277827452[17] = state[17];
}
void F_fun(double *state, double dt, double *out_715380118175320270) {
   out_715380118175320270[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_715380118175320270[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_715380118175320270[2] = 0;
   out_715380118175320270[3] = 0;
   out_715380118175320270[4] = 0;
   out_715380118175320270[5] = 0;
   out_715380118175320270[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_715380118175320270[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_715380118175320270[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_715380118175320270[9] = 0;
   out_715380118175320270[10] = 0;
   out_715380118175320270[11] = 0;
   out_715380118175320270[12] = 0;
   out_715380118175320270[13] = 0;
   out_715380118175320270[14] = 0;
   out_715380118175320270[15] = 0;
   out_715380118175320270[16] = 0;
   out_715380118175320270[17] = 0;
   out_715380118175320270[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_715380118175320270[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_715380118175320270[20] = 0;
   out_715380118175320270[21] = 0;
   out_715380118175320270[22] = 0;
   out_715380118175320270[23] = 0;
   out_715380118175320270[24] = 0;
   out_715380118175320270[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_715380118175320270[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_715380118175320270[27] = 0;
   out_715380118175320270[28] = 0;
   out_715380118175320270[29] = 0;
   out_715380118175320270[30] = 0;
   out_715380118175320270[31] = 0;
   out_715380118175320270[32] = 0;
   out_715380118175320270[33] = 0;
   out_715380118175320270[34] = 0;
   out_715380118175320270[35] = 0;
   out_715380118175320270[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_715380118175320270[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_715380118175320270[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_715380118175320270[39] = 0;
   out_715380118175320270[40] = 0;
   out_715380118175320270[41] = 0;
   out_715380118175320270[42] = 0;
   out_715380118175320270[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_715380118175320270[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_715380118175320270[45] = 0;
   out_715380118175320270[46] = 0;
   out_715380118175320270[47] = 0;
   out_715380118175320270[48] = 0;
   out_715380118175320270[49] = 0;
   out_715380118175320270[50] = 0;
   out_715380118175320270[51] = 0;
   out_715380118175320270[52] = 0;
   out_715380118175320270[53] = 0;
   out_715380118175320270[54] = 0;
   out_715380118175320270[55] = 0;
   out_715380118175320270[56] = 0;
   out_715380118175320270[57] = 1;
   out_715380118175320270[58] = 0;
   out_715380118175320270[59] = 0;
   out_715380118175320270[60] = 0;
   out_715380118175320270[61] = 0;
   out_715380118175320270[62] = 0;
   out_715380118175320270[63] = 0;
   out_715380118175320270[64] = 0;
   out_715380118175320270[65] = 0;
   out_715380118175320270[66] = dt;
   out_715380118175320270[67] = 0;
   out_715380118175320270[68] = 0;
   out_715380118175320270[69] = 0;
   out_715380118175320270[70] = 0;
   out_715380118175320270[71] = 0;
   out_715380118175320270[72] = 0;
   out_715380118175320270[73] = 0;
   out_715380118175320270[74] = 0;
   out_715380118175320270[75] = 0;
   out_715380118175320270[76] = 1;
   out_715380118175320270[77] = 0;
   out_715380118175320270[78] = 0;
   out_715380118175320270[79] = 0;
   out_715380118175320270[80] = 0;
   out_715380118175320270[81] = 0;
   out_715380118175320270[82] = 0;
   out_715380118175320270[83] = 0;
   out_715380118175320270[84] = 0;
   out_715380118175320270[85] = dt;
   out_715380118175320270[86] = 0;
   out_715380118175320270[87] = 0;
   out_715380118175320270[88] = 0;
   out_715380118175320270[89] = 0;
   out_715380118175320270[90] = 0;
   out_715380118175320270[91] = 0;
   out_715380118175320270[92] = 0;
   out_715380118175320270[93] = 0;
   out_715380118175320270[94] = 0;
   out_715380118175320270[95] = 1;
   out_715380118175320270[96] = 0;
   out_715380118175320270[97] = 0;
   out_715380118175320270[98] = 0;
   out_715380118175320270[99] = 0;
   out_715380118175320270[100] = 0;
   out_715380118175320270[101] = 0;
   out_715380118175320270[102] = 0;
   out_715380118175320270[103] = 0;
   out_715380118175320270[104] = dt;
   out_715380118175320270[105] = 0;
   out_715380118175320270[106] = 0;
   out_715380118175320270[107] = 0;
   out_715380118175320270[108] = 0;
   out_715380118175320270[109] = 0;
   out_715380118175320270[110] = 0;
   out_715380118175320270[111] = 0;
   out_715380118175320270[112] = 0;
   out_715380118175320270[113] = 0;
   out_715380118175320270[114] = 1;
   out_715380118175320270[115] = 0;
   out_715380118175320270[116] = 0;
   out_715380118175320270[117] = 0;
   out_715380118175320270[118] = 0;
   out_715380118175320270[119] = 0;
   out_715380118175320270[120] = 0;
   out_715380118175320270[121] = 0;
   out_715380118175320270[122] = 0;
   out_715380118175320270[123] = 0;
   out_715380118175320270[124] = 0;
   out_715380118175320270[125] = 0;
   out_715380118175320270[126] = 0;
   out_715380118175320270[127] = 0;
   out_715380118175320270[128] = 0;
   out_715380118175320270[129] = 0;
   out_715380118175320270[130] = 0;
   out_715380118175320270[131] = 0;
   out_715380118175320270[132] = 0;
   out_715380118175320270[133] = 1;
   out_715380118175320270[134] = 0;
   out_715380118175320270[135] = 0;
   out_715380118175320270[136] = 0;
   out_715380118175320270[137] = 0;
   out_715380118175320270[138] = 0;
   out_715380118175320270[139] = 0;
   out_715380118175320270[140] = 0;
   out_715380118175320270[141] = 0;
   out_715380118175320270[142] = 0;
   out_715380118175320270[143] = 0;
   out_715380118175320270[144] = 0;
   out_715380118175320270[145] = 0;
   out_715380118175320270[146] = 0;
   out_715380118175320270[147] = 0;
   out_715380118175320270[148] = 0;
   out_715380118175320270[149] = 0;
   out_715380118175320270[150] = 0;
   out_715380118175320270[151] = 0;
   out_715380118175320270[152] = 1;
   out_715380118175320270[153] = 0;
   out_715380118175320270[154] = 0;
   out_715380118175320270[155] = 0;
   out_715380118175320270[156] = 0;
   out_715380118175320270[157] = 0;
   out_715380118175320270[158] = 0;
   out_715380118175320270[159] = 0;
   out_715380118175320270[160] = 0;
   out_715380118175320270[161] = 0;
   out_715380118175320270[162] = 0;
   out_715380118175320270[163] = 0;
   out_715380118175320270[164] = 0;
   out_715380118175320270[165] = 0;
   out_715380118175320270[166] = 0;
   out_715380118175320270[167] = 0;
   out_715380118175320270[168] = 0;
   out_715380118175320270[169] = 0;
   out_715380118175320270[170] = 0;
   out_715380118175320270[171] = 1;
   out_715380118175320270[172] = 0;
   out_715380118175320270[173] = 0;
   out_715380118175320270[174] = 0;
   out_715380118175320270[175] = 0;
   out_715380118175320270[176] = 0;
   out_715380118175320270[177] = 0;
   out_715380118175320270[178] = 0;
   out_715380118175320270[179] = 0;
   out_715380118175320270[180] = 0;
   out_715380118175320270[181] = 0;
   out_715380118175320270[182] = 0;
   out_715380118175320270[183] = 0;
   out_715380118175320270[184] = 0;
   out_715380118175320270[185] = 0;
   out_715380118175320270[186] = 0;
   out_715380118175320270[187] = 0;
   out_715380118175320270[188] = 0;
   out_715380118175320270[189] = 0;
   out_715380118175320270[190] = 1;
   out_715380118175320270[191] = 0;
   out_715380118175320270[192] = 0;
   out_715380118175320270[193] = 0;
   out_715380118175320270[194] = 0;
   out_715380118175320270[195] = 0;
   out_715380118175320270[196] = 0;
   out_715380118175320270[197] = 0;
   out_715380118175320270[198] = 0;
   out_715380118175320270[199] = 0;
   out_715380118175320270[200] = 0;
   out_715380118175320270[201] = 0;
   out_715380118175320270[202] = 0;
   out_715380118175320270[203] = 0;
   out_715380118175320270[204] = 0;
   out_715380118175320270[205] = 0;
   out_715380118175320270[206] = 0;
   out_715380118175320270[207] = 0;
   out_715380118175320270[208] = 0;
   out_715380118175320270[209] = 1;
   out_715380118175320270[210] = 0;
   out_715380118175320270[211] = 0;
   out_715380118175320270[212] = 0;
   out_715380118175320270[213] = 0;
   out_715380118175320270[214] = 0;
   out_715380118175320270[215] = 0;
   out_715380118175320270[216] = 0;
   out_715380118175320270[217] = 0;
   out_715380118175320270[218] = 0;
   out_715380118175320270[219] = 0;
   out_715380118175320270[220] = 0;
   out_715380118175320270[221] = 0;
   out_715380118175320270[222] = 0;
   out_715380118175320270[223] = 0;
   out_715380118175320270[224] = 0;
   out_715380118175320270[225] = 0;
   out_715380118175320270[226] = 0;
   out_715380118175320270[227] = 0;
   out_715380118175320270[228] = 1;
   out_715380118175320270[229] = 0;
   out_715380118175320270[230] = 0;
   out_715380118175320270[231] = 0;
   out_715380118175320270[232] = 0;
   out_715380118175320270[233] = 0;
   out_715380118175320270[234] = 0;
   out_715380118175320270[235] = 0;
   out_715380118175320270[236] = 0;
   out_715380118175320270[237] = 0;
   out_715380118175320270[238] = 0;
   out_715380118175320270[239] = 0;
   out_715380118175320270[240] = 0;
   out_715380118175320270[241] = 0;
   out_715380118175320270[242] = 0;
   out_715380118175320270[243] = 0;
   out_715380118175320270[244] = 0;
   out_715380118175320270[245] = 0;
   out_715380118175320270[246] = 0;
   out_715380118175320270[247] = 1;
   out_715380118175320270[248] = 0;
   out_715380118175320270[249] = 0;
   out_715380118175320270[250] = 0;
   out_715380118175320270[251] = 0;
   out_715380118175320270[252] = 0;
   out_715380118175320270[253] = 0;
   out_715380118175320270[254] = 0;
   out_715380118175320270[255] = 0;
   out_715380118175320270[256] = 0;
   out_715380118175320270[257] = 0;
   out_715380118175320270[258] = 0;
   out_715380118175320270[259] = 0;
   out_715380118175320270[260] = 0;
   out_715380118175320270[261] = 0;
   out_715380118175320270[262] = 0;
   out_715380118175320270[263] = 0;
   out_715380118175320270[264] = 0;
   out_715380118175320270[265] = 0;
   out_715380118175320270[266] = 1;
   out_715380118175320270[267] = 0;
   out_715380118175320270[268] = 0;
   out_715380118175320270[269] = 0;
   out_715380118175320270[270] = 0;
   out_715380118175320270[271] = 0;
   out_715380118175320270[272] = 0;
   out_715380118175320270[273] = 0;
   out_715380118175320270[274] = 0;
   out_715380118175320270[275] = 0;
   out_715380118175320270[276] = 0;
   out_715380118175320270[277] = 0;
   out_715380118175320270[278] = 0;
   out_715380118175320270[279] = 0;
   out_715380118175320270[280] = 0;
   out_715380118175320270[281] = 0;
   out_715380118175320270[282] = 0;
   out_715380118175320270[283] = 0;
   out_715380118175320270[284] = 0;
   out_715380118175320270[285] = 1;
   out_715380118175320270[286] = 0;
   out_715380118175320270[287] = 0;
   out_715380118175320270[288] = 0;
   out_715380118175320270[289] = 0;
   out_715380118175320270[290] = 0;
   out_715380118175320270[291] = 0;
   out_715380118175320270[292] = 0;
   out_715380118175320270[293] = 0;
   out_715380118175320270[294] = 0;
   out_715380118175320270[295] = 0;
   out_715380118175320270[296] = 0;
   out_715380118175320270[297] = 0;
   out_715380118175320270[298] = 0;
   out_715380118175320270[299] = 0;
   out_715380118175320270[300] = 0;
   out_715380118175320270[301] = 0;
   out_715380118175320270[302] = 0;
   out_715380118175320270[303] = 0;
   out_715380118175320270[304] = 1;
   out_715380118175320270[305] = 0;
   out_715380118175320270[306] = 0;
   out_715380118175320270[307] = 0;
   out_715380118175320270[308] = 0;
   out_715380118175320270[309] = 0;
   out_715380118175320270[310] = 0;
   out_715380118175320270[311] = 0;
   out_715380118175320270[312] = 0;
   out_715380118175320270[313] = 0;
   out_715380118175320270[314] = 0;
   out_715380118175320270[315] = 0;
   out_715380118175320270[316] = 0;
   out_715380118175320270[317] = 0;
   out_715380118175320270[318] = 0;
   out_715380118175320270[319] = 0;
   out_715380118175320270[320] = 0;
   out_715380118175320270[321] = 0;
   out_715380118175320270[322] = 0;
   out_715380118175320270[323] = 1;
}
void h_4(double *state, double *unused, double *out_8448258150288935568) {
   out_8448258150288935568[0] = state[6] + state[9];
   out_8448258150288935568[1] = state[7] + state[10];
   out_8448258150288935568[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6467565593524033461) {
   out_6467565593524033461[0] = 0;
   out_6467565593524033461[1] = 0;
   out_6467565593524033461[2] = 0;
   out_6467565593524033461[3] = 0;
   out_6467565593524033461[4] = 0;
   out_6467565593524033461[5] = 0;
   out_6467565593524033461[6] = 1;
   out_6467565593524033461[7] = 0;
   out_6467565593524033461[8] = 0;
   out_6467565593524033461[9] = 1;
   out_6467565593524033461[10] = 0;
   out_6467565593524033461[11] = 0;
   out_6467565593524033461[12] = 0;
   out_6467565593524033461[13] = 0;
   out_6467565593524033461[14] = 0;
   out_6467565593524033461[15] = 0;
   out_6467565593524033461[16] = 0;
   out_6467565593524033461[17] = 0;
   out_6467565593524033461[18] = 0;
   out_6467565593524033461[19] = 0;
   out_6467565593524033461[20] = 0;
   out_6467565593524033461[21] = 0;
   out_6467565593524033461[22] = 0;
   out_6467565593524033461[23] = 0;
   out_6467565593524033461[24] = 0;
   out_6467565593524033461[25] = 1;
   out_6467565593524033461[26] = 0;
   out_6467565593524033461[27] = 0;
   out_6467565593524033461[28] = 1;
   out_6467565593524033461[29] = 0;
   out_6467565593524033461[30] = 0;
   out_6467565593524033461[31] = 0;
   out_6467565593524033461[32] = 0;
   out_6467565593524033461[33] = 0;
   out_6467565593524033461[34] = 0;
   out_6467565593524033461[35] = 0;
   out_6467565593524033461[36] = 0;
   out_6467565593524033461[37] = 0;
   out_6467565593524033461[38] = 0;
   out_6467565593524033461[39] = 0;
   out_6467565593524033461[40] = 0;
   out_6467565593524033461[41] = 0;
   out_6467565593524033461[42] = 0;
   out_6467565593524033461[43] = 0;
   out_6467565593524033461[44] = 1;
   out_6467565593524033461[45] = 0;
   out_6467565593524033461[46] = 0;
   out_6467565593524033461[47] = 1;
   out_6467565593524033461[48] = 0;
   out_6467565593524033461[49] = 0;
   out_6467565593524033461[50] = 0;
   out_6467565593524033461[51] = 0;
   out_6467565593524033461[52] = 0;
   out_6467565593524033461[53] = 0;
}
void h_10(double *state, double *unused, double *out_7723716202278938104) {
   out_7723716202278938104[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7723716202278938104[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7723716202278938104[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_2203730435840599590) {
   out_2203730435840599590[0] = 0;
   out_2203730435840599590[1] = 9.8100000000000005*cos(state[1]);
   out_2203730435840599590[2] = 0;
   out_2203730435840599590[3] = 0;
   out_2203730435840599590[4] = -state[8];
   out_2203730435840599590[5] = state[7];
   out_2203730435840599590[6] = 0;
   out_2203730435840599590[7] = state[5];
   out_2203730435840599590[8] = -state[4];
   out_2203730435840599590[9] = 0;
   out_2203730435840599590[10] = 0;
   out_2203730435840599590[11] = 0;
   out_2203730435840599590[12] = 1;
   out_2203730435840599590[13] = 0;
   out_2203730435840599590[14] = 0;
   out_2203730435840599590[15] = 1;
   out_2203730435840599590[16] = 0;
   out_2203730435840599590[17] = 0;
   out_2203730435840599590[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_2203730435840599590[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_2203730435840599590[20] = 0;
   out_2203730435840599590[21] = state[8];
   out_2203730435840599590[22] = 0;
   out_2203730435840599590[23] = -state[6];
   out_2203730435840599590[24] = -state[5];
   out_2203730435840599590[25] = 0;
   out_2203730435840599590[26] = state[3];
   out_2203730435840599590[27] = 0;
   out_2203730435840599590[28] = 0;
   out_2203730435840599590[29] = 0;
   out_2203730435840599590[30] = 0;
   out_2203730435840599590[31] = 1;
   out_2203730435840599590[32] = 0;
   out_2203730435840599590[33] = 0;
   out_2203730435840599590[34] = 1;
   out_2203730435840599590[35] = 0;
   out_2203730435840599590[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_2203730435840599590[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_2203730435840599590[38] = 0;
   out_2203730435840599590[39] = -state[7];
   out_2203730435840599590[40] = state[6];
   out_2203730435840599590[41] = 0;
   out_2203730435840599590[42] = state[4];
   out_2203730435840599590[43] = -state[3];
   out_2203730435840599590[44] = 0;
   out_2203730435840599590[45] = 0;
   out_2203730435840599590[46] = 0;
   out_2203730435840599590[47] = 0;
   out_2203730435840599590[48] = 0;
   out_2203730435840599590[49] = 0;
   out_2203730435840599590[50] = 1;
   out_2203730435840599590[51] = 0;
   out_2203730435840599590[52] = 0;
   out_2203730435840599590[53] = 1;
}
void h_13(double *state, double *unused, double *out_3557376465177495290) {
   out_3557376465177495290[0] = state[3];
   out_3557376465177495290[1] = state[4];
   out_3557376465177495290[2] = state[5];
}
void H_13(double *state, double *unused, double *out_4368547271868817226) {
   out_4368547271868817226[0] = 0;
   out_4368547271868817226[1] = 0;
   out_4368547271868817226[2] = 0;
   out_4368547271868817226[3] = 1;
   out_4368547271868817226[4] = 0;
   out_4368547271868817226[5] = 0;
   out_4368547271868817226[6] = 0;
   out_4368547271868817226[7] = 0;
   out_4368547271868817226[8] = 0;
   out_4368547271868817226[9] = 0;
   out_4368547271868817226[10] = 0;
   out_4368547271868817226[11] = 0;
   out_4368547271868817226[12] = 0;
   out_4368547271868817226[13] = 0;
   out_4368547271868817226[14] = 0;
   out_4368547271868817226[15] = 0;
   out_4368547271868817226[16] = 0;
   out_4368547271868817226[17] = 0;
   out_4368547271868817226[18] = 0;
   out_4368547271868817226[19] = 0;
   out_4368547271868817226[20] = 0;
   out_4368547271868817226[21] = 0;
   out_4368547271868817226[22] = 1;
   out_4368547271868817226[23] = 0;
   out_4368547271868817226[24] = 0;
   out_4368547271868817226[25] = 0;
   out_4368547271868817226[26] = 0;
   out_4368547271868817226[27] = 0;
   out_4368547271868817226[28] = 0;
   out_4368547271868817226[29] = 0;
   out_4368547271868817226[30] = 0;
   out_4368547271868817226[31] = 0;
   out_4368547271868817226[32] = 0;
   out_4368547271868817226[33] = 0;
   out_4368547271868817226[34] = 0;
   out_4368547271868817226[35] = 0;
   out_4368547271868817226[36] = 0;
   out_4368547271868817226[37] = 0;
   out_4368547271868817226[38] = 0;
   out_4368547271868817226[39] = 0;
   out_4368547271868817226[40] = 0;
   out_4368547271868817226[41] = 1;
   out_4368547271868817226[42] = 0;
   out_4368547271868817226[43] = 0;
   out_4368547271868817226[44] = 0;
   out_4368547271868817226[45] = 0;
   out_4368547271868817226[46] = 0;
   out_4368547271868817226[47] = 0;
   out_4368547271868817226[48] = 0;
   out_4368547271868817226[49] = 0;
   out_4368547271868817226[50] = 0;
   out_4368547271868817226[51] = 0;
   out_4368547271868817226[52] = 0;
   out_4368547271868817226[53] = 0;
}
void h_14(double *state, double *unused, double *out_5615766574460706046) {
   out_5615766574460706046[0] = state[6];
   out_5615766574460706046[1] = state[7];
   out_5615766574460706046[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3384777161228661165) {
   out_3384777161228661165[0] = 0;
   out_3384777161228661165[1] = 0;
   out_3384777161228661165[2] = 0;
   out_3384777161228661165[3] = 0;
   out_3384777161228661165[4] = 0;
   out_3384777161228661165[5] = 0;
   out_3384777161228661165[6] = 1;
   out_3384777161228661165[7] = 0;
   out_3384777161228661165[8] = 0;
   out_3384777161228661165[9] = 0;
   out_3384777161228661165[10] = 0;
   out_3384777161228661165[11] = 0;
   out_3384777161228661165[12] = 0;
   out_3384777161228661165[13] = 0;
   out_3384777161228661165[14] = 0;
   out_3384777161228661165[15] = 0;
   out_3384777161228661165[16] = 0;
   out_3384777161228661165[17] = 0;
   out_3384777161228661165[18] = 0;
   out_3384777161228661165[19] = 0;
   out_3384777161228661165[20] = 0;
   out_3384777161228661165[21] = 0;
   out_3384777161228661165[22] = 0;
   out_3384777161228661165[23] = 0;
   out_3384777161228661165[24] = 0;
   out_3384777161228661165[25] = 1;
   out_3384777161228661165[26] = 0;
   out_3384777161228661165[27] = 0;
   out_3384777161228661165[28] = 0;
   out_3384777161228661165[29] = 0;
   out_3384777161228661165[30] = 0;
   out_3384777161228661165[31] = 0;
   out_3384777161228661165[32] = 0;
   out_3384777161228661165[33] = 0;
   out_3384777161228661165[34] = 0;
   out_3384777161228661165[35] = 0;
   out_3384777161228661165[36] = 0;
   out_3384777161228661165[37] = 0;
   out_3384777161228661165[38] = 0;
   out_3384777161228661165[39] = 0;
   out_3384777161228661165[40] = 0;
   out_3384777161228661165[41] = 0;
   out_3384777161228661165[42] = 0;
   out_3384777161228661165[43] = 0;
   out_3384777161228661165[44] = 1;
   out_3384777161228661165[45] = 0;
   out_3384777161228661165[46] = 0;
   out_3384777161228661165[47] = 0;
   out_3384777161228661165[48] = 0;
   out_3384777161228661165[49] = 0;
   out_3384777161228661165[50] = 0;
   out_3384777161228661165[51] = 0;
   out_3384777161228661165[52] = 0;
   out_3384777161228661165[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_6316962376878717030) {
  err_fun(nom_x, delta_x, out_6316962376878717030);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2486354416350675894) {
  inv_err_fun(nom_x, true_x, out_2486354416350675894);
}
void pose_H_mod_fun(double *state, double *out_5713404591469320854) {
  H_mod_fun(state, out_5713404591469320854);
}
void pose_f_fun(double *state, double dt, double *out_5650402834277827452) {
  f_fun(state,  dt, out_5650402834277827452);
}
void pose_F_fun(double *state, double dt, double *out_715380118175320270) {
  F_fun(state,  dt, out_715380118175320270);
}
void pose_h_4(double *state, double *unused, double *out_8448258150288935568) {
  h_4(state, unused, out_8448258150288935568);
}
void pose_H_4(double *state, double *unused, double *out_6467565593524033461) {
  H_4(state, unused, out_6467565593524033461);
}
void pose_h_10(double *state, double *unused, double *out_7723716202278938104) {
  h_10(state, unused, out_7723716202278938104);
}
void pose_H_10(double *state, double *unused, double *out_2203730435840599590) {
  H_10(state, unused, out_2203730435840599590);
}
void pose_h_13(double *state, double *unused, double *out_3557376465177495290) {
  h_13(state, unused, out_3557376465177495290);
}
void pose_H_13(double *state, double *unused, double *out_4368547271868817226) {
  H_13(state, unused, out_4368547271868817226);
}
void pose_h_14(double *state, double *unused, double *out_5615766574460706046) {
  h_14(state, unused, out_5615766574460706046);
}
void pose_H_14(double *state, double *unused, double *out_3384777161228661165) {
  H_14(state, unused, out_3384777161228661165);
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
