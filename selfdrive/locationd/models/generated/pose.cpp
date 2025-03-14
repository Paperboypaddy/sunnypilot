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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7827726718003508843) {
   out_7827726718003508843[0] = delta_x[0] + nom_x[0];
   out_7827726718003508843[1] = delta_x[1] + nom_x[1];
   out_7827726718003508843[2] = delta_x[2] + nom_x[2];
   out_7827726718003508843[3] = delta_x[3] + nom_x[3];
   out_7827726718003508843[4] = delta_x[4] + nom_x[4];
   out_7827726718003508843[5] = delta_x[5] + nom_x[5];
   out_7827726718003508843[6] = delta_x[6] + nom_x[6];
   out_7827726718003508843[7] = delta_x[7] + nom_x[7];
   out_7827726718003508843[8] = delta_x[8] + nom_x[8];
   out_7827726718003508843[9] = delta_x[9] + nom_x[9];
   out_7827726718003508843[10] = delta_x[10] + nom_x[10];
   out_7827726718003508843[11] = delta_x[11] + nom_x[11];
   out_7827726718003508843[12] = delta_x[12] + nom_x[12];
   out_7827726718003508843[13] = delta_x[13] + nom_x[13];
   out_7827726718003508843[14] = delta_x[14] + nom_x[14];
   out_7827726718003508843[15] = delta_x[15] + nom_x[15];
   out_7827726718003508843[16] = delta_x[16] + nom_x[16];
   out_7827726718003508843[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6277241276544534268) {
   out_6277241276544534268[0] = -nom_x[0] + true_x[0];
   out_6277241276544534268[1] = -nom_x[1] + true_x[1];
   out_6277241276544534268[2] = -nom_x[2] + true_x[2];
   out_6277241276544534268[3] = -nom_x[3] + true_x[3];
   out_6277241276544534268[4] = -nom_x[4] + true_x[4];
   out_6277241276544534268[5] = -nom_x[5] + true_x[5];
   out_6277241276544534268[6] = -nom_x[6] + true_x[6];
   out_6277241276544534268[7] = -nom_x[7] + true_x[7];
   out_6277241276544534268[8] = -nom_x[8] + true_x[8];
   out_6277241276544534268[9] = -nom_x[9] + true_x[9];
   out_6277241276544534268[10] = -nom_x[10] + true_x[10];
   out_6277241276544534268[11] = -nom_x[11] + true_x[11];
   out_6277241276544534268[12] = -nom_x[12] + true_x[12];
   out_6277241276544534268[13] = -nom_x[13] + true_x[13];
   out_6277241276544534268[14] = -nom_x[14] + true_x[14];
   out_6277241276544534268[15] = -nom_x[15] + true_x[15];
   out_6277241276544534268[16] = -nom_x[16] + true_x[16];
   out_6277241276544534268[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3091072411791179619) {
   out_3091072411791179619[0] = 1.0;
   out_3091072411791179619[1] = 0.0;
   out_3091072411791179619[2] = 0.0;
   out_3091072411791179619[3] = 0.0;
   out_3091072411791179619[4] = 0.0;
   out_3091072411791179619[5] = 0.0;
   out_3091072411791179619[6] = 0.0;
   out_3091072411791179619[7] = 0.0;
   out_3091072411791179619[8] = 0.0;
   out_3091072411791179619[9] = 0.0;
   out_3091072411791179619[10] = 0.0;
   out_3091072411791179619[11] = 0.0;
   out_3091072411791179619[12] = 0.0;
   out_3091072411791179619[13] = 0.0;
   out_3091072411791179619[14] = 0.0;
   out_3091072411791179619[15] = 0.0;
   out_3091072411791179619[16] = 0.0;
   out_3091072411791179619[17] = 0.0;
   out_3091072411791179619[18] = 0.0;
   out_3091072411791179619[19] = 1.0;
   out_3091072411791179619[20] = 0.0;
   out_3091072411791179619[21] = 0.0;
   out_3091072411791179619[22] = 0.0;
   out_3091072411791179619[23] = 0.0;
   out_3091072411791179619[24] = 0.0;
   out_3091072411791179619[25] = 0.0;
   out_3091072411791179619[26] = 0.0;
   out_3091072411791179619[27] = 0.0;
   out_3091072411791179619[28] = 0.0;
   out_3091072411791179619[29] = 0.0;
   out_3091072411791179619[30] = 0.0;
   out_3091072411791179619[31] = 0.0;
   out_3091072411791179619[32] = 0.0;
   out_3091072411791179619[33] = 0.0;
   out_3091072411791179619[34] = 0.0;
   out_3091072411791179619[35] = 0.0;
   out_3091072411791179619[36] = 0.0;
   out_3091072411791179619[37] = 0.0;
   out_3091072411791179619[38] = 1.0;
   out_3091072411791179619[39] = 0.0;
   out_3091072411791179619[40] = 0.0;
   out_3091072411791179619[41] = 0.0;
   out_3091072411791179619[42] = 0.0;
   out_3091072411791179619[43] = 0.0;
   out_3091072411791179619[44] = 0.0;
   out_3091072411791179619[45] = 0.0;
   out_3091072411791179619[46] = 0.0;
   out_3091072411791179619[47] = 0.0;
   out_3091072411791179619[48] = 0.0;
   out_3091072411791179619[49] = 0.0;
   out_3091072411791179619[50] = 0.0;
   out_3091072411791179619[51] = 0.0;
   out_3091072411791179619[52] = 0.0;
   out_3091072411791179619[53] = 0.0;
   out_3091072411791179619[54] = 0.0;
   out_3091072411791179619[55] = 0.0;
   out_3091072411791179619[56] = 0.0;
   out_3091072411791179619[57] = 1.0;
   out_3091072411791179619[58] = 0.0;
   out_3091072411791179619[59] = 0.0;
   out_3091072411791179619[60] = 0.0;
   out_3091072411791179619[61] = 0.0;
   out_3091072411791179619[62] = 0.0;
   out_3091072411791179619[63] = 0.0;
   out_3091072411791179619[64] = 0.0;
   out_3091072411791179619[65] = 0.0;
   out_3091072411791179619[66] = 0.0;
   out_3091072411791179619[67] = 0.0;
   out_3091072411791179619[68] = 0.0;
   out_3091072411791179619[69] = 0.0;
   out_3091072411791179619[70] = 0.0;
   out_3091072411791179619[71] = 0.0;
   out_3091072411791179619[72] = 0.0;
   out_3091072411791179619[73] = 0.0;
   out_3091072411791179619[74] = 0.0;
   out_3091072411791179619[75] = 0.0;
   out_3091072411791179619[76] = 1.0;
   out_3091072411791179619[77] = 0.0;
   out_3091072411791179619[78] = 0.0;
   out_3091072411791179619[79] = 0.0;
   out_3091072411791179619[80] = 0.0;
   out_3091072411791179619[81] = 0.0;
   out_3091072411791179619[82] = 0.0;
   out_3091072411791179619[83] = 0.0;
   out_3091072411791179619[84] = 0.0;
   out_3091072411791179619[85] = 0.0;
   out_3091072411791179619[86] = 0.0;
   out_3091072411791179619[87] = 0.0;
   out_3091072411791179619[88] = 0.0;
   out_3091072411791179619[89] = 0.0;
   out_3091072411791179619[90] = 0.0;
   out_3091072411791179619[91] = 0.0;
   out_3091072411791179619[92] = 0.0;
   out_3091072411791179619[93] = 0.0;
   out_3091072411791179619[94] = 0.0;
   out_3091072411791179619[95] = 1.0;
   out_3091072411791179619[96] = 0.0;
   out_3091072411791179619[97] = 0.0;
   out_3091072411791179619[98] = 0.0;
   out_3091072411791179619[99] = 0.0;
   out_3091072411791179619[100] = 0.0;
   out_3091072411791179619[101] = 0.0;
   out_3091072411791179619[102] = 0.0;
   out_3091072411791179619[103] = 0.0;
   out_3091072411791179619[104] = 0.0;
   out_3091072411791179619[105] = 0.0;
   out_3091072411791179619[106] = 0.0;
   out_3091072411791179619[107] = 0.0;
   out_3091072411791179619[108] = 0.0;
   out_3091072411791179619[109] = 0.0;
   out_3091072411791179619[110] = 0.0;
   out_3091072411791179619[111] = 0.0;
   out_3091072411791179619[112] = 0.0;
   out_3091072411791179619[113] = 0.0;
   out_3091072411791179619[114] = 1.0;
   out_3091072411791179619[115] = 0.0;
   out_3091072411791179619[116] = 0.0;
   out_3091072411791179619[117] = 0.0;
   out_3091072411791179619[118] = 0.0;
   out_3091072411791179619[119] = 0.0;
   out_3091072411791179619[120] = 0.0;
   out_3091072411791179619[121] = 0.0;
   out_3091072411791179619[122] = 0.0;
   out_3091072411791179619[123] = 0.0;
   out_3091072411791179619[124] = 0.0;
   out_3091072411791179619[125] = 0.0;
   out_3091072411791179619[126] = 0.0;
   out_3091072411791179619[127] = 0.0;
   out_3091072411791179619[128] = 0.0;
   out_3091072411791179619[129] = 0.0;
   out_3091072411791179619[130] = 0.0;
   out_3091072411791179619[131] = 0.0;
   out_3091072411791179619[132] = 0.0;
   out_3091072411791179619[133] = 1.0;
   out_3091072411791179619[134] = 0.0;
   out_3091072411791179619[135] = 0.0;
   out_3091072411791179619[136] = 0.0;
   out_3091072411791179619[137] = 0.0;
   out_3091072411791179619[138] = 0.0;
   out_3091072411791179619[139] = 0.0;
   out_3091072411791179619[140] = 0.0;
   out_3091072411791179619[141] = 0.0;
   out_3091072411791179619[142] = 0.0;
   out_3091072411791179619[143] = 0.0;
   out_3091072411791179619[144] = 0.0;
   out_3091072411791179619[145] = 0.0;
   out_3091072411791179619[146] = 0.0;
   out_3091072411791179619[147] = 0.0;
   out_3091072411791179619[148] = 0.0;
   out_3091072411791179619[149] = 0.0;
   out_3091072411791179619[150] = 0.0;
   out_3091072411791179619[151] = 0.0;
   out_3091072411791179619[152] = 1.0;
   out_3091072411791179619[153] = 0.0;
   out_3091072411791179619[154] = 0.0;
   out_3091072411791179619[155] = 0.0;
   out_3091072411791179619[156] = 0.0;
   out_3091072411791179619[157] = 0.0;
   out_3091072411791179619[158] = 0.0;
   out_3091072411791179619[159] = 0.0;
   out_3091072411791179619[160] = 0.0;
   out_3091072411791179619[161] = 0.0;
   out_3091072411791179619[162] = 0.0;
   out_3091072411791179619[163] = 0.0;
   out_3091072411791179619[164] = 0.0;
   out_3091072411791179619[165] = 0.0;
   out_3091072411791179619[166] = 0.0;
   out_3091072411791179619[167] = 0.0;
   out_3091072411791179619[168] = 0.0;
   out_3091072411791179619[169] = 0.0;
   out_3091072411791179619[170] = 0.0;
   out_3091072411791179619[171] = 1.0;
   out_3091072411791179619[172] = 0.0;
   out_3091072411791179619[173] = 0.0;
   out_3091072411791179619[174] = 0.0;
   out_3091072411791179619[175] = 0.0;
   out_3091072411791179619[176] = 0.0;
   out_3091072411791179619[177] = 0.0;
   out_3091072411791179619[178] = 0.0;
   out_3091072411791179619[179] = 0.0;
   out_3091072411791179619[180] = 0.0;
   out_3091072411791179619[181] = 0.0;
   out_3091072411791179619[182] = 0.0;
   out_3091072411791179619[183] = 0.0;
   out_3091072411791179619[184] = 0.0;
   out_3091072411791179619[185] = 0.0;
   out_3091072411791179619[186] = 0.0;
   out_3091072411791179619[187] = 0.0;
   out_3091072411791179619[188] = 0.0;
   out_3091072411791179619[189] = 0.0;
   out_3091072411791179619[190] = 1.0;
   out_3091072411791179619[191] = 0.0;
   out_3091072411791179619[192] = 0.0;
   out_3091072411791179619[193] = 0.0;
   out_3091072411791179619[194] = 0.0;
   out_3091072411791179619[195] = 0.0;
   out_3091072411791179619[196] = 0.0;
   out_3091072411791179619[197] = 0.0;
   out_3091072411791179619[198] = 0.0;
   out_3091072411791179619[199] = 0.0;
   out_3091072411791179619[200] = 0.0;
   out_3091072411791179619[201] = 0.0;
   out_3091072411791179619[202] = 0.0;
   out_3091072411791179619[203] = 0.0;
   out_3091072411791179619[204] = 0.0;
   out_3091072411791179619[205] = 0.0;
   out_3091072411791179619[206] = 0.0;
   out_3091072411791179619[207] = 0.0;
   out_3091072411791179619[208] = 0.0;
   out_3091072411791179619[209] = 1.0;
   out_3091072411791179619[210] = 0.0;
   out_3091072411791179619[211] = 0.0;
   out_3091072411791179619[212] = 0.0;
   out_3091072411791179619[213] = 0.0;
   out_3091072411791179619[214] = 0.0;
   out_3091072411791179619[215] = 0.0;
   out_3091072411791179619[216] = 0.0;
   out_3091072411791179619[217] = 0.0;
   out_3091072411791179619[218] = 0.0;
   out_3091072411791179619[219] = 0.0;
   out_3091072411791179619[220] = 0.0;
   out_3091072411791179619[221] = 0.0;
   out_3091072411791179619[222] = 0.0;
   out_3091072411791179619[223] = 0.0;
   out_3091072411791179619[224] = 0.0;
   out_3091072411791179619[225] = 0.0;
   out_3091072411791179619[226] = 0.0;
   out_3091072411791179619[227] = 0.0;
   out_3091072411791179619[228] = 1.0;
   out_3091072411791179619[229] = 0.0;
   out_3091072411791179619[230] = 0.0;
   out_3091072411791179619[231] = 0.0;
   out_3091072411791179619[232] = 0.0;
   out_3091072411791179619[233] = 0.0;
   out_3091072411791179619[234] = 0.0;
   out_3091072411791179619[235] = 0.0;
   out_3091072411791179619[236] = 0.0;
   out_3091072411791179619[237] = 0.0;
   out_3091072411791179619[238] = 0.0;
   out_3091072411791179619[239] = 0.0;
   out_3091072411791179619[240] = 0.0;
   out_3091072411791179619[241] = 0.0;
   out_3091072411791179619[242] = 0.0;
   out_3091072411791179619[243] = 0.0;
   out_3091072411791179619[244] = 0.0;
   out_3091072411791179619[245] = 0.0;
   out_3091072411791179619[246] = 0.0;
   out_3091072411791179619[247] = 1.0;
   out_3091072411791179619[248] = 0.0;
   out_3091072411791179619[249] = 0.0;
   out_3091072411791179619[250] = 0.0;
   out_3091072411791179619[251] = 0.0;
   out_3091072411791179619[252] = 0.0;
   out_3091072411791179619[253] = 0.0;
   out_3091072411791179619[254] = 0.0;
   out_3091072411791179619[255] = 0.0;
   out_3091072411791179619[256] = 0.0;
   out_3091072411791179619[257] = 0.0;
   out_3091072411791179619[258] = 0.0;
   out_3091072411791179619[259] = 0.0;
   out_3091072411791179619[260] = 0.0;
   out_3091072411791179619[261] = 0.0;
   out_3091072411791179619[262] = 0.0;
   out_3091072411791179619[263] = 0.0;
   out_3091072411791179619[264] = 0.0;
   out_3091072411791179619[265] = 0.0;
   out_3091072411791179619[266] = 1.0;
   out_3091072411791179619[267] = 0.0;
   out_3091072411791179619[268] = 0.0;
   out_3091072411791179619[269] = 0.0;
   out_3091072411791179619[270] = 0.0;
   out_3091072411791179619[271] = 0.0;
   out_3091072411791179619[272] = 0.0;
   out_3091072411791179619[273] = 0.0;
   out_3091072411791179619[274] = 0.0;
   out_3091072411791179619[275] = 0.0;
   out_3091072411791179619[276] = 0.0;
   out_3091072411791179619[277] = 0.0;
   out_3091072411791179619[278] = 0.0;
   out_3091072411791179619[279] = 0.0;
   out_3091072411791179619[280] = 0.0;
   out_3091072411791179619[281] = 0.0;
   out_3091072411791179619[282] = 0.0;
   out_3091072411791179619[283] = 0.0;
   out_3091072411791179619[284] = 0.0;
   out_3091072411791179619[285] = 1.0;
   out_3091072411791179619[286] = 0.0;
   out_3091072411791179619[287] = 0.0;
   out_3091072411791179619[288] = 0.0;
   out_3091072411791179619[289] = 0.0;
   out_3091072411791179619[290] = 0.0;
   out_3091072411791179619[291] = 0.0;
   out_3091072411791179619[292] = 0.0;
   out_3091072411791179619[293] = 0.0;
   out_3091072411791179619[294] = 0.0;
   out_3091072411791179619[295] = 0.0;
   out_3091072411791179619[296] = 0.0;
   out_3091072411791179619[297] = 0.0;
   out_3091072411791179619[298] = 0.0;
   out_3091072411791179619[299] = 0.0;
   out_3091072411791179619[300] = 0.0;
   out_3091072411791179619[301] = 0.0;
   out_3091072411791179619[302] = 0.0;
   out_3091072411791179619[303] = 0.0;
   out_3091072411791179619[304] = 1.0;
   out_3091072411791179619[305] = 0.0;
   out_3091072411791179619[306] = 0.0;
   out_3091072411791179619[307] = 0.0;
   out_3091072411791179619[308] = 0.0;
   out_3091072411791179619[309] = 0.0;
   out_3091072411791179619[310] = 0.0;
   out_3091072411791179619[311] = 0.0;
   out_3091072411791179619[312] = 0.0;
   out_3091072411791179619[313] = 0.0;
   out_3091072411791179619[314] = 0.0;
   out_3091072411791179619[315] = 0.0;
   out_3091072411791179619[316] = 0.0;
   out_3091072411791179619[317] = 0.0;
   out_3091072411791179619[318] = 0.0;
   out_3091072411791179619[319] = 0.0;
   out_3091072411791179619[320] = 0.0;
   out_3091072411791179619[321] = 0.0;
   out_3091072411791179619[322] = 0.0;
   out_3091072411791179619[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1071594590870993149) {
   out_1071594590870993149[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1071594590870993149[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1071594590870993149[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1071594590870993149[3] = dt*state[12] + state[3];
   out_1071594590870993149[4] = dt*state[13] + state[4];
   out_1071594590870993149[5] = dt*state[14] + state[5];
   out_1071594590870993149[6] = state[6];
   out_1071594590870993149[7] = state[7];
   out_1071594590870993149[8] = state[8];
   out_1071594590870993149[9] = state[9];
   out_1071594590870993149[10] = state[10];
   out_1071594590870993149[11] = state[11];
   out_1071594590870993149[12] = state[12];
   out_1071594590870993149[13] = state[13];
   out_1071594590870993149[14] = state[14];
   out_1071594590870993149[15] = state[15];
   out_1071594590870993149[16] = state[16];
   out_1071594590870993149[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1482318386001013498) {
   out_1482318386001013498[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1482318386001013498[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1482318386001013498[2] = 0;
   out_1482318386001013498[3] = 0;
   out_1482318386001013498[4] = 0;
   out_1482318386001013498[5] = 0;
   out_1482318386001013498[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1482318386001013498[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1482318386001013498[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1482318386001013498[9] = 0;
   out_1482318386001013498[10] = 0;
   out_1482318386001013498[11] = 0;
   out_1482318386001013498[12] = 0;
   out_1482318386001013498[13] = 0;
   out_1482318386001013498[14] = 0;
   out_1482318386001013498[15] = 0;
   out_1482318386001013498[16] = 0;
   out_1482318386001013498[17] = 0;
   out_1482318386001013498[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1482318386001013498[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1482318386001013498[20] = 0;
   out_1482318386001013498[21] = 0;
   out_1482318386001013498[22] = 0;
   out_1482318386001013498[23] = 0;
   out_1482318386001013498[24] = 0;
   out_1482318386001013498[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1482318386001013498[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1482318386001013498[27] = 0;
   out_1482318386001013498[28] = 0;
   out_1482318386001013498[29] = 0;
   out_1482318386001013498[30] = 0;
   out_1482318386001013498[31] = 0;
   out_1482318386001013498[32] = 0;
   out_1482318386001013498[33] = 0;
   out_1482318386001013498[34] = 0;
   out_1482318386001013498[35] = 0;
   out_1482318386001013498[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1482318386001013498[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1482318386001013498[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1482318386001013498[39] = 0;
   out_1482318386001013498[40] = 0;
   out_1482318386001013498[41] = 0;
   out_1482318386001013498[42] = 0;
   out_1482318386001013498[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1482318386001013498[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1482318386001013498[45] = 0;
   out_1482318386001013498[46] = 0;
   out_1482318386001013498[47] = 0;
   out_1482318386001013498[48] = 0;
   out_1482318386001013498[49] = 0;
   out_1482318386001013498[50] = 0;
   out_1482318386001013498[51] = 0;
   out_1482318386001013498[52] = 0;
   out_1482318386001013498[53] = 0;
   out_1482318386001013498[54] = 0;
   out_1482318386001013498[55] = 0;
   out_1482318386001013498[56] = 0;
   out_1482318386001013498[57] = 1;
   out_1482318386001013498[58] = 0;
   out_1482318386001013498[59] = 0;
   out_1482318386001013498[60] = 0;
   out_1482318386001013498[61] = 0;
   out_1482318386001013498[62] = 0;
   out_1482318386001013498[63] = 0;
   out_1482318386001013498[64] = 0;
   out_1482318386001013498[65] = 0;
   out_1482318386001013498[66] = dt;
   out_1482318386001013498[67] = 0;
   out_1482318386001013498[68] = 0;
   out_1482318386001013498[69] = 0;
   out_1482318386001013498[70] = 0;
   out_1482318386001013498[71] = 0;
   out_1482318386001013498[72] = 0;
   out_1482318386001013498[73] = 0;
   out_1482318386001013498[74] = 0;
   out_1482318386001013498[75] = 0;
   out_1482318386001013498[76] = 1;
   out_1482318386001013498[77] = 0;
   out_1482318386001013498[78] = 0;
   out_1482318386001013498[79] = 0;
   out_1482318386001013498[80] = 0;
   out_1482318386001013498[81] = 0;
   out_1482318386001013498[82] = 0;
   out_1482318386001013498[83] = 0;
   out_1482318386001013498[84] = 0;
   out_1482318386001013498[85] = dt;
   out_1482318386001013498[86] = 0;
   out_1482318386001013498[87] = 0;
   out_1482318386001013498[88] = 0;
   out_1482318386001013498[89] = 0;
   out_1482318386001013498[90] = 0;
   out_1482318386001013498[91] = 0;
   out_1482318386001013498[92] = 0;
   out_1482318386001013498[93] = 0;
   out_1482318386001013498[94] = 0;
   out_1482318386001013498[95] = 1;
   out_1482318386001013498[96] = 0;
   out_1482318386001013498[97] = 0;
   out_1482318386001013498[98] = 0;
   out_1482318386001013498[99] = 0;
   out_1482318386001013498[100] = 0;
   out_1482318386001013498[101] = 0;
   out_1482318386001013498[102] = 0;
   out_1482318386001013498[103] = 0;
   out_1482318386001013498[104] = dt;
   out_1482318386001013498[105] = 0;
   out_1482318386001013498[106] = 0;
   out_1482318386001013498[107] = 0;
   out_1482318386001013498[108] = 0;
   out_1482318386001013498[109] = 0;
   out_1482318386001013498[110] = 0;
   out_1482318386001013498[111] = 0;
   out_1482318386001013498[112] = 0;
   out_1482318386001013498[113] = 0;
   out_1482318386001013498[114] = 1;
   out_1482318386001013498[115] = 0;
   out_1482318386001013498[116] = 0;
   out_1482318386001013498[117] = 0;
   out_1482318386001013498[118] = 0;
   out_1482318386001013498[119] = 0;
   out_1482318386001013498[120] = 0;
   out_1482318386001013498[121] = 0;
   out_1482318386001013498[122] = 0;
   out_1482318386001013498[123] = 0;
   out_1482318386001013498[124] = 0;
   out_1482318386001013498[125] = 0;
   out_1482318386001013498[126] = 0;
   out_1482318386001013498[127] = 0;
   out_1482318386001013498[128] = 0;
   out_1482318386001013498[129] = 0;
   out_1482318386001013498[130] = 0;
   out_1482318386001013498[131] = 0;
   out_1482318386001013498[132] = 0;
   out_1482318386001013498[133] = 1;
   out_1482318386001013498[134] = 0;
   out_1482318386001013498[135] = 0;
   out_1482318386001013498[136] = 0;
   out_1482318386001013498[137] = 0;
   out_1482318386001013498[138] = 0;
   out_1482318386001013498[139] = 0;
   out_1482318386001013498[140] = 0;
   out_1482318386001013498[141] = 0;
   out_1482318386001013498[142] = 0;
   out_1482318386001013498[143] = 0;
   out_1482318386001013498[144] = 0;
   out_1482318386001013498[145] = 0;
   out_1482318386001013498[146] = 0;
   out_1482318386001013498[147] = 0;
   out_1482318386001013498[148] = 0;
   out_1482318386001013498[149] = 0;
   out_1482318386001013498[150] = 0;
   out_1482318386001013498[151] = 0;
   out_1482318386001013498[152] = 1;
   out_1482318386001013498[153] = 0;
   out_1482318386001013498[154] = 0;
   out_1482318386001013498[155] = 0;
   out_1482318386001013498[156] = 0;
   out_1482318386001013498[157] = 0;
   out_1482318386001013498[158] = 0;
   out_1482318386001013498[159] = 0;
   out_1482318386001013498[160] = 0;
   out_1482318386001013498[161] = 0;
   out_1482318386001013498[162] = 0;
   out_1482318386001013498[163] = 0;
   out_1482318386001013498[164] = 0;
   out_1482318386001013498[165] = 0;
   out_1482318386001013498[166] = 0;
   out_1482318386001013498[167] = 0;
   out_1482318386001013498[168] = 0;
   out_1482318386001013498[169] = 0;
   out_1482318386001013498[170] = 0;
   out_1482318386001013498[171] = 1;
   out_1482318386001013498[172] = 0;
   out_1482318386001013498[173] = 0;
   out_1482318386001013498[174] = 0;
   out_1482318386001013498[175] = 0;
   out_1482318386001013498[176] = 0;
   out_1482318386001013498[177] = 0;
   out_1482318386001013498[178] = 0;
   out_1482318386001013498[179] = 0;
   out_1482318386001013498[180] = 0;
   out_1482318386001013498[181] = 0;
   out_1482318386001013498[182] = 0;
   out_1482318386001013498[183] = 0;
   out_1482318386001013498[184] = 0;
   out_1482318386001013498[185] = 0;
   out_1482318386001013498[186] = 0;
   out_1482318386001013498[187] = 0;
   out_1482318386001013498[188] = 0;
   out_1482318386001013498[189] = 0;
   out_1482318386001013498[190] = 1;
   out_1482318386001013498[191] = 0;
   out_1482318386001013498[192] = 0;
   out_1482318386001013498[193] = 0;
   out_1482318386001013498[194] = 0;
   out_1482318386001013498[195] = 0;
   out_1482318386001013498[196] = 0;
   out_1482318386001013498[197] = 0;
   out_1482318386001013498[198] = 0;
   out_1482318386001013498[199] = 0;
   out_1482318386001013498[200] = 0;
   out_1482318386001013498[201] = 0;
   out_1482318386001013498[202] = 0;
   out_1482318386001013498[203] = 0;
   out_1482318386001013498[204] = 0;
   out_1482318386001013498[205] = 0;
   out_1482318386001013498[206] = 0;
   out_1482318386001013498[207] = 0;
   out_1482318386001013498[208] = 0;
   out_1482318386001013498[209] = 1;
   out_1482318386001013498[210] = 0;
   out_1482318386001013498[211] = 0;
   out_1482318386001013498[212] = 0;
   out_1482318386001013498[213] = 0;
   out_1482318386001013498[214] = 0;
   out_1482318386001013498[215] = 0;
   out_1482318386001013498[216] = 0;
   out_1482318386001013498[217] = 0;
   out_1482318386001013498[218] = 0;
   out_1482318386001013498[219] = 0;
   out_1482318386001013498[220] = 0;
   out_1482318386001013498[221] = 0;
   out_1482318386001013498[222] = 0;
   out_1482318386001013498[223] = 0;
   out_1482318386001013498[224] = 0;
   out_1482318386001013498[225] = 0;
   out_1482318386001013498[226] = 0;
   out_1482318386001013498[227] = 0;
   out_1482318386001013498[228] = 1;
   out_1482318386001013498[229] = 0;
   out_1482318386001013498[230] = 0;
   out_1482318386001013498[231] = 0;
   out_1482318386001013498[232] = 0;
   out_1482318386001013498[233] = 0;
   out_1482318386001013498[234] = 0;
   out_1482318386001013498[235] = 0;
   out_1482318386001013498[236] = 0;
   out_1482318386001013498[237] = 0;
   out_1482318386001013498[238] = 0;
   out_1482318386001013498[239] = 0;
   out_1482318386001013498[240] = 0;
   out_1482318386001013498[241] = 0;
   out_1482318386001013498[242] = 0;
   out_1482318386001013498[243] = 0;
   out_1482318386001013498[244] = 0;
   out_1482318386001013498[245] = 0;
   out_1482318386001013498[246] = 0;
   out_1482318386001013498[247] = 1;
   out_1482318386001013498[248] = 0;
   out_1482318386001013498[249] = 0;
   out_1482318386001013498[250] = 0;
   out_1482318386001013498[251] = 0;
   out_1482318386001013498[252] = 0;
   out_1482318386001013498[253] = 0;
   out_1482318386001013498[254] = 0;
   out_1482318386001013498[255] = 0;
   out_1482318386001013498[256] = 0;
   out_1482318386001013498[257] = 0;
   out_1482318386001013498[258] = 0;
   out_1482318386001013498[259] = 0;
   out_1482318386001013498[260] = 0;
   out_1482318386001013498[261] = 0;
   out_1482318386001013498[262] = 0;
   out_1482318386001013498[263] = 0;
   out_1482318386001013498[264] = 0;
   out_1482318386001013498[265] = 0;
   out_1482318386001013498[266] = 1;
   out_1482318386001013498[267] = 0;
   out_1482318386001013498[268] = 0;
   out_1482318386001013498[269] = 0;
   out_1482318386001013498[270] = 0;
   out_1482318386001013498[271] = 0;
   out_1482318386001013498[272] = 0;
   out_1482318386001013498[273] = 0;
   out_1482318386001013498[274] = 0;
   out_1482318386001013498[275] = 0;
   out_1482318386001013498[276] = 0;
   out_1482318386001013498[277] = 0;
   out_1482318386001013498[278] = 0;
   out_1482318386001013498[279] = 0;
   out_1482318386001013498[280] = 0;
   out_1482318386001013498[281] = 0;
   out_1482318386001013498[282] = 0;
   out_1482318386001013498[283] = 0;
   out_1482318386001013498[284] = 0;
   out_1482318386001013498[285] = 1;
   out_1482318386001013498[286] = 0;
   out_1482318386001013498[287] = 0;
   out_1482318386001013498[288] = 0;
   out_1482318386001013498[289] = 0;
   out_1482318386001013498[290] = 0;
   out_1482318386001013498[291] = 0;
   out_1482318386001013498[292] = 0;
   out_1482318386001013498[293] = 0;
   out_1482318386001013498[294] = 0;
   out_1482318386001013498[295] = 0;
   out_1482318386001013498[296] = 0;
   out_1482318386001013498[297] = 0;
   out_1482318386001013498[298] = 0;
   out_1482318386001013498[299] = 0;
   out_1482318386001013498[300] = 0;
   out_1482318386001013498[301] = 0;
   out_1482318386001013498[302] = 0;
   out_1482318386001013498[303] = 0;
   out_1482318386001013498[304] = 1;
   out_1482318386001013498[305] = 0;
   out_1482318386001013498[306] = 0;
   out_1482318386001013498[307] = 0;
   out_1482318386001013498[308] = 0;
   out_1482318386001013498[309] = 0;
   out_1482318386001013498[310] = 0;
   out_1482318386001013498[311] = 0;
   out_1482318386001013498[312] = 0;
   out_1482318386001013498[313] = 0;
   out_1482318386001013498[314] = 0;
   out_1482318386001013498[315] = 0;
   out_1482318386001013498[316] = 0;
   out_1482318386001013498[317] = 0;
   out_1482318386001013498[318] = 0;
   out_1482318386001013498[319] = 0;
   out_1482318386001013498[320] = 0;
   out_1482318386001013498[321] = 0;
   out_1482318386001013498[322] = 0;
   out_1482318386001013498[323] = 1;
}
void h_4(double *state, double *unused, double *out_1405613756174050193) {
   out_1405613756174050193[0] = state[6] + state[9];
   out_1405613756174050193[1] = state[7] + state[10];
   out_1405613756174050193[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2061445973247901116) {
   out_2061445973247901116[0] = 0;
   out_2061445973247901116[1] = 0;
   out_2061445973247901116[2] = 0;
   out_2061445973247901116[3] = 0;
   out_2061445973247901116[4] = 0;
   out_2061445973247901116[5] = 0;
   out_2061445973247901116[6] = 1;
   out_2061445973247901116[7] = 0;
   out_2061445973247901116[8] = 0;
   out_2061445973247901116[9] = 1;
   out_2061445973247901116[10] = 0;
   out_2061445973247901116[11] = 0;
   out_2061445973247901116[12] = 0;
   out_2061445973247901116[13] = 0;
   out_2061445973247901116[14] = 0;
   out_2061445973247901116[15] = 0;
   out_2061445973247901116[16] = 0;
   out_2061445973247901116[17] = 0;
   out_2061445973247901116[18] = 0;
   out_2061445973247901116[19] = 0;
   out_2061445973247901116[20] = 0;
   out_2061445973247901116[21] = 0;
   out_2061445973247901116[22] = 0;
   out_2061445973247901116[23] = 0;
   out_2061445973247901116[24] = 0;
   out_2061445973247901116[25] = 1;
   out_2061445973247901116[26] = 0;
   out_2061445973247901116[27] = 0;
   out_2061445973247901116[28] = 1;
   out_2061445973247901116[29] = 0;
   out_2061445973247901116[30] = 0;
   out_2061445973247901116[31] = 0;
   out_2061445973247901116[32] = 0;
   out_2061445973247901116[33] = 0;
   out_2061445973247901116[34] = 0;
   out_2061445973247901116[35] = 0;
   out_2061445973247901116[36] = 0;
   out_2061445973247901116[37] = 0;
   out_2061445973247901116[38] = 0;
   out_2061445973247901116[39] = 0;
   out_2061445973247901116[40] = 0;
   out_2061445973247901116[41] = 0;
   out_2061445973247901116[42] = 0;
   out_2061445973247901116[43] = 0;
   out_2061445973247901116[44] = 1;
   out_2061445973247901116[45] = 0;
   out_2061445973247901116[46] = 0;
   out_2061445973247901116[47] = 1;
   out_2061445973247901116[48] = 0;
   out_2061445973247901116[49] = 0;
   out_2061445973247901116[50] = 0;
   out_2061445973247901116[51] = 0;
   out_2061445973247901116[52] = 0;
   out_2061445973247901116[53] = 0;
}
void h_10(double *state, double *unused, double *out_7299504663005157793) {
   out_7299504663005157793[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7299504663005157793[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7299504663005157793[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_212622638766991425) {
   out_212622638766991425[0] = 0;
   out_212622638766991425[1] = 9.8100000000000005*cos(state[1]);
   out_212622638766991425[2] = 0;
   out_212622638766991425[3] = 0;
   out_212622638766991425[4] = -state[8];
   out_212622638766991425[5] = state[7];
   out_212622638766991425[6] = 0;
   out_212622638766991425[7] = state[5];
   out_212622638766991425[8] = -state[4];
   out_212622638766991425[9] = 0;
   out_212622638766991425[10] = 0;
   out_212622638766991425[11] = 0;
   out_212622638766991425[12] = 1;
   out_212622638766991425[13] = 0;
   out_212622638766991425[14] = 0;
   out_212622638766991425[15] = 1;
   out_212622638766991425[16] = 0;
   out_212622638766991425[17] = 0;
   out_212622638766991425[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_212622638766991425[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_212622638766991425[20] = 0;
   out_212622638766991425[21] = state[8];
   out_212622638766991425[22] = 0;
   out_212622638766991425[23] = -state[6];
   out_212622638766991425[24] = -state[5];
   out_212622638766991425[25] = 0;
   out_212622638766991425[26] = state[3];
   out_212622638766991425[27] = 0;
   out_212622638766991425[28] = 0;
   out_212622638766991425[29] = 0;
   out_212622638766991425[30] = 0;
   out_212622638766991425[31] = 1;
   out_212622638766991425[32] = 0;
   out_212622638766991425[33] = 0;
   out_212622638766991425[34] = 1;
   out_212622638766991425[35] = 0;
   out_212622638766991425[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_212622638766991425[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_212622638766991425[38] = 0;
   out_212622638766991425[39] = -state[7];
   out_212622638766991425[40] = state[6];
   out_212622638766991425[41] = 0;
   out_212622638766991425[42] = state[4];
   out_212622638766991425[43] = -state[3];
   out_212622638766991425[44] = 0;
   out_212622638766991425[45] = 0;
   out_212622638766991425[46] = 0;
   out_212622638766991425[47] = 0;
   out_212622638766991425[48] = 0;
   out_212622638766991425[49] = 0;
   out_212622638766991425[50] = 1;
   out_212622638766991425[51] = 0;
   out_212622638766991425[52] = 0;
   out_212622638766991425[53] = 1;
}
void h_13(double *state, double *unused, double *out_8700349808613667701) {
   out_8700349808613667701[0] = state[3];
   out_8700349808613667701[1] = state[4];
   out_8700349808613667701[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5273719798580233917) {
   out_5273719798580233917[0] = 0;
   out_5273719798580233917[1] = 0;
   out_5273719798580233917[2] = 0;
   out_5273719798580233917[3] = 1;
   out_5273719798580233917[4] = 0;
   out_5273719798580233917[5] = 0;
   out_5273719798580233917[6] = 0;
   out_5273719798580233917[7] = 0;
   out_5273719798580233917[8] = 0;
   out_5273719798580233917[9] = 0;
   out_5273719798580233917[10] = 0;
   out_5273719798580233917[11] = 0;
   out_5273719798580233917[12] = 0;
   out_5273719798580233917[13] = 0;
   out_5273719798580233917[14] = 0;
   out_5273719798580233917[15] = 0;
   out_5273719798580233917[16] = 0;
   out_5273719798580233917[17] = 0;
   out_5273719798580233917[18] = 0;
   out_5273719798580233917[19] = 0;
   out_5273719798580233917[20] = 0;
   out_5273719798580233917[21] = 0;
   out_5273719798580233917[22] = 1;
   out_5273719798580233917[23] = 0;
   out_5273719798580233917[24] = 0;
   out_5273719798580233917[25] = 0;
   out_5273719798580233917[26] = 0;
   out_5273719798580233917[27] = 0;
   out_5273719798580233917[28] = 0;
   out_5273719798580233917[29] = 0;
   out_5273719798580233917[30] = 0;
   out_5273719798580233917[31] = 0;
   out_5273719798580233917[32] = 0;
   out_5273719798580233917[33] = 0;
   out_5273719798580233917[34] = 0;
   out_5273719798580233917[35] = 0;
   out_5273719798580233917[36] = 0;
   out_5273719798580233917[37] = 0;
   out_5273719798580233917[38] = 0;
   out_5273719798580233917[39] = 0;
   out_5273719798580233917[40] = 0;
   out_5273719798580233917[41] = 1;
   out_5273719798580233917[42] = 0;
   out_5273719798580233917[43] = 0;
   out_5273719798580233917[44] = 0;
   out_5273719798580233917[45] = 0;
   out_5273719798580233917[46] = 0;
   out_5273719798580233917[47] = 0;
   out_5273719798580233917[48] = 0;
   out_5273719798580233917[49] = 0;
   out_5273719798580233917[50] = 0;
   out_5273719798580233917[51] = 0;
   out_5273719798580233917[52] = 0;
   out_5273719798580233917[53] = 0;
}
void h_14(double *state, double *unused, double *out_1187658249473560640) {
   out_1187658249473560640[0] = state[6];
   out_1187658249473560640[1] = state[7];
   out_1187658249473560640[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6024686829587385645) {
   out_6024686829587385645[0] = 0;
   out_6024686829587385645[1] = 0;
   out_6024686829587385645[2] = 0;
   out_6024686829587385645[3] = 0;
   out_6024686829587385645[4] = 0;
   out_6024686829587385645[5] = 0;
   out_6024686829587385645[6] = 1;
   out_6024686829587385645[7] = 0;
   out_6024686829587385645[8] = 0;
   out_6024686829587385645[9] = 0;
   out_6024686829587385645[10] = 0;
   out_6024686829587385645[11] = 0;
   out_6024686829587385645[12] = 0;
   out_6024686829587385645[13] = 0;
   out_6024686829587385645[14] = 0;
   out_6024686829587385645[15] = 0;
   out_6024686829587385645[16] = 0;
   out_6024686829587385645[17] = 0;
   out_6024686829587385645[18] = 0;
   out_6024686829587385645[19] = 0;
   out_6024686829587385645[20] = 0;
   out_6024686829587385645[21] = 0;
   out_6024686829587385645[22] = 0;
   out_6024686829587385645[23] = 0;
   out_6024686829587385645[24] = 0;
   out_6024686829587385645[25] = 1;
   out_6024686829587385645[26] = 0;
   out_6024686829587385645[27] = 0;
   out_6024686829587385645[28] = 0;
   out_6024686829587385645[29] = 0;
   out_6024686829587385645[30] = 0;
   out_6024686829587385645[31] = 0;
   out_6024686829587385645[32] = 0;
   out_6024686829587385645[33] = 0;
   out_6024686829587385645[34] = 0;
   out_6024686829587385645[35] = 0;
   out_6024686829587385645[36] = 0;
   out_6024686829587385645[37] = 0;
   out_6024686829587385645[38] = 0;
   out_6024686829587385645[39] = 0;
   out_6024686829587385645[40] = 0;
   out_6024686829587385645[41] = 0;
   out_6024686829587385645[42] = 0;
   out_6024686829587385645[43] = 0;
   out_6024686829587385645[44] = 1;
   out_6024686829587385645[45] = 0;
   out_6024686829587385645[46] = 0;
   out_6024686829587385645[47] = 0;
   out_6024686829587385645[48] = 0;
   out_6024686829587385645[49] = 0;
   out_6024686829587385645[50] = 0;
   out_6024686829587385645[51] = 0;
   out_6024686829587385645[52] = 0;
   out_6024686829587385645[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_7827726718003508843) {
  err_fun(nom_x, delta_x, out_7827726718003508843);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6277241276544534268) {
  inv_err_fun(nom_x, true_x, out_6277241276544534268);
}
void pose_H_mod_fun(double *state, double *out_3091072411791179619) {
  H_mod_fun(state, out_3091072411791179619);
}
void pose_f_fun(double *state, double dt, double *out_1071594590870993149) {
  f_fun(state,  dt, out_1071594590870993149);
}
void pose_F_fun(double *state, double dt, double *out_1482318386001013498) {
  F_fun(state,  dt, out_1482318386001013498);
}
void pose_h_4(double *state, double *unused, double *out_1405613756174050193) {
  h_4(state, unused, out_1405613756174050193);
}
void pose_H_4(double *state, double *unused, double *out_2061445973247901116) {
  H_4(state, unused, out_2061445973247901116);
}
void pose_h_10(double *state, double *unused, double *out_7299504663005157793) {
  h_10(state, unused, out_7299504663005157793);
}
void pose_H_10(double *state, double *unused, double *out_212622638766991425) {
  H_10(state, unused, out_212622638766991425);
}
void pose_h_13(double *state, double *unused, double *out_8700349808613667701) {
  h_13(state, unused, out_8700349808613667701);
}
void pose_H_13(double *state, double *unused, double *out_5273719798580233917) {
  H_13(state, unused, out_5273719798580233917);
}
void pose_h_14(double *state, double *unused, double *out_1187658249473560640) {
  h_14(state, unused, out_1187658249473560640);
}
void pose_H_14(double *state, double *unused, double *out_6024686829587385645) {
  H_14(state, unused, out_6024686829587385645);
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
