#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_702749055828734914) {
   out_702749055828734914[0] = delta_x[0] + nom_x[0];
   out_702749055828734914[1] = delta_x[1] + nom_x[1];
   out_702749055828734914[2] = delta_x[2] + nom_x[2];
   out_702749055828734914[3] = delta_x[3] + nom_x[3];
   out_702749055828734914[4] = delta_x[4] + nom_x[4];
   out_702749055828734914[5] = delta_x[5] + nom_x[5];
   out_702749055828734914[6] = delta_x[6] + nom_x[6];
   out_702749055828734914[7] = delta_x[7] + nom_x[7];
   out_702749055828734914[8] = delta_x[8] + nom_x[8];
   out_702749055828734914[9] = delta_x[9] + nom_x[9];
   out_702749055828734914[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3284229240671530261) {
   out_3284229240671530261[0] = -nom_x[0] + true_x[0];
   out_3284229240671530261[1] = -nom_x[1] + true_x[1];
   out_3284229240671530261[2] = -nom_x[2] + true_x[2];
   out_3284229240671530261[3] = -nom_x[3] + true_x[3];
   out_3284229240671530261[4] = -nom_x[4] + true_x[4];
   out_3284229240671530261[5] = -nom_x[5] + true_x[5];
   out_3284229240671530261[6] = -nom_x[6] + true_x[6];
   out_3284229240671530261[7] = -nom_x[7] + true_x[7];
   out_3284229240671530261[8] = -nom_x[8] + true_x[8];
   out_3284229240671530261[9] = -nom_x[9] + true_x[9];
   out_3284229240671530261[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3877143077739777724) {
   out_3877143077739777724[0] = 1.0;
   out_3877143077739777724[1] = 0;
   out_3877143077739777724[2] = 0;
   out_3877143077739777724[3] = 0;
   out_3877143077739777724[4] = 0;
   out_3877143077739777724[5] = 0;
   out_3877143077739777724[6] = 0;
   out_3877143077739777724[7] = 0;
   out_3877143077739777724[8] = 0;
   out_3877143077739777724[9] = 0;
   out_3877143077739777724[10] = 0;
   out_3877143077739777724[11] = 0;
   out_3877143077739777724[12] = 1.0;
   out_3877143077739777724[13] = 0;
   out_3877143077739777724[14] = 0;
   out_3877143077739777724[15] = 0;
   out_3877143077739777724[16] = 0;
   out_3877143077739777724[17] = 0;
   out_3877143077739777724[18] = 0;
   out_3877143077739777724[19] = 0;
   out_3877143077739777724[20] = 0;
   out_3877143077739777724[21] = 0;
   out_3877143077739777724[22] = 0;
   out_3877143077739777724[23] = 0;
   out_3877143077739777724[24] = 1.0;
   out_3877143077739777724[25] = 0;
   out_3877143077739777724[26] = 0;
   out_3877143077739777724[27] = 0;
   out_3877143077739777724[28] = 0;
   out_3877143077739777724[29] = 0;
   out_3877143077739777724[30] = 0;
   out_3877143077739777724[31] = 0;
   out_3877143077739777724[32] = 0;
   out_3877143077739777724[33] = 0;
   out_3877143077739777724[34] = 0;
   out_3877143077739777724[35] = 0;
   out_3877143077739777724[36] = 1.0;
   out_3877143077739777724[37] = 0;
   out_3877143077739777724[38] = 0;
   out_3877143077739777724[39] = 0;
   out_3877143077739777724[40] = 0;
   out_3877143077739777724[41] = 0;
   out_3877143077739777724[42] = 0;
   out_3877143077739777724[43] = 0;
   out_3877143077739777724[44] = 0;
   out_3877143077739777724[45] = 0;
   out_3877143077739777724[46] = 0;
   out_3877143077739777724[47] = 0;
   out_3877143077739777724[48] = 1.0;
   out_3877143077739777724[49] = 0;
   out_3877143077739777724[50] = 0;
   out_3877143077739777724[51] = 0;
   out_3877143077739777724[52] = 0;
   out_3877143077739777724[53] = 0;
   out_3877143077739777724[54] = 0;
   out_3877143077739777724[55] = 0;
   out_3877143077739777724[56] = 0;
   out_3877143077739777724[57] = 0;
   out_3877143077739777724[58] = 0;
   out_3877143077739777724[59] = 0;
   out_3877143077739777724[60] = 1.0;
   out_3877143077739777724[61] = 0;
   out_3877143077739777724[62] = 0;
   out_3877143077739777724[63] = 0;
   out_3877143077739777724[64] = 0;
   out_3877143077739777724[65] = 0;
   out_3877143077739777724[66] = 0;
   out_3877143077739777724[67] = 0;
   out_3877143077739777724[68] = 0;
   out_3877143077739777724[69] = 0;
   out_3877143077739777724[70] = 0;
   out_3877143077739777724[71] = 0;
   out_3877143077739777724[72] = 1.0;
   out_3877143077739777724[73] = 0;
   out_3877143077739777724[74] = 0;
   out_3877143077739777724[75] = 0;
   out_3877143077739777724[76] = 0;
   out_3877143077739777724[77] = 0;
   out_3877143077739777724[78] = 0;
   out_3877143077739777724[79] = 0;
   out_3877143077739777724[80] = 0;
   out_3877143077739777724[81] = 0;
   out_3877143077739777724[82] = 0;
   out_3877143077739777724[83] = 0;
   out_3877143077739777724[84] = 1.0;
   out_3877143077739777724[85] = 0;
   out_3877143077739777724[86] = 0;
   out_3877143077739777724[87] = 0;
   out_3877143077739777724[88] = 0;
   out_3877143077739777724[89] = 0;
   out_3877143077739777724[90] = 0;
   out_3877143077739777724[91] = 0;
   out_3877143077739777724[92] = 0;
   out_3877143077739777724[93] = 0;
   out_3877143077739777724[94] = 0;
   out_3877143077739777724[95] = 0;
   out_3877143077739777724[96] = 1.0;
   out_3877143077739777724[97] = 0;
   out_3877143077739777724[98] = 0;
   out_3877143077739777724[99] = 0;
   out_3877143077739777724[100] = 0;
   out_3877143077739777724[101] = 0;
   out_3877143077739777724[102] = 0;
   out_3877143077739777724[103] = 0;
   out_3877143077739777724[104] = 0;
   out_3877143077739777724[105] = 0;
   out_3877143077739777724[106] = 0;
   out_3877143077739777724[107] = 0;
   out_3877143077739777724[108] = 1.0;
   out_3877143077739777724[109] = 0;
   out_3877143077739777724[110] = 0;
   out_3877143077739777724[111] = 0;
   out_3877143077739777724[112] = 0;
   out_3877143077739777724[113] = 0;
   out_3877143077739777724[114] = 0;
   out_3877143077739777724[115] = 0;
   out_3877143077739777724[116] = 0;
   out_3877143077739777724[117] = 0;
   out_3877143077739777724[118] = 0;
   out_3877143077739777724[119] = 0;
   out_3877143077739777724[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_1162225855609439512) {
   out_1162225855609439512[0] = dt*state[3] + state[0];
   out_1162225855609439512[1] = dt*state[4] + state[1];
   out_1162225855609439512[2] = dt*state[5] + state[2];
   out_1162225855609439512[3] = state[3];
   out_1162225855609439512[4] = state[4];
   out_1162225855609439512[5] = state[5];
   out_1162225855609439512[6] = dt*state[7] + state[6];
   out_1162225855609439512[7] = dt*state[8] + state[7];
   out_1162225855609439512[8] = state[8];
   out_1162225855609439512[9] = state[9];
   out_1162225855609439512[10] = state[10];
}
void F_fun(double *state, double dt, double *out_2321032320784632618) {
   out_2321032320784632618[0] = 1;
   out_2321032320784632618[1] = 0;
   out_2321032320784632618[2] = 0;
   out_2321032320784632618[3] = dt;
   out_2321032320784632618[4] = 0;
   out_2321032320784632618[5] = 0;
   out_2321032320784632618[6] = 0;
   out_2321032320784632618[7] = 0;
   out_2321032320784632618[8] = 0;
   out_2321032320784632618[9] = 0;
   out_2321032320784632618[10] = 0;
   out_2321032320784632618[11] = 0;
   out_2321032320784632618[12] = 1;
   out_2321032320784632618[13] = 0;
   out_2321032320784632618[14] = 0;
   out_2321032320784632618[15] = dt;
   out_2321032320784632618[16] = 0;
   out_2321032320784632618[17] = 0;
   out_2321032320784632618[18] = 0;
   out_2321032320784632618[19] = 0;
   out_2321032320784632618[20] = 0;
   out_2321032320784632618[21] = 0;
   out_2321032320784632618[22] = 0;
   out_2321032320784632618[23] = 0;
   out_2321032320784632618[24] = 1;
   out_2321032320784632618[25] = 0;
   out_2321032320784632618[26] = 0;
   out_2321032320784632618[27] = dt;
   out_2321032320784632618[28] = 0;
   out_2321032320784632618[29] = 0;
   out_2321032320784632618[30] = 0;
   out_2321032320784632618[31] = 0;
   out_2321032320784632618[32] = 0;
   out_2321032320784632618[33] = 0;
   out_2321032320784632618[34] = 0;
   out_2321032320784632618[35] = 0;
   out_2321032320784632618[36] = 1;
   out_2321032320784632618[37] = 0;
   out_2321032320784632618[38] = 0;
   out_2321032320784632618[39] = 0;
   out_2321032320784632618[40] = 0;
   out_2321032320784632618[41] = 0;
   out_2321032320784632618[42] = 0;
   out_2321032320784632618[43] = 0;
   out_2321032320784632618[44] = 0;
   out_2321032320784632618[45] = 0;
   out_2321032320784632618[46] = 0;
   out_2321032320784632618[47] = 0;
   out_2321032320784632618[48] = 1;
   out_2321032320784632618[49] = 0;
   out_2321032320784632618[50] = 0;
   out_2321032320784632618[51] = 0;
   out_2321032320784632618[52] = 0;
   out_2321032320784632618[53] = 0;
   out_2321032320784632618[54] = 0;
   out_2321032320784632618[55] = 0;
   out_2321032320784632618[56] = 0;
   out_2321032320784632618[57] = 0;
   out_2321032320784632618[58] = 0;
   out_2321032320784632618[59] = 0;
   out_2321032320784632618[60] = 1;
   out_2321032320784632618[61] = 0;
   out_2321032320784632618[62] = 0;
   out_2321032320784632618[63] = 0;
   out_2321032320784632618[64] = 0;
   out_2321032320784632618[65] = 0;
   out_2321032320784632618[66] = 0;
   out_2321032320784632618[67] = 0;
   out_2321032320784632618[68] = 0;
   out_2321032320784632618[69] = 0;
   out_2321032320784632618[70] = 0;
   out_2321032320784632618[71] = 0;
   out_2321032320784632618[72] = 1;
   out_2321032320784632618[73] = dt;
   out_2321032320784632618[74] = 0;
   out_2321032320784632618[75] = 0;
   out_2321032320784632618[76] = 0;
   out_2321032320784632618[77] = 0;
   out_2321032320784632618[78] = 0;
   out_2321032320784632618[79] = 0;
   out_2321032320784632618[80] = 0;
   out_2321032320784632618[81] = 0;
   out_2321032320784632618[82] = 0;
   out_2321032320784632618[83] = 0;
   out_2321032320784632618[84] = 1;
   out_2321032320784632618[85] = dt;
   out_2321032320784632618[86] = 0;
   out_2321032320784632618[87] = 0;
   out_2321032320784632618[88] = 0;
   out_2321032320784632618[89] = 0;
   out_2321032320784632618[90] = 0;
   out_2321032320784632618[91] = 0;
   out_2321032320784632618[92] = 0;
   out_2321032320784632618[93] = 0;
   out_2321032320784632618[94] = 0;
   out_2321032320784632618[95] = 0;
   out_2321032320784632618[96] = 1;
   out_2321032320784632618[97] = 0;
   out_2321032320784632618[98] = 0;
   out_2321032320784632618[99] = 0;
   out_2321032320784632618[100] = 0;
   out_2321032320784632618[101] = 0;
   out_2321032320784632618[102] = 0;
   out_2321032320784632618[103] = 0;
   out_2321032320784632618[104] = 0;
   out_2321032320784632618[105] = 0;
   out_2321032320784632618[106] = 0;
   out_2321032320784632618[107] = 0;
   out_2321032320784632618[108] = 1;
   out_2321032320784632618[109] = 0;
   out_2321032320784632618[110] = 0;
   out_2321032320784632618[111] = 0;
   out_2321032320784632618[112] = 0;
   out_2321032320784632618[113] = 0;
   out_2321032320784632618[114] = 0;
   out_2321032320784632618[115] = 0;
   out_2321032320784632618[116] = 0;
   out_2321032320784632618[117] = 0;
   out_2321032320784632618[118] = 0;
   out_2321032320784632618[119] = 0;
   out_2321032320784632618[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_6213137486689232702) {
   out_6213137486689232702[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_6115281955807808636) {
   out_6115281955807808636[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6115281955807808636[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6115281955807808636[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6115281955807808636[3] = 0;
   out_6115281955807808636[4] = 0;
   out_6115281955807808636[5] = 0;
   out_6115281955807808636[6] = 1;
   out_6115281955807808636[7] = 0;
   out_6115281955807808636[8] = 0;
   out_6115281955807808636[9] = 0;
   out_6115281955807808636[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_3029045169310487465) {
   out_3029045169310487465[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_2605204431652788069) {
   out_2605204431652788069[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2605204431652788069[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2605204431652788069[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2605204431652788069[3] = 0;
   out_2605204431652788069[4] = 0;
   out_2605204431652788069[5] = 0;
   out_2605204431652788069[6] = 1;
   out_2605204431652788069[7] = 0;
   out_2605204431652788069[8] = 0;
   out_2605204431652788069[9] = 1;
   out_2605204431652788069[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_7592276607120576223) {
   out_7592276607120576223[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_4738219559717688498) {
   out_4738219559717688498[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[6] = 0;
   out_4738219559717688498[7] = 1;
   out_4738219559717688498[8] = 0;
   out_4738219559717688498[9] = 0;
   out_4738219559717688498[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_7592276607120576223) {
   out_7592276607120576223[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_4738219559717688498) {
   out_4738219559717688498[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4738219559717688498[6] = 0;
   out_4738219559717688498[7] = 1;
   out_4738219559717688498[8] = 0;
   out_4738219559717688498[9] = 0;
   out_4738219559717688498[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_702749055828734914) {
  err_fun(nom_x, delta_x, out_702749055828734914);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3284229240671530261) {
  inv_err_fun(nom_x, true_x, out_3284229240671530261);
}
void gnss_H_mod_fun(double *state, double *out_3877143077739777724) {
  H_mod_fun(state, out_3877143077739777724);
}
void gnss_f_fun(double *state, double dt, double *out_1162225855609439512) {
  f_fun(state,  dt, out_1162225855609439512);
}
void gnss_F_fun(double *state, double dt, double *out_2321032320784632618) {
  F_fun(state,  dt, out_2321032320784632618);
}
void gnss_h_6(double *state, double *sat_pos, double *out_6213137486689232702) {
  h_6(state, sat_pos, out_6213137486689232702);
}
void gnss_H_6(double *state, double *sat_pos, double *out_6115281955807808636) {
  H_6(state, sat_pos, out_6115281955807808636);
}
void gnss_h_20(double *state, double *sat_pos, double *out_3029045169310487465) {
  h_20(state, sat_pos, out_3029045169310487465);
}
void gnss_H_20(double *state, double *sat_pos, double *out_2605204431652788069) {
  H_20(state, sat_pos, out_2605204431652788069);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7592276607120576223) {
  h_7(state, sat_pos_vel, out_7592276607120576223);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4738219559717688498) {
  H_7(state, sat_pos_vel, out_4738219559717688498);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7592276607120576223) {
  h_21(state, sat_pos_vel, out_7592276607120576223);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4738219559717688498) {
  H_21(state, sat_pos_vel, out_4738219559717688498);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
