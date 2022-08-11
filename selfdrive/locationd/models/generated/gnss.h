#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_702749055828734914);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3284229240671530261);
void gnss_H_mod_fun(double *state, double *out_3877143077739777724);
void gnss_f_fun(double *state, double dt, double *out_1162225855609439512);
void gnss_F_fun(double *state, double dt, double *out_2321032320784632618);
void gnss_h_6(double *state, double *sat_pos, double *out_6213137486689232702);
void gnss_H_6(double *state, double *sat_pos, double *out_6115281955807808636);
void gnss_h_20(double *state, double *sat_pos, double *out_3029045169310487465);
void gnss_H_20(double *state, double *sat_pos, double *out_2605204431652788069);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7592276607120576223);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4738219559717688498);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7592276607120576223);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4738219559717688498);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}