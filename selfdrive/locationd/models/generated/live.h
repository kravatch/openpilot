#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1710202414824543892);
void live_err_fun(double *nom_x, double *delta_x, double *out_5471801279801416329);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2016136487027356106);
void live_H_mod_fun(double *state, double *out_4326736777899737312);
void live_f_fun(double *state, double dt, double *out_6381004973772240512);
void live_F_fun(double *state, double dt, double *out_7688962136229660845);
void live_h_4(double *state, double *unused, double *out_8910729110262818823);
void live_H_4(double *state, double *unused, double *out_5149317107166622839);
void live_h_9(double *state, double *unused, double *out_2985488469675191362);
void live_H_9(double *state, double *unused, double *out_2137901828097824631);
void live_h_10(double *state, double *unused, double *out_8035615168955792815);
void live_H_10(double *state, double *unused, double *out_6950019072027878977);
void live_h_12(double *state, double *unused, double *out_3672646691063821029);
void live_H_12(double *state, double *unused, double *out_6916168589500195781);
void live_h_35(double *state, double *unused, double *out_4997180468052778618);
void live_H_35(double *state, double *unused, double *out_2615702333190352665);
void live_h_32(double *state, double *unused, double *out_8970276157722010948);
void live_H_32(double *state, double *unused, double *out_5237168111897183397);
void live_h_13(double *state, double *unused, double *out_3153068502881594014);
void live_H_13(double *state, double *unused, double *out_7701039023330007817);
void live_h_14(double *state, double *unused, double *out_2985488469675191362);
void live_H_14(double *state, double *unused, double *out_2137901828097824631);
void live_h_33(double *state, double *unused, double *out_2937579763789009053);
void live_H_33(double *state, double *unused, double *out_5634455447245484522);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}