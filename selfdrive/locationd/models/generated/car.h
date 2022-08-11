#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4329134677672719662);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8663601870348907847);
void car_H_mod_fun(double *state, double *out_666742920972236807);
void car_f_fun(double *state, double dt, double *out_9041872531305286328);
void car_F_fun(double *state, double dt, double *out_62538366870240575);
void car_h_25(double *state, double *unused, double *out_6359509751087296679);
void car_H_25(double *state, double *unused, double *out_7842149085654755664);
void car_h_24(double *state, double *unused, double *out_2086835435358409646);
void car_H_24(double *state, double *unused, double *out_5669499486649256098);
void car_h_30(double *state, double *unused, double *out_2280367725631245205);
void car_H_30(double *state, double *unused, double *out_3687904646563179197);
void car_h_26(double *state, double *unused, double *out_5829116421031607295);
void car_H_26(double *state, double *unused, double *out_4100645766780699440);
void car_h_27(double *state, double *unused, double *out_7085561054961409292);
void car_H_27(double *state, double *unused, double *out_5862667958363604108);
void car_h_29(double *state, double *unused, double *out_3448111526707386942);
void car_H_29(double *state, double *unused, double *out_7576030685233155141);
void car_h_28(double *state, double *unused, double *out_1632275864907283034);
void car_H_28(double *state, double *unused, double *out_5788314371406865901);
void car_h_31(double *state, double *unused, double *out_7810141143716179846);
void car_H_31(double *state, double *unused, double *out_7872795047531716092);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}