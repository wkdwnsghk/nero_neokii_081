/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6172310900975591351);
void inv_err_fun(double *nom_x, double *true_x, double *out_5820966726132440401);
void H_mod_fun(double *state, double *out_2717401705716432712);
void f_fun(double *state, double dt, double *out_2081527664976926419);
void F_fun(double *state, double dt, double *out_944017234319852679);
void h_25(double *state, double *unused, double *out_7595325382477308992);
void H_25(double *state, double *unused, double *out_3572581248280151507);
void h_24(double *state, double *unused, double *out_4967085268437632262);
void H_24(double *state, double *unused, double *out_3690903359279584538);
void h_30(double *state, double *unused, double *out_8997289840627765960);
void H_30(double *state, double *unused, double *out_5260263914480216829);
void h_26(double *state, double *unused, double *out_4447780965420940011);
void H_26(double *state, double *unused, double *out_2266345476325310362);
void h_27(double *state, double *unused, double *out_6569854849988582116);
void H_27(double *state, double *unused, double *out_3972681926643591517);
void h_29(double *state, double *unused, double *out_1043797413437559003);
void H_29(double *state, double *unused, double *out_1436465145351384876);
void h_28(double *state, double *unused, double *out_7821817060538049217);
void H_28(double *state, double *unused, double *out_3326929118956910328);
#define DIM 8
#define EDIM 8
#define MEDIM 8
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_25 = 3.841459;
void update_25(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_24 = 5.991465;
void update_24(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_30 = 3.841459;
void update_30(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_26 = 3.841459;
void update_26(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_27 = 3.841459;
void update_27(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_29 = 3.841459;
void update_29(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_28 = 5.991465;
void update_28(double *, double *, double *, double *, double *);
void set_mass(double x);

void set_rotational_inertia(double x);

void set_center_to_front(double x);

void set_center_to_rear(double x);

void set_stiffness_front(double x);

void set_stiffness_rear(double x);
