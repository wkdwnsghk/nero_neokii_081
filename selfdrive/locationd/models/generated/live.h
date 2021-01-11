/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2002862878943973367);
void inv_err_fun(double *nom_x, double *true_x, double *out_3073152847115848839);
void H_mod_fun(double *state, double *out_5669238332561590490);
void f_fun(double *state, double dt, double *out_4120904080798699263);
void F_fun(double *state, double dt, double *out_3708296214517378524);
void h_3(double *state, double *unused, double *out_4127740227549778880);
void H_3(double *state, double *unused, double *out_7123714006302879509);
void h_4(double *state, double *unused, double *out_7546759659301489879);
void H_4(double *state, double *unused, double *out_6280951571414933013);
void h_9(double *state, double *unused, double *out_6038863602690813126);
void H_9(double *state, double *unused, double *out_2797257669788905808);
void h_10(double *state, double *unused, double *out_2551030473251362283);
void H_10(double *state, double *unused, double *out_9056737659161614029);
void h_12(double *state, double *unused, double *out_1029359794113398790);
void H_12(double *state, double *unused, double *out_8387507083154142582);
void h_31(double *state, double *unused, double *out_868593504493004049);
void H_31(double *state, double *unused, double *out_8335969382714460727);
void h_32(double *state, double *unused, double *out_1947050830567613664);
void H_32(double *state, double *unused, double *out_8160272969347339242);
void h_13(double *state, double *unused, double *out_2995273580430189044);
void H_13(double *state, double *unused, double *out_6898433951268632293);
void h_14(double *state, double *unused, double *out_6038863602690813126);
void H_14(double *state, double *unused, double *out_2797257669788905808);
void h_19(double *state, double *unused, double *out_8650338528301369882);
void H_19(double *state, double *unused, double *out_6033779564297893501);
#define DIM 23
#define EDIM 22
#define MEDIM 22
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_3 = 3.841459;
void update_3(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814728;
void update_4(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_9 = 7.814728;
void update_9(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_10 = 7.814728;
void update_10(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_12 = 7.814728;
void update_12(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_31 = 7.814728;
void update_31(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_32 = 9.487729;
void update_32(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_13 = 7.814728;
void update_13(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_14 = 7.814728;
void update_14(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_19 = 7.814728;
void update_19(double *, double *, double *, double *, double *);