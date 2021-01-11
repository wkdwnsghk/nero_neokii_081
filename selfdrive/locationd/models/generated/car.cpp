
extern "C"{

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}

}
extern "C" {
#include <math.h>
/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6172310900975591351) {
   out_6172310900975591351[0] = delta_x[0] + nom_x[0];
   out_6172310900975591351[1] = delta_x[1] + nom_x[1];
   out_6172310900975591351[2] = delta_x[2] + nom_x[2];
   out_6172310900975591351[3] = delta_x[3] + nom_x[3];
   out_6172310900975591351[4] = delta_x[4] + nom_x[4];
   out_6172310900975591351[5] = delta_x[5] + nom_x[5];
   out_6172310900975591351[6] = delta_x[6] + nom_x[6];
   out_6172310900975591351[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5820966726132440401) {
   out_5820966726132440401[0] = -nom_x[0] + true_x[0];
   out_5820966726132440401[1] = -nom_x[1] + true_x[1];
   out_5820966726132440401[2] = -nom_x[2] + true_x[2];
   out_5820966726132440401[3] = -nom_x[3] + true_x[3];
   out_5820966726132440401[4] = -nom_x[4] + true_x[4];
   out_5820966726132440401[5] = -nom_x[5] + true_x[5];
   out_5820966726132440401[6] = -nom_x[6] + true_x[6];
   out_5820966726132440401[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_2717401705716432712) {
   out_2717401705716432712[0] = 1.0;
   out_2717401705716432712[1] = 0.0;
   out_2717401705716432712[2] = 0.0;
   out_2717401705716432712[3] = 0.0;
   out_2717401705716432712[4] = 0.0;
   out_2717401705716432712[5] = 0.0;
   out_2717401705716432712[6] = 0.0;
   out_2717401705716432712[7] = 0.0;
   out_2717401705716432712[8] = 0.0;
   out_2717401705716432712[9] = 1.0;
   out_2717401705716432712[10] = 0.0;
   out_2717401705716432712[11] = 0.0;
   out_2717401705716432712[12] = 0.0;
   out_2717401705716432712[13] = 0.0;
   out_2717401705716432712[14] = 0.0;
   out_2717401705716432712[15] = 0.0;
   out_2717401705716432712[16] = 0.0;
   out_2717401705716432712[17] = 0.0;
   out_2717401705716432712[18] = 1.0;
   out_2717401705716432712[19] = 0.0;
   out_2717401705716432712[20] = 0.0;
   out_2717401705716432712[21] = 0.0;
   out_2717401705716432712[22] = 0.0;
   out_2717401705716432712[23] = 0.0;
   out_2717401705716432712[24] = 0.0;
   out_2717401705716432712[25] = 0.0;
   out_2717401705716432712[26] = 0.0;
   out_2717401705716432712[27] = 1.0;
   out_2717401705716432712[28] = 0.0;
   out_2717401705716432712[29] = 0.0;
   out_2717401705716432712[30] = 0.0;
   out_2717401705716432712[31] = 0.0;
   out_2717401705716432712[32] = 0.0;
   out_2717401705716432712[33] = 0.0;
   out_2717401705716432712[34] = 0.0;
   out_2717401705716432712[35] = 0.0;
   out_2717401705716432712[36] = 1.0;
   out_2717401705716432712[37] = 0.0;
   out_2717401705716432712[38] = 0.0;
   out_2717401705716432712[39] = 0.0;
   out_2717401705716432712[40] = 0.0;
   out_2717401705716432712[41] = 0.0;
   out_2717401705716432712[42] = 0.0;
   out_2717401705716432712[43] = 0.0;
   out_2717401705716432712[44] = 0.0;
   out_2717401705716432712[45] = 1.0;
   out_2717401705716432712[46] = 0.0;
   out_2717401705716432712[47] = 0.0;
   out_2717401705716432712[48] = 0.0;
   out_2717401705716432712[49] = 0.0;
   out_2717401705716432712[50] = 0.0;
   out_2717401705716432712[51] = 0.0;
   out_2717401705716432712[52] = 0.0;
   out_2717401705716432712[53] = 0.0;
   out_2717401705716432712[54] = 1.0;
   out_2717401705716432712[55] = 0.0;
   out_2717401705716432712[56] = 0.0;
   out_2717401705716432712[57] = 0.0;
   out_2717401705716432712[58] = 0.0;
   out_2717401705716432712[59] = 0.0;
   out_2717401705716432712[60] = 0.0;
   out_2717401705716432712[61] = 0.0;
   out_2717401705716432712[62] = 0.0;
   out_2717401705716432712[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_2081527664976926419) {
   out_2081527664976926419[0] = state[0];
   out_2081527664976926419[1] = state[1];
   out_2081527664976926419[2] = state[2];
   out_2081527664976926419[3] = state[3];
   out_2081527664976926419[4] = state[4];
   out_2081527664976926419[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2081527664976926419[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2081527664976926419[7] = state[7];
}
void F_fun(double *state, double dt, double *out_944017234319852679) {
   out_944017234319852679[0] = 1;
   out_944017234319852679[1] = 0;
   out_944017234319852679[2] = 0;
   out_944017234319852679[3] = 0;
   out_944017234319852679[4] = 0;
   out_944017234319852679[5] = 0;
   out_944017234319852679[6] = 0;
   out_944017234319852679[7] = 0;
   out_944017234319852679[8] = 0;
   out_944017234319852679[9] = 1;
   out_944017234319852679[10] = 0;
   out_944017234319852679[11] = 0;
   out_944017234319852679[12] = 0;
   out_944017234319852679[13] = 0;
   out_944017234319852679[14] = 0;
   out_944017234319852679[15] = 0;
   out_944017234319852679[16] = 0;
   out_944017234319852679[17] = 0;
   out_944017234319852679[18] = 1;
   out_944017234319852679[19] = 0;
   out_944017234319852679[20] = 0;
   out_944017234319852679[21] = 0;
   out_944017234319852679[22] = 0;
   out_944017234319852679[23] = 0;
   out_944017234319852679[24] = 0;
   out_944017234319852679[25] = 0;
   out_944017234319852679[26] = 0;
   out_944017234319852679[27] = 1;
   out_944017234319852679[28] = 0;
   out_944017234319852679[29] = 0;
   out_944017234319852679[30] = 0;
   out_944017234319852679[31] = 0;
   out_944017234319852679[32] = 0;
   out_944017234319852679[33] = 0;
   out_944017234319852679[34] = 0;
   out_944017234319852679[35] = 0;
   out_944017234319852679[36] = 1;
   out_944017234319852679[37] = 0;
   out_944017234319852679[38] = 0;
   out_944017234319852679[39] = 0;
   out_944017234319852679[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_944017234319852679[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_944017234319852679[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_944017234319852679[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_944017234319852679[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_944017234319852679[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_944017234319852679[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_944017234319852679[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_944017234319852679[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_944017234319852679[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_944017234319852679[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_944017234319852679[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_944017234319852679[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_944017234319852679[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_944017234319852679[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_944017234319852679[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_944017234319852679[56] = 0;
   out_944017234319852679[57] = 0;
   out_944017234319852679[58] = 0;
   out_944017234319852679[59] = 0;
   out_944017234319852679[60] = 0;
   out_944017234319852679[61] = 0;
   out_944017234319852679[62] = 0;
   out_944017234319852679[63] = 1;
}
void h_25(double *state, double *unused, double *out_7595325382477308992) {
   out_7595325382477308992[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3572581248280151507) {
   out_3572581248280151507[0] = 0;
   out_3572581248280151507[1] = 0;
   out_3572581248280151507[2] = 0;
   out_3572581248280151507[3] = 0;
   out_3572581248280151507[4] = 0;
   out_3572581248280151507[5] = 0;
   out_3572581248280151507[6] = 1;
   out_3572581248280151507[7] = 0;
}
void h_24(double *state, double *unused, double *out_4967085268437632262) {
   out_4967085268437632262[0] = state[4];
   out_4967085268437632262[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3690903359279584538) {
   out_3690903359279584538[0] = 0;
   out_3690903359279584538[1] = 0;
   out_3690903359279584538[2] = 0;
   out_3690903359279584538[3] = 0;
   out_3690903359279584538[4] = 1;
   out_3690903359279584538[5] = 0;
   out_3690903359279584538[6] = 0;
   out_3690903359279584538[7] = 0;
   out_3690903359279584538[8] = 0;
   out_3690903359279584538[9] = 0;
   out_3690903359279584538[10] = 0;
   out_3690903359279584538[11] = 0;
   out_3690903359279584538[12] = 0;
   out_3690903359279584538[13] = 1;
   out_3690903359279584538[14] = 0;
   out_3690903359279584538[15] = 0;
}
void h_30(double *state, double *unused, double *out_8997289840627765960) {
   out_8997289840627765960[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5260263914480216829) {
   out_5260263914480216829[0] = 0;
   out_5260263914480216829[1] = 0;
   out_5260263914480216829[2] = 0;
   out_5260263914480216829[3] = 0;
   out_5260263914480216829[4] = 1;
   out_5260263914480216829[5] = 0;
   out_5260263914480216829[6] = 0;
   out_5260263914480216829[7] = 0;
}
void h_26(double *state, double *unused, double *out_4447780965420940011) {
   out_4447780965420940011[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2266345476325310362) {
   out_2266345476325310362[0] = 0;
   out_2266345476325310362[1] = 0;
   out_2266345476325310362[2] = 0;
   out_2266345476325310362[3] = 0;
   out_2266345476325310362[4] = 0;
   out_2266345476325310362[5] = 0;
   out_2266345476325310362[6] = 0;
   out_2266345476325310362[7] = 1;
}
void h_27(double *state, double *unused, double *out_6569854849988582116) {
   out_6569854849988582116[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3972681926643591517) {
   out_3972681926643591517[0] = 0;
   out_3972681926643591517[1] = 0;
   out_3972681926643591517[2] = 0;
   out_3972681926643591517[3] = 1;
   out_3972681926643591517[4] = 0;
   out_3972681926643591517[5] = 0;
   out_3972681926643591517[6] = 0;
   out_3972681926643591517[7] = 0;
}
void h_29(double *state, double *unused, double *out_1043797413437559003) {
   out_1043797413437559003[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1436465145351384876) {
   out_1436465145351384876[0] = 0;
   out_1436465145351384876[1] = 1;
   out_1436465145351384876[2] = 0;
   out_1436465145351384876[3] = 0;
   out_1436465145351384876[4] = 0;
   out_1436465145351384876[5] = 0;
   out_1436465145351384876[6] = 0;
   out_1436465145351384876[7] = 0;
}
void h_28(double *state, double *unused, double *out_7821817060538049217) {
   out_7821817060538049217[0] = state[5];
   out_7821817060538049217[1] = state[6];
}
void H_28(double *state, double *unused, double *out_3326929118956910328) {
   out_3326929118956910328[0] = 0;
   out_3326929118956910328[1] = 0;
   out_3326929118956910328[2] = 0;
   out_3326929118956910328[3] = 0;
   out_3326929118956910328[4] = 0;
   out_3326929118956910328[5] = 1;
   out_3326929118956910328[6] = 0;
   out_3326929118956910328[7] = 0;
   out_3326929118956910328[8] = 0;
   out_3326929118956910328[9] = 0;
   out_3326929118956910328[10] = 0;
   out_3326929118956910328[11] = 0;
   out_3326929118956910328[12] = 0;
   out_3326929118956910328[13] = 0;
   out_3326929118956910328[14] = 1;
   out_3326929118956910328[15] = 0;
}
}

extern "C"{
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



extern "C"{

      void update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
      }
    
      void update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<2,3,0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
      }
    
      void update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
      }
    
      void update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
      }
    
      void update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
      }
    
      void update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
      }
    
      void update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<2,3,0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
      }
    
}
