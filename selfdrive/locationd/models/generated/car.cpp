#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

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
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4474920018550507185) {
   out_4474920018550507185[0] = delta_x[0] + nom_x[0];
   out_4474920018550507185[1] = delta_x[1] + nom_x[1];
   out_4474920018550507185[2] = delta_x[2] + nom_x[2];
   out_4474920018550507185[3] = delta_x[3] + nom_x[3];
   out_4474920018550507185[4] = delta_x[4] + nom_x[4];
   out_4474920018550507185[5] = delta_x[5] + nom_x[5];
   out_4474920018550507185[6] = delta_x[6] + nom_x[6];
   out_4474920018550507185[7] = delta_x[7] + nom_x[7];
   out_4474920018550507185[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2872096011199435276) {
   out_2872096011199435276[0] = -nom_x[0] + true_x[0];
   out_2872096011199435276[1] = -nom_x[1] + true_x[1];
   out_2872096011199435276[2] = -nom_x[2] + true_x[2];
   out_2872096011199435276[3] = -nom_x[3] + true_x[3];
   out_2872096011199435276[4] = -nom_x[4] + true_x[4];
   out_2872096011199435276[5] = -nom_x[5] + true_x[5];
   out_2872096011199435276[6] = -nom_x[6] + true_x[6];
   out_2872096011199435276[7] = -nom_x[7] + true_x[7];
   out_2872096011199435276[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7026224005864505857) {
   out_7026224005864505857[0] = 1.0;
   out_7026224005864505857[1] = 0.0;
   out_7026224005864505857[2] = 0.0;
   out_7026224005864505857[3] = 0.0;
   out_7026224005864505857[4] = 0.0;
   out_7026224005864505857[5] = 0.0;
   out_7026224005864505857[6] = 0.0;
   out_7026224005864505857[7] = 0.0;
   out_7026224005864505857[8] = 0.0;
   out_7026224005864505857[9] = 0.0;
   out_7026224005864505857[10] = 1.0;
   out_7026224005864505857[11] = 0.0;
   out_7026224005864505857[12] = 0.0;
   out_7026224005864505857[13] = 0.0;
   out_7026224005864505857[14] = 0.0;
   out_7026224005864505857[15] = 0.0;
   out_7026224005864505857[16] = 0.0;
   out_7026224005864505857[17] = 0.0;
   out_7026224005864505857[18] = 0.0;
   out_7026224005864505857[19] = 0.0;
   out_7026224005864505857[20] = 1.0;
   out_7026224005864505857[21] = 0.0;
   out_7026224005864505857[22] = 0.0;
   out_7026224005864505857[23] = 0.0;
   out_7026224005864505857[24] = 0.0;
   out_7026224005864505857[25] = 0.0;
   out_7026224005864505857[26] = 0.0;
   out_7026224005864505857[27] = 0.0;
   out_7026224005864505857[28] = 0.0;
   out_7026224005864505857[29] = 0.0;
   out_7026224005864505857[30] = 1.0;
   out_7026224005864505857[31] = 0.0;
   out_7026224005864505857[32] = 0.0;
   out_7026224005864505857[33] = 0.0;
   out_7026224005864505857[34] = 0.0;
   out_7026224005864505857[35] = 0.0;
   out_7026224005864505857[36] = 0.0;
   out_7026224005864505857[37] = 0.0;
   out_7026224005864505857[38] = 0.0;
   out_7026224005864505857[39] = 0.0;
   out_7026224005864505857[40] = 1.0;
   out_7026224005864505857[41] = 0.0;
   out_7026224005864505857[42] = 0.0;
   out_7026224005864505857[43] = 0.0;
   out_7026224005864505857[44] = 0.0;
   out_7026224005864505857[45] = 0.0;
   out_7026224005864505857[46] = 0.0;
   out_7026224005864505857[47] = 0.0;
   out_7026224005864505857[48] = 0.0;
   out_7026224005864505857[49] = 0.0;
   out_7026224005864505857[50] = 1.0;
   out_7026224005864505857[51] = 0.0;
   out_7026224005864505857[52] = 0.0;
   out_7026224005864505857[53] = 0.0;
   out_7026224005864505857[54] = 0.0;
   out_7026224005864505857[55] = 0.0;
   out_7026224005864505857[56] = 0.0;
   out_7026224005864505857[57] = 0.0;
   out_7026224005864505857[58] = 0.0;
   out_7026224005864505857[59] = 0.0;
   out_7026224005864505857[60] = 1.0;
   out_7026224005864505857[61] = 0.0;
   out_7026224005864505857[62] = 0.0;
   out_7026224005864505857[63] = 0.0;
   out_7026224005864505857[64] = 0.0;
   out_7026224005864505857[65] = 0.0;
   out_7026224005864505857[66] = 0.0;
   out_7026224005864505857[67] = 0.0;
   out_7026224005864505857[68] = 0.0;
   out_7026224005864505857[69] = 0.0;
   out_7026224005864505857[70] = 1.0;
   out_7026224005864505857[71] = 0.0;
   out_7026224005864505857[72] = 0.0;
   out_7026224005864505857[73] = 0.0;
   out_7026224005864505857[74] = 0.0;
   out_7026224005864505857[75] = 0.0;
   out_7026224005864505857[76] = 0.0;
   out_7026224005864505857[77] = 0.0;
   out_7026224005864505857[78] = 0.0;
   out_7026224005864505857[79] = 0.0;
   out_7026224005864505857[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5957602480347530017) {
   out_5957602480347530017[0] = state[0];
   out_5957602480347530017[1] = state[1];
   out_5957602480347530017[2] = state[2];
   out_5957602480347530017[3] = state[3];
   out_5957602480347530017[4] = state[4];
   out_5957602480347530017[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5957602480347530017[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5957602480347530017[7] = state[7];
   out_5957602480347530017[8] = state[8];
}
void F_fun(double *state, double dt, double *out_823148608051821538) {
   out_823148608051821538[0] = 1;
   out_823148608051821538[1] = 0;
   out_823148608051821538[2] = 0;
   out_823148608051821538[3] = 0;
   out_823148608051821538[4] = 0;
   out_823148608051821538[5] = 0;
   out_823148608051821538[6] = 0;
   out_823148608051821538[7] = 0;
   out_823148608051821538[8] = 0;
   out_823148608051821538[9] = 0;
   out_823148608051821538[10] = 1;
   out_823148608051821538[11] = 0;
   out_823148608051821538[12] = 0;
   out_823148608051821538[13] = 0;
   out_823148608051821538[14] = 0;
   out_823148608051821538[15] = 0;
   out_823148608051821538[16] = 0;
   out_823148608051821538[17] = 0;
   out_823148608051821538[18] = 0;
   out_823148608051821538[19] = 0;
   out_823148608051821538[20] = 1;
   out_823148608051821538[21] = 0;
   out_823148608051821538[22] = 0;
   out_823148608051821538[23] = 0;
   out_823148608051821538[24] = 0;
   out_823148608051821538[25] = 0;
   out_823148608051821538[26] = 0;
   out_823148608051821538[27] = 0;
   out_823148608051821538[28] = 0;
   out_823148608051821538[29] = 0;
   out_823148608051821538[30] = 1;
   out_823148608051821538[31] = 0;
   out_823148608051821538[32] = 0;
   out_823148608051821538[33] = 0;
   out_823148608051821538[34] = 0;
   out_823148608051821538[35] = 0;
   out_823148608051821538[36] = 0;
   out_823148608051821538[37] = 0;
   out_823148608051821538[38] = 0;
   out_823148608051821538[39] = 0;
   out_823148608051821538[40] = 1;
   out_823148608051821538[41] = 0;
   out_823148608051821538[42] = 0;
   out_823148608051821538[43] = 0;
   out_823148608051821538[44] = 0;
   out_823148608051821538[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_823148608051821538[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_823148608051821538[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_823148608051821538[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_823148608051821538[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_823148608051821538[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_823148608051821538[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_823148608051821538[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_823148608051821538[53] = -9.8000000000000007*dt;
   out_823148608051821538[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_823148608051821538[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_823148608051821538[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_823148608051821538[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_823148608051821538[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_823148608051821538[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_823148608051821538[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_823148608051821538[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_823148608051821538[62] = 0;
   out_823148608051821538[63] = 0;
   out_823148608051821538[64] = 0;
   out_823148608051821538[65] = 0;
   out_823148608051821538[66] = 0;
   out_823148608051821538[67] = 0;
   out_823148608051821538[68] = 0;
   out_823148608051821538[69] = 0;
   out_823148608051821538[70] = 1;
   out_823148608051821538[71] = 0;
   out_823148608051821538[72] = 0;
   out_823148608051821538[73] = 0;
   out_823148608051821538[74] = 0;
   out_823148608051821538[75] = 0;
   out_823148608051821538[76] = 0;
   out_823148608051821538[77] = 0;
   out_823148608051821538[78] = 0;
   out_823148608051821538[79] = 0;
   out_823148608051821538[80] = 1;
}
void h_25(double *state, double *unused, double *out_4948611342472557250) {
   out_4948611342472557250[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2122532180679841017) {
   out_2122532180679841017[0] = 0;
   out_2122532180679841017[1] = 0;
   out_2122532180679841017[2] = 0;
   out_2122532180679841017[3] = 0;
   out_2122532180679841017[4] = 0;
   out_2122532180679841017[5] = 0;
   out_2122532180679841017[6] = 1;
   out_2122532180679841017[7] = 0;
   out_2122532180679841017[8] = 0;
}
void h_24(double *state, double *unused, double *out_7882170571962217135) {
   out_7882170571962217135[0] = state[4];
   out_7882170571962217135[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1277535571896145618) {
   out_1277535571896145618[0] = 0;
   out_1277535571896145618[1] = 0;
   out_1277535571896145618[2] = 0;
   out_1277535571896145618[3] = 0;
   out_1277535571896145618[4] = 1;
   out_1277535571896145618[5] = 0;
   out_1277535571896145618[6] = 0;
   out_1277535571896145618[7] = 0;
   out_1277535571896145618[8] = 0;
   out_1277535571896145618[9] = 0;
   out_1277535571896145618[10] = 0;
   out_1277535571896145618[11] = 0;
   out_1277535571896145618[12] = 0;
   out_1277535571896145618[13] = 0;
   out_1277535571896145618[14] = 1;
   out_1277535571896145618[15] = 0;
   out_1277535571896145618[16] = 0;
   out_1277535571896145618[17] = 0;
}
void h_30(double *state, double *unused, double *out_5223805404757063139) {
   out_5223805404757063139[0] = state[4];
}
void H_30(double *state, double *unused, double *out_395800777827407610) {
   out_395800777827407610[0] = 0;
   out_395800777827407610[1] = 0;
   out_395800777827407610[2] = 0;
   out_395800777827407610[3] = 0;
   out_395800777827407610[4] = 1;
   out_395800777827407610[5] = 0;
   out_395800777827407610[6] = 0;
   out_395800777827407610[7] = 0;
   out_395800777827407610[8] = 0;
}
void h_26(double *state, double *unused, double *out_3743877300182716179) {
   out_3743877300182716179[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5864035499553897241) {
   out_5864035499553897241[0] = 0;
   out_5864035499553897241[1] = 0;
   out_5864035499553897241[2] = 0;
   out_5864035499553897241[3] = 0;
   out_5864035499553897241[4] = 0;
   out_5864035499553897241[5] = 0;
   out_5864035499553897241[6] = 0;
   out_5864035499553897241[7] = 1;
   out_5864035499553897241[8] = 0;
}
void h_27(double *state, double *unused, double *out_7656520890721244418) {
   out_7656520890721244418[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2619394849011350827) {
   out_2619394849011350827[0] = 0;
   out_2619394849011350827[1] = 0;
   out_2619394849011350827[2] = 0;
   out_2619394849011350827[3] = 1;
   out_2619394849011350827[4] = 0;
   out_2619394849011350827[5] = 0;
   out_2619394849011350827[6] = 0;
   out_2619394849011350827[7] = 0;
   out_2619394849011350827[8] = 0;
}
void h_29(double *state, double *unused, double *out_3181851414887163514) {
   out_3181851414887163514[0] = state[1];
}
void H_29(double *state, double *unused, double *out_906032122141799794) {
   out_906032122141799794[0] = 0;
   out_906032122141799794[1] = 1;
   out_906032122141799794[2] = 0;
   out_906032122141799794[3] = 0;
   out_906032122141799794[4] = 0;
   out_906032122141799794[5] = 0;
   out_906032122141799794[6] = 0;
   out_906032122141799794[7] = 0;
   out_906032122141799794[8] = 0;
}
void h_28(double *state, double *unused, double *out_8942129508512501858) {
   out_8942129508512501858[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4176366894927730780) {
   out_4176366894927730780[0] = 1;
   out_4176366894927730780[1] = 0;
   out_4176366894927730780[2] = 0;
   out_4176366894927730780[3] = 0;
   out_4176366894927730780[4] = 0;
   out_4176366894927730780[5] = 0;
   out_4176366894927730780[6] = 0;
   out_4176366894927730780[7] = 0;
   out_4176366894927730780[8] = 0;
}
void h_31(double *state, double *unused, double *out_5001472049973254374) {
   out_5001472049973254374[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2091886218802880589) {
   out_2091886218802880589[0] = 0;
   out_2091886218802880589[1] = 0;
   out_2091886218802880589[2] = 0;
   out_2091886218802880589[3] = 0;
   out_2091886218802880589[4] = 0;
   out_2091886218802880589[5] = 0;
   out_2091886218802880589[6] = 0;
   out_2091886218802880589[7] = 0;
   out_2091886218802880589[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_4474920018550507185) {
  err_fun(nom_x, delta_x, out_4474920018550507185);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2872096011199435276) {
  inv_err_fun(nom_x, true_x, out_2872096011199435276);
}
void car_H_mod_fun(double *state, double *out_7026224005864505857) {
  H_mod_fun(state, out_7026224005864505857);
}
void car_f_fun(double *state, double dt, double *out_5957602480347530017) {
  f_fun(state,  dt, out_5957602480347530017);
}
void car_F_fun(double *state, double dt, double *out_823148608051821538) {
  F_fun(state,  dt, out_823148608051821538);
}
void car_h_25(double *state, double *unused, double *out_4948611342472557250) {
  h_25(state, unused, out_4948611342472557250);
}
void car_H_25(double *state, double *unused, double *out_2122532180679841017) {
  H_25(state, unused, out_2122532180679841017);
}
void car_h_24(double *state, double *unused, double *out_7882170571962217135) {
  h_24(state, unused, out_7882170571962217135);
}
void car_H_24(double *state, double *unused, double *out_1277535571896145618) {
  H_24(state, unused, out_1277535571896145618);
}
void car_h_30(double *state, double *unused, double *out_5223805404757063139) {
  h_30(state, unused, out_5223805404757063139);
}
void car_H_30(double *state, double *unused, double *out_395800777827407610) {
  H_30(state, unused, out_395800777827407610);
}
void car_h_26(double *state, double *unused, double *out_3743877300182716179) {
  h_26(state, unused, out_3743877300182716179);
}
void car_H_26(double *state, double *unused, double *out_5864035499553897241) {
  H_26(state, unused, out_5864035499553897241);
}
void car_h_27(double *state, double *unused, double *out_7656520890721244418) {
  h_27(state, unused, out_7656520890721244418);
}
void car_H_27(double *state, double *unused, double *out_2619394849011350827) {
  H_27(state, unused, out_2619394849011350827);
}
void car_h_29(double *state, double *unused, double *out_3181851414887163514) {
  h_29(state, unused, out_3181851414887163514);
}
void car_H_29(double *state, double *unused, double *out_906032122141799794) {
  H_29(state, unused, out_906032122141799794);
}
void car_h_28(double *state, double *unused, double *out_8942129508512501858) {
  h_28(state, unused, out_8942129508512501858);
}
void car_H_28(double *state, double *unused, double *out_4176366894927730780) {
  H_28(state, unused, out_4176366894927730780);
}
void car_h_31(double *state, double *unused, double *out_5001472049973254374) {
  h_31(state, unused, out_5001472049973254374);
}
void car_H_31(double *state, double *unused, double *out_2091886218802880589) {
  H_31(state, unused, out_2091886218802880589);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
