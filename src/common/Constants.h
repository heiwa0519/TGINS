//
// Created by hw on 8/27/22.
//

#ifndef FUSING_CONSTANTS_H
#define FUSING_CONSTANTS_H

/*imu related*/

#define D2R             (M_PI/180.0)            /* deg to rad */
#define R2D             (180.0/M_PI)            /* rad to deg */
#define DPH2RPS         4.84813681109536e-06    /* deg/h to rad/s */
#define RPS2DPH         206264.806247096        /* rad/s to deg/h */
#define DPSH2RPSS       2.90888208665722e-4     /* deg/sqrt(h) to rad/sqrt(s) */
#define RPSS2DPSH       3437.74677078493        /* rad/sqrt(s) to deg/sqrt(h) */
#define DPS2DPH         3600.0
#define DPHPSHZ2RPSS    4.84813681109536e-06    /* deg/h/sqrt(Hz) to rad/sqrt(s) */
#define RPSS2DPHPSHZ    206264.806247096        /* rad/sqrt(s) to deg/h/sqrt(Hz) */
#define G2MPS2          9.7803267714            /* g0 to m/s^2 */
#define MPS22G          0.101971621297793       /* m/s^2 to g0 */
#define MG2MPS2         9.7803267714e-3         /* millo-g0(mg) to m/s^2 */
#define UG2MPS2         9.7803267714e-6
#define MPS22MG         101.971621297793        /* m/s^2 to milli-g0(mg) */
#define MPS22UG         101971.621297793        /* m/s^2 to micro-g0(ug) */
#define MPSPSH2MPSSS     0.01666666666667
#define MPSSS2MPSPSH     60.0
#define GAL2MPS2        0.01                    /* gal to m/s^2 */
#define MPS22GAL        100.0                   /* m/s^2 to gal */
#define MGAL2MPS2       1E-5                    /* mGal to m/s^2 */
#define MPS22MGAL       100000                  /* m/s^2 to mGal */
#define SCALE2PPM       1E6
#define PPM2SCALE       1E-6

#define WGS84_OMGE      7.2921151467E-5
#define WGS84_RE        6378137.0
#define WGS84_RP        6356752.31425
#define WGS84_MU        3.986004418E14
#define WGS84_J2        1.082627E-3
#define WGS84_FE        (1.0/298.257223563)
#define WGS84_ECC       0.081819221455523
#define WGS84_ECC2      0.006694384999588
#define GRAVITY0        9.7803267714

/*gnss related*/
#define JD2MJD      2400000.5           /* JD to MJD */

#define MAXLEAPS    64                  /* max number of leap seconds table */


#endif //FUSING_CONSTANTS_H
