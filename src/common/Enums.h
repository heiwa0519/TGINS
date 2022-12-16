//
// Created by hw on 8/27/22.
//

#ifndef FUSING_ENUMS_H
#define FUSING_ENUMS_H

#define BETTER_ENUMS_DEFAULT_CONSTRUCTOR(Enum) \
public:                                        \
	Enum() = default;
#include "../../3rdparty/enum.h"

BETTER_ENUM(E_SwitchOpt, short int,
            OFF,
            ON)

BETTER_ENUM(E_FileType, short int,
            TEXT,
            BINARY)

BETTER_ENUM(E_Estimator, short int,
            FKF,
            BKF,
            FBS,
            RTS,
            FGO)

BETTER_ENUM(E_ImuFmt, short int,
            DEFAULT,
            NOVATEL,
            M39,
            PSINS,
            GINS,
            A15)

BETTER_ENUM(E_AttDefination, short int,
            NED_FRD,
            ENU_RFU)

BETTER_ENUM(E_InsNavCoord, short int,
            ECEF,
            LLH)

BETTER_ENUM(E_InsAlign, short int,
            MANUAL,
            MOVING,
            GPSSOL,
            GPSOBS,
            DUALANT)

BETTER_ENUM(E_GnssPvFmt, short int,
            PSINS,
            RTKLIB,
            GINS)

BETTER_ENUM(E_RefSolFmt, short int,
            IE,
            PSINS,
            RTKLIB,
            GINS,
            FUSING)

BETTER_ENUM(E_PrcMode, short int,
            GNSS,
            INS,
            IGLC,
            IGSTC,
            IGTC,
            VO,
            VIO,
            GVIO)

BETTER_ENUM(E_GnssMode, short int,
            SOL,
            SINGLE,
            DGPS,
            KINEMATIC,
            STATIC,
            STATIC_START,
            MOVED,
            FIXED,
            PPP_KINEMA,
            PPP_STATIC,
            PPP_FIXED)

BETTER_ENUM(E_TideCorr, short int,
            OFF,
            ON,
            OTL)

BETTER_ENUM(E_IonoOpt, short int,
            OFF,
            BRDC,
            SBAS,
            IF,
            UC,
            GIM)

BETTER_ENUM(E_TropOpt, short int,
            OFF,
            SAAS,
            SBAS,
            EST_ZTD,
            EST_ZTD_GRAD)

BETTER_ENUM(E_SatEph, short int,
            BRDC,
            PRECISE,
            BRDC_SBAS,
            BRDC_SSRAPC,
            BRDC_SSRCOM)

BETTER_ENUM(E_ARMode, short int,
            OFF,
            CONTINUOUS,
            INSTANTANEOUS,
            FIX_AND_HOLD)

BETTER_ENUM(E_PosFmt, short int,
            LLH,
            XYZ,
            SPP,
            POS_FILE,
            RINEX_HEAD,
            RTCM,
            RAW)

BETTER_ENUM(E_SolFmt, short int,
            LLH,
            XYZ,
            ENU,
            NMEA)

BETTER_ENUM(E_TimeSys, short int,
            GPST,
            UTC,
            JST)

BETTER_ENUM(E_TimeFmt, short int,
            TOW,
            HMS)

BETTER_ENUM(E_DegFmt, short int,
            DEG,
            DMS)

BETTER_ENUM(E_HeightFmt, short int,
            ELLIPSOIDAL,
            GEODETIC)

BETTER_ENUM(E_SolStat, short int,
            OFF,
            STATE,
            RESIDUAL)

BETTER_ENUM(E_WeightMode, short int,
            ELEVATION,
            SNR)
#endif //FUSING_ENUMS_H
