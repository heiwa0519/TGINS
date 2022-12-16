//
// Created by hw on 8/30/22.
//

#ifndef FUSING_CONFIG_H
#define FUSING_CONFIG_H

#include <string>
#include <yaml-cpp/yaml.h>
#include "Enums.h"
#include "EigenInc.h"
#include "GTime.h"
#include "rtklib.h"

using namespace std;

struct imuProperty_t;
struct prcopt_t;
struct solopt_t;

typedef struct {
    string prc_dir;
    vector<string>  rnx_files;
    vector<string>  nav_files;
    vector<string>  sp3_files;
    vector<string>  clk_files;

    vector<string>  erp_files;
    vector<string>  obx_files;
    vector<string>  bia_files;
    vector<string>  dcb_files;
    string  bsx_file;
    string  blq_file;
    string  ion_file;
    string  atx_file;
    string  snx_file;

    string	egm_file;
    string	jpl_file;

    string  imu_file;
    string  imup_file;
    string  pos_file;
    string  sol_file;
    string  ref_file;
    string  err_file;
    string  trace_file;
    string  solstat_file;
    vector<string>  log_files;
}inputopt_t;

typedef struct {
    double sow_start=0.0,sow_end=0.0;
    E_PrcMode prc_mode=E_PrcMode::INS;
    E_Estimator filter_type=E_Estimator::FKF;
    vector<int> prc_date;
    string prc_site;
}commonopt_t;

typedef struct {
    double ts;
    int    n;
    double spacing;
    vector<double> duration;
}simGpsLoss_t;

typedef struct{
    E_InsAlign align_method = E_InsAlign::MANUAL;
    E_InsNavCoord nav_coord = E_InsNavCoord::LLH;
    E_AttDefination att_def = E_AttDefination::ENU_RFU;
    E_GnssPvFmt  gnsspv_fmt = E_GnssPvFmt::RTKLIB;
    simGpsLoss_t sim_gps_loss = {0};
    E_SwitchOpt est_Sa=E_SwitchOpt::OFF,est_Sg=E_SwitchOpt::OFF;
    E_SwitchOpt dop_aid=E_SwitchOpt::OFF;
    E_SwitchOpt tdcp_aid=E_SwitchOpt::OFF;
    E_SwitchOpt zupt_aid=E_SwitchOpt::OFF;
    E_SwitchOpt nhc_aid=E_SwitchOpt::OFF;
}imuopt_t;

typedef struct {
    double sample_rate=30.0;
    E_GnssMode mode=E_GnssMode::KINEMATIC;
    string freq="l1+l2";
    int nf=2;
    double el_mask=15.0*D2R;
    E_SwitchOpt snrmask[2]={E_SwitchOpt::OFF,E_SwitchOpt::OFF};
    vector<double> snrmask_L[NFREQ];
    E_SwitchOpt dynamics  = E_SwitchOpt::OFF;
    E_TideCorr  tide_corr = E_TideCorr::OFF;
    E_IonoOpt   iono_opt  = E_IonoOpt::BRDC;
    E_TropOpt   trop_opt  = E_TropOpt::SAAS;
    E_SatEph    sat_eph   = E_SatEph::BRDC;
    string      pre_ac;

    E_SwitchOpt sat_pcv = E_SwitchOpt::OFF;
    E_SwitchOpt rec_pcv = E_SwitchOpt::OFF;
    E_SwitchOpt phw = E_SwitchOpt::OFF;
    E_SwitchOpt eclipsing = E_SwitchOpt::OFF;
    E_SwitchOpt raim = E_SwitchOpt::OFF;
    E_SwitchOpt clkjump = E_SwitchOpt::OFF;
    E_SwitchOpt sd_gnss = E_SwitchOpt::OFF;

    int excl_sats[MAXSAT]={0};
    int nav_sys;

    E_ARMode ar_mode        = E_ARMode::CONTINUOUS;
    E_SwitchOpt glo_ar_mode = E_SwitchOpt::OFF;
    E_SwitchOpt bds_ar_mode = E_SwitchOpt::OFF;
    E_SwitchOpt ar_filter   = E_SwitchOpt::OFF;
    vector<double> arthres = {3.0,0.1, 0.0, 1E-09, 1E-05};
    double ratio_min = 2.5;
    double ratio_max = 3.0;
    double varholdmab  = 0.1;
    double gainholdamb = 0.01;
    int arlock_cnt   = 10;
    int minfix_sats  = 4;
    int minhold_sats = 5;
    int mindrop_sats = 10;
    int armin_fix    = 20;
    int armax_iter   = 1;
    double elmask_ar = 0.0;
    double elmask_hold  = 0.0;
    int arout_cnt    = 20;

    double max_age       = 30;
    E_SwitchOpt sync_sol = E_SwitchOpt::OFF;
    double slip_thres    = 0.05;
    double rej_inno[2]   = {30,30};
    double rej_dop       = 30;
    int niter            = 1;
    double base_len      = 0.0;
    double base_sig      = 0.0;

    E_WeightMode   weight_mode=E_WeightMode::ELEVATION;
    vector<double> eratio = {100,100,100};
    double err_phase    = 0.003;
    double err_phase_el = 0.003;
    double err_phase_bl = 0.0;
    double err_doppler  = 1.0;
    double snr_max      = 52.0;
    double err_snr      = 0.0;
    double err_rcv      = 0.0;
    double std_bias     = 30.0;
    double std_iono     = 0.03;
    double std_trop     = 0.3;
    double prn_accelh   = 3.0;
    double prn_accelv   = 1.0;
    double prn_bias     = 0.0;
    double prn_iono     = 0.001;
    double prn_trop     = 0.0001;
    double prn_pos      = 0.0;
    double clk_stab     = 5e-12;

    E_PosFmt ant_pos_fmt[2] = {E_PosFmt::LLH,E_PosFmt::LLH};
    Vector3d ant_pos[2]     = {Vector3d::Zero(),Vector3d::Zero()};
    string   ant_type[2];
    Vector3d ant_del[2]     = {Vector3d::Zero(),Vector3d::Zero()};
    double maxaveep         = 0.0;
    E_SwitchOpt initrst     = E_SwitchOpt::OFF;
}gnssopt_t;

typedef struct {

}cameraopt_t;

typedef struct {
    E_SolFmt sol_fmt     = E_SolFmt::LLH;
    E_SwitchOpt out_head = E_SwitchOpt::OFF;
    E_SwitchOpt out_opt  = E_SwitchOpt::OFF;
    E_SwitchOpt out_vel  = E_SwitchOpt::OFF;
    E_SwitchOpt out_ins  = E_SwitchOpt::OFF;
    E_TimeSys   time_sys = E_TimeSys::GPST;
    E_TimeFmt   time_fmt = E_TimeFmt::TOW;
    int time_ndec = 3;
    string field_sep;
    E_DegFmt deg_fmt = E_DegFmt::DEG;
    E_SwitchOpt out_single = E_SwitchOpt::OFF;
    double max_sol_std = 0;
    E_HeightFmt height = E_HeightFmt::ELLIPSOIDAL;
    double nmea_intv1 = 0;
    double nmea_intv2 = 0;
    E_SolStat out_sol_stat = E_SolStat::OFF;
    int trace;
}outputopt_t;

typedef struct {
    prcopt_t prc_opt=prcopt_default;
    solopt_t sol_opt=solopt_default;
}rtklibopt_t;

typedef struct{
    commonopt_t common;
    imuopt_t    imu;
    gnssopt_t   gnss;
    cameraopt_t camera;
    inputopt_t  input;
    outputopt_t output;
    rtklibopt_t rtklib;
}fusingopt_t;

class Config {
public:
    Config()=default;
    Config(string file_path);
    ~Config()=default;

private:
    bool parse();

public:
    bool parse(string file_path);
    void getRtklibOpts(prcopt_t *popt,solopt_t *sopt);
    static bool parseImuConf(string file_path,imuProperty_t& imuc,E_AttDefination att_def);

private:
    string config_file_;
    YAML::Node yaml_;

public:
    fusingopt_t fusingopt_;
};


#endif //FUSING_CONFIG_H
