//
// Created by hw on 8/30/22.
//

#include "Logger.h"
#include "Config.h"
#include "EigenInc.h"
#include "Coordinate.h"
#include "Rotation.h"
#include "Imu.h"

typedef tuple<YAML::Node, string> NodeStack;
map<string, bool> availableOptions;

string stringify(string value)
{
    return ((string) "\"") + value + "\"";
}

template<typename TYPE>
string stringify(TYPE value)
{
    string output;
    output += std::to_string(value);
    return output;
}

template<typename TYPE>
string stringify(vector<TYPE>& vec)
{
    string output;
    output += "[";

    for (int i = 0; i < vec.size(); i++)
    {
        output += stringify(vec[i]);

        if (i < vec.size() - 1)
        {
            output += ", ";
        }
    }
    output += "]";
    return output;
}

string nonNumericStack(string stack, bool colon = true)
{
    string token;

    std::stringstream ss(stack);
    string newStack;

    while (getline(ss, token, ':'))
    {
        size_t found = token.find_first_not_of("0123456789: ");
        if (found != std::string::npos)
        {
            token = token.substr(found);
            newStack += token;
            if (colon)
                newStack += ":";
        }
    }

    return newStack;
}

NodeStack stringsToYamlObject(NodeStack yamlBase, vector<string> yamlNodeDescriptor, string comment = "", string defaultValue = "")
{
    YAML::Node currentNode;

    auto [node, stack] = yamlBase;
    currentNode.reset(node);

    for (int i = 0; i < yamlNodeDescriptor.size(); i++)
    {
        auto& desc = yamlNodeDescriptor[i];

        string shortDesc = nonNumericStack(desc, false);

        currentNode.reset(currentNode[shortDesc]);
        stack += desc + ":";
    }

    return {currentNode, stack};
}

template<typename TYPE>
bool trySetFromYaml(TYPE& output, NodeStack yamlBase, vector<string> yamlNodeDescriptor, string comment = "")
{
    auto [optNode, stack] = stringsToYamlObject(yamlBase, yamlNodeDescriptor, comment, stringify(output));
    availableOptions[nonNumericStack(stack)] = true;

    try
    {
        output = optNode.template as<TYPE>();
        return true;
    }
    catch (...)
    {
        if (optNode.Scalar().empty() == false)
        {
            LOG(WARNING)<<"Warning: Yaml entry '" << stack << "' was found but its value is incorrectly formatted";
        }
    }
    return false;
}

template <typename ENUM>
void trySetEnumOpt(
        ENUM&			out,							///< Variable to output to
        NodeStack		yamlBase,						///< Yaml node to search within
        vector<string>	yamlNodeDescriptor,				///< List of strings of keys to trace hierarcy
        ENUM			(&_from_string)(const char*),	///< Function to decode enum strings
        string			comment = "")					///< Description to provide to user for automatic documentation
{
    string enumOptions = " {";

    for (int i = 0; i < ENUM::_size(); i++)
    {
        string		enumOption		= ENUM::_names() [i];
        transform(enumOption.begin(), enumOption.end(), enumOption.begin(), ::tolower);
        if (i != 0)
            enumOptions += ",";
        enumOptions += enumOption;
    }
    enumOptions += "}";

    auto [optNode, stack] = stringsToYamlObject(yamlBase, yamlNodeDescriptor, comment + enumOptions, out._to_string());

    availableOptions[nonNumericStack(stack)] = true;

    string value;
    try
    {
        value = optNode.template as<string>();
    }
    catch (...)
    {
//        LOG(WARNING)<<"Warning: Yaml entry '" << stack << "' was found but its value is incorrectly formatted";
        return;
    }

    try
    {
        out = _from_string(value.c_str());
    }
    catch (...)
    {
        LOG(ERROR)
                << "\nError: " << value << " is not a valid entry for option: " << yamlNodeDescriptor.back() << ".\n"
                << "Valid options include:";

        for (const char* name : ENUM::_names())
        {
            LOG(ERROR) << name;
        }
        exit(0);
    }
}


Config::Config(std::string file_path) {
    config_file_=file_path;
}

bool Config::parse(std::string file_path) {
    try{
        yaml_.reset();
        yaml_ = YAML::LoadFile(file_path);
    }
    catch(const YAML::BadFile &e){
        LOG(ERROR)<<"yaml file parse error, file path: "<<file_path;
        return false;
    }

    vector<double> vec;
    /*input options*/
    {
        auto inputs = stringsToYamlObject({yaml_,""},{"inputs"});
        trySetFromYaml(fusingopt_.input.prc_dir,     inputs,{"prc_dir"},  "[string] List of prc dir to use");
        trySetFromYaml(fusingopt_.input.rnx_files,   inputs,{"rnx_files"},"[string] List of rnx files to use");
        trySetFromYaml(fusingopt_.input.nav_files,   inputs,{"nav_files"},"[string] List of nav files to use");
        trySetFromYaml(fusingopt_.input.sp3_files,   inputs,{"sp3_files"},"[string] List of sp3 files to use");
        trySetFromYaml(fusingopt_.input.clk_files,   inputs,{"clk_files"},"[string] List of clk files to use");

        trySetFromYaml(fusingopt_.input.erp_files,   inputs,{"erp_files"},"[string] List of erp files to use");
        trySetFromYaml(fusingopt_.input.dcb_files,   inputs,{"dcb_files"},"[string] List of dcb files to use");
        trySetFromYaml(fusingopt_.input.bsx_file,    inputs,{"bsx_file"}, "[string] List of bsx file to use");
        trySetFromYaml(fusingopt_.input.blq_file,    inputs,{"blq_file"}, "[string] List of blq file to use");
        trySetFromYaml(fusingopt_.input.ion_file,    inputs,{"ion_file"}, "[string] List of ion file to use");
        trySetFromYaml(fusingopt_.input.atx_file,    inputs,{"atx_file"}, "[string] List of atx file to use");
        trySetFromYaml(fusingopt_.input.snx_file,    inputs,{"snx_file"}, "[string] List of snx file to use");

        trySetFromYaml(fusingopt_.input.egm_file,    inputs,{"egm_file"}, "[string] List of egm file to use");
        trySetFromYaml(fusingopt_.input.jpl_file,    inputs,{"jpl_file"}, "[string] List of jpl file to use");
        trySetFromYaml(fusingopt_.input.imu_file,    inputs,{"imu_file"}, "[string] List of imu file to use");
        trySetFromYaml(fusingopt_.input.imup_file,   inputs,{"imup_file"}, "[string] List of imuc file to use");
        trySetFromYaml(fusingopt_.input.pos_file,    inputs,{"pos_file"}, "[string] List of pos file to use");
        trySetFromYaml(fusingopt_.input.sol_file,    inputs,{"sol_file"}, "[string] List of out file to use");
        trySetFromYaml(fusingopt_.input.ref_file,    inputs,{"ref_file"}, "[string] List of ref file to use");
        trySetFromYaml(fusingopt_.input.err_file,    inputs,{"err_file"}, "[string] List of err file to use");
        trySetFromYaml(fusingopt_.input.log_files,    inputs,{"log_files"}, "[string] List of err file to use");

        trySetFromYaml(fusingopt_.input.solstat_file,inputs,{"solstat_file"},"[string] List of solstat file to use");
        trySetFromYaml(fusingopt_.input.trace_file,  inputs,{"trace_file"},"[string] List of trace file to use");
    }

    /*common options*/
    {
        auto common = stringsToYamlObject({yaml_,""},{"common options"});
        GTime ts,te;
        vector<string> v;

        trySetFromYaml(v, common,{"start_sow"},"[string] process start time");
        if(v.size()>0){
            if(stoi(v[0])==0){
                GTime::str2time(v[1].c_str(), 0, 28, "-", ":", ts);
                fusingopt_.common.sow_start=ts.time2gpst(ts, nullptr, nullptr);
            }
            else if(stoi(v[0])==1){
                fusingopt_.common.sow_start= stod(v[1]);
            }
        }

        v.clear();
        trySetFromYaml(v, common,{"end_sow"},"[string] process end time");
        if(v.size()>0){
            if(stoi(v[0])==0){
                GTime::str2time(v[1].c_str(), 0, 28, "-", ":", te);
                fusingopt_.common.sow_end=te.time2gpst(te, nullptr, nullptr);
            }
            else if(stoi(v[0])==1){
                fusingopt_.common.sow_end= stod(v[1]);
            }
        }

        trySetEnumOpt (fusingopt_.common.prc_mode,    common,{"process_mode" },E_PrcMode::_from_string_nocase);
        LOG(INFO)<<"PROCESS MODE: "<<fusingopt_.common.prc_mode._to_string();
        trySetEnumOpt (fusingopt_.common.filter_type,    common,{"filter_type" },E_Estimator::_from_string_nocase);
        LOG(INFO)<<"FILTER  MODE: "<<fusingopt_.common.filter_type._to_string();

        trySetFromYaml(fusingopt_.common.prc_date,    common,{"prc_date"}, "[int] process date [year doy]");
        trySetFromYaml(fusingopt_.common.prc_site,    common,{"prc_site"}, "[string] process site");
    }

    /*imu options*/
    {
        vector<double> v;
        auto imu = stringsToYamlObject({yaml_,""},{"imu options"});
        trySetEnumOpt (fusingopt_.imu.nav_coord,   imu,{"nav_coord" },E_InsNavCoord::_from_string_nocase);
        trySetEnumOpt (fusingopt_.imu.att_def,     imu,{"att_def" },E_AttDefination::_from_string_nocase);
        trySetEnumOpt (fusingopt_.imu.gnsspv_fmt,  imu,{"gnsspv_fmt" },E_GnssPvFmt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.imu.align_method,imu,{"ins_align" },E_InsAlign::_from_string_nocase);
        trySetFromYaml(v, imu,{"simgps_loss"},"gps measurement loss simulation");
        if(v.size()>=2){
            fusingopt_.imu.sim_gps_loss.ts=v[0];
            fusingopt_.imu.sim_gps_loss.n=(int)v[1];
            for(int i=2;i<v.size()-1;i++){
                fusingopt_.imu.sim_gps_loss.duration.push_back(v[i]);
            }
            fusingopt_.imu.sim_gps_loss.spacing=v.back();
        }
        v.clear();
        trySetEnumOpt (fusingopt_.imu.est_Sg,    imu,{"est_sg" },E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.imu.est_Sa,    imu,{"est_sa" },E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.imu.dop_aid,   imu,{"dop_aid" },E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.imu.tdcp_aid,  imu,{"tdcp_aid" },E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.imu.zupt_aid,  imu,{"zupt_aid" },E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.imu.nhc_aid,   imu,{"nhc_aid" },E_SwitchOpt::_from_string_nocase);
    }

    /*gnss options*/
    {
        auto gnss = stringsToYamlObject({yaml_,""},{"gnss options"});
        trySetFromYaml(fusingopt_.gnss.sample_rate,   gnss,{"sample_rate"},"[double] gnss sample rate(s)");
        trySetEnumOpt (fusingopt_.gnss.mode,          gnss,{"mode" },E_GnssMode::_from_string_nocase);
        LOG(INFO)<<"GNSS    MODE: "<<fusingopt_.gnss.mode._to_string();
        trySetFromYaml(fusingopt_.gnss.freq,          gnss,{"freq"},"[string] gnss used frequency");
        trySetFromYaml(fusingopt_.gnss.el_mask,       gnss,{"el_mask"},"[double] gnss elevation angle mask");
        fusingopt_.gnss.el_mask*=D2R;
        trySetEnumOpt (fusingopt_.gnss.snrmask[0],    gnss,{"snrmask_r" },E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.snrmask[1],    gnss,{"snrmask_b" },E_SwitchOpt::_from_string_nocase);
        string snrmask_[NFREQ];
        trySetFromYaml(snrmask_[0],  gnss,{"snrmask_L1"},"[string] gnss snr mask on L1");
        trySetFromYaml(snrmask_[1],  gnss,{"snrmask_L2"},"[string] gnss snr mask on L2");
        trySetFromYaml(snrmask_[2],  gnss,{"snrmask_L5"},"[string] gnss snr mask on L5");
        {
            for(int i=0;i<NFREQ;i++){
                char buff[1024];
                strcpy(buff,snrmask_[i].c_str());
                for(char *p= strtok(buff,","),j=0;p&&j<9;p= strtok(NULL,",")){
                    fusingopt_.gnss.snrmask_L[i].push_back(atof(p));
                }
            }
        }
        trySetEnumOpt (fusingopt_.gnss.dynamics,      gnss,{"dynamics" },E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.tide_corr,     gnss,{"tidecorr" },E_TideCorr::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.iono_opt,      gnss,{"ionoopt" },E_IonoOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.trop_opt,      gnss,{"tropopt" },E_TropOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.sat_eph,       gnss,{"sateph" },E_SatEph::_from_string_nocase);
        trySetFromYaml(fusingopt_.gnss.pre_ac,        gnss,{"pre_ac" },"[string] precise products ac");
        trySetEnumOpt (fusingopt_.gnss.sat_pcv,       gnss,{"posopt","satpcv"},E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.rec_pcv,       gnss,{"posopt","recpcv"},E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.phw,           gnss,{"posopt","phw"},E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.eclipsing,     gnss,{"posopt","eclipsing"},E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.raim,          gnss,{"posopt","raim"},E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.clkjump,       gnss,{"posopt","clkjump"},E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.sd_gnss,       gnss,{"posopt","sd_gnss"},E_SwitchOpt::_from_string_nocase);
        string excl_sats;
        trySetFromYaml(excl_sats, gnss,{"exclsats"},"[string] gnss snr mask on L5");
        {
            int sat;
            char *p,*id,buff[1024];
            strcpy(buff,excl_sats.c_str());
            for(p= strtok(buff," ");p;p= strtok(NULL," ")){
                if(*p=='+') id=p+1;else id=p;
                if(!(sat= satid2no(id))) continue;
                fusingopt_.gnss.excl_sats[sat-1]=*p=='+'?2:1;
            }
        }
        trySetFromYaml(fusingopt_.gnss.nav_sys,       gnss,{"navsys"},"[int] gnss snr mask on L5");
        trySetEnumOpt (fusingopt_.gnss.ar_mode,       gnss,{"aropts","armode"},E_ARMode::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.glo_ar_mode,   gnss,{"aropts","gloarmode"},E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.bds_ar_mode,   gnss,{"aropts","bdsarmode"},E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt (fusingopt_.gnss.ar_filter,     gnss,{"aropts","arfilter"},E_SwitchOpt::_from_string_nocase);
        trySetFromYaml(fusingopt_.gnss.arthres,       gnss,{"aropts","arthres"},"[double]");
        trySetFromYaml(fusingopt_.gnss.ratio_min,     gnss,{"aropts","ratio_min"},"[double]");
        trySetFromYaml(fusingopt_.gnss.ratio_max,     gnss,{"aropts","ratio_max"},"[double]");
        trySetFromYaml(fusingopt_.gnss.varholdmab,    gnss,{"aropts","varholdamb"},"[double] variance of fixed ambiguity for hold");
        trySetFromYaml(fusingopt_.gnss.gainholdamb,   gnss,{"aropts","gainholdamb"},"[double] ");
        trySetFromYaml(fusingopt_.gnss.arlock_cnt,    gnss,{"aropts","arlockcnt"},"[int]");
        trySetFromYaml(fusingopt_.gnss.minfix_sats,   gnss,{"aropts","minfixsats"},"[int]");
        trySetFromYaml(fusingopt_.gnss.minhold_sats,  gnss,{"aropts","minholdsats"},"[int]");
        trySetFromYaml(fusingopt_.gnss.mindrop_sats,  gnss,{"aropts","mindropsats"},"[int]");
        trySetFromYaml(fusingopt_.gnss.armin_fix,     gnss,{"aropts","arminfix"},"[int]");
        trySetFromYaml(fusingopt_.gnss.armax_iter,    gnss,{"aropts","armaxiter"},"[int]");
        trySetFromYaml(fusingopt_.gnss.elmask_ar,     gnss,{"aropts","elmaskar"},"[double]");
        trySetFromYaml(fusingopt_.gnss.elmask_hold,   gnss,{"aropts","elmaskhold"},"[double]");
        fusingopt_.gnss.elmask_hold*=D2R;
        trySetFromYaml(fusingopt_.gnss.arout_cnt,     gnss,{"aropts","aroutcnt"},"[int]");
        trySetFromYaml(fusingopt_.gnss.max_age,       gnss,{"maxage"},"[int]");
        trySetEnumOpt(fusingopt_.gnss.sync_sol,       gnss,{"syncsol"},E_SwitchOpt::_from_string_nocase);
        trySetFromYaml(fusingopt_.gnss.slip_thres,    gnss,{"slipthres"},"[double]");
        vector<double>tmp;
        trySetFromYaml(tmp,      gnss,{"rejinno"},"[double]");
        fusingopt_.gnss.rej_inno[0]=tmp[0];fusingopt_.gnss.rej_inno[1]=tmp[1];
        trySetFromYaml(fusingopt_.gnss.rej_dop,       gnss,{"rejdop"},"[double]");
        trySetFromYaml(fusingopt_.gnss.niter,         gnss,{"niter"},"[int]");
        trySetFromYaml(fusingopt_.gnss.base_len,      gnss,{"baselen"},"[double]");
        trySetFromYaml(fusingopt_.gnss.base_sig,      gnss,{"basesig"},"[double]");

        trySetEnumOpt (fusingopt_.gnss.weight_mode,   gnss,{"weightmode"},E_WeightMode::_from_string_nocase);
        trySetFromYaml(fusingopt_.gnss.eratio,        gnss,{"eratio"},"[vector<double>]");
        trySetFromYaml(fusingopt_.gnss.err_phase,     gnss,{"errphase"},"[double]");
        trySetFromYaml(fusingopt_.gnss.err_phase_el,  gnss,{"errphaseel"},"[double]");
        trySetFromYaml(fusingopt_.gnss.err_phase_bl,  gnss,{"errphasebl"},"[double]");
        trySetFromYaml(fusingopt_.gnss.err_doppler,   gnss,{"errdoppler"},"[double]");
        trySetFromYaml(fusingopt_.gnss.snr_max,       gnss,{"snrmax"},"[double]");
        trySetFromYaml(fusingopt_.gnss.err_snr,       gnss,{"errsnr"},"[double]");
        trySetFromYaml(fusingopt_.gnss.err_rcv,       gnss,{"errrcv"},"[double]");
        trySetFromYaml(fusingopt_.gnss.std_bias,      gnss,{"stdbias"},"[double]");
        trySetFromYaml(fusingopt_.gnss.std_iono,      gnss,{"stdiono"},"[double]");
        trySetFromYaml(fusingopt_.gnss.std_trop,      gnss,{"stdtrop"},"[double]");
        trySetFromYaml(fusingopt_.gnss.prn_accelh,    gnss,{"prnaccelh"},"[double]");
        trySetFromYaml(fusingopt_.gnss.prn_accelv,    gnss,{"prnaccelv"},"[double]");
        trySetFromYaml(fusingopt_.gnss.prn_bias,      gnss,{"prnbias"},"[double]");
        trySetFromYaml(fusingopt_.gnss.prn_trop,      gnss,{"prntrop"},"[double]");
        trySetFromYaml(fusingopt_.gnss.prn_pos,       gnss,{"prnpos"},"[double]");
        trySetFromYaml(fusingopt_.gnss.clk_stab,      gnss,{"clkstab"},"[double]");

        trySetEnumOpt(fusingopt_.gnss.ant_pos_fmt[0], gnss,{"antenna","posfmt1"},E_PosFmt::_from_string_nocase);
        trySetFromYaml(vec,                 gnss,{"antenna","pos1"},"[double]");
        if(vec.size()){
            if(fusingopt_.gnss.ant_pos_fmt[0]==+E_PosFmt::LLH){
                Vector3d llh;
                llh=Eigen::Map<Eigen::Vector3d>(vec.data());vec.clear();
                llh[0]*=D2R;llh[1]*=D2R;
                fusingopt_.gnss.ant_pos[0]=Coordinate::llh2ecef(llh);
            }
        }
        trySetFromYaml(fusingopt_.gnss.ant_type[0],   gnss,{"anttype1"},"[string]");
        trySetFromYaml(vec,    gnss,{"antdel1"},"[double]");
        if(vec.size()) fusingopt_.gnss.ant_del[0]=Eigen::Map<Eigen::Vector3d>(vec.data());vec.clear();

        trySetEnumOpt(fusingopt_.gnss.ant_pos_fmt[1], gnss,{"antenna","posfmt2"},E_PosFmt::_from_string_nocase);
        trySetFromYaml(vec,                 gnss,{"antenna","pos2"},"[double]");
        if(vec.size()){
            if(fusingopt_.gnss.ant_pos_fmt[1]==+E_PosFmt::LLH){
                Vector3d llh;
                llh=Eigen::Map<Eigen::Vector3d>(vec.data());vec.clear();
                llh[0]*=D2R;llh[1]*=D2R;
                fusingopt_.gnss.ant_pos[1]=Coordinate::llh2ecef(llh);
            }
        }
        trySetFromYaml(fusingopt_.gnss.ant_type[1],   gnss,{"anttype2"},"[string]");
        trySetFromYaml(vec,    gnss,{"antdel2"},"[double]");
        if(vec.size()) fusingopt_.gnss.ant_del[1]=Eigen::Map<Eigen::Vector3d>(vec.data());vec.clear();
        trySetFromYaml(fusingopt_.gnss.maxaveep,      gnss,{"maxaveep"},"[double]");
        trySetEnumOpt( fusingopt_.gnss.initrst,       gnss,{"initrst"},E_SwitchOpt::_from_string_nocase);
    }

    /*camera options*/
    {

    }

    /*output options*/
    {
        auto outputs = stringsToYamlObject({yaml_,""},{"outputs"});
        trySetEnumOpt(fusingopt_.output.sol_fmt,      outputs,{"solformat"},E_SolFmt::_from_string_nocase);
        trySetEnumOpt(fusingopt_.output.out_head,     outputs,{"outhead"},  E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt(fusingopt_.output.out_opt,      outputs,{"outopt"},  E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt(fusingopt_.output.out_vel,      outputs,{"outvel"},  E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt(fusingopt_.output.out_ins,      outputs,{"outins"},  E_SwitchOpt::_from_string_nocase);
        trySetEnumOpt(fusingopt_.output.time_sys,     outputs,{"timesys"},  E_TimeSys::_from_string_nocase);
        trySetEnumOpt(fusingopt_.output.time_fmt,     outputs,{"timefmt"},  E_TimeFmt::_from_string_nocase);
        trySetFromYaml(fusingopt_.output.time_ndec,   outputs,{"timendec"},"[int]");
        trySetEnumOpt(fusingopt_.output.deg_fmt,      outputs,{"degfmt"},  E_DegFmt::_from_string_nocase);
        trySetFromYaml(fusingopt_.output.field_sep,   outputs,{"fieldsep"},"[string]");
        trySetEnumOpt(fusingopt_.output.out_single,   outputs,{"outsingle"},  E_SwitchOpt::_from_string_nocase);
        trySetFromYaml(fusingopt_.output.max_sol_std, outputs,{"max_sol_std"},"[string]");
        trySetEnumOpt(fusingopt_.output.height,       outputs,{"height"},  E_HeightFmt::_from_string_nocase);
//        trySetEnumOpt(output_.geo,       outputs,{"height"},  E_HeightFmt::_from_string_nocase);
//        trySetEnumOpt(output_.out_sol_stat,       outputs,{"solstatic"},  E_SolS::_from_string_nocase);
        trySetFromYaml(fusingopt_.output.nmea_intv1,  outputs,{"nmeaintv1"},"[double]");
        trySetFromYaml(fusingopt_.output.nmea_intv2,  outputs,{"nmeaintv2"},"[double]");
        trySetEnumOpt(fusingopt_.output.out_sol_stat, outputs,{"outstat"},  E_SolStat::_from_string_nocase);

    }
    return true;
}

bool Config::parse() {
    return parse(config_file_);
}

bool Config::parseImuConf(std::string file_path, imuProperty_t& imup,E_AttDefination att_def) {
    LOG(TRACE)<<"parseImuConf(): ";
    if(file_path.empty()){
        LOG(ERROR)<<"imu configuration file set error\n";
        return false;
    }

    YAML::Node imuc_yaml;
    imuc_yaml=YAML::LoadFile(file_path);

    std::vector<double> vec;
    /* global options */
    {
        auto global_options = stringsToYamlObject({imuc_yaml,""}, {"Global Options"},"");
        trySetEnumOpt (imup.fmt,	global_options, {"str_fmt" },E_ImuFmt::_from_string_nocase);
        trySetFromYaml(imup.rate,	global_options, {"imu_frq"	});
        trySetFromYaml(imup.week,		global_options, {"gps_week"});
    }

    /* imu error options */
    {
        auto imu_error_options = stringsToYamlObject({imuc_yaml,""}, {"Imu Error Options"},"");
    }

    /* imu filter options */
    {
        auto imu_filter_options = stringsToYamlObject({imuc_yaml,""}, {"Imu Filter Options"},"");
        vec.clear();
        trySetFromYaml(vec,		imu_filter_options, {"Initial Standard Deviation Values","pos_std"});
        if(vec.size()==1) imup.imu_error.std_pos.setConstant(vec[0]);
        else imup.imu_error.std_pos = Eigen::Map<Eigen::Vector3d>(vec.data());
        vec.clear();
        trySetFromYaml(vec,		imu_filter_options, {"Initial Standard Deviation Values","vel_std"});
        if(vec.size()==1) imup.imu_error.std_vel.setConstant(vec[0]);
        else imup.imu_error.std_vel = Eigen::Map<Eigen::Vector3d>(vec.data());
        vec.clear();
        trySetFromYaml(vec,		imu_filter_options, {"Initial Standard Deviation Values","att_std"});
        if(vec.size()==1) imup.imu_error.std_att.setConstant(vec[0]*D2R);
        else imup.imu_error.std_att = Eigen::Map<Eigen::Vector3d>(vec.data())*D2R;
        vec.clear();
        trySetFromYaml(vec,		imu_filter_options, {"Initial Standard Deviation Values","ba_std"});
        if(vec.size()==2){
            if(vec[0]==0) imup.imu_error.std_ba.setConstant(vec[1]*MG2MPS2);
            else if(vec[0]==1) imup.imu_error.std_ba.setConstant(vec[1]*MGAL2MPS2);
        }
        else{
            if(vec[0]==0) imup.imu_error.std_ba = Vector3d(vec[1]*MG2MPS2,vec[2]*MG2MPS2,vec[3]*MG2MPS2);
            else if(vec[0]==1) imup.imu_error.std_ba = Vector3d(vec[1]*MGAL2MPS2,vec[2]*MGAL2MPS2,vec[3]*MGAL2MPS2);
        }
        vec.clear();
        trySetFromYaml(vec,		imu_filter_options, {"Initial Standard Deviation Values","bg_std"});
        if(vec.size()==1) imup.imu_error.std_bg.setConstant(vec[0]*DPH2RPS);
        else imup.imu_error.std_bg = (Eigen::Map<Eigen::Vector3d>(vec.data()))*DPH2RPS;
        vec.clear();
        trySetFromYaml(vec,		imu_filter_options, {"Initial Standard Deviation Values","sa_std"});
        if(vec.size()==1) imup.imu_error.std_sa.setConstant(vec[0]*PPM2SCALE);
        else imup.imu_error.std_sa = (Eigen::Map<Eigen::Vector3d>(vec.data()))*PPM2SCALE;
        vec.clear();
        trySetFromYaml(vec,		imu_filter_options, {"Initial Standard Deviation Values","sg_std"});
        if(vec.size()==1) imup.imu_error.std_sg.setConstant(vec[0]*PPM2SCALE);
        else imup.imu_error.std_sg = (Eigen::Map<Eigen::Vector3d>(vec.data()))*PPM2SCALE;

        vec.clear();
        trySetFromYaml(vec,		imu_filter_options, {"Spectral Densities","acc_arw"});
        /*TODO: Check unit convert*/
        if(vec.size()==2){
            if(vec[0]==0) imup.imu_noise.acc_vrw.setConstant(vec[1]*MG2MPS2);
            else if(vec[0]==1) imup.imu_noise.acc_vrw.setConstant(vec[1]*MPSPSH2MPSSS);
        }
        else{
            if(vec[0]==0) imup.imu_noise.acc_vrw = Vector3d(vec[1]*MG2MPS2,vec[2]*MG2MPS2,vec[3]*MG2MPS2);
            else if(vec[0]==1) imup.imu_noise.acc_vrw = Vector3d(vec[1]*MPSPSH2MPSSS,vec[2]*MPSPSH2MPSSS,vec[3]*MPSPSH2MPSSS);
        }
        vec.clear();
        trySetFromYaml(vec,		imu_filter_options, {"Spectral Densities","gyr_arw"});
        if(vec.size()==1) imup.imu_noise.gyr_arw.setConstant(vec[0]*DPSH2RPSS);
        else imup.imu_noise.gyr_arw = (Eigen::Map<Eigen::Vector3d>(vec.data()))*DPSH2RPSS;
        vec.clear();
        trySetFromYaml(vec,		imu_filter_options, {"Spectral Densities","ab_rw"});
        if(vec.size()==1) imup.imu_noise.ab_std.setConstant(vec[0]); /*m/s^2/sqrt(s) */
        else imup.imu_noise.ab_std = Eigen::Map<Eigen::Vector3d>(vec.data());
        vec.clear();
        trySetFromYaml(vec,		imu_filter_options, {"Spectral Densities","gb_rw"});
        if(vec.size()==1) imup.imu_noise.gb_std.setConstant(vec[0]);  /*rad/s/sqrt(s)*/
        else imup.imu_noise.gb_std = Eigen::Map<Eigen::Vector3d>(vec.data());
        vec.clear();
        trySetFromYaml(imup.imu_noise.tau_ba,		imu_filter_options, {"Spectral Densities","tao_ba"});
        trySetFromYaml(imup.imu_noise.tau_bg,		imu_filter_options, {"Spectral Densities","tao_bg"});
        imup.imu_noise.tau_ba*=3600.0;
        imup.imu_noise.tau_bg*=3600.0;
    }

    /* imu initialization options */
    {
        auto init_options = stringsToYamlObject({imuc_yaml,""}, {"Imu Initialization Options"},"");
        vec.clear();
        trySetFromYaml(vec,		init_options, {"init_time"});
        imup.imu_init.sow = vec[0];vec.clear();
        trySetFromYaml(vec,		init_options, {"init_pos"});
        imup.imu_init.pos = Eigen::Map<Eigen::Vector3d>(&vec[1]);
        if(vec[0]==0){ /*llh-deg*/
            Vector3d llh=imup.imu_init.pos;
            llh[0]*=D2R;llh[1]*=D2R;
            imup.imu_init.pos=Coordinate::llh2ecef(llh);
        }
        else if(vec[0]==1){ /*llh-rad*/
            Vector3d llh=imup.imu_init.pos;
            imup.imu_init.pos=Coordinate::llh2ecef(llh);
        }

        vec.clear();
        trySetFromYaml(vec,		init_options, {"init_vel"});
        imup.imu_init.vel = Eigen::Map<Eigen::Vector3d>(&vec[1]);
        if(vec[0]==0){ /*llh-ned*/
            Vector3d llh=Coordinate::ecef2llh(imup.imu_init.pos);
            Matrix3d Cne=Rotation::getCne(llh,E_AttDefination::NED_FRD);
            imup.imu_init.vel=Cne*imup.imu_init.vel;
        }

        vec.clear();
        trySetFromYaml(vec,		init_options, {"init_att"});
        imup.imu_init.att = Eigen::Map<Eigen::Vector3d>(&vec[1]);
        if(vec[0]==0){ /*deg*/
            imup.imu_init.att*=D2R;
        }
        vec.clear();
        trySetFromYaml(vec,		init_options, {"init_ba"});
        imup.imu_init.ba = (Eigen::Map<Eigen::Vector3d>(vec.data()))*MG2MPS2;
        vec.clear();
        trySetFromYaml(vec,		init_options, {"init_bg"});
        imup.imu_init.bg = (Eigen::Map<Eigen::Vector3d>(vec.data()))*DPH2RPS;
    }

    /* lever arm options */
    {
        auto lever_options = stringsToYamlObject({imuc_yaml,""}, {"Lever Arm Options"},"");
        trySetFromYaml(vec,		lever_options, {"imu_to_gps"});
        imup.ig_lever = Eigen::Map<Eigen::Vector3d>(vec.data());
        if(att_def==+E_AttDefination::ENU_RFU){
            imup.ig_lever[0]=vec[1],imup.ig_lever[1]=vec[0],imup.ig_lever[2]=-vec[2];
        }
    }

    return true;
}

void Config::getRtklibOpts(prcopt_t *popt, solopt_t *sopt) {
    popt->mode=fusingopt_.gnss.mode._value-1;
    popt->navsys=fusingopt_.gnss.nav_sys;
    popt->nf=2;
    popt->elmin=fusingopt_.gnss.el_mask;
    popt->snrmask.ena[0]=fusingopt_.gnss.snrmask[0]._value;
    popt->snrmask.ena[1]=fusingopt_.gnss.snrmask[1]._value;
    std::copy(fusingopt_.gnss.snrmask_L[0].begin(),fusingopt_.gnss.snrmask_L[0].end(),popt->snrmask.mask[0]);
    std::copy(fusingopt_.gnss.snrmask_L[1].begin(),fusingopt_.gnss.snrmask_L[1].end(),popt->snrmask.mask[1]);
    std::copy(fusingopt_.gnss.snrmask_L[2].begin(),fusingopt_.gnss.snrmask_L[2].end(),popt->snrmask.mask[2]);
    popt->dynamics=fusingopt_.gnss.dynamics._value;
    popt->tidecorr=fusingopt_.gnss.tide_corr._value;
    popt->ionoopt=fusingopt_.gnss.iono_opt._value;
    popt->tropopt=fusingopt_.gnss.trop_opt._value;
    popt->sateph=fusingopt_.gnss.sat_eph._value;
    popt->posopt[0]=fusingopt_.gnss.sat_pcv._value;
    popt->posopt[1]=fusingopt_.gnss.rec_pcv._value;
    popt->posopt[2]=fusingopt_.gnss.phw._value;
    popt->posopt[3]=fusingopt_.gnss.eclipsing._value;
    popt->posopt[4]=fusingopt_.gnss.raim._value;
    popt->posopt[5]=fusingopt_.gnss.clkjump._value;
    popt->posopt[6]=fusingopt_.gnss.sd_gnss._value;

    memcpy(popt->exsats,fusingopt_.gnss.excl_sats,MAXSAT*sizeof(int));
    popt->navsys=fusingopt_.gnss.nav_sys;
    popt->modear=fusingopt_.gnss.ar_mode._value;
    popt->glomodear=fusingopt_.gnss.glo_ar_mode._value;
    popt->bdsmodear=fusingopt_.gnss.bds_ar_mode._value;
    popt->arfilter=fusingopt_.gnss.ar_filter._value;
    popt->thresar[0]=fusingopt_.gnss.arthres[0];
    popt->thresar[1]=fusingopt_.gnss.arthres[1];
    popt->thresar[2]=fusingopt_.gnss.arthres[2];
    popt->thresar[3]=fusingopt_.gnss.arthres[3];
    popt->thresar[4]=fusingopt_.gnss.arthres[4];
    popt->thresar[5]=fusingopt_.gnss.ratio_min;
    popt->thresar[6]=fusingopt_.gnss.ratio_max;
    popt->varholdamb=fusingopt_.gnss.varholdmab;
    popt->gainholdamb=fusingopt_.gnss.gainholdamb;
    popt->minlock=fusingopt_.gnss.arlock_cnt;
    popt->minfixsats=fusingopt_.gnss.minfix_sats;
    popt->minholdsats=fusingopt_.gnss.minhold_sats;
    popt->mindropsats=fusingopt_.gnss.mindrop_sats;
    popt->minfix=fusingopt_.gnss.armin_fix;
    popt->armaxiter=fusingopt_.gnss.armax_iter;
    popt->elmaskar=fusingopt_.gnss.elmask_ar;
    popt->elmaskhold=fusingopt_.gnss.elmask_hold;
    popt->maxout=fusingopt_.gnss.arout_cnt;
    popt->maxtdiff=fusingopt_.gnss.max_age;
    popt->syncsol=fusingopt_.gnss.sync_sol._value;
    popt->thresslip=fusingopt_.gnss.slip_thres;
    popt->maxinno[0]=fusingopt_.gnss.rej_inno[0];
    popt->maxinno[1]=fusingopt_.gnss.rej_inno[1];
    popt->thresdop=fusingopt_.gnss.rej_dop;
    popt->niter=fusingopt_.gnss.niter;
    popt->baseline[0]=fusingopt_.gnss.base_len;
    popt->baseline[1]=fusingopt_.gnss.base_sig;
    std::copy(fusingopt_.gnss.eratio.begin(),fusingopt_.gnss.eratio.end(),popt->eratio);
    popt->err[1]=fusingopt_.gnss.err_phase;
    popt->err[2]=fusingopt_.gnss.err_phase_el;
    popt->err[3]=fusingopt_.gnss.err_phase_bl;
    popt->err[4]=fusingopt_.gnss.err_doppler;
    popt->err[5]=fusingopt_.gnss.snr_max;
    popt->err[6]=fusingopt_.gnss.err_snr;
    popt->err[7]=fusingopt_.gnss.err_rcv;
    popt->std[0]=fusingopt_.gnss.std_bias;
    popt->std[1]=fusingopt_.gnss.std_iono;
    popt->std[2]=fusingopt_.gnss.std_trop;
    popt->prn[0]=fusingopt_.gnss.prn_bias;
    popt->prn[1]=fusingopt_.gnss.prn_iono;
    popt->prn[2]=fusingopt_.gnss.prn_trop;
    popt->prn[3]=fusingopt_.gnss.prn_accelh;
    popt->prn[4]=fusingopt_.gnss.prn_accelv;
    popt->prn[5]=fusingopt_.gnss.prn_pos;
    popt->sclkstab=fusingopt_.gnss.clk_stab;
    strcpy(popt->anttype[0],fusingopt_.gnss.ant_type[0].c_str());
    strcpy(popt->anttype[1],fusingopt_.gnss.ant_type[1].c_str());
    matcpy(popt->rb,fusingopt_.gnss.ant_pos[0].data(),3,1);
    matcpy(popt->ru,fusingopt_.gnss.ant_pos[1].data(),3,1);
    matcpy(popt->antdel[0],fusingopt_.gnss.ant_del[0].data(),3,1);
    matcpy(popt->antdel[1],fusingopt_.gnss.ant_del[1].data(),3,1);
    popt->refpos=1;
    popt->maxaveep=fusingopt_.gnss.maxaveep;
    popt->initrst=fusingopt_.gnss.initrst;

    sopt->posf=fusingopt_.output.sol_fmt._value;
    sopt->times=fusingopt_.output.time_sys._value;
    sopt->timef=fusingopt_.output.time_fmt._value;
    sopt->timeu=fusingopt_.output.time_ndec;
    sopt->degf=fusingopt_.output.deg_fmt._value;
    sopt->outhead=fusingopt_.output.out_head._value;
    sopt->outopt=fusingopt_.output.out_opt._value;
    sopt->outvel=fusingopt_.output.out_vel._value;
    sopt->outins=fusingopt_.output.out_ins._value;
//    sopt->datum=fusingopt_.output.d
    sopt->height=fusingopt_.output.height._value;
//    sopt->geoid=fusingopt_.output.
//    sopt->solstatic=fusingopt_.output.so
    sopt->sstat=fusingopt_.output.out_sol_stat._value;
//    sopt->trace
//    strcpy(sopt->sep,fusingopt_.output.field_sep.c_str());
}

