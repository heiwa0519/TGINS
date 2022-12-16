//
// Created by hw on 11/19/22.
//

#include <dirent.h>
#include <string>
#include <algorithm>
#include "MatchFiles.h"
#include "Fusing.h"
#include "rtklib.h"

using namespace std;

extern bool isExist(const fs::path& path, bool is_directory)
{
    bool stat=false;
    std::error_code error;
    auto file_status=fs::status(path,error);
    if(error){
        return false;
    }

    if(!(stat=fs::exists(file_status))){
        return false;;
    }

    return is_directory?fs::is_directory(file_status)&&is_directory:stat;
}

static bool matchObsFile(const fusingopt_t& opts,vector<string>& obs_path,string *base_path,string *pos_path,string &site_name)
{
    string prc_dir=opts.input.prc_dir;
    string year,doy,file_name;
    year= to_string(opts.common.prc_date[0]),doy= to_string(opts.common.prc_date[1]);
    int yy=opts.common.prc_date[0]>2000?opts.common.prc_date[0]-2000:opts.common.prc_date[0]-1900;
    bool find_obs=false,find_base=false,find_pos=false;

    obs_path.clear();
    if(base_path) *base_path="";
    if(pos_path) *pos_path="";

    fs::path pt,sub_dir;
    pt.assign(prc_dir);

    if(site_name.empty()){
        ((pt/=year)/=doy)/="obs";
    }
    else{
        string tmp="obs";
        tmp=tmp+"_"+site_name;
        ((pt/=year)/=doy)/=tmp;
    }

    if(!(site_name.empty()||site_name=="null")){
        file_name=opts.common.prc_site;
        transform(file_name.begin(),file_name.end(),file_name.begin(),::toupper);
        char file_name_[60]={'\0'};
        sprintf(file_name_,"%s%03d0",file_name.c_str(),opts.common.prc_date[1]);
        file_name=file_name_;
    }

    if(isExist(pt,true)){
        for(auto &itr:fs::directory_iterator(pt)){
            if(fs::is_directory(itr)){
                continue;
            }
            else{
                string fn=itr.path().stem();
                string ext=itr.path().extension();
                string base=fn.substr(fn.size()-4,4);

                if(!pos_path){
                    if(file_name.empty()){
                        if((atoi(ext.substr(1,2).c_str())==yy)&&(ext.substr(3,1)=="o"||ext.substr(3,1)=="O")&&base!="base"){
                            obs_path.push_back(itr.path());
                            for(int i=0;i<obs_path.size();i++){
                                LOG(INFO)<<"==> OBS  FILE: "<<obs_path[0];
                            }
                            site_name=fn.substr(0,4);
                            find_obs=true;
                        }
                    }
                    else{
                        if(file_name==fn&&(atoi(ext.substr(1,2).c_str())==yy)&&(ext.substr(3,1)=="o"||ext.substr(3,1)=="O")){
                            obs_path.push_back(itr.path());
                            LOG(INFO)<<"==> OBS  FILE: "<<obs_path[0];
                            find_obs=true;
                        }
                    }
                }

                if(base_path&&(atoi(ext.substr(1,2).c_str())==yy)&&(ext.substr(3,1)=="o"||ext.substr(3,1)=="O")&&base=="base"){
                    *base_path=itr.path();
                    LOG(INFO)<<"==> BASE FILE: "<<*base_path;
                    find_base=true;
                }
                if(pos_path&&(ext==".pos"||ext==".sol")){
                    *pos_path=itr.path();
                    LOG(INFO)<<"==> POS  FILE: "<<*pos_path;
                    find_pos=true;
                }
            }
        }
    }
    else{
        LOG(ERROR)<<"observation directory is no exists: "<<pt;
    }

    return pos_path?find_pos:(base_path?(find_obs&find_base):find_obs);
}

static bool matchNavFile(const fusingopt_t &opts, vector<string> &nav_path)
{
    string prc_dir=opts.input.prc_dir;
    string year,doy;
    year= to_string(opts.common.prc_date[0]),doy= to_string(opts.common.prc_date[1]);
    int yy=opts.common.prc_date[0]>2000?opts.common.prc_date[0]-2000:opts.common.prc_date[0]-1900;
    bool find_nav=false;

    nav_path.clear();

    fs::path pt1,sub_dir;
    pt1.assign(prc_dir);
    (((pt1/=year)/=doy)/="products")/="nav";

    if(isExist(pt1,true)){
        for(auto &itr:fs::directory_iterator(pt1)){
            if(fs::is_directory(itr)){
                continue;
            }
            else{
                string file_name=itr.path().stem();
                string ext=itr.path().extension();
                string base=file_name.substr(file_name.size()-4,4);
                if(atoi(ext.substr(1,2).c_str())==yy&&(ext.substr(3,1)=="p"||ext.substr(3,1)=="n")){
                    nav_path.push_back(itr.path());
                    for(int i=0;i<nav_path.size();i++){
                        LOG(INFO)<<"==> NAV  FILE: "<<nav_path[i];
                    }
                    find_nav=true;
                }
            }
        }
    }
    else{
        LOG(ERROR)<<"products or products/nav directory is no exists: "<<pt1;
    }
    return find_nav;
}

static bool findOneDayProduct(gtime_t t,const fusingopt_t& opts,string& clk_path,string& orb_path,string& erp_path,
                              string& bia_path,string& obx_path ,bool cur_day)
{
    string prc_dir=opts.input.prc_dir;
    string yyyy,ddd,ac=opts.gnss.pre_ac;
    int year,doy,week,wod;

    time2gpst_new(t,&week,&wod);
    doy=(int)time2doy_new(t,&year);
    yyyy= to_string(year);
    ddd= to_string(doy);
    bool find_clk=false,find_orb=false,find_erp=false;

    fs::path pt0,pt1,pt2,sub_dir;
    pt1.assign(prc_dir);

    clk_path.clear();orb_path.clear();
    if(ac.empty()){
        (((pt1/=yyyy)/=ddd)/="products");
    }
    else{
        (((pt1/=yyyy)/=ddd)/="products")/=(ac);
    }

    /*find current day*/
    if(isExist(pt1,true)){
        for(auto &itr:fs::directory_iterator(pt1)){
            if(fs::is_directory(itr)){
                continue;
            }
            else{
                string file_name=itr.path().stem();
                string ext=itr.path().extension();
                int y,d,w;
                if(file_name.size()>8){ /*long name*/
                    y=atoi(file_name.substr(11,4).c_str());
                    d=atoi(file_name.substr(15,3).c_str());
                }
                else{ /*short name*/
                    w=atoi(file_name.substr(3,4).c_str());
                    d=atoi(file_name.substr(file_name.size()-1,1).c_str());
                }

                if((ext==".clk"&&d==wod&&w==week)||(ext==".CLK"&&d==doy&&y==year)){
                    clk_path=itr.path();
                    LOG(INFO)<<"==> CLK  FILE: "<<clk_path;
                    find_clk=true;
                }
                if((ext==".sp3"&&d==wod&&w==week)||(ext==".SP3"&&d==doy&&y==year)){
                    orb_path=itr.path();
                    LOG(INFO)<<"==> SP3  FILE: "<<orb_path;
                    find_orb=true;
                }
                if((ext==".erp"&&d==wod&&w==week)||(ext==".ERP"&&d==doy&&y==year)){
                    erp_path=itr.path();
                    LOG(INFO)<<"==> ERP  FILE: "<<erp_path;
                }
                if((ext==".bia"&&d==wod&&w==week)||(ext==".BIA"&&(d==doy)&&y==year)){
                    LOG(INFO)<<"==> BIA  FILE: "<<bia_path;
                }
                if((ext==".obx"&&d==wod&&w==week)||(ext==".OBX"&&(d==doy)&&y==year)){
                    LOG(INFO)<<"==> OBX  FILE: "<<obx_path;
                }
            }
        }
    }
    else{
       if(cur_day) LOG(ERROR)<<"products directory is no exists: "<<pt1;
    }

    return find_orb&find_clk;
}

static bool matchPreOrbClk(const fusingopt_t& opts,vector<string>& clk_path,vector<string>& orb_path,
                           vector<string>& erp_path,vector<string>& bia_path,vector<string>& obx_path){
    gtime_t t1= yrdoy2time(opts.common.prc_date[0],opts.common.prc_date[1]),t0,t2;
    t0= timeadd(t1,-86400.0);
    t2= timeadd(t1,86400.0);

    clk_path.clear();
    orb_path.clear();
    erp_path.clear();
    bia_path.clear();
    obx_path.clear();

    bool find_ok=false;
    string clk,orb,erp,bia,obx;

    if(find_ok=findOneDayProduct(t1,opts,clk,orb,erp,bia,obx,true)){
        clk_path.push_back(clk);orb_path.push_back(orb);erp_path.push_back(erp);bia_path.push_back(erp);obx_path.push_back(obx);
        if(findOneDayProduct(t0,opts,clk,orb,erp,bia,obx,false)){
            clk_path.push_back(clk);orb_path.push_back(orb);erp_path.push_back(erp);bia_path.push_back(erp);obx_path.push_back(obx);
        }
        if(findOneDayProduct(t2,opts,clk,orb,erp,bia,obx,false)){
            clk_path.push_back(clk);orb_path.push_back(orb);erp_path.push_back(erp);bia_path.push_back(erp);obx_path.push_back(obx);
        }
    }

    return find_ok;
}

static bool matchErpFile(const fusingopt_t& opts,vector<string>& erp_path)
{
    if(!erp_path.empty()) return true;
    string prc_dir=opts.input.prc_dir;
    string year,doy,wwww,ddd;
    int week, wod;
    bool find_erp=false;
    gtime_t t= yrdoy2time(opts.common.prc_date[0],opts.common.prc_date[1]);
    time2gpst_new(t,&week,&wod);

    year=to_string(opts.common.prc_date[0]);
    wwww= to_string(week);
    ddd = to_string(wod);
    fs::path pt,sub_dir;
    pt.assign(prc_dir);
    ((pt/=year)/="erp");

    string file_name;
    char file_name_[80];
    sprintf(file_name_,"igs%04d7",week);
    file_name=file_name_;

    /*find current day*/
    if(isExist(pt,true)){
        for(auto &itr:fs::directory_iterator(pt)){
            if(fs::is_directory(itr)){
                continue;
            }
            else{
                string fn=itr.path().stem();
                string ext=itr.path().extension();
                int d=atoi(file_name.substr(file_name.size()-1,1).c_str());
                if(ext==".erp"&&fn==file_name){
                    LOG(INFO)<<"==> ERP  FILE: "<<itr.path();
                    find_erp=true;
                }
            }
        }
    }
    else{
       LOG(ERROR)<<"erp directory is no exists: "<<pt;
    }

    return find_erp;
}

static bool matchDcbFile(const fusingopt_t& opts,vector<string> dcb_path)
{
    string prc_dir=opts.input.prc_dir;
    string year;
    int y,m;
    bool find_dcb=false;
    y=opts.common.prc_date[0];
    gtime_t t= yrdoy2time(y,opts.common.prc_date[1]);
    double ep[6];
    time2epoch(t,ep);
    m=(int)ep[1];

    dcb_path.clear();

    year=to_string(y);
    y=opts.common.prc_date[0]>2000?opts.common.prc_date[0]-2000:opts.common.prc_date[0]-1900;

    fs::path pt,sub_dir;
    pt.assign(prc_dir);
    ((pt/=year)/="dcb");

    /*find current day*/
    if(isExist(pt,true)){
        for(auto &itr:fs::directory_iterator(pt)){
            if(fs::is_directory(itr)){
                continue;
            }
            else{
                string fn=itr.path().stem();
                string ext=itr.path().extension();
                int yy,mm;
                yy=atoi(fn.substr(4,2).c_str());
                mm=atoi(fn.substr(6,2).c_str());
                if(ext==".DCB"&&yy==y&&mm==m){
                    char file_name[80],file_path[MAXSTRPATH];
                    sprintf(file_name,"*%02d%02d*.DCB",yy,mm);
                    sprintf(file_path,"%s%c%s",pt.c_str(),FILEPATHSEP,file_name);
                    LOG(INFO)<<"==> DCB  FILE: "<<file_path;
                    find_dcb=true;
                    break;
                }
            }
        }
    }
    else{
        LOG(ERROR)<<"dcb directory is no exists: "<<pt;
    }

    return find_dcb;
}

static bool matchPreMiscFiles(const fusingopt_t& opts,string& atx_path,string& blq_path)
{
    string prc_dir=opts.input.prc_dir;
    bool find_atx=false,find_blq=false;
    gtime_t t= yrdoy2time(opts.common.prc_date[0],opts.common.prc_date[1]);
    double ep[6];
    time2epoch(t,ep);
    int week,wod,min_dif=1000;
    time2gpst(t,&week);
    vector<string> atxs;
    vector<int> weeks;

    atx_path="";
    blq_path="";

    fs::path pt,sub_dir;
    pt.assign(prc_dir);
    (pt/="misc");

    if(isExist(pt,true)){
        for(auto &itr:fs::directory_iterator(pt)){
            if(fs::is_directory(itr)){
                continue;
            }
            else{
                string fn=itr.path().stem();
                string ext=itr.path().extension();
                string base=fn.substr(fn.size()-4,4);
                int w=atoi(fn.substr(6,4).c_str());
                if(w>week) continue;
                if(ext==".atx"){
                    if(fabs(week-w)<min_dif){
                        atx_path=itr.path();
                        min_dif=fabs(week-w);
                    }
                    find_atx=true;
                }
                if(ext==".blq"){
                    blq_path=itr.path();
                    LOG(INFO)<<"==> BLQ  FILE: "<<blq_path;
                    find_blq=true;
                }
            }
        }
    }
    else{
        LOG(ERROR)<<"misc precise product directory is no exists: "<<pt;
    }

    LOG(INFO)<<"==> ATX  FILE: "<<atx_path;

    return find_atx&find_blq;
}

static bool matchImuFile(const fusingopt_t& opts,string& imu_path,string& imup_path,string& site_name)
{
    string prc_dir=opts.input.prc_dir;
    string year,doy,file_name;
    year=to_string(opts.common.prc_date[0]),doy= to_string(opts.common.prc_date[1]);
    bool find_imu=false,find_imup=false;

    imu_path="";
    imup_path="";

    fs::path pt,sub_dir;
    pt.assign(prc_dir);
    if(site_name.empty()){
        ((pt/=year)/=doy)/="obs";
    }
    else{
        string tmp="obs";
        tmp=tmp+"_"+site_name;
        ((pt/=year)/=doy)/=tmp;
    }

    if(!(site_name.empty()||site_name=="null")){
        file_name=site_name;
        transform(file_name.begin(),file_name.end(),file_name.begin(),::toupper);
        char file_name_[60]={'\0'};
        sprintf(file_name_,"%s%03d0",file_name.c_str(),opts.common.prc_date[1]);
        file_name=file_name_;
    }

    if(isExist(pt,true)){
        for(auto &itr:fs::directory_iterator(pt)){
            if(fs::is_directory(itr)){
                continue;
            }
            else{
                string fn=itr.path().stem();
                string ext=itr.path().extension();
                string base=fn.substr(fn.size()-4,4);
                if(file_name.empty()){
                    if(ext==".imu"){
                        imu_path=itr.path();
                        LOG(INFO)<<"==> IMU  FILE: "<<imu_path;
                        if(site_name.empty()) site_name=fn.substr(0,4);
                        find_imu=true;
                    }
                    if(ext==".yaml"){
                        imup_path=itr.path();
                        LOG(INFO)<<"==> IMUP FILE: "<<imup_path;
                        find_imup=true;
                    }
                }
                else{
                    if(ext==".imu"&&file_name==fn){
                        imu_path=itr.path();
                        LOG(INFO)<<"==> IMU  FILE: "<<imu_path;
                        find_imu=true;
                    }
                    if(ext==".imup"&&file_name==fn){
                        imup_path=itr.path();
                        LOG(INFO)<<"==> IMUP FILE: "<<imup_path;
                        find_imup=true;
                    }
                }
            }
        }
    }
    else{
        LOG(ERROR)<<"observation directory is no exists: "<<pt;
    }

    return find_imu&find_imup;
}

extern bool matchRefFiles(const fusingopt_t& opts,string site_name,string &ref_path)
{
    string prc_dir=opts.input.prc_dir;
    string year,doy,file_name;
    year=to_string(opts.common.prc_date[0]),doy= to_string(opts.common.prc_date[1]);
    bool find_ref=false;

    ref_path="";
    fs::path pt;
    pt.assign(prc_dir);

    (((pt/=year)/=doy)/="ref");

    bool gnss=opts.common.prc_mode==+E_PrcMode::GNSS?true:false;
    if(isExist(pt,true)){
        for(auto &itr:fs::directory_iterator(pt)){
            if(fs::is_directory(itr)){
                continue;
            }
            else{
                string fn=itr.path().stem();
                string ext=itr.path().extension();
                string site=fn.substr(0,4);
                if(gnss&&ext==".gnss"&&site==site_name){
                    ref_path=itr.path();
                    LOG(INFO)<<"==> REF  FILE: "<<ref_path;
                    find_ref=true;
                }
                if(!gnss&&ext==".tc"&&site==site_name){
                    ref_path=itr.path();
                    LOG(INFO)<<"==> REF  FILE: "<<ref_path;
                    find_ref=true;
                }
            }
        }
    }

    return find_ref;
}

extern bool matchOutFiles(const fusing_t& fusing, fusingopt_t& opts,string& sol_path,string* err_path,vector<string>& log_path,int log_level)
{
    string prc_dir=opts.input.prc_dir;
    string year,doy;
    year=to_string(opts.common.prc_date[0]),doy= to_string(opts.common.prc_date[1]);
    const char *s2[]={"SF","DF","TF","QF"};
    const char *s3[]={"CONT","INST","HOLD"};
    const char *s4[]={"FKF","BKF","FBS","RTS","FGO"};
    const char *s5[]={"E","L"};
    const char *s6[]={"NF","ER"};
    const char *s7[]={"SAAS","SBAS","EST","GRAD"};
    const char *s8[]={"BRDC","SBAS","IF","UC","GIM"};
    bool find_sol=false;

    sol_path="";
    if(err_path) *err_path="";
    log_path.clear();

    fs::path pt,pt1,pt2,pt3;
    pt.assign(prc_dir);

    (((pt/=year)/=doy)/="result");
    char sub_dir_[50];
    if(opts.common.prc_mode==+E_PrcMode::GNSS){
        sprintf(sub_dir_,"GNSS_%s",opts.gnss.mode._to_string());
    }
    else if(opts.common.prc_mode==+E_PrcMode::INS){
        sprintf(sub_dir_,"INS");
    }
    else{
        sprintf(sub_dir_,"%s_%s",opts.common.prc_mode._to_string(),opts.gnss.mode._to_string());
    }
    (pt/=sub_dir_);

    if(!isExist(pt,true)){
        fs::create_directories(pt);
        find_sol=true;
    }

    char file_name[80],sol_file_name[80],err_file_name[80],log_file_name[80];
    E_PrcMode mode=opts.common.prc_mode;
    bool kine=(mode==+E_GnssMode::KINEMATIC||mode==+E_GnssMode::PPP_KINEMA)||mode>=+E_PrcMode::INS;
    bool ar=(opts.gnss.ar_mode>+E_ARMode::OFF);
    string sys;
    if(opts.gnss.nav_sys&SYS_GPS) sys+="G";
    if(opts.gnss.nav_sys&SYS_GLO) sys+="R";
    if(opts.gnss.nav_sys&SYS_GAL) sys+="E";
    if(opts.gnss.nav_sys&SYS_CMP) sys+="C";
    if(opts.gnss.nav_sys&SYS_QZS) sys+="J";

    if(fusing.isSPP||fusing.isDGPS){
        sprintf(file_name,"%s_%s_%s_K_BRD",opts.common.prc_site.c_str(),sys.c_str(),s2[opts.gnss.nf-1]);
    }
    else if(fusing.isPPK){
        sprintf(file_name,"%s_%s_%s_%s_BRD",opts.common.prc_site.c_str(),sys.c_str(),s2[opts.gnss.nf-1],kine?"K":"S");
    }
    else if(fusing.isPPP&&(!(opts.gnss.pre_ac.empty()||(opts.gnss.pre_ac=="null")))){
        sprintf(file_name,"%s_%s_%s_%s_%s",opts.common.prc_site.c_str(),sys.c_str(),s2[opts.gnss.nf-1],kine?"K":"S",opts.gnss.pre_ac.c_str());
    }
    else{
        sprintf(file_name,"%s",opts.common.prc_site.c_str());
    }

    if(opts.gnss.mode>+E_GnssMode::SOL){
        if(ar){
            sprintf(file_name,"%s_%s",file_name,s3[opts.gnss.ar_mode._value-1]);
        }

        if(fusing.opts.gnss.trop_opt>+E_TropOpt::OFF){
            sprintf(file_name,"%s_%s",file_name,s7[opts.gnss.trop_opt._value-1]);

        }
        if(fusing.opts.gnss.iono_opt>+E_TropOpt::OFF){
            sprintf(file_name,"%s_%s",file_name,s8[opts.gnss.iono_opt._value-1]);
        }
    }

    sprintf(file_name,"%s_%s",file_name,s4[opts.common.filter_type._value]);

    if(fusing.iglc_sol||fusing.iglc_obs||fusing.igstc||fusing.igtc){
        sprintf(file_name,"%s_%s%s",file_name,s5[opts.imu.nav_coord._value],s6[opts.imu.att_def._value]);
    }

    sprintf(sol_file_name,"%s.sol",file_name);
    if(err_path) sprintf(err_file_name,"%s.err",file_name);


    pt1=pt2=pt3=pt;
    (pt/=sol_file_name);
    if(err_path) (pt1/=err_file_name);

    LOG(INFO)<<"<== SOL  FILE: "<<pt.c_str();
    if(err_path) LOG(INFO)<<"<== ERR  FILE: "<<pt1.c_str();
    sol_path=pt;
    if(err_path) *err_path=pt1;

    el::Level level=static_cast<el::Level>(log_level);
    string trace_path,debug_path,warning_path,verbose_path,info_path;
    if(level>=el::Level::Global) {
        if (level == el::Level::Global) {
            sprintf(log_file_name, "%s.log", file_name);
            (pt2 /= file_name);
            log_path.push_back(pt2);
        } else {
            pt2/="log";
            if(!isExist(pt2,true)){
                fs::create_directories(pt2);
            }
            pt2=pt3;
            sprintf(log_file_name, "%s_trace.log", file_name);
            trace_path = ((pt2 /= "log") /= log_file_name);
            pt2=pt3;
            sprintf(log_file_name, "%s_debug.log", file_name);
            debug_path = ((pt2 /= "log") /= log_file_name);
            pt2=pt3;
            sprintf(log_file_name, "%s_warning.log", file_name);
            warning_path = ((pt2 /= "log") /= log_file_name);
            pt2=pt3;
            sprintf(log_file_name, "%s_verbose.log", file_name);
            verbose_path = ((pt2 /= "log") /= log_file_name);
            pt2=pt3;
            sprintf(log_file_name, "%s_info.log", file_name);
            info_path = ((pt2 /= "log") /= log_file_name);
            if (level == el::Level::Info) {
                log_path.push_back(trace_path);
                log_path.push_back(debug_path);
                log_path.push_back(warning_path);
                log_path.push_back(verbose_path);
                log_path.push_back(info_path);
            } else if (level == el::Level::Verbose) {
                log_path.push_back(trace_path);
                log_path.push_back(debug_path);
                log_path.push_back(warning_path);
                log_path.push_back(verbose_path);
            } else if (level == el::Level::Warning) {
                log_path.push_back(trace_path);
                log_path.push_back(debug_path);
                log_path.push_back(warning_path);
            } else if (level == el::Level::Debug) {
                log_path.push_back(trace_path);
                log_path.push_back(debug_path);
            } else if (level == el::Level::Trace) {
                log_path.push_back(trace_path);
            }
        }
        for(int i=0;i<log_path.size();i++){
            LOG(INFO)<<"<== LOG  PATH: "<<log_path[i];
        }
    }

    return find_sol;
}

extern bool autoMatchFiles(const fusing_t &fusing,fusingopt_t& opts,int log_level)
{
    bool file_ok=false,base,pos;
    string base_path,pos_path;
    vector<string> nav_path;
    base=(fusing.isPPK||fusing.isDGPS)?true: false;
    pos=(fusing.iglc_sol)?true:false;

    if(fusing.opts.input.prc_dir.empty()||fusing.opts.input.prc_dir=="null"){
        LOG(ERROR)<<"no set process directory ...";
        return false;
    }

    if(!(file_ok=matchObsFile(opts,opts.input.rnx_files,base?&base_path:nullptr,pos?&pos_path: nullptr,opts.common.prc_site))){
        LOG(ERROR)<<"lack observation file ...";
    }
    else{
        if(!base_path.empty()) opts.input.rnx_files.push_back(base_path);
        if(!pos_path.empty())  opts.input.pos_file=pos_path;
    }

    if(!pos&&!(file_ok&=matchNavFile(opts,opts.input.nav_files))){
        LOG(ERROR)<<"lack navigation file ...";
    }

    if(!pos&&fusing.isPPP){
        if(!(file_ok&=matchPreOrbClk(opts,opts.input.clk_files,opts.input.sp3_files,opts.input.erp_files,opts.input.bia_files,opts.input.obx_files))){
            LOG(ERROR)<<"lack precise products ...";
        }

        if(!(file_ok&=matchErpFile(opts,opts.input.erp_files))){
            LOG(ERROR)<<"lack erp file ...";
        }

        if(!(file_ok&=matchPreMiscFiles(opts,opts.input.atx_file,opts.input.blq_file))){
            LOG(ERROR)<<"lack atx or blq file ...";
        }
    }

    if(!pos&&opts.input.bia_files.empty()&&!(fusing.isPPK||fusing.isDGPS)){
        if(!matchDcbFile(opts,opts.input.dcb_files)){
            LOG(WARNING)<<"lack dcb file ...";
        }
    }

    if(fusing.iglc_sol||fusing.iglc_obs||fusing.igstc||fusing.igtc){
        if(!(file_ok&=matchImuFile(opts,opts.input.imu_file,opts.input.imup_file,opts.common.prc_site))){
            LOG(ERROR)<<"lack imu file ...";
        }
    }

    matchRefFiles(opts,opts.common.prc_site,opts.input.ref_file);

    matchOutFiles(fusing,opts,opts.input.sol_file,&opts.input.err_file, opts.input.log_files,log_level);

    return file_ok;
}

extern bool checkInputFiles(const fusing_t &fusing,fusingopt_t& opts,int log_level)
{
    bool stat=true;
    if(!fusing.opts.input.prc_dir.empty()&&fusing.opts.input.prc_dir!="null"){
        LOG(INFO)<<fusing.opts.input.prc_dir<<": automatically match the required files";
        stat=autoMatchFiles(fusing,opts,log_level);
    }
    else{
        LOG(INFO)<<"check input files from configuration file...";
        if(fusing.opts.gnss.mode>+E_GnssMode::SOL){
            if(opts.input.rnx_files[0].empty()){
                stat=false;
                LOG(INFO)<<"lack gnss rover rinex file ...";
            }
            else{
                if(isExist(opts.input.rnx_files[0],false)){
                    LOG(INFO)<<"==> OBS  FILE: "<<opts.input.rnx_files[0];
                }
                else stat=false;
            }
            if(opts.input.nav_files[0].empty()){
                stat=false;
                LOG(INFO)<<"lack gnss navigation file ...";
            }
            else{
                for(int i=0;i<opts.input.nav_files.size();i++){
                    if(isExist(opts.input.nav_files[i],false)){
                        LOG(INFO)<<"==> NAV  FILE: "<<opts.input.rnx_files[i];
                    }
                    else stat=false;
                }
            }
        }
        if(fusing.isDGPS||fusing.isPPK){
            if(opts.input.rnx_files[1].empty()){
                stat=false;
                LOG(INFO)<<"lack gnss base rinex file ...";
            }
            else{
                if(isExist(opts.input.rnx_files[1],false)){
                    LOG(INFO)<<"==> BASE FILE: "<<opts.input.rnx_files[1];
                }
                else stat=false;
            }
        }
        if(fusing.isPPP){
            if(opts.input.clk_files.empty()){
                stat=false;
                LOG(INFO)<<"lack precise clock file ...";
            }
            else{
                for(int i=0;i<opts.input.clk_files.size();i++){
                    if(isExist(opts.input.clk_files[i],false)){
                        LOG(INFO)<<"==> CLK  FILE: "<<opts.input.clk_files[i];
                    }
                    else stat=false;
                }
            }
            if(opts.input.sp3_files.empty()){
                stat=false;
                LOG(INFO)<<"lack precise orbit file ...";
            }
            else{
                for(int i=0;i<opts.input.clk_files.size();i++){
                    if(isExist(opts.input.sp3_files[i],false)){
                        LOG(INFO)<<"==> SP3  FILE: "<<opts.input.sp3_files[i];
                    }
                    else stat=false;
                }
            }
            if(opts.gnss.tide_corr>+E_TideCorr::OFF&&(opts.input.erp_files.empty()||opts.input.blq_file.empty())){
                stat=false;
                LOG(INFO)<<"lack erp file ...";
            }
            else{
                for(int i=0;i<opts.input.erp_files.size();i++){
                    if(isExist(opts.input.erp_files[i],false)){
                        LOG(INFO)<<"==> ERP  FILE: "<<opts.input.erp_files[i];
                    }
                    else stat=false;
                }
                if(isExist(opts.input.blq_file,false)){
                    LOG(INFO)<<"==> BLQ  FILE: "<<opts.input.blq_file;
                }
            }
            if(opts.input.atx_file.empty()){
                stat=false;
                LOG(INFO)<<"lack atx file ...";
            }
            else{
                if(isExist(opts.input.atx_file,false)){
                    LOG(INFO)<<"==> ATX  FILE: "<<opts.input.atx_file;
                }
                else
                {
                    LOG(INFO)<<"wrong atx file path: "<<opts.input.atx_file;
                    stat=false;
                }
            }

            if(opts.input.bia_files.empty()){

            }
            else{
                for(int i=0;i<opts.input.bia_files.size();i++){
                    if(isExist(opts.input.bia_files[i],false)){
                        LOG(INFO)<<"==> BIA  FILE: "<<opts.input.bia_files[i];
                    }
                    else stat=false;
                }
            }
            if(opts.input.obx_files.empty()){

            }
            else{
                for(int i=0;i<opts.input.obx_files.size();i++){
                    if(isExist(opts.input.obx_files[i],false)){
                        LOG(INFO)<<"==> OBX  FILE: "<<opts.input.obx_files[i];
                    }
                    else stat=false;
                }
            }
        }

        if(fusing.iglc_sol||fusing.iglc_obs||fusing.igstc||fusing.igtc){
            if(opts.input.imu_file.empty()){
                stat=false;
                LOG(INFO)<<"miss imu file ...";
            }
            else{
                if(isExist(opts.input.imu_file,false)){
                    LOG(INFO)<<"==> IMU  FILE: "<<opts.input.imu_file;
                }
                else stat=false;
            }
            if(opts.input.imup_file.empty()){
                stat=false;
                LOG(INFO)<<"miss imup file ...";
            }
            else{
                if(isExist(opts.input.imup_file,false)){
                    LOG(INFO)<<"==> IMUP FILE: "<<opts.input.imup_file;
                }
                else stat=false;
            }
        }

        if(fusing.iglc_sol){
            if(opts.input.pos_file.empty()){
                stat=false;
                LOG(INFO)<<"miss pos file ...";
            }
            else{
                if(isExist(opts.input.pos_file,false)){
                    LOG(INFO)<<"==> POS  FILE: "<<opts.input.pos_file;
                }
                else stat=false;
            }
        }

        if(stat){
            if(!opts.input.ref_file.empty()){
                if(isExist(opts.input.ref_file,false)){
                    LOG(INFO)<<"==> REF  FILE: "<<opts.input.ref_file;
                }
            }
            LOG(INFO)<<"<== SOL  FILE: "<<opts.input.sol_file;
            LOG(INFO)<<"<== ERR  FILE: "<<opts.input.err_file;
            for(int i=0;i<opts.input.log_files.size();i++){
                LOG(INFO)<<"<== LOG  FILE  "<<opts.input.log_files[i];
            }
        }

    }
    return stat;
}