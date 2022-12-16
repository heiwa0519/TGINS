//
// Created by hw on 10/2/22.
//

#include "cxxopts.hpp"
#include "Config.h"
#include "Gnss.h"
#include "rtklib/rtklib.h"
#include "Fusing.h"
#include "MatchFiles.h"
#include "ProcessBar.h"

// trace=2,debug=4,fatal=8, error=16, warning=32, verbose=64, info=128
int main(int argc,char **argv)
{
    long t1,t2;
    t1=clock();

    cxxopts::Options options("Fusing","-c config_file");
    options.add_options()
            ("c,config","yaml-format configuration file", cxxopts::value<std::string>())
            ("l,logc",  "log system configuration file path",cxxopts::value<std::string>())
            ("f,format","reference solution format",cxxopts::value<std::string>())
            ("d,debug", "debug  level(T2,D4,F8,E16,W32,V64,I128)",cxxopts::value<std::int8_t>())
            ;

    auto result=options.parse(argc,argv);

    if(result.count("help")){
        cout<<options.help()<<endl;
    }


    string conf_file,logc_file;
    if(result.count("config")){
        conf_file=result["config"].as<string>();
    }
    else{
        cout<<options.help();
        return -1;
    }

    E_RefSolFmt ref_format=E_RefSolFmt::IE;
    if(result.count("format")){
        ref_format=E_RefSolFmt::_from_string_nocase((result["format"].as<string>()).c_str());
    }

    if(result.count("logc")){
        logc_file=result["logc"].as<string>();
    }

    int log_level=8;
    if(result.count("debug")){
        log_level=result["debug"].as<int8_t>();
    }

    /*initialize logger system*/
    initLogger(argc,argv,logc_file,log_level);

    LOG(INFO)<<"Welcome to TGINS";

    Config config;
    if(!config.parse(conf_file)){
        LOG(ERROR)<<"msf: parse configuration file error";
        return -1;
    }

    fusing_t  fusing;
    fusingMeas_t meas={0};

    /*initialize FUSING system*/
    initFusing(fusing,config);

    if(!checkInputFiles(fusing,fusing.opts,log_level)){
        LOG(INFO)<<"TGINS: miss necessary files, exit ...";
        return -1;
    }

    if(!fusing.opts.input.log_files.empty()){
        setLogFile(fusing.opts.input.log_files,log_level);
    }

    /*load required data*/
    if(!loadMeasurement(fusing,fusing.opts,meas)){
        LOG(ERROR)<<"TGINS: load measurement failed, exit...";
        return -1;
    }

    /*multi sensor fusion*/
    if(!multiSensorFusing(meas,fusing)){
        LOG(ERROR)<<"TGINS: GNSS/INS fusion process failed, exit...";
        return -1;
    }

    /*compare solutions*/
    if(!fusing.opts.input.ref_file.empty()&&!fusing.opts.input.err_file.empty()){
        char s[128];
        sprintf(s,"%s -c %s -d %d -f %s" ,"/home/hw/Workspace/NewLab/Fusing/build/Bin/compareSol",conf_file.c_str(),log_level,(result["format"].as<string>()).c_str());
        cout<<endl<<s<<endl;
        system(s);
    }

    t2=clock();
    double t=(double)(t2-t1)/CLOCKS_PER_SEC;
    fprintf(stdout,"TGINS running time: %7.2f\n",t);
    fflush(stdout);

    return 1;
}