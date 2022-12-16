//
// Created by hw on 8/28/22.
//

#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP
#include "Logger.h"

extern string logTime(double sow,int ins_epoch,int ig_epoch)
{
    char s[124];
    sprintf(s,"%9.3f-[%d %d]:",sow,ins_epoch,ig_epoch);

    return s;
}

extern void initLogger(int argc, char **argv, string log_conf_file,int debug_level)
{
    START_EASYLOGGINGPP(argc,argv);

//    defaultConf.setToDefault();

    if(log_conf_file.empty()){
        el::Configurations defaultConf;

        el::Loggers::addFlag(el::LoggingFlag::DisableApplicationAbortOnFatalLog);
        ///
        el::Loggers::addFlag(el::LoggingFlag::HierarchicalLogging);
        ///
        el::Loggers::addFlag(el::LoggingFlag::ColoredTerminalOutput);

        el::Loggers::setLoggingLevel(static_cast<el::Level>(debug_level));

        defaultConf.setGlobally(el::ConfigurationType::Enabled,"true");
        defaultConf.setGlobally(el::ConfigurationType::ToFile,"true");
        defaultConf.setGlobally(el::ConfigurationType::ToStandardOutput,"true");
        defaultConf.setGlobally(el::ConfigurationType::Format,"%datetime{%s:%g} [%level] %msg");
//        defaultConf.setGlobally(el::ConfigurationType::Filename,"log/%datatime{%Y-%M-%d}.log");
        defaultConf.setGlobally(el::ConfigurationType::MillisecondsWidth,"3");
        defaultConf.setGlobally(el::ConfigurationType::PerformanceTracking,"false");
        defaultConf.setGlobally(el::ConfigurationType::MaxLogFileSize,"1048576");
        defaultConf.setGlobally(el::ConfigurationType::LogFlushThreshold,"0");

        /*trace*/
        defaultConf.set(el::Level::Trace,el::ConfigurationType::ToStandardOutput,"false");
        defaultConf.set(el::Level::Trace,el::ConfigurationType::ToFile,"true");

        /*debug*/
        defaultConf.set(el::Level::Debug,el::ConfigurationType::ToStandardOutput,"false");
        defaultConf.set(el::Level::Debug,el::ConfigurationType::ToFile,"true");

        /*fatal*/
        defaultConf.set(el::Level::Fatal,el::ConfigurationType::ToStandardOutput,"false");
        defaultConf.set(el::Level::Fatal,el::ConfigurationType::ToFile,"true");

        /*error*/
        defaultConf.set(el::Level::Trace,el::ConfigurationType::ToStandardOutput,"false");
        defaultConf.set(el::Level::Trace,el::ConfigurationType::ToFile,"true");

        /*warning*/
        defaultConf.set(el::Level::Warning,el::ConfigurationType::ToStandardOutput,"false");
        defaultConf.set(el::Level::Warning,el::ConfigurationType::ToFile,"true");

        /*info*/
        defaultConf.set(el::Level::Info,el::ConfigurationType::ToStandardOutput,"true");
        defaultConf.set(el::Level::Info,el::ConfigurationType::ToFile,"false");

        /*verbose*/
        defaultConf.set(el::Level::Verbose,el::ConfigurationType::ToStandardOutput,"false");
        defaultConf.set(el::Level::Verbose,el::ConfigurationType::ToFile,"true");
        el::Loggers::reconfigureLogger("default", defaultConf);
    }
    else{
        el::Configurations conf(log_conf_file);
//        el::Loggers::reconfigureLogger("default",conf);
//        el::Configurations c;
//        c.setToDefault();
//        el::Loggers::reconfigureLogger("default", c);

    }

}

extern void setLogFile(vector<string> log_paths,int log_level)
{
    el::Configurations defaultConf;

    if(static_cast<el::Level>(log_level) == el::Level::Global){
        defaultConf.setGlobally(el::ConfigurationType::Filename,log_paths[0]);
    }
    else{
        /*trace*/
        if(log_paths.size()==4){
            defaultConf.set(el::Level::Trace,el::ConfigurationType::Filename,log_paths[0]);
            defaultConf.set(el::Level::Debug,el::ConfigurationType::Filename,log_paths[1]);
            defaultConf.set(el::Level::Fatal,el::ConfigurationType::Filename,log_paths[2]);
            defaultConf.set(el::Level::Error,el::ConfigurationType::Filename,log_paths[2]);
            defaultConf.set(el::Level::Warning,el::ConfigurationType::Filename,log_paths[2]);
            defaultConf.set(el::Level::Verbose,el::ConfigurationType::Filename,log_paths[3]);
        }
        else if(log_paths.size()==3){
            defaultConf.set(el::Level::Trace,el::ConfigurationType::Filename,log_paths[0]);
            defaultConf.set(el::Level::Debug,el::ConfigurationType::Filename,log_paths[1]);
            defaultConf.set(el::Level::Fatal,el::ConfigurationType::Filename,log_paths[2]);
            defaultConf.set(el::Level::Error,el::ConfigurationType::Filename,log_paths[2]);
            defaultConf.set(el::Level::Warning,el::ConfigurationType::Filename,log_paths[2]);
        }
        else if(log_paths.size()==2){
            defaultConf.set(el::Level::Trace,el::ConfigurationType::Filename,log_paths[0]);
            defaultConf.set(el::Level::Debug,el::ConfigurationType::Filename,log_paths[1]);
        }
        else if(log_paths.size()==1){
            defaultConf.set(el::Level::Trace,el::ConfigurationType::Filename,log_paths[0]);
        }
    }
    el::Loggers::reconfigureLogger("default", defaultConf);
}