//
// Created by hw on 10/11/22.
//

#include "cxxopts.hpp"
#include "RefSol.h"
#include "Rotation.h"
#include "Coordinate.h"
#include "Ins.h"
#include "Fbs.h"
#include "MatchFiles.h"
#include "Statistics.h"


static int extraSolData(const solbuf_t *sol,int type,double *data)
{
    int i,j,n=sol->n;

    if(type==0){
        for(i=0;i<n;i++){
            if(sol->data[i].rr[0]==0.0&&sol->data[i].rr[1]==0.0&&sol->data[i].rr[2]==0.0){
                return 0;
            }
            else{
                data[i+0*n]=sol->data[i].rr[0];
                data[i+1*n]=sol->data[i].rr[1];
                data[i+2*n]=sol->data[i].rr[2];
            }
        }
    }
    else if(type==1){
        for(i=0;i<sol->n;i++){
            if(sol->data[i].rr[3]==0.0&&sol->data[i].rr[4]==0.0&&sol->data[i].rr[5]==0.0){
                return 0;
            }
            else{
                data[i+0*n]=sol->data[i].rr[3];
                data[i+1*n]=sol->data[i].rr[4];
                data[i+2*n]=sol->data[i].rr[5];
            }
        }
    }
    else if(type==2){
        for(i=0;i<sol->n;i++){
            if(sol->data[i].rpy[0]==0.0&&sol->data[i].rpy[1]==0.0&&sol->data[i].rpy[2]==0.0){
                return 0;
            }
            else{
                data[i+0*n]=sol->data[i].rpy[0]*R2D;
                data[i+1*n]=sol->data[i].rpy[1]*R2D;
                data[i+2*n]=sol->data[i].rpy[2]*R2D;
            }
        }
    }
    else if(type==3){
        for(i=0;i<sol->n;i++){
            if(sol->data[n>100?100:0].bg[0]==0.0&&sol->data[n>100?100:0].bg[1]==0.0&&sol->data[n>100?100:0].bg[2]==0.0){
                return 0;
            }
            else{
                data[i+0*n]=sol->data[i].bg[0]*RPS2DPH;
                data[i+1*n]=sol->data[i].bg[1]*RPS2DPH;
                data[i+2*n]=sol->data[i].bg[2]*RPS2DPH;
            }
        }
    }
    else if(type==4){
        for(i=0;i<sol->n;i++){
            if(sol->data[n>100?100:0].ba[0]==0.0&&sol->data[n>100?100:0].ba[1]==0.0&&sol->data[n>100?100:0].ba[2]==0.0){
                return 0;
            }
            else{
                data[i+0*n]=sol->data[i].ba[0]*MPS22MG;
                data[i+1*n]=sol->data[i].ba[1]*MPS22MG;
                data[i+2*n]=sol->data[i].ba[2]*MPS22MG;
            }
        }
    }
    else if(type==5){
        for(i=0;i<sol->n;i++){
            if(sol->data[n>100?100:0].sg[0]==0.0&&sol->data[n>100?100:0].sg[1]==0.0&&sol->data[n>100?100:0].sg[2]==0.0){
                return 0;
            }
            else{
                data[i+0*n]=sol->data[i].sg[0]*SCALE2PPM;
                data[i+1*n]=sol->data[i].sg[1]*SCALE2PPM;
                data[i+2*n]=sol->data[i].sg[2]*SCALE2PPM;
            }
        }
    }
    else {
        for (i = 0; i < sol->n; i++) {
            if (sol->data[n>100?100:0].sa[0] == 0.0 && sol->data[n>100?100:0].sa[1] == 0.0 && sol->data[n>100?100:0].sa[2] == 0.0) {
                return 0;
            } else {
                data[i + 0 * n] = sol->data[i].sa[0] * SCALE2PPM;
                data[i + 1 * n] = sol->data[i].sa[1] * SCALE2PPM;
                data[i + 2 * n] = sol->data[i].sa[2] * SCALE2PPM;
            }
        }
    }

    return 1;
}

static void solStatistics(string site,const solbuf_t *sol)
{
    int i,j,n=sol->n;
    double *data,rmse,std,ave,max,min,CEP68,CEP95;
    if(sol->n<=0) return;

    data= mat(sol->n,3);
    fprintf(stdout,"+ + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + \n");
    fprintf(stdout,"%5s | %9s | %9s | %9s | %9s | %9s |\n",site.empty()?"SITE":site.c_str(),"STD","RMSE","CEP68","CEP95","MAX");
    fprintf(stdout,"- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");

    if(extraSolData(sol,0,data)){
        std=calcSTD(data,n,false);
        rmse= calcRMSE(data,n,false);
        CEP68= calcCEP(data,n,1);
        CEP95= calcCEP(data,n,2);
        max= calcMax(data,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | m\n","POSE",std,rmse,CEP68,CEP95,max);    std=calcSTD(data,n,false);
        std=calcSTD(data+n,n,false);
        rmse= calcRMSE(data+n,n,false);
        CEP68= calcCEP(data+n,n,1);
        CEP95= calcCEP(data+n,n,2);
        max= calcMax(data+n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | m\n","POSN",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+2*n,n,false);
        rmse=calcRMSE(data+2*n,n,false);
        CEP68= calcCEP(data+2*n,n,1);
        CEP95= calcCEP(data+2*n,n,2);
        max= calcMax(data+2*n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | m\n","POSU",std,rmse,CEP68,CEP95,max);
        fprintf(stdout,"- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
    }

    if(extraSolData(sol,1,data)){
        std=calcSTD(data,n,false);
        rmse= calcRMSE(data,n,false);
        CEP68= calcCEP(data,n,1);
        CEP95= calcCEP(data,n,2);
        max= calcMax(data,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | m/s\n","VELE",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+n,n,false);
        rmse= calcRMSE(data+n,n,false);
        CEP68= calcCEP(data+n,n,1);
        CEP95= calcCEP(data+n,n,2);
        max= calcMax(data+n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | m/s\n","VELN",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+2*n,n,false);
        rmse=calcRMSE(data+2*n,n,false);
        CEP68= calcCEP(data+2*n,n,1);
        CEP95= calcCEP(data+2*n,n,2);
        max= calcMax(data+2*n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | m/s\n","VELU",std,rmse,CEP68,CEP95,max);
        fprintf(stdout,"- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
    }

    if(extraSolData(sol,2,data)){
        std=calcSTD(data,n,false);
        rmse= calcRMSE(data,n,false);
        CEP68= calcCEP(data,n,1);
        CEP95= calcCEP(data,n,2);
        max= calcMax(data,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | deg\n","ATTR",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+n,n,false);
        rmse= calcRMSE(data+n,n,false);
        CEP68= calcCEP(data+n,n,1);
        CEP95= calcCEP(data+n,n,2);
        max= calcMax(data+n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | deg\n","ATTP",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+2*n,n,false);
        rmse=calcRMSE(data+2*n,n,false);
        CEP68= calcCEP(data+2*n,n,1);
        CEP95= calcCEP(data+2*n,n,2);
        max= calcMax(data+2*n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | deg\n","ATTY",std,rmse,CEP68,CEP95,max);
        fprintf(stdout,"- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
    }

    if(extraSolData(sol,3,data)){
        std=calcSTD(data,n,false);
        rmse= calcRMSE(data,n,false);
        CEP68= calcCEP(data,n,1);
        CEP95= calcCEP(data,n,2);
        max= calcMax(data,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | deg/h\n","BG_X",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+n,n,false);
        rmse= calcRMSE(data+n,n,false);
        CEP68= calcCEP(data+n,n,1);
        CEP95= calcCEP(data+n,n,2);
        max= calcMax(data+n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | deg/h\n","BG_Y",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+2*n,n,false);
        rmse=calcRMSE(data+2*n,n,false);
        CEP68= calcCEP(data+2*n,n,1);
        CEP95= calcCEP(data+2*n,n,2);
        max= calcMax(data+2*n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | deg/h\n","BG_Z",std,rmse,CEP68,CEP95,max);
        fprintf(stdout,"- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
    }

    if(extraSolData(sol,4,data)){
        std=calcSTD(data,n,false);
        rmse= calcRMSE(data,n,false);
        CEP68= calcCEP(data,n,1);
        CEP95= calcCEP(data,n,2);
        max= calcMax(data,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | mg\n","BA_X",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+n,n,false);
        rmse= calcRMSE(data+n,n,false);
        CEP68= calcCEP(data+n,n,1);
        CEP95= calcCEP(data+n,n,2);
        max= calcMax(data+n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | mg\n","BA_Y",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+2*n,n,false);
        rmse=calcRMSE(data+2*n,n,false);
        CEP68= calcCEP(data+2*n,n,1);
        CEP95= calcCEP(data+2*n,n,2);
        max= calcMax(data+2*n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | mg\n","BA_Z",std,rmse,CEP68,CEP95,max);
        fprintf(stdout,"- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
    }

    if(extraSolData(sol,5,data)){
        std=calcSTD(data,n,false);
        rmse= calcRMSE(data,n,false);
        CEP68= calcCEP(data,n,1);
        CEP95= calcCEP(data,n,2);
        max= calcMax(data,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | ppm\n","SG_X",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+n,n,false);
        rmse= calcRMSE(data+n,n,false);
        CEP68= calcCEP(data+n,n,1);
        CEP95= calcCEP(data+n,n,2);
        max= calcMax(data+n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | ppm\n","SG_Y",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+2*n,n,false);
        rmse=calcRMSE(data+2*n,n,false);
        CEP68= calcCEP(data+2*n,n,1);
        CEP95= calcCEP(data+2*n,n,2);
        max= calcMax(data+2*n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | ppm\n","SG_Z",std,rmse,CEP68,CEP95,max);
        fprintf(stdout,"- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
    }

    if(extraSolData(sol,6,data)){
        std=calcSTD(data,n,false);
        rmse= calcRMSE(data,n,false);
        CEP68= calcCEP(data,n,1);
        CEP95= calcCEP(data,n,2);
        max= calcMax(data,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | ppm\n","SA_X",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+n,n,false);
        rmse= calcRMSE(data+n,n,false);
        CEP68= calcCEP(data+n,n,1);
        CEP95= calcCEP(data+n,n,2);
        max= calcMax(data+n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | ppm\n","SA_Y",std,rmse,CEP68,CEP95,max);
        std=calcSTD(data+2*n,n,false);
        rmse=calcRMSE(data+2*n,n,false);
        CEP68= calcCEP(data+2*n,n,1);
        CEP95= calcCEP(data+2*n,n,2);
        max= calcMax(data+2*n,n);
        fprintf(stdout,"%5s | %9.3f | %9.3f | %9.3f | %9.3f | %9.3f | ppm\n","SA_Z",std,rmse,CEP68,CEP95,max);
        fprintf(stdout,"- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
    }

//    fprintf(stdout,"+ + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + \n");

    free(data);
}

// trace=2,debug=4,fatal=8, error=16, warning=32, verbose=64, info=128
int main(int argc,char **argv){

    cxxopts::Options options("Fusing-compareSol","compare fusing-sols to reference");
    options.add_options()
            ("c,config","yaml-format configuration file", cxxopts::value<std::string>())
            ("i,ifile", "input  file path (optional)", cxxopts::value<std::string>())
            ("o,ofile", "output file path", cxxopts::value<std::string>())
            ("f,format","reference solution format",cxxopts::value<std::string>())
            ("d,debug", "debug  level(T2,D4,F8,E16,W32,V64,I128)",cxxopts::value<std::int8_t>())
            ("h,help",  "help document")
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
    int debug_level=8;
    if(result.count("debug")){
        debug_level=result["debug"].as<int8_t>();
    }

    E_RefSolFmt ref_format=E_RefSolFmt::IE;
    if(result.count("format")){
        ref_format=E_RefSolFmt::_from_string_nocase((result["format"].as<string>()).c_str());
    }


    initLogger(argc,argv,logc_file,debug_level);

    Config config;
    if(!config.parse(conf_file)){
        return 0;
    }
    config.getRtklibOpts(&config.fusingopt_.rtklib.prc_opt,&config.fusingopt_.rtklib.sol_opt);

    fusing_t fusing;
    initFusing(fusing,config);
    if(!(fusing.opts.input.prc_dir.empty()||fusing.opts.input.prc_dir=="null")){
        matchRefFiles(fusing.opts,fusing.opts.common.prc_site,fusing.opts.input.ref_file);
        matchOutFiles(fusing,fusing.opts,fusing.opts.input.sol_file,&fusing.opts.input.err_file, fusing.opts.input.log_files,debug_level);
    }
    else{
        if(isExist(fusing.opts.input.ref_file,false)){
            LOG(INFO)<<"==> REF  FILE: "<<fusing.opts.input.ref_file;
        }
        if(isExist(fusing.opts.input.sol_file,false)){
            LOG(INFO)<<"==> SOL  FILE: "<<fusing.opts.input.sol_file;
        }
        LOG(INFO)<<"<== ERR  FILE: "<<fusing.opts.input.err_file;
    }

    bool back=config.fusingopt_.common.filter_type==+E_Estimator::BKF?true:false;

    solbuf_t ref_sols={0},sols={0},err_sols={0};

    FILE *fp= fopen(fusing.opts.input.err_file.c_str(),"w");

    /*reference solution*/
    if(!loadRef(fusing.opts.input.ref_file,ref_format,&ref_sols)){
        LOG(ERROR)<<"load reference solutions failed, process error";
        return -1;
    }

    /*solution*/
    if(!loadRef(fusing.opts.input.sol_file,E_RefSolFmt::RTKLIB,&sols)){
        LOG(ERROR)<<"load solutions failed, process error";
        return -1;
    }

    if(sols.n<=0||ref_sols.n<=0) return 0;

    makeSolDiff(&sols,&ref_sols,&err_sols);

    config.fusingopt_.rtklib.sol_opt.posf=SOLF_XYZ;
    config.fusingopt_.rtklib.sol_opt.outvel=1;
    outsolhead(fp,&config.fusingopt_.rtklib.sol_opt);
    sols.rb[0]=0.001;

    solStatistics(fusing.opts.common.prc_site,&err_sols);
    for(int i=0;i<err_sols.n;i++){
        outsol(fp,&err_sols.data[i],sols.rb,&config.fusingopt_.rtklib.sol_opt,1);
    }


    fclose(fp);
    return 1;
}