//
// Created by hw on 8/28/22.
//

#ifndef FUSING_LOGGER_H
#define FUSING_LOGGER_H

#include <string>
#include <iomanip>

#include "easylogging++.h"

using namespace std;

extern string logTime(double sow,int ins_epoch,int ig_epoch);
extern void initLogger(int argc, char **argv, string log_conf_file,int debug_level);
extern void setLogFile(vector<string> log_paths,int log_level);

#endif //FUSING_LOGGER_H
