//
// Created by hw on 11/19/22.
//

#ifndef FUSING_MATCHFILES_H
#define FUSING_MATCHFILES_H

#include "Enums.h"
#include "Logger.h"
#include "Config.h"
#include "filesystem.hpp"

namespace fs=ghc::filesystem;

struct fusing_t;

extern bool isExist(const fs::path& path, bool is_directory);
extern bool matchOutFiles(const fusing_t& fusing, fusingopt_t& opts,string& sol_path,string* err_path,vector<string>& log_path,int log_level);
extern bool matchRefFiles(const fusingopt_t& opts,string site_name,string &ref_path);
extern bool autoMatchFiles(const fusing_t &fusing,fusingopt_t& opts,int log_level);
extern bool checkInputFiles(const fusing_t &fusing,fusingopt_t& opts,int log_level);
#endif //FUSING_MATCHFILES_H
