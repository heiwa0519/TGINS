//
// Created by hw on 10/16/22.
//

#ifndef FUSING_FBS_H
#define FUSING_FBS_H

#include <cstdio>

struct fusing_t;
struct track_t;

extern int sortTrack(track_t *track);
extern bool fbSmoother(FILE *fp,fusing_t& fusing);

#endif //FUSING_FBSMOOTH_H
