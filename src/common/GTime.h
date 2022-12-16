//
// Created by hw on 8/27/22.
//

#ifndef FUSING_GTIME_H
#define FUSING_GTIME_H

#include <time.h>
#include <string>
#include <cstring>

using namespace std;

#include "Constants.h"

extern const double gpst0[]; /* gps time reference */
extern const double gst0 []; /* galileo system time reference */
extern const double bdt0 []; /* beidou time reference */
extern const double leaps[MAXLEAPS+1][7];

typedef struct {
    time_t  time;
    double  sec;
}timebase_t;

class GTime {

public:
    GTime() = default;
    ~GTime() = default;
    GTime(int n,double sec){
        t_.time=n;
        t_.sec=sec;
    }
    GTime(const double *ep){
        if(ep){
            *this=ymdhms2time(ep);
        }
    }

    bool operator == (const GTime &t2) const{
        if(this->t_.time!=t2.t_.time) return false;
        if(this->t_.time!=t2.t_.time) return false;
        else return true;
    }

    bool operator != (const GTime &t2) const {
        return !(*this==t2);
    }

    bool operator < (const GTime &t2) const {
        if(this->t_.time<t2.t_.time) return true;
        if(this->t_.time>t2.t_.time) return false;
        if(this->t_.sec<t2.t_.sec)   return true;
        else                         return false;
    }

    bool operator > (const GTime &t2) const{
        if(this->t_.time>t2.t_.time) return true;
        if(this->t_.time<t2.t_.time) return false;
        if(this->t_.sec>t2.t_.sec)   return true;
        else                         return false;
    }

    GTime operator + (const double t) const{
        GTime tt=*this;
        tt.t_.sec+=t;

        double frac_sec=tt.t_.sec-(int)tt.t_.sec;
        double int_sec=tt.t_.sec-frac_sec;

        tt.t_.time+=int_sec;
        tt.t_.sec-=int_sec;

        if(tt.t_.sec<0){
            tt.t_.sec+=1;
            tt.t_.time-=1;
        }

        return tt;
    }

    GTime operator + (const int t) const{
        return *this+(double)t;
    }

    GTime& operator += (const double t)
    {
        *this=*this+t;
        return *this;
    }

    GTime operator - (const GTime t2) const{
        GTime tt=*this;
        tt.t_.time -= t2.t_.time;
        tt.t_.sec  -= t2.t_.sec;

        if(tt.t_.sec<0){
            tt.t_.time-=1;
            tt.t_.sec+=1;
        }

        return tt;
    }

    GTime operator - (const double t2) const{
        GTime tt=*this+(-t2);
        return tt;
    }

    operator double() const{
        return this->t_.time + this->t_.sec;
    }

    GTime& operator ++(int){
        this->t_.time++;
        return *this;
    }

    double getSow() const {
        return time2gpst(*this, nullptr, nullptr);
    }

public:

    GTime gpst2utc();
    GTime utc2gpst();
    GTime gpst2bdt();
    GTime bdt2gpst();

    static GTime gpst2utc(GTime t2);
    static GTime utc2gpst(GTime t2);
    static GTime gpst2bdt(GTime t2);
    static GTime bdt2gpst(GTime t2);

    static GTime gpst2time(int week,double sec);
    static double time2gpst(GTime t2,int *week,int *dow);
    static GTime galt2time(int week,double sec);
    static double time2galt(GTime t2,int *week,int *dow);
    static GTime bdt2time(int week,double sec);
    static double time2bdt(GTime t2,int *week,int *dow);

    static GTime yrdoy2time(int year,int doy);
    static int time2yrdoy(GTime t2, int *year);
    static GTime ymdhms2time(const double *ep);
    static void  time2ymdhms(GTime t2,double *ep);

    static GTime yds2time(const double *yds);
    static void jd2ymdhms(double jd,double *ep);
    static double ymdhms2jd(const double *ep);
    static double gpst2mjd(GTime t2);
    static double utc2mjd(GTime t2);

    static int str2time(const char *s,int i,int n,const char *sep1, const char *sep2,GTime& t2);
    static void time2str(GTime t2,const char *sep1,const char *sep2,int n,char *s);

private:
    timebase_t t_={0};
};


#endif //FUSING_GTIME_H
