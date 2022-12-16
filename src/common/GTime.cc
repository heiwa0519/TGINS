//
// Created by hw on 8/27/22.
//

#include <cmath>
#include "GTime.h"


const double gpst0[] = {1980, 1, 6, 0, 0, 0}; /* gps time reference */
const double gst0[] = {1999, 8, 22, 0, 0, 0}; /* galileo system time reference */
const double bdt0[] = {2006, 1, 1, 0, 0, 0}; /* beidou time reference */

const double leaps[MAXLEAPS + 1][7] =
{
        /* leap seconds (y,m,d,h,m,s,utc-gpst) */
        {2017, 1, 1, 0, 0, 0, -18},
        {2015, 7, 1, 0, 0, 0, -17},
        {2012, 7, 1, 0, 0, 0, -16},
        {2009, 1, 1, 0, 0, 0, -15},
        {2006, 1, 1, 0, 0, 0, -14},
        {1999, 1, 1, 0, 0, 0, -13},
        {1997, 7, 1, 0, 0, 0, -12},
        {1996, 1, 1, 0, 0, 0, -11},
        {1994, 7, 1, 0, 0, 0, -10},
        {1993, 7, 1, 0, 0, 0, -9},
        {1992, 7, 1, 0, 0, 0, -8},
        {1991, 1, 1, 0, 0, 0, -7},
        {1990, 1, 1, 0, 0, 0, -6},
        {1988, 1, 1, 0, 0, 0, -5},
        {1985, 7, 1, 0, 0, 0, -4},
        {1983, 7, 1, 0, 0, 0, -3},
        {1982, 7, 1, 0, 0, 0, -2},
        {1981, 7, 1, 0, 0, 0, -1},
        {0}
};

GTime GTime::yrdoy2time(int year,int doy) {
    double ep[6]={ 0 };
    ep[0] = year; ep[1] = 1; ep[2] = 1;
    ep[3] = 0;    ep[4] = 0; ep[5] = 0;
    GTime t_jan1 = GTime::ymdhms2time(ep);
    double dt = (doy-1)*86400.0;
    GTime tt = t_jan1+dt;

    return tt;
}

int GTime::time2yrdoy(GTime t2, int *year) {
    double ep[6]={0};
    GTime::time2ymdhms(t2, ep);
    if (year) *year = (int)ep[0];

    /* time at yyyy-01-01 00:00:00 */
    ep[1] = 1; ep[2] = 1; ep[3] = 0; ep[4] = 0; ep[5] = 0;
    GTime t_jan1 = GTime::ymdhms2time(ep);

    double dt = t2 - t_jan1;
    int doy=0;
    doy = (int)(dt/86400.0 + 1.0e-9) + 1;
    return doy;
}

GTime GTime::ymdhms2time(const double *ep) {
    const int doy[] = {1,32,60,91,121,152,182,213,244,274,305,335};
    GTime tt={};

    int year=(int)ep[0];
    int mon=(int)ep[1];
    int day=(int)ep[2];

    if(year<1970||year>2099||mon<1||mon>12){
        return tt;
    }

    int days	= (year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    int sec		= (int) floor(ep[5]);
    tt.t_.time	= (time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    tt.t_.sec	= ep[5] - sec;

    return  tt;
}

void GTime::time2ymdhms(const GTime t2,double *ep) {
    const int mday[]=
    {
      31,28,31,30,31,30,31,31,30,31,30,31,
      31,28,31,30,31,30,31,31,30,31,30,31,
      31,29,31,30,31,30,31,31,30,31,30,31,
      31,28,31,30,31,30,31,31,30,31,30,31
    };

    int days	= (int) (t2.t_.time / 86400);
    int sec		= (int) (t2.t_.time - (time_t) days * 86400);

    int doy = days % (365*4+1);
    int mon;
    for (mon = 0; mon < 48; mon++)
    {
        if (doy >= mday[mon])
            doy -= mday[mon];
        else
            break;
    }
    ep[0] = 1970 + days / 1461 * 4 + mon / 12;
    ep[1] = mon % 12 + 1;
    ep[2] = doy + 1;
    ep[3] = sec / 3600;
    ep[4] = sec % 3600 / 60;
    ep[5] = sec % 60 + t2.t_.sec;
}

GTime GTime::gpst2utc(){
    int i;
    for(i=0;leaps[i][0]>0;i++){
        GTime tu=*this+leaps[i][6];
        if((tu-ymdhms2time(leaps[i]))>=0){
            return tu;
        }
    }
    return *this;
}

GTime GTime::utc2gpst() {
    int i;
    for(i=0;leaps[i][0]>0;i++){
        if((*this-ymdhms2time(leaps[i]))>=0){
            return *this-leaps[i][6];
        }
    }
    return *this;
}

GTime GTime::gpst2bdt() {
    return *this-14.0;
}

GTime GTime::bdt2gpst() {
    return *this+14.0;
}

GTime GTime::gpst2utc(GTime t2){
    int i;
    for(i=0;leaps[i][0]>0;i++){
        GTime tu=t2+leaps[i][6];
        if((tu-ymdhms2time(leaps[i]))>=0){
            return tu;
        }
    }
    return t2;
}

GTime GTime::utc2gpst(GTime t2) {
    int i;
    for(i=0;leaps[i][0]>0;i++){
        if((t2-ymdhms2time(leaps[i]))>=0){
            return t2-leaps[i][6];
        }
    }
    return t2;
}

GTime GTime::gpst2bdt(GTime t2) {
    return t2-14.0;
}

GTime GTime::bdt2gpst(GTime t2) {
    return t2+14.0;
}

GTime GTime::gpst2time(int week, double sec) {
    GTime t2= GTime::ymdhms2time(gpst0);
    if(sec<-1E9||sec>1E9){
        sec=0;
    }
    t2.t_.time+=60*60*24*7*week+(int)sec;
    t2.t_.sec=sec-(int)sec;
    return t2;
}

double GTime::time2gpst(GTime t2, int *week, int *dow) {
    GTime t0= GTime::ymdhms2time(gpst0);

    time_t sec=t2.t_.time-t0.t_.time;
    int w=(int)(sec/(86400*7));
    int day=(int)((sec-w*86400*7)/86400);

    if(week) *week=w;
    if(dow)  *dow=day;

    return (double)(sec-w*86400*7)+t2.t_.sec;
}

GTime GTime::galt2time(int week, double sec) {
    GTime t2= GTime::ymdhms2time(gst0);
    if(sec<-1E9||sec>1E9){
        sec=0;
    }
    t2.t_.time=60*60*24*7*week+(int)sec;
    t2.t_.sec=sec-(int)sec;
    return t2;
}

double GTime::time2galt(GTime t2, int *week, int *dow) {
    GTime t0= GTime::ymdhms2time(gst0);

    time_t sec=t2.t_.time-t0.t_.time;
    int w=(int)(sec/(86400*7));
    int day=(int)((sec-w*86400*7)/86400);

    if(week) *week=w;
    if(dow)  *dow=day;

    return (double)(sec-w*86400*7)+t2.t_.sec;
}

GTime GTime::bdt2time(int week, double sec) {
    GTime t2= GTime::ymdhms2time(bdt0);
    if(sec<-1E9||sec>1E9){
        sec=0;
    }
    t2.t_.time=60*60*24*7*week+(int)sec;
    t2.t_.sec=sec-(int)sec;
    return t2;
}

double GTime::time2bdt(GTime t2, int *week, int *dow) {
    GTime t0= GTime::ymdhms2time(bdt0);

    time_t sec=t2.t_.time-t0.t_.time;
    int w=(int)(sec/(86400*7));
    int day=(int)((sec-w*86400*7)/86400);

    if(week) *week=w;
    if(dow)  *dow=day;

    return (double)(sec-w*86400*7)+t2.t_.sec;
}

int GTime::str2time(const char *s, int i, int n,const char *sep1, const char *sep2, GTime &t2) {
    double ep[6];
    char str[256],*p=str;
    const char *s1= reinterpret_cast<const char *>(' '),*s2= reinterpret_cast<const char *>(' ');

    if(i<0||(int)strlen(s)<i||(int)sizeof(str)-1<i) return -1;

    for(s+=i;*s&&--n>=0;){
        *p++=*s++;
    }
    *p='\0';

    if(sep1) s1=sep1;
    if(sep2) s2=sep2;
    int read_count= sscanf(str,"%lf-%lf-%lf %lf:%lf:%lf",
                           ep,ep+1,ep+2,ep+3,ep+4,ep+5);
    if(read_count<6) return -1;

    if(ep[0]<100) ep[0]+=ep[0]<80?2000:1900;

    t2=GTime::ymdhms2time(ep);

    return 1;
}

void GTime::time2str(GTime t2, const char *sep1, const char *sep2, int n, char *s) {
    double ep[6];

    if(n<0) n=0;
    else if(n>12) n=12;

    if(1-t2.t_.sec<0.5/pow(10,n)){
        t2.t_.time++;
        t2.t_.sec=0;
    }

    time2ymdhms(t2,ep);

    const char *s1= reinterpret_cast<char *>(' '),*s2= reinterpret_cast<char *>(' ');
    if(sep1) s1=sep1;
    if(sep2) s2=sep2;
    sprintf(s,"%04.0f%c%02.0f%c%02.0f %02.0f%c%02.0f%c%0*.*f",
            ep[0],s1,ep[1],s1,ep[2],ep[3],s2,ep[4],s2,n<=0?2:n+3,n<=0?0:n,ep[5]);
}

GTime GTime::yds2time(const double *yds) {
    int year =(int)yds[0];
    int doy  =(int)yds[1];
    double sec=yds[2];

    if(year<1970||2099<year||doy<1||366<doy){
        return {};
    }

    int days=(year-1970)*365+(year-1969)/4+doy-2;

    GTime t2={};
    t2.t_.time=(time_t)(days*86400+sec);

    return t2;
}

static double setdigits(const double n)
{
    char str[128];
    double m;

    sprintf(str,"%.4f",n);
    sscanf(str,"%lf",&m);

    return m;
}

void GTime::jd2ymdhms(double jd, double *ep) {
    int b,c,d,e;
    double t1,t2,i1;

    modf(jd+0.5,&i1);
    b=i1+1537;
    modf((b-122.1)/365.25,&i1);
    c=i1;
    modf(365.25*c,&i1);
    d=i1;
    modf((b-d)/30.6001,&i1);
    e=i1;

    t1=(jd+0.5-floor(jd+0.5))*24*3600;

    t2=modf(t1/3600,&ep[3]);            /* hour */
    ep[2]=b-d-floor(30.6001*e);         /* day */
    ep[1]=e-1-12*floor(e/14);           /* month */
    ep[0]=c-4715-floor((7+ep[1])/10);   /* year */
    t2=modf(t2*60,&ep[4]);              /* minute */
    ep[5]=t2*60;                        /* second */
    ep[5]=setdigits(ep[5]);

    if (setdigits(ep[5])==60)
    {
        ep[5]=0;
        ep[4]+=1;
        if (ep[4]==60)
        {
            ep[4]=0;
            ep[3]+=1;
        }
    }
}

double GTime::ymdhms2jd(const double *ep) {
    double i,j;

    double yr	= ep[0];
    double mon	= ep[1];
// 	double day	= ep[2];
    double hr	= ep[3];
    double min	= ep[4];
    double sec	= ep[5];

    if (yr<=0||yr>=2099) return 0;

    if (mon>2)	{i=yr;	j=mon;    	}
    else		{i=yr-1;j=mon+12;	}

    double day=ep[2]
                    +hr/24
                    +min/24/60
                    +sec/24/60/60;

    double jd = (floor(365.25*i)+floor(30.6001*(j+1))+day+1720981.5);

    return jd;
}

double GTime::gpst2mjd(GTime t2) {
    double ep[6];
    time2ymdhms(t2,ep);
    double jd= ymdhms2jd(ep);
    return jd-JD2MJD;
}

double GTime::utc2mjd(GTime t2) {
    double ep[6];
    time2ymdhms(t2,ep);
    double jd= ymdhms2jd(ep);
    return  jd-JD2MJD;
}