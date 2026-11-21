#include "fifo_guard.h"
#include <math.h>
#include <string.h>
fifo_count_state_t fifo_count_confirm(const uint16_t *c,const uint64_t *t,size_t n,uint16_t capacity,unsigned hz,uint16_t *out)
{
    if(out)*out=0;
    if(!c || !t || !out || !n || n>FIFO_COUNT_MAX_OBSERVATIONS || capacity<6 || !hz || hz>1000)return FIFO_COUNT_INVALID;
    for(size_t i=0;i<n;++i) {
        if(c[i]>capacity)return FIFO_COUNT_INVALID;
        if(i){
            if(t[i]<t[i-1] || t[i]-t[0]>FIFO_COUNT_CONFIRM_BUDGET_US || c[i]<c[i-1])return FIFO_COUNT_INVALID;
            const uint64_t expected=((t[i]-t[i-1])*hz+999999)/1000000+2;
            if((uint64_t)(c[i]-c[i-1])>expected*6)return FIFO_COUNT_INVALID;
        }
    }
    if(n>=2 && c[n-2]%6==0 && c[n-1]%6==0) {
        *out=c[n-2];return FIFO_COUNT_READY;
    }
    return n==FIFO_COUNT_MAX_OBSERVATIONS?FIFO_COUNT_INVALID:FIFO_COUNT_WAIT;
}
bool fifo_count_select(const uint16_t *c,const uint64_t *t,size_t n,uint16_t capacity,unsigned hz,uint16_t *out)
{ return fifo_count_confirm(c,t,n,capacity,hz,out)==FIFO_COUNT_READY; }
void frame_guard_init(frame_guard_t *g){if(g)memset(g,0,sizeof(*g));}
static void clear_window(frame_guard_t *g,uint64_t now)
{
    g->start_us=now;g->samples=0;
    memset(g->fifo_sum,0,sizeof(g->fifo_sum));memset(g->direct_sum,0,sizeof(g->direct_sum));
    memset(g->fifo_square,0,sizeof(g->fifo_square));memset(g->direct_square,0,sizeof(g->direct_square));
}
frame_check_t frame_guard_feed(frame_guard_t *g,const int16_t f[3],const int16_t d[3],uint64_t now,unsigned pending)
{
    if(!g || !f || !d || !now)return FRAME_TIME_ERROR;
    if(g->last_us && now<g->last_us){frame_guard_init(g);return FRAME_TIME_ERROR;}
    g->last_us=now;
    if(!g->start_us)clear_window(g,now);
    if(!pending || pending>32){clear_window(g,now);g->bad_windows=0;return FRAME_DYNAMIC;}
    ++g->samples;
    for(unsigned a=0;a<3;++a){g->fifo_sum[a]+=f[a];g->direct_sum[a]+=d[a];g->fifo_square[a]+=(double)f[a]*f[a];g->direct_square[a]+=(double)d[a]*d[a];}
    if(now-g->start_us<100000 || g->samples<24)return FRAME_COLLECTING;
    double norm=0,diff=0,fs=0,ds=0;
    for(unsigned a=0;a<3;++a){
        const double x=g->fifo_sum[a]/g->samples,y=g->direct_sum[a]/g->samples;
        fs=fmax(fs,sqrt(fmax(0,g->fifo_square[a]/g->samples-x*x)));
        ds=fmax(ds,sqrt(fmax(0,g->direct_square[a]/g->samples-y*y)));
        norm+=y*y;diff+=(x-y)*(x-y);
    }
    g->difference_lsb=sqrt(diff);g->max_fifo_sigma=fs;g->max_direct_sigma=ds;clear_window(g,now);
    if(norm<6500.0*6500 || norm>10000.0*10000 || fs>400 || ds>400){g->bad_windows=0;return FRAME_DYNAMIC;}
    if(g->difference_lsb>2458){g->consistent=false;g->good_windows=0;if(g->bad_windows<UINT32_MAX)++g->bad_windows;return FRAME_MISMATCH;}
    g->bad_windows=0;if(g->good_windows<UINT32_MAX)++g->good_windows;
    if(g->good_windows>=2)g->consistent=true;
    return FRAME_AGREEMENT;
}
