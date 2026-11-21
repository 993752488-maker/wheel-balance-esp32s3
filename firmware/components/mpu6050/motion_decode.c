#include "motion_decode.h"
#include <string.h>
static int16_t be(const uint8_t *p)
{
    const uint16_t value=((uint16_t)p[0]<<8)|p[1];
    return value<0x8000?(int16_t)value:(int16_t)((int32_t)value-65536);
}
bool motion_decode(const uint8_t *data,size_t n,motion_snapshot_t *out)
{
    if(!out)return false;
    memset(out,0,sizeof(*out));
    if(!data || n!=14)return false;
    for(unsigned i=0;i<3;++i){out->accel[i]=be(data+2*i);out->gyro[i]=be(data+8+2*i);}
    out->temperature=be(data+6);return true;
}
