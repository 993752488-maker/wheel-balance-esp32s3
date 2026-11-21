#include "frame_probe.h"
#include "board_config.h"
#include "fifo_guard.h"
#include "mpu6050.h"
#include "serial_console.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

/* This executable deliberately fills / misaligns FIFO. It is a separate,
 * compile-time bench mode, not reachable from the phone or capture console.
 * All comparisons assume a stationary sensor and are not phase calibration. */
enum { REG_FIFO_EN=0x23, REG_INT_ENABLE=0x38, REG_INT_STATUS=0x3a,
       REG_FIFO_COUNT=0x72, REG_FIFO_RW=0x74 };
static unsigned s_passed, s_failed;

static void delay_ms(unsigned ms)
{
    TickType_t ticks=pdMS_TO_TICKS(ms);
    vTaskDelay(ticks?ticks:1);
}

static esp_err_t read_reg(mpu6050_t *d,uint8_t reg,uint8_t *out,size_t n)
{
    return i2c_master_transmit_receive(d->device,&reg,1,out,n,50);
}

static esp_err_t write_reg(mpu6050_t *d,uint8_t reg,uint8_t value)
{
    uint8_t packet[]={reg,value},actual=0;
    esp_err_t err=i2c_master_transmit(d->device,packet,sizeof(packet),50);
    if(err==ESP_OK)err=read_reg(d,reg,&actual,1);
    return err==ESP_OK && actual!=value?ESP_ERR_INVALID_RESPONSE:err;
}

static esp_err_t raw_count(mpu6050_t *d,uint16_t *count)
{
    uint8_t bytes[2]={0};
    const esp_err_t err=read_reg(d,REG_FIFO_COUNT,bytes,sizeof(bytes));
    if(err==ESP_OK)*count=((uint16_t)bytes[0]<<8)|bytes[1];
    return err;
}

static int16_t signed_be(const uint8_t *b)
{
    const uint16_t bits=((uint16_t)b[0]<<8)|b[1];
    return bits<0x8000?(int16_t)bits:(int16_t)((int32_t)bits-65536);
}

static bool agreement(mpu6050_t *d,unsigned milliseconds,const char *name)
{
    frame_guard_t guard;frame_guard_init(&guard);
    uint32_t frames_read=0;
    esp_err_t err=ESP_OK;
    const int64_t begin=esp_timer_get_time();
    while(esp_timer_get_time()-begin<(int64_t)milliseconds*1000) {
        uint16_t count=0;uint8_t status=0;
        err=mpu6050_fifo_count(d,&count,&status);
        if(err!=ESP_OK || (status&0x10) || count>=800)break;
        size_t n=count/6;if(n>32)n=32;
        if(!n){delay_ms(2);continue;}
        mpu6050_raw_t fifo[32],direct;
        err=mpu6050_fifo_read(d,fifo,n);
        if(err==ESP_OK)err=mpu6050_read_accel_registers(d,&direct);
        if(err!=ESP_OK)break;
        const int16_t value[]={direct.x,direct.y,direct.z};
        const uint64_t now=(uint64_t)esp_timer_get_time();
        for(size_t i=0;i<n;++i) {
            const int16_t f[]={fifo[i].x,fifo[i].y,fifo[i].z};
            (void)frame_guard_feed(&guard,f,value,now,count/6);
        }
        frames_read+=(uint32_t)n;
        delay_ms(10);
    }
    const bool pass=err==ESP_OK && guard.consistent && !guard.bad_windows && frames_read>100;
    serial_console_printf("FRAME_PROBE_CHECK name=%s pass=%u frames=%" PRIu32
        " consistent=%u diff_lsb=%.1f bad_windows=%u err=%s qualified=0\n",
        name,pass,frames_read,guard.consistent,guard.difference_lsb,guard.bad_windows,esp_err_to_name(err));
    if(pass)++s_passed;else ++s_failed;
    return pass;
}

static void partial_read(mpu6050_t *d,unsigned bytes)
{
    esp_err_t err=mpu6050_configure(d,BOARD_MPU_SAMPLE_HZ);
    delay_ms(40);
    uint8_t scratch[4]={0};
    if(err==ESP_OK)err=read_reg(d,REG_FIFO_RW,scratch,bytes);
    uint16_t observed=0,selected=0;uint8_t status=0;
    if(err==ESP_OK)err=raw_count(d,&observed);
    const uint32_t before=d->fifo_count_anomalies;
    const esp_err_t rejected=err==ESP_OK?mpu6050_fifo_count(d,&selected,&status):err;
    const bool pass=err==ESP_OK && rejected==ESP_ERR_INVALID_RESPONSE &&
        d->fifo_count_anomalies==before+1 && !d->sample_hz && selected==0;
    serial_console_printf("FRAME_PROBE_INJECTION kind=partial_fifo_read bytes=%u count=%u"
        " pass=%u guard_err=%s sample_hz=%u count_faults=%" PRIu32 " qualified=0\n",
        bytes,observed,pass,esp_err_to_name(rejected),d->sample_hz,d->fifo_count_anomalies);
    if(pass)++s_passed;else ++s_failed;
    err=mpu6050_configure(d,BOARD_MPU_SAMPLE_HZ);
    if(err==ESP_OK)(void)agreement(d,800,"after_partial_reconfigure");
    else ++s_failed;
}

static void stale_count_read(mpu6050_t *d,uint8_t interrupts)
{
    esp_err_t err=mpu6050_configure(d,BOARD_MPU_SAMPLE_HZ);
    if(err==ESP_OK)err=write_reg(d,REG_INT_ENABLE,interrupts);
    delay_ms(50);
    uint16_t count=0;uint8_t status=0;
    if(err==ESP_OK)err=mpu6050_fifo_count(d,&count,&status);
    const bool snapshot_ok=err==ESP_OK && !(status&0x10) && count>=6 && count<828;
    size_t n=count/6;if(n>32)n=32;
    /* Deliberate pause AFTER a valid count/status, BEFORE consuming bytes.
     * This tests the real driver without a production fault-injection hook. */
    delay_ms(500);
    mpu6050_raw_t batch[32];
    const uint32_t before=d->fifo_read_overflows;
    const esp_err_t rejected=snapshot_ok?mpu6050_fifo_read(d,batch,n):ESP_FAIL;
    const bool pass=snapshot_ok && rejected==ESP_ERR_INVALID_SIZE &&
        d->fifo_read_overflows==before+1 && !d->sample_hz;
    serial_console_printf("FRAME_PROBE_INJECTION kind=stale_count_read int_enable=0x%02x"
        " cached_bytes=%u requested_frames=%u pass=%u guard_err=%s post_read_overflows=%" PRIu32
        " sample_hz=%u discard_entire_batch=1 deliberate_stall=1 qualified=0\n",
        interrupts,count,(unsigned)n,pass,esp_err_to_name(rejected),d->fifo_read_overflows,d->sample_hz);
    if(pass)++s_passed;else ++s_failed;
    err=mpu6050_configure(d,BOARD_MPU_SAMPLE_HZ);
    if(err==ESP_OK)(void)agreement(d,800,"after_stale_count_reconfigure");
    else ++s_failed;
}

static void fill_and_inspect(mpu6050_t *d,uint8_t interrupts)
{
    esp_err_t err=mpu6050_configure(d,BOARD_MPU_SAMPLE_HZ);
    if(err==ESP_OK)err=write_reg(d,REG_INT_ENABLE,interrupts);
    uint8_t initial=0;
    if(err==ESP_OK)err=read_reg(d,REG_INT_STATUS,&initial,1);
    uint8_t seen_status=0;
    uint16_t max_count=0,previous=0;
    const int64_t begin=esp_timer_get_time();
    for(unsigned i=0;i<16 && err==ESP_OK;++i) {
        delay_ms(40);
        uint16_t count=0;uint8_t status=0;
        err=raw_count(d,&count);
        if(err==ESP_OK)err=read_reg(d,REG_INT_STATUS,&status,1);
        seen_status|=status;
        if(count>max_count)max_count=count;
        serial_console_printf("FRAME_PROBE_FILL int_enable=0x%02x elapsed_us=%" PRId64
            " count=%u previous=%u status=0x%02x err=%s deliberate_stall=1 qualified=0\n",
            interrupts,esp_timer_get_time()-begin,count,previous,status,esp_err_to_name(err));
        previous=count;
    }
    if(err==ESP_OK)err=write_reg(d,REG_FIFO_EN,0); /* freeze before forensic reads */
    uint16_t frozen=0;
    if(err==ESP_OK)err=raw_count(d,&frozen);
    mpu6050_raw_t direct={0};
    if(err==ESP_OK)err=mpu6050_read_accel_registers(d,&direct);
    double mean[3]={0};unsigned samples=0;
    /* Bypass the production API ONLY to observe what a full FIFO contains.
     * These values are never sent to acquisition or stored as valid capture. */
    for(unsigned i=0;i<24 && err==ESP_OK && frozen>=144;++i) {
        uint8_t bytes[6];err=read_reg(d,REG_FIFO_RW,bytes,sizeof(bytes));
        if(err!=ESP_OK)break;
        for(unsigned a=0;a<3;++a)mean[a]+=signed_be(bytes+2*a);
        ++samples;
    }
    double difference[3]={0};
    const int16_t direct_axis[]={direct.x,direct.y,direct.z};
    if(samples)for(unsigned a=0;a<3;++a)mean[a]/=samples;
    for(unsigned shift=0;shift<3;++shift)for(unsigned a=0;a<3;++a) {
        const double delta=mean[a]-direct_axis[(a+shift)%3];
        difference[shift]+=delta*delta;
    }
    uint16_t selected=0;uint8_t status=0;
    const esp_err_t rejected=err==ESP_OK?mpu6050_fifo_count(d,&selected,&status):err;
    const bool blocked=err==ESP_OK && (rejected!=ESP_OK || (status&0x10) || selected>=828);
    serial_console_printf("FRAME_PROBE_FULL int_enable=0x%02x max_count=%u frozen_count=%u"
        " overflow_seen=%u fifo_mean=%.1f:%.1f:%.1f direct=%d:%d:%d"
        " identity_diff=%.1f shift1_diff=%.1f shift2_diff=%.1f"
        " production_blocked=%u guard_err=%s deliberate_stall=1 qualified=0\n",
        interrupts,max_count,frozen,!!(seen_status&0x10),mean[0],mean[1],mean[2],
        direct.x,direct.y,direct.z,sqrt(difference[0]),sqrt(difference[1]),sqrt(difference[2]),
        blocked,esp_err_to_name(rejected));
    if(blocked && samples==24)++s_passed;else ++s_failed;
    err=mpu6050_configure(d,BOARD_MPU_SAMPLE_HZ);
    if(err==ESP_OK)(void)agreement(d,800,"after_full_reconfigure");
    else ++s_failed;
}

void frame_probe_run(void)
{
    serial_console_printf("FRAME_PROBE_BEGIN source=physical_static_bench optical=DISABLED"
        " balance=DISABLED deliberate_fault_injection=1 qualified=0\n");
    mpu6050_t sensor={0};
    esp_err_t err=mpu6050_create(&sensor,BOARD_MPU_SDA_GPIO,BOARD_MPU_SCL_GPIO,
        BOARD_MPU_ADDRESS,BOARD_I2C_SPEED_HZ);
    if(err==ESP_OK)err=mpu6050_configure(&sensor,BOARD_MPU_SAMPLE_HZ);
    if(err==ESP_OK) {
        (void)agreement(&sensor,1500,"baseline");
        partial_read(&sensor,2);
        partial_read(&sensor,4);
        /* Same physical chip, same bus, only the source-enable bits differ. */
        fill_and_inspect(&sensor,0x00);
        fill_and_inspect(&sensor,0x10);
        stale_count_read(&sensor,0x00);
        stale_count_read(&sensor,0x10);
        err=mpu6050_configure(&sensor,BOARD_MPU_SAMPLE_HZ);
        if(err==ESP_OK)(void)agreement(&sensor,3000,"final_recovery");
        else ++s_failed;
    }else ++s_failed;
    if(sensor.device) {
        (void)write_reg(&sensor,REG_FIFO_EN,0);
        mpu6050_destroy(&sensor);
    }
    while(true) {
        serial_console_printf("FRAME_PROBE_DONE passed=%u failed=%u sensor_stopped=1"
            " source=physical_static_bench qualified=0\n",s_passed,s_failed);
        delay_ms(1000);
    }
}
