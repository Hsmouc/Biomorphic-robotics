/******************** ********************

 * 描述         ：获取姿态信息
 * 
 *
**********************************************************************************/
#include "common.h"
#include "include.h"
#include "math.h"


/********************************DMP参数****************************/
unsigned char whoAmI = 0;
unsigned char gyo_XH = 0;
unsigned char gyo_XL = 0;
unsigned char gyo_YH = 0;
unsigned char gyo_YL = 0;
unsigned char gyo_ZH = 0;
unsigned char gyo_ZL = 0;
unsigned char acc_XH = 0;
unsigned char acc_XL = 0;
unsigned char acc_YH = 0;
unsigned char acc_YL = 0;
unsigned char acc_ZH = 0;
unsigned char acc_ZL = 0;

short gyo_X = 0;
short gyo_Y = 0;
short gyo_Z = 0;
short acc_X = 0;
short acc_Y = 0;
short acc_Z = 0;
short gyo_sj;

short int mag_X = 0;
short int mag_Y = 0;
short int mag_Z = 0;

#define q30  1073741824.0f
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

long int quat[4];
float Pitch = 0.0f;//俯仰
float Roll = 0.0f;//翻滚
float Yaw = 0.0f;//偏航


/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

struct rx_s {
  unsigned char header[3];
  unsigned char cmd;
};

struct hal_s {
  unsigned char sensors;
  unsigned char dmp_on;
  unsigned char wait_for_tap;
  volatile unsigned char new_gyro;
  unsigned short report;
  unsigned short dmp_features;
  unsigned char motion_int_mode;
  struct rx_s rx;
};
static struct hal_s hal = {0};

/* The sensors can be mounted onto the board in any orientation. The mounting
* matrix seen below tells the MPL how to rotate the raw data from thei
* driver(s).
* TODO: The following matrices refer to the configuration on an internal test
* board at Invensense. If needed, please modify the matrices to match the
* chip-to-body matrix for your particular set up.
*/
static signed char gyro_orientation[9] = {-1,  0,  0,
0, -1,  0,
0,  0,  1};

enum packet_type_e {
  PACKET_TYPE_ACCEL,
  PACKET_TYPE_GYRO,
  PACKET_TYPE_QUAT,
  PACKET_TYPE_TAP,
  PACKET_TYPE_ANDROID_ORIENT,
  PACKET_TYPE_PEDO,
  PACKET_TYPE_MISC
};




void send_client(void)
{
  //send_packet(PACKET_TYPE_QUAT, quat);
}

/* These next two functions converts the orientation matrix (see
* gyro_orientation) to a scalar representation for use by the DMP.
* NOTE: These functions are borrowed from Invensense's MPL.
*/
static inline unsigned short inv_row_2_scale(const signed char *row) {
  unsigned short b;
  
  if (row[0] > 0)
    b = 0;
  else if (row[0] < 0)
    b = 4;
  else if (row[1] > 0)
    b = 1;
  else if (row[1] < 0)
    b = 5;
  else if (row[2] > 0)
    b = 2;
  else if (row[2] < 0)
    b = 6;
  else
    b = 7; // error
  return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) {
  unsigned short scalar;
  
  /*
  XYZ  010_001_000 Identity Matrix
  XZY  001_010_000
  YXZ  010_000_001
  YZX  000_010_001
  ZXY  001_000_010
  ZYX  000_001_010
  */
  
  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;
  
  return scalar;
}

/* Handle sensor on/off combinations. */
static void setup_gyro(void) {
  unsigned char mask = 0;
  if (hal.sensors & ACCEL_ON)
    mask |= INV_XYZ_ACCEL;
  if (hal.sensors & GYRO_ON)
    mask |= INV_XYZ_GYRO;
  /* If you need a power transition, this function should be called with a
  * mask of the sensors still enabled. The driver turns off any sensors
  * excluded from this mask.
  */
  mpu_set_sensors(mask);
  if (!hal.dmp_on)
    mpu_configure_fifo(mask);
}

static void tap_cb(unsigned char direction, unsigned char count) {
  char data[2];
  data[0] = (char)direction;
  data[1] = (char)count;
  //send_packet(PACKET_TYPE_TAP, data);
}

static void android_orient_cb(unsigned char orientation) {
  //send_packet(PACKET_TYPE_ANDROID_ORIENT, &orientation);
}

static inline void run_self_test(void) {
  int result;
  char test_packet[4] = {0};
  long gyro[3], accel[3];
  
  result = mpu_run_self_test(gyro, accel);
  if (result == 0x7) {
    /* Test passed. We can trust the gyro data here, so let's push it down
    * to the DMP.
    */
    float sens;
    unsigned short accel_sens;
    mpu_get_gyro_sens(&sens);
    gyro[0] = (long) (gyro[0] * sens);
    gyro[1] = (long) (gyro[1] * sens);
    gyro[2] = (long) (gyro[2] * sens);
    dmp_set_gyro_bias(gyro);
    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    dmp_set_accel_bias(accel);
  }
  
  /* Report results. */
  test_packet[0] = 't';
  test_packet[1] = result;
  //send_packet(PACKET_TYPE_MISC, test_packet);
}

static void gyro_data_ready_cb(void) {
  hal.new_gyro = 1;
}
/********************************DMP参数***************************/

u8 dp;

#define ABS(i)  (((i)>=0)?(i):(0-(i)))//绝对值



void DMP_init()
{
     /******************************DMP初始化********************/ 
  
    int result;
    /* unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    struct int_param_s int_param;
    unsigned long sensor_timestamp;
    //short gyro[3], accel[3];
    unsigned char  more;
    
    short gyro[3], accel[3], sensors;*/
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    struct int_param_s int_param;
    //unsigned char more;
    
    // pic32 hardware initialize
    
    #ifdef DELUG_USE_TERMINAL
    UART_send_str(">>> Hardware Init Successful!\n");
    #endif
    
    /* Set up gyro.
    * Every function preceded by mpu_ is a driver function and can be found
    * in inv_mpu.h.
    */
    int_param.cb = gyro_data_ready_cb;
    int_param.active_low = 1;
    result = mpu_init(&int_param);
    #ifdef DELUG_USE_TERMINAL
    if (0 == result)
    UART_send_str(">>> Mpu6050  Init Successful!\n");
    #endif
    
    /* If you're not using an MPU9150 AND you're not using DMP features, this
    * function will place all slaves on the primary bus.
    * mpu_set_bypass(1);
    */
    
    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    
    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof (hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.report = PRINT_QUAT;
    
    /* To initialize the DMP:
    * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
    *    inv_mpu_dmp_motion_driver.h into the MPU memory.
    * 2. Push the gyro and accel orientation matrix to the DMP.
    * 3. Register gesture callbacks. Don't worry, these callbacks won't be
    *    executed unless the corresponding feature is enabled.
    * 4. Call dmp_enable_feature(mask) to enable different features.
    * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
    * 6. Call any feature-specific control functions.
    *
    * To enable the DMP, just call mpu_set_dmp_state(1). This function can
    * be called repeatedly to enable and disable the DMP at runtime.
    *
    * The following is a short summary of the features supported in the DMP
    * image provided in inv_mpu_dmp_motion_driver.c:
    * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
    * 200Hz. Integrating the gyro data at higher rates reduces numerical
    * errors (compared to integration on the MCU at a lower sampling rate).
    * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
    * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
    * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
    * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
    * an event at the four orientations where the screen should rotate.
    * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
    * no motion.
    * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
    * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
    * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
    * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
    */
    result = dmp_load_motion_driver_firmware();
    #ifdef DELUG_USE_TERMINAL
    if (0 == result)
    UART_send_str(">>> Load Firmware Successful!\n");
    #endif
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                       DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                       DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
    DelayMs(2);

/******************************DMP初始化********************/ 
}

void zitaishuju()
{

    unsigned long sensor_timestamp;
    //short gyro[3], accel[3];
    unsigned char  more;
    
    short gyro[3], accel[3], sensors;
    //unsigned long sensor_timestamp;
    //gpio_set(PORTE,4,1);
    hal.new_gyro = 1;
    // 未使能传感器或未收到新数据
    if (!hal.sensors || !hal.new_gyro)
    {
    // 可以在此处休眠，以降低功耗
    //continue;
    }
    // 传感器数据准备好并且打开DMP功能
    if (hal.new_gyro && hal.dmp_on)
    {
        /* short gyro[3], accel[3], sensors;
        unsigned char more;*/
        //long quat[4];
        /* This function gets new data from the FIFO when the DMP is in
        * use. The FIFO can contain any combination of gyro, accel,
        * quaternion, and gesture data. The sensors parameter tells the
        * caller which data fields were actually populated with new data.
        * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
        * the FIFO isn't being filled with accel data.
        * The driver parses the gesture data to determine if a gesture
        * event has occurred; on an event, the application will be notified
        * via a callback (assuming that a callback function was properly
        * registered). The more parameter is non-zero if there are
        * leftover packets in the FIFO.
        */
        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
        //acc_X = mpu6050_getdata('A', 'X');
        // acc_Y = mpu6050_getdata('A', 'Y');
        // acc_Z = mpu6050_getdata('A', 'Z');
        //HMC_SingleWrite(0x0A, 0x01);
        //DelayMs(7);
        /* mag_X = HMC_SingleRead(HXH);
        mag_X = mag_X<<8;
        mag_X += HMC_SingleRead(HXL);
        mag_Y = HMC_SingleRead(HYH);
        mag_Y = mag_Y<<8;
        mag_Y += HMC_SingleRead(HYL);
        mag_Z = HMC_SingleRead(HZH);
        mag_Z = mag_Z<<8;
        mag_Z += HMC_SingleRead(HZL);*/
        
        if (!more)
        {
            hal.new_gyro = 0;
        }
        /* Gyro and accel data are written to the FIFO by the DMP in chip
        * frame and hardware units. This behavior is convenient because it
        * keeps the gyro and accel outputs of dmp_read_fifo and
        * mpu_read_fifo consistent.
        */
        //if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
        //send_packet(PACKET_TYPE_GYRO, gyro);
        //if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
        //send_packet(PACKET_TYPE_ACCEL, accel);
        /* Unlike gyro and accel, quaternions are written to the FIFO in
        * the body frame, q30. The orientation is set by the scalar passed
        * to dmp_set_orientation during initialization.
        */
        if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
        {
            q0=quat[0] / q30;
            q1=quat[1] / q30;
            q2=quat[2] / q30;
            q3=quat[3] / q30;
            
            
            
            Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
            Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
            Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3+180;//agnel; //yaw
            if(Yaw>360)
            {
              Yaw=Yaw-360;
            }
        }
    }
    else if (hal.new_gyro)
    {
        short gyro[3], accel[3];
        unsigned char sensors, more;
        /* This function gets new data from the FIFO. The FIFO can contain
        * gyro, accel, both, or neither. The sensors parameter tells the
        * caller which data fields were actually populated with new data.
        * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
        * being filled with accel data. The more parameter is non-zero if
        * there are leftover packets in the FIFO.
        */
        mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more);
        if (!more)
          hal.new_gyro = 0;
        //if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO);
          //send_packet(PACKET_TYPE_GYRO, gyro);
        //if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL);
          //send_packet(PACKET_TYPE_ACCEL, accel);
    }
    
    //DelayMs(2);
    //gpio_turn(PORTE,4);
}
