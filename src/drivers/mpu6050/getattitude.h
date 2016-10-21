#ifndef _GETATTITUDE_H_
#define _GETATTITUDE_H_

extern void send_client(void);
static inline unsigned short inv_row_2_scale(const signed char *row);
static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
static void setup_gyro(void);
static void tap_cb(unsigned char direction, unsigned char count);
static void android_orient_cb(unsigned char orientation);
static inline void run_self_test(void);
static void gyro_data_ready_cb(void);
extern void DMP_init();
extern void zitaishuju();

#endif