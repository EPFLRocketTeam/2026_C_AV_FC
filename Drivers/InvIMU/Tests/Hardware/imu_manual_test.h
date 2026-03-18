#ifndef INVIMU_TESTS_HARDWARE_IMU_MANUAL_TEST_H_
#define INVIMU_TESTS_HARDWARE_IMU_MANUAL_TEST_H_

#ifdef __cplusplus
extern "C" {
#endif

int manual_test_imu();
void imu_exti_callback();
void imu_dma_callback();

#ifdef __cplusplus
}
#endif

#endif
