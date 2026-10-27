#pragma once
#include <zephyr/drivers/can.h>
#include <esb_rx.h>

/* === CANopen / EPOS4 상수들 (STM 코드에서 복사) === */
#define CAN_ID_MCU 0x000
#define CAN_ID_MOTOR1 0x001
#define CAN_ID_MOTOR2 0x002
#define CAN_ID_MOTOR3 0x003
#define CAN_ID_MOTOR4 0x004

#define COB_ID_SDO_TX  0x00000580
#define COB_ID_SDO_RX  0x00000600
#define COB_ID_PDO1_TX 0x00000180
#define COB_ID_PDO1_RX 0x00000200
#define COB_ID_PDO2_TX 0x00000280
#define COB_ID_PDO2_RX 0x00000300
#define COB_ID_PDO3_TX 0x00000380
#define COB_ID_PDO3_RX 0x00000400
#define COB_ID_PDO4_TX 0x00000480
#define COB_ID_PDO4_RX 0x00000500

#define CS_READ_OD          0x40
#define CS_WRITE_OD_4BYTES  0x23
#define CS_WRITE_OD_3BYTES  0x27
#define CS_WRITE_OD_2BYTES  0x2B
#define CS_WRITE_OD_1BYTES  0x2F
#define CS_NMT_ENTER_OP     0x01
#define CS_NMT_ENTER_PREOP  0x80

#define INDEX_RxPDO1_PAR 0x1400
#define INDEX_RxPDO1_MAP 0x1600
#define INDEX_TxPDO1_PAR 0x1800
#define INDEX_TxPDO1_MAP 0x1A00

#define INDEX_ModesOfOperation   0x6060
#define INDEX_TargetTorque       0x6071
#define INDEX_TargetVelocity     0x60FF
#define INDEX_MotorRatedTorque   0x6076
#define OBJ_Controlword          0x60400010
#define OBJ_TargetTorque         0x60710010

#define PROFILE_POSITION_MODE              1
#define PROFILE_VELOCITY_MODE              3
#define HOMING_MODE                        6
#define CYCLIC_SYNCHRONOUS_POSITION_MODE   8
#define CYCLIC_SYNCHRONOUS_VELOCITY_MODE   9
#define CYCLIC_SYNCHRONOUS_TORQUE_MODE     10

#define EPOS4_CMD_SDOWN   0x06
#define EPOS4_CMD_SWION   0x07
#define EPOS4_CMD_ENA_OP  0x0F
#define EPOS4_CMD_DIS_VOL 0x00
#define EPOS4_CMD_QSTOP   0x02
#define EPOS4_CMD_DIS_OP  0x07
#define EPOS4_CMD_FLT_RST 0x80

#define CAN_BITRATE_BPS 1000000  // 1 Mbps

extern const struct device *can_dev;

/* CAN TX Frame */
extern struct can_frame TxCAN;
extern struct can_frame TxCAN_CSV;

/* Motor feedback values */
extern int32_t actual_position;
extern int32_t actual_velocity;
extern volatile bool pos_updated;

/* CAN functions */
void can_init(void);

void setup_can_filters(void);
void send_frame(const struct can_frame *f);
uint32_t get_can_send_fail_count(void);
void Enable_operation(void);

void pack_sdo(struct can_frame *f, uint32_t cob_id,
                     uint8_t CS, uint16_t Index,
                     uint8_t Subindex, uint32_t Value);
void pack_pdo_Controlword(struct can_frame *f, uint32_t cob_id,
                                 uint16_t Controlword);
void pack_pdo_CSV(struct can_frame *f, uint32_t cob_id, int32_t velocity);
void pack_pdo_CST(struct can_frame *f, uint32_t cob_id, int16_t torque);

void unpack_rpy(const struct can_frame *frame);
void tx_callback(const struct device *dev, int error, void *user_data);
void rx_callback(const struct device *dev,
                        struct can_frame *frame,
                        void *user_data);
