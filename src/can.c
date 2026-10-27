#include <can.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(can, LOG_LEVEL_INF);

static atomic_t can_send_fail_count = ATOMIC_INIT(0);

uint32_t get_can_send_fail_count(void)
{
    return (uint32_t)atomic_get(&can_send_fail_count);
}

const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

/* CAN 프레임 구조 */
struct can_frame TxCAN;
struct can_frame TxCAN_CSV;

int32_t actual_position = 0;
int32_t actual_velocity = 0;
volatile bool pos_updated = false;

void can_init(void){
	/* MCP2515 드라이버 준비 확인 */
    if (!device_is_ready(can_dev)) {
        LOG_WRN("MCP2515 not ready");
        return;
    }

    k_msleep(1); 

    /* 2) CAN 시작 */
    int ret = can_start(can_dev);
    if (ret) {
        LOG_WRN("can_start failed (%d)", ret);
        return;
    }
    k_msleep(1);

    setup_can_filters();
    k_msleep(1);

    /* EPOS4 설정 */
    pack_sdo(&TxCAN,
             COB_ID_SDO_RX + 1,
             CS_WRITE_OD_1BYTES,
             INDEX_ModesOfOperation,
             0,
             CYCLIC_SYNCHRONOUS_VELOCITY_MODE);
    LOG_INF("EPOS4: set ModesOfOperation (SDO)");
    send_frame(&TxCAN);
    k_msleep(10);

    // NMT Operational
    LOG_INF("EPOS4: send NMT Operational");
    pack_sdo(&TxCAN,
             0,
             CS_NMT_ENTER_OP,
             0x00, 0, 0);
    send_frame(&TxCAN);
    k_msleep(10);

    LOG_INF("EPOS4: Enable_operation()");
    Enable_operation();
    k_msleep(10);
}

/* ----------- SDO / PDO 패킹 함수 그대로 유지 ----------- */
void pack_sdo(struct can_frame *f, uint32_t cob_id,
                     uint8_t CS, uint16_t Index,
                     uint8_t Subindex, uint32_t Value)
{
    f->id    = cob_id;
    f->dlc   = 8;
    f->flags = 0;

    f->data[0] = CS;
    f->data[1] = Index & 0xFF;
    f->data[2] = Index >> 8;
    f->data[3] = Subindex;
    f->data[4] = Value & 0xFF;
    f->data[5] = (Value >> 8)  & 0xFF;
    f->data[6] = (Value >> 16) & 0xFF;
    f->data[7] = (Value >> 24) & 0xFF;
}

void pack_pdo_Controlword(struct can_frame *f, uint32_t cob_id,
                                 uint16_t Controlword)
{
    f->id    = cob_id;
    f->dlc   = 2;
    f->flags = 0;

    f->data[0] = Controlword & 0xFF;
    f->data[1] = (Controlword >> 8) & 0xFF;
}

void pack_pdo_CST(struct can_frame *f, uint32_t cob_id, int16_t torque)
{
    f->id    = cob_id;
    f->dlc   = 2;
    f->flags = 0;

    f->data[0] = torque & 0xFF;
    f->data[1] = (torque >> 8) & 0xFF;
}

void pack_pdo_CSV(struct can_frame *f, uint32_t cob_id, int32_t velocity)
{
    f->id    = cob_id;      // COB-ID 예: 0x300 + node_id
    f->dlc   = 4;           // CSV는 4바이트
    f->flags = 0;           // Standard CAN frame

    f->data[0] = velocity & 0xFF;
    f->data[1] = (velocity >> 8) & 0xFF;
    f->data[2] = (velocity >> 16) & 0xFF;
    f->data[3] = (velocity >> 24) & 0xFF;
}

/* ----------- EPOS4 PDO 수신 파싱 (모터 1번만) ----------- */
void unpack_rpy(const struct can_frame *frame)
{
    uint32_t id = frame->id;

    /* TxPDO2 (Actual Position) → ID = 0x280 + 1 = 0x281 */
    if (id == COB_ID_PDO2_TX + CAN_ID_MOTOR1)
    {
        actual_position =
            (frame->data[3] << 24) |
            (frame->data[2] << 16) |
            (frame->data[1] <<  8) |
             frame->data[0];

        pos_updated = true;
    }
    /* TxPDO3 (Actual Velocity) → ID = 0x380 + 1 = 0x381 */
    else if (id == COB_ID_PDO3_TX + CAN_ID_MOTOR1)
    {
        actual_velocity =
            (frame->data[3] << 24) |
            (frame->data[2] << 16) |
            (frame->data[1] <<  8) |
             frame->data[0];
    }
}

/* ----------- CAN 공통 함수 ----------- */
void tx_callback(const struct device *dev, int error, void *user_data)
{
    if (error != 0) {
        LOG_WRN("CAN TX error: %d", error);
    }
}

void send_frame(const struct can_frame *f)
{
    int ret = can_send(can_dev, (struct can_frame *)f,
                   K_NO_WAIT,            
                   tx_callback,      
                   NULL);                
    if (ret) {
        atomic_inc(&can_send_fail_count);
    }
}

void Enable_operation(void)
{
    int i = 1;  // 모터 1번만

    pack_pdo_Controlword(&TxCAN, COB_ID_PDO1_RX + i, EPOS4_CMD_FLT_RST);
    send_frame(&TxCAN);
    k_msleep(1);

    pack_pdo_Controlword(&TxCAN, COB_ID_PDO1_RX + i, EPOS4_CMD_SDOWN);
    send_frame(&TxCAN);
    k_msleep(1);

    pack_pdo_Controlword(&TxCAN, COB_ID_PDO1_RX + i, EPOS4_CMD_SWION);
    send_frame(&TxCAN);
    k_msleep(1);

    pack_pdo_Controlword(&TxCAN, COB_ID_PDO1_RX + i, EPOS4_CMD_ENA_OP);
    send_frame(&TxCAN);
    k_msleep(1);
}

/* RX 콜백 */
void rx_callback(const struct device *dev,
                        struct can_frame *frame,
                        void *user_data)
{
    unpack_rpy(frame);
}

/* 모든 메시지 수신 */
void setup_can_filters(void)
{
    struct can_filter filter = {
        .id    = 0x000,
        .mask  = 0x000,
        .flags = 0,
    };

    int filter_id = can_add_rx_filter(can_dev, rx_callback, NULL, &filter);
    if (filter_id < 0) {
        LOG_WRN("RX filter install failed (%d)", filter_id);
    } else {
        LOG_INF("RX filter installed: %d", filter_id);
    }
}
