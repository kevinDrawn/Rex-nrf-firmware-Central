#include <uart_bridge.h>

LOG_MODULE_REGISTER(uart_bridge_c, LOG_LEVEL_INF);

#define DATA_UART_NODE DT_NODELABEL(uart0)

static volatile bool tx_busy;
static uint8_t tx_buf[FRAME_LEN];
const struct device *uart_dev;

/* UART TX 큐 */
K_MSGQ_DEFINE(uart_tx_q, FRAME_LEN, 64, 4);

static void uart_evt_cb(const struct device *dev,
                        struct uart_event *evt,
                        void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    switch (evt->type) {
    case UART_TX_DONE:
    case UART_TX_ABORTED:
        tx_busy = false;
        uart_tx_kick();
        break;
    default:
        break;
    }
}

// /* 다음 전송 시도 */
// void uart_tx_kick(void)
// {
//     if (tx_busy || !uart_dev) return;

//     if (k_msgq_get(&uart_tx_q, tx_buf, K_NO_WAIT) != 0)
//         return;

//     int rc = uart_tx(uart_dev, tx_buf, FRAME_LEN, SYS_FOREVER_MS);
//     if (rc == 0)
//         tx_busy = true;
//     else if (rc == -EBUSY)
//         (void)k_msgq_put(&uart_tx_q, tx_buf, K_NO_WAIT);
//     else {
//         (void)k_msgq_put(&uart_tx_q, tx_buf, K_NO_WAIT);
//         k_sleep(K_MSEC(1));
//     }
// }

void uart_tx_kick(void)
{
    if (tx_busy) return;

    if (k_msgq_get(&uart_tx_q, tx_buf, K_NO_WAIT) != 0)
        return;

    tx_busy = true;
    uart_tx(uart_dev, tx_buf, FRAME_LEN, SYS_FOREVER_MS);
}

int uart_bridge_init(void)
{
    uart_dev = DEVICE_DT_GET(DATA_UART_NODE);
    if (!device_is_ready(uart_dev))
        return -ENODEV;

    int rc = uart_callback_set(uart_dev, uart_evt_cb, NULL);
    if (rc)
        return rc;

    tx_busy = false;

    LOG_INF("Clear UART Init");
    return 0;
}
