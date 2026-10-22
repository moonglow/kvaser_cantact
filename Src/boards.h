#define DEFAULT               0
#define ENTREE                1
#define UCAN                  2

#if BOARD_ID==ENTREE
#define IOPIN_LED0    GPIO_PIN_1
#define IOPIN_LED1    GPIO_PIN_0
#define IOPIN_PORT    GPIOB
#elif BOARD_ID==UCAN
/* FYSETC UCAN V1.0: LED1 (TX) PA0, LED2 (RX) PA1, both active high */
#define IOPIN_LED0    GPIO_PIN_0
#define IOPIN_LED1    GPIO_PIN_1
#define IOPIN_PORT    GPIOA
/* SN65HVD230 Rs pin (net CAN_S, 10k pull-down R18): low = normal, high = standby */
#define IOPIN_CAN_S_PIN   GPIO_PIN_13
#define IOPIN_CAN_S_PORT  GPIOC
// default
#else
#define IOPIN_LED0    GPIO_PIN_1
#define IOPIN_LED1    GPIO_PIN_0
#define IOPIN_PORT    GPIOB
#endif
