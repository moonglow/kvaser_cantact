#define DEFAULT               0
#define ENTREE                1

#if BOARD_ID==ENTREE
#define IOPIN_LED0    GPIO_PIN_1
#define IOPIN_LED1    GPIO_PIN_0
#define IOPIN_PORT    GPIOB
// default
#else
#define IOPIN_LED0    GPIO_PIN_1
#define IOPIN_LED1    GPIO_PIN_0
#define IOPIN_PORT    GPIOB
#endif
