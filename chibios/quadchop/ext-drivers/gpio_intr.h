#ifndef GPIO_INT_H
#define GPIO_INT_H

typedef enum {
    GPIO_INT_FALLING,
    GPIO_INT_RISING,
    GPIO_INT_HIGH,
    GPIO_INT_LOW,
} int_type;

typedef void (*gpio_intr_func)(void *);

typedef struct _gpio_intr {
    //tbd PORT              <currently defaulted to port D>
    PeripheralPinConfig  pin;
    int_type    type;
    uint32_t    peripheral_id;
    gpio_intr_func  func;
    void  *ctx;
}gpio_intr_cfg_t;

#endif
