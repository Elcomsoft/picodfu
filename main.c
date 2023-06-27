#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#include "pico/stdio.h"
#include "pico/multicore.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

#include "lightning_rx.pio.h"
#include "lightning_tx.pio.h"

#define PIN_ID0 3
#define PIN_ID1 2
const uint led_pin = 25;

bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;

    // Must disable interrupts, as interrupt handlers may be in flash, and we
    // are about to temporarily disable flash access!
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Note we can't call into any sleep functions in flash right now
    for (volatile int i = 0; i < 1000; ++i);

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // Note the button pulls the pin *low* when pressed.
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

    // Need to restore the state of chip select, else we are going to have a
    // bad time when we return to code in flash!
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);

    return button_state;
}

void configure_rx_internal(PIO pio, uint sm, int pin_sdq) {
    pio_sm_set_enabled(pio, sm, false);
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &lightning_rx_program);
    pio_sm_config c = lightning_rx_program_init(pio, sm, offset, pin_sdq, 125.0/2.0);
}

void configure_rx(PIO pio) {
    configure_rx_internal(pio, 0, PIN_ID0);
    configure_rx_internal(pio, 1, PIN_ID1);
}

void lightning_send_wake(int pid_sdq) {
    gpio_init(pid_sdq);
    gpio_set_dir(pid_sdq, GPIO_OUT);
    gpio_put(pid_sdq, 0);
    
    sleep_us(20);
    
    gpio_set_dir(pid_sdq, GPIO_IN);
    
    sleep_us(1000);
}

void tamarin_reset_tristar(PIO pio) {
    lightning_send_wake(PIN_ID0);    
    lightning_send_wake(PIN_ID1);    
    configure_rx(pio);
}

unsigned char reverse_byte(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

enum state {
    RESTART_ENUMERATION,
    WAITING_FOR_INIT,
    READING_TRISTAR_REQUEST,
    HANDLE_TRISTAR_REQUEST,
    HANDLE_JTAG,
    // Force JTAG mode if the cable is already in JTAG mode
    // (e.g. after the tamarin cable was reset but not the device)
    FORCE_JTAG
};

volatile enum state gState = RESTART_ENUMERATION;

enum command {
    CMD_DEFAULT,
    CMD_RESET,
    CMD_AUTO_DFU,
    // Used as 'second stage' for the DFU command. Saves us a state machine.
    CMD_INTERNAL_AUTO_DFU_2,
    CMD_MAX,
};

enum default_command {
    DEFAULT_CMD_DCSD = 0,
    DEFAULT_CMD_JTAG
};

// Command that should be sent automatically (on reboot, plugin, etc.)
// Automatically changed when selecting DCSD/JTAG
enum default_command gDefaultCommand = DEFAULT_CMD_DCSD;

// Next command to send
enum command gCommand = CMD_DEFAULT;

enum responses {
    // JTAG mode
    RSP_USB_UART_JTAG,
    // DCSD mode
    RSP_USB_UART,
    // Reset
    RSP_RESET,
    // DFU
    RSP_DFU,
    RSP_MAX
};

// To generate CRCs:
// hex(pwnlib.util.crc.generic_crc(b"\x75\x00\x00\x02\x00\x00\x00", 0x31, 8, 0xff, True, True, False))
const uint8_t bootloader_response[RSP_MAX][8] = {
    [RSP_USB_UART_JTAG] = {0x75, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40},
    [RSP_USB_UART] = {0x75, 0x20, 0x00, 0x10, 0x00, 0x00, 0x00, 0x92},
    [RSP_RESET] = {0x75, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x83},
    [RSP_DFU] = {0x75, 0x20, 0x00, 0x02, 0x00, 0x00, 0x00, 0xad},
};

void set_idbus_high_impedance_internal(int pin_sdq) {
    gpio_pull_up (pin_sdq);
    gpio_init(pin_sdq);
    gpio_set_dir(pin_sdq, GPIO_IN);
}
void set_idbus_high_impedance() {
    set_idbus_high_impedance_internal(PIN_ID0);
    set_idbus_high_impedance_internal(PIN_ID1);
}

void respond_lightning_internal(PIO pio, uint sm, int pin_sdq, const uint8_t *data, size_t data_length) {
    pio_sm_set_enabled(pio, sm, false);
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &lightning_tx_program);
    lightning_tx_program_init(pio, sm, offset, pin_sdq, 125.0/2.0);
    for(size_t i=0; i < data_length; i++) {
        pio_sm_put_blocking(pio, sm, data[i]);
    }
    while (!pio_sm_is_tx_fifo_empty(pio, sm)) {
        sleep_us(500);
    }
}

void respond_lightning(PIO pio, const uint8_t *data, size_t data_length) {
    respond_lightning_internal(pio, 0, PIN_ID0, data, data_length);
    respond_lightning_internal(pio, 1, PIN_ID1, data, data_length);
}

void set_dfu(){
    printf("Called set_DFU!\n");
    gCommand = CMD_AUTO_DFU;
    gState = RESTART_ENUMERATION;
}

void set_reset(){
    printf("Called set_DFU!\n");
    gCommand = CMD_RESET;
    gState = RESTART_ENUMERATION;
}

int buttonCNT = 0;

void output_state_machine() {
    PIO pio = pio1;

    uint8_t i = 0;
    uint8_t buf[4];

    uint32_t value, value_b;
    while(1) {
        if (get_bootsel_button()){
            if (++buttonCNT == 5){
                set_reset();
                buttonCNT = 0;
                for (size_t i = 0; i < 10; i++) {
                    gpio_put(led_pin, 0);
                    sleep_ms(100);
                    gpio_put(led_pin, 1);
                    sleep_ms(100);
                }                
            }else{
                set_dfu();
                gpio_put(led_pin, 0);
                sleep_ms(1000);
                gpio_put(led_pin, 1);
            }
        }
        switch(gState) {
            case RESTART_ENUMERATION:
                gpio_put(led_pin, 0);
                printf("Restarting enumeration!\r\n");
                tamarin_reset_tristar(pio);
                printf("Done restarting enumeration!\r\n");
                gState = WAITING_FOR_INIT;
                break;
                
            case WAITING_FOR_INIT:
                gpio_put(led_pin, 1);
                for (uint8_t sm = 0; sm < 2; sm++){
                    if (pio_sm_is_rx_fifo_empty(pio, sm)) continue;
                    value = pio_sm_get_blocking(pio, sm);
                    value_b = reverse_byte(value & 0xFF);
                    gpio_put(led_pin, 0);

                    if(value_b == 0x74) {
                        gState = READING_TRISTAR_REQUEST;
                        buf[0] = value_b;
                        i = 1;
                    } else {
                        printf("Tristar >> 0x%x (unknown, ignoring)\r\n", value_b);
                    }                    

                    sleep_us(100); // Breaks without this...
                    break;
                }
                
                break;
            case READING_TRISTAR_REQUEST:
                for (uint8_t sm = 0; sm < 2; sm++){
                    if (pio_sm_is_rx_fifo_empty(pio, sm)) continue;
                    value = pio_sm_get_blocking(pio, sm);
                    value_b = reverse_byte(value & 0xFF);

                    gpio_put(led_pin, 0);

                    buf[i++] = value_b;
                    if(i == 4) {
                        gState = HANDLE_TRISTAR_REQUEST;
                        i = 0;
                    }
                    
                    sleep_us(100); // Breaks without this...
                    break;
                }
                
                break;
            case HANDLE_TRISTAR_REQUEST:
                switch(gCommand) {
                    case CMD_DEFAULT:
                        switch (gDefaultCommand) {
                            case DEFAULT_CMD_DCSD:
                                respond_lightning(pio, bootloader_response[RSP_USB_UART], 8);
                                break;
                                
                            case DEFAULT_CMD_JTAG:
                                respond_lightning(pio, bootloader_response[RSP_USB_UART_JTAG], 8);
                                gState = FORCE_JTAG;
                                continue;
                        }
                        break;
                    case CMD_RESET:
                        respond_lightning(pio, bootloader_response[RSP_RESET], 8);
                        gpio_put(led_pin, 0);
                        sleep_us(1000);
                        gCommand = CMD_DEFAULT;
                        break;
                    case CMD_AUTO_DFU:
                        respond_lightning(pio, bootloader_response[RSP_RESET], 8);
                        // Measured with logic analyzer
                        gpio_put(led_pin, 0);
                        sleep_us(900);
                        gCommand = CMD_INTERNAL_AUTO_DFU_2;
                        break;
                    case CMD_INTERNAL_AUTO_DFU_2:
                        respond_lightning(pio, bootloader_response[RSP_DFU], 8);
                        printf("Device should now be in DFU mode.\r\n");
                        for (size_t i = 0; i < 3; i++) {
                            gpio_put(led_pin, 0);
                            sleep_ms(200);
                            gpio_put(led_pin, 1);
                            sleep_ms(200);
                        }    
                        break;
                    default:
                        printf("UNKNOWN MODE. Send help. Locking up.\r\n");
                        while(1) {}
                        break;
                }
                
                gState = WAITING_FOR_INIT;
                configure_rx(pio);
                break;
                
            default:
                printf("UNEXPECTED STATE!!!\n");
                break;
        }
    }
}

int main() {
    stdio_init_all();
    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    printf("Init");

    set_dfu();
    // set_reset();

    output_state_machine();
}