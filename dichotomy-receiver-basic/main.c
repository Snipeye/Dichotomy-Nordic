
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "nrf_drv_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_gzll.h"

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */


#define RX_PIN_NUMBER  25
#define TX_PIN_NUMBER  24
#define CTS_PIN_NUMBER 23
#define RTS_PIN_NUMBER 22
#define HWFC           false


// Define payload length
#define TX_PAYLOAD_LENGTH 7 // 7-byte payload length for each half: 3 for keystate, 1 for rotary encoder button, 2 for optical encoder, 1 for rotary encoder

#define CHANNEL_SIZE 4
static uint8_t gzll_channels[CHANNEL_SIZE] = {1, 41, 83, 119};

// How long before we assume a keyboard is inactive and 0 out the values?  Hard to say, but experimentally this works well with MAX_SEND_DELAY 100 in the transmitter
#define INACTIVE 100000

// Binary printing
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x10 ? '#' : '.'), \
  (byte & 0x08 ? '#' : '.'), \
  (byte & 0x04 ? '#' : '.'), \
  (byte & 0x02 ? '#' : '.'), \
  (byte & 0x01 ? '#' : '.')


// Data and acknowledgement payloads
static uint8_t data_payload_left[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; // Placeholder for data payload received from host.
static uint8_t data_payload_right[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; // Placeholder for data payload received from host.
static uint8_t ack_payload[TX_PAYLOAD_LENGTH]; // Payload to attach to ACK sent to device.
static uint8_t key_data_buffer[11] = {0};

// Debug helper variables
extern nrf_gzll_error_code_t nrf_gzll_error_code; // Error code
static bool packet_received_left, packet_received_right;
uint32_t left_active = 0;
uint32_t right_active = 0;
uint8_t c;


void uart_error_handle(app_uart_evt_t * p_event) {
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

#define LB0_MASK 0x03
#define RB0_MASK 0xFC
#define LB1_MASK 0xF0
#define RB1_MASK 0x0F
#define LB2_MASK 0x3F
#define RB2_MASK 0xC0
#define LB3_MASK 0x03
#define RB3_MASK 0xFC
#define LB4_MASK 0xF1
#define RB4_MASK 0x0E
#define LB5_MASK 0xC7
#define RB5_MASK 0x38


int main(void) {
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
        {
            RX_PIN_NUMBER,
            TX_PIN_NUMBER,
            RTS_PIN_NUMBER,
            CTS_PIN_NUMBER,
            APP_UART_FLOW_CONTROL_DISABLED,
            false,
            UART_BAUDRATE_BAUDRATE_Baud1M
            //UART_BAUDRATE_BAUDRATE_Baud115200
        };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);

    // Initialize Gazell
    nrf_gzll_init(NRF_GZLL_MODE_HOST);
    nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_4_DBM);
    nrf_gzll_set_channel_table(gzll_channels, CHANNEL_SIZE);
    nrf_gzll_set_datarate(NRF_GZLL_DATARATE_2MBIT);
    nrf_gzll_set_sync_lifetime(8);
    nrf_gzll_set_timeslot_period(600);
    nrf_gzll_set_timeslots_per_channel(2);

    // Addressing - these are arbitrary, but work.  There will be interference if you have other devices using these addresses
    nrf_gzll_set_base_address_0(0x01020304);
    nrf_gzll_set_base_address_1(0x05060708);

    // Load data into TX queue
    ack_payload[0] = 0x55;
    nrf_gzll_add_packet_to_tx_fifo(0, data_payload_left, TX_PAYLOAD_LENGTH);
    nrf_gzll_add_packet_to_tx_fifo(1, data_payload_left, TX_PAYLOAD_LENGTH);

    // Enable Gazell to start sending over the air
    nrf_gzll_enable();

    // Main loop
    while (true) {
        // Detecting received packet from interupt, and unpacking
        if (packet_received_left) {
            packet_received_left = false;

            // First, 0-out old keystates from the left side
    	    key_data_buffer[0] = key_data_buffer[0] & LB0_MASK;
    	    key_data_buffer[1] = key_data_buffer[1] & LB1_MASK;
    	    key_data_buffer[2] = key_data_buffer[2] & LB2_MASK;
    	    key_data_buffer[3] = key_data_buffer[3] & LB3_MASK;
    	    key_data_buffer[4] = key_data_buffer[4] & LB4_MASK;
    	    key_data_buffer[5] = key_data_buffer[5] & LB5_MASK;

            // Now read in/decode the new keystates.
            key_data_buffer[0] |= ((data_payload_left[0] & 1<<2) ? 1:0) << 7 |
    				              ((data_payload_left[0] & 1<<3) ? 1:0) << 6 |
                				  ((data_payload_left[0] & 1<<4) ? 1:0) << 5 |
                				  ((data_payload_left[0] & 1<<5) ? 1:0) << 4 |
                				  ((data_payload_left[0] & 1<<6) ? 1:0) << 3 |
                				  ((data_payload_left[0] & 1<<7) ? 1:0) << 2;

    	    key_data_buffer[1] |= ((data_payload_left[1] & 1<<4) ? 1:0) << 3 |
                				  ((data_payload_left[1] & 1<<5) ? 1:0) << 2 |
                				  ((data_payload_left[1] & 1<<6) ? 1:0) << 1 |
                				  ((data_payload_left[1] & 1<<7) ? 1:0) << 0;

    	    key_data_buffer[2] |= ((data_payload_left[0] & 1<<0) ? 1:0) << 7 |
                				  ((data_payload_left[0] & 1<<1) ? 1:0) << 6;

    	    key_data_buffer[3] |= ((data_payload_left[2] & 1<<6) ? 1:0) << 7 |
                				  ((data_payload_left[2] & 1<<7) ? 1:0) << 6 |
                				  ((data_payload_left[1] & 1<<0) ? 1:0) << 5 |
                				  ((data_payload_left[1] & 1<<1) ? 1:0) << 4 |
                				  ((data_payload_left[1] & 1<<2) ? 1:0) << 3 |
                				  ((data_payload_left[1] & 1<<3) ? 1:0) << 2;

    	    key_data_buffer[4] |= ((data_payload_left[2] & 1<<3) ? 1:0) << 3 |
                				  ((data_payload_left[2] & 1<<4) ? 1:0) << 2 |
                				  ((data_payload_left[2] & 1<<5) ? 1:0) << 1;

    	    key_data_buffer[5] |= ((data_payload_left[2] & 1<<0) ? 1:0) << 5 |
                				  ((data_payload_left[2] & 1<<1) ? 1:0) << 4 |
                				  ((data_payload_left[2] & 1<<2) ? 1:0) << 3;

            // Be sure to prevent overflow on the "analog" values
    	    int16_t mouseAdd = (int8_t) key_data_buffer[6];
    	    mouseAdd += (int8_t) data_payload_left[4];
    	    if (mouseAdd > 127) {
		          mouseAdd = 127;
    	    } else if (mouseAdd < -127) {
		          mouseAdd = -127;
    	    }
    	    key_data_buffer[6] = (uint8_t) mouseAdd;

    	    mouseAdd = (int8_t) key_data_buffer[7];
    	    mouseAdd += (int8_t) data_payload_left[5];
    	    if (mouseAdd > 127) {
    		    mouseAdd = 127;
    	    } else if (mouseAdd < -127) {
    		    mouseAdd = -127;
    	    }
    	    key_data_buffer[7] = (uint8_t) mouseAdd;

    	    mouseAdd = (int8_t) key_data_buffer[9];
    	    mouseAdd += (int8_t) data_payload_left[6];
    	    if (mouseAdd > 127) {
    		    mouseAdd = 127;
    	    } else if (mouseAdd < -127) {
    		    mouseAdd = -127;
    	    }
    	    key_data_buffer[9] = (uint8_t) mouseAdd;

            // Also ensure we include the rotary encoder button
    	    if (data_payload_left[3] & 1<<7) {
    		    key_data_buffer[10] |= 0x80;
    	    } else {
    		    key_data_buffer[10] &= 0x7F;
    	    }
        }

        if (packet_received_right) {
            packet_received_right = false;

            // First, 0-out old keystates from the right side
    	    key_data_buffer[0] = key_data_buffer[0] & RB0_MASK;
    	    key_data_buffer[1] = key_data_buffer[1] & RB1_MASK;
    	    key_data_buffer[2] = key_data_buffer[2] & RB2_MASK;
    	    key_data_buffer[3] = key_data_buffer[3] & RB3_MASK;
    	    key_data_buffer[4] = key_data_buffer[4] & RB4_MASK;
    	    key_data_buffer[5] = key_data_buffer[5] & RB5_MASK;

            // Now read in/decode the new keystates.
            key_data_buffer[0] |= ((data_payload_right[0] & 1<<7) ? 1:0) << 1 |
                				  ((data_payload_right[0] & 1<<6) ? 1:0) << 0;

    	    key_data_buffer[1] |= ((data_payload_right[0] & 1<<5) ? 1:0) << 7 |
                				  ((data_payload_right[0] & 1<<4) ? 1:0) << 6 |
                				  ((data_payload_right[0] & 1<<3) ? 1:0) << 5 |
                				  ((data_payload_right[0] & 1<<2) ? 1:0) << 4;

    	    key_data_buffer[2] |= ((data_payload_right[0] & 1<<1) ? 1:0) << 5 |
                				  ((data_payload_right[0] & 1<<0) ? 1:0) << 4 |
                				  ((data_payload_right[1] & 1<<7) ? 1:0) << 3 |
                				  ((data_payload_right[1] & 1<<6) ? 1:0) << 2 |
                				  ((data_payload_right[1] & 1<<5) ? 1:0) << 1 |
                				  ((data_payload_right[1] & 1<<4) ? 1:0) << 0;

    	    key_data_buffer[3] |= ((data_payload_right[1] & 1<<3) ? 1:0) << 1 |
                				  ((data_payload_right[1] & 1<<2) ? 1:0) << 0;

    	    key_data_buffer[4] |= ((data_payload_right[1] & 1<<1) ? 1:0) << 7 |
                				  ((data_payload_right[1] & 1<<0) ? 1:0) << 6 |
                				  ((data_payload_right[2] & 1<<7) ? 1:0) << 5 |
                				  ((data_payload_right[2] & 1<<6) ? 1:0) << 4 |
                				  ((data_payload_right[2] & 1<<5) ? 1:0) << 0;

    	    key_data_buffer[5] |= ((data_payload_right[2] & 1<<4) ? 1:0) << 7 |
                				  ((data_payload_right[2] & 1<<3) ? 1:0) << 6 |
                				  ((data_payload_right[2] & 1<<2) ? 1:0) << 2 |
                				  ((data_payload_right[2] & 1<<1) ? 1:0) << 1 |
                				  ((data_payload_right[2] & 1<<0) ? 1:0) << 0;

            // Be sure to prevent overflow on the "analog" values
    	    int16_t mouseAdd = (int8_t) key_data_buffer[6];
    	    mouseAdd += (int8_t) data_payload_right[4];
    	    if (mouseAdd > 127) {
    		    mouseAdd = 127;
    	    } else if (mouseAdd < -127) {
    		    mouseAdd = -127;
    	    }
    	    key_data_buffer[6] = (uint8_t) mouseAdd;

    	    mouseAdd = (int8_t) key_data_buffer[7];
    	    mouseAdd += (int8_t) data_payload_right[5];
    	    if (mouseAdd > 127) {
    		    mouseAdd = 127;
    	    } else if (mouseAdd < -127) {
    		    mouseAdd = -127;
    	    }
    	    key_data_buffer[7] = (uint8_t) mouseAdd;

    	    mouseAdd = (int8_t) key_data_buffer[8];
    	    mouseAdd += (int8_t) data_payload_right[6];
    	    if (mouseAdd > 127) {
    		    mouseAdd = 127;
    	    } else if (mouseAdd < -127) {
    		    mouseAdd = -127;
    	    }
    	    key_data_buffer[8] = (uint8_t) mouseAdd;

            // Also ensure we include the rotary encoder button
    	    if (data_payload_right[3] & 1<<7) {
    		    key_data_buffer[10] |= 0x40;
    	    } else {
    		    key_data_buffer[10] &= 0xBF;
    	    }
        }

        // Checking for a poll request from QMK
        if (app_uart_get(&c) == NRF_SUCCESS && c == 's') {
            // Sending data to QMK, including a checksum in the last half-byte
            uint8_t checksum = 0x00;
            for (uint8_t z=0; z<11; z++){
                checksum = checksum^key_data_buffer[z];
            }
            // Smash the checksum from 1 byte into 4 bits
            checksum = (checksum ^ ((checksum & 0xF0)>>4)) & 0x0F;
    	    key_data_buffer[10] |= checksum; // This is to ensure all the info arrived properly
    	    for (uint8_t z=0; z<11; z++) {
    		    app_uart_put(key_data_buffer[z]);
    	    }
            // Once sent, we 0-out the "analog" values like mouse and encoder movement
    	    key_data_buffer[6] = 0;
    	    key_data_buffer[7] = 0;
    	    key_data_buffer[8] = 0;
    	    key_data_buffer[9] = 0;
            key_data_buffer[10] &= 0xF0;
        }

        // Allowing UART buffers to clear
        nrf_delay_us(80);

        // If no packets are received from each half within INACTIVE periods, assume
        // out of range, or sleeping due to no keys pressed, update keystates to off
        left_active++;
        right_active++;
        if (left_active > INACTIVE) {
    	    key_data_buffer[0] = key_data_buffer[0] & LB0_MASK;
    	    key_data_buffer[1] = key_data_buffer[1] & LB1_MASK;
    	    key_data_buffer[2] = key_data_buffer[2] & LB2_MASK;
    	    key_data_buffer[3] = key_data_buffer[3] & LB3_MASK;
    	    key_data_buffer[4] = key_data_buffer[4] & LB4_MASK;
    	    key_data_buffer[5] = key_data_buffer[5] & LB5_MASK;
    	    key_data_buffer[10] &= 0x7F;
            left_active = 0;
        }
        if (right_active > INACTIVE) {
    	    key_data_buffer[0] = key_data_buffer[0] & RB0_MASK;
    	    key_data_buffer[1] = key_data_buffer[1] & RB1_MASK;
    	    key_data_buffer[2] = key_data_buffer[2] & RB2_MASK;
    	    key_data_buffer[3] = key_data_buffer[3] & RB3_MASK;
    	    key_data_buffer[4] = key_data_buffer[4] & RB4_MASK;
    	    key_data_buffer[5] = key_data_buffer[5] & RB5_MASK;
    	    key_data_buffer[10] &= 0xBF;
            right_active = 0;
        }
    }
}


// Callbacks not needed for this
void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_disabled() {}

// If a data packet was received, identify half, and throw flag
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
    uint32_t data_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    if (pipe == 0) {
        packet_received_left = true;
        left_active = 0;
        // Pop packet and write first byte of the payload to the GPIO port.
        nrf_gzll_fetch_packet_from_rx_fifo(pipe, data_payload_left, &data_payload_length);
    } else if (pipe == 1) {
        packet_received_right = true;
        right_active = 0;
        // Pop packet and write first byte of the payload to the GPIO port.
        nrf_gzll_fetch_packet_from_rx_fifo(pipe, data_payload_right, &data_payload_length);
    }

    // Not sure if required, I guess if enough packets are missed during blocking uart
    nrf_gzll_flush_rx_fifo(pipe);

    // Load ACK payload into TX queue
    ack_payload[0] =  0x55;
    nrf_gzll_add_packet_to_tx_fifo(pipe, ack_payload, TX_PAYLOAD_LENGTH);
}
