
#define COMPILE_RIGHT
//#define COMPILE_LEFT

#include "dichotomy.h"
#include "nrf_drv_config.h"
#include "nrf_gzll.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"


/*****************************************************************************/
/** Configuration */
/*****************************************************************************/

const nrf_drv_rtc_t rtc_maint = NRF_DRV_RTC_INSTANCE(0); // Setting up the "maintenance" timer for watching mouse movement
const nrf_drv_rtc_t rtc_deb = NRF_DRV_RTC_INSTANCE(1); // Setting up the debounce timer for key presses


// Define payload length
#define TX_PAYLOAD_LENGTH 7 // 7-byte payload length when transmitting (3 for button state, 1 for scroll wheel button, 2 for mouse x/y, 1 for mouse scroll)

// Data and acknowledgement payloads
static uint8_t data_payload[TX_PAYLOAD_LENGTH]; // Payload to send to Host.
static uint8_t ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; // Placeholder for received ACK payloads from Host.

// The MAX_SEND_DELAY is the maximum number of ms between any non-0 packets
#define MAX_SEND_DELAY 100

#define CHANNEL_SIZE 4 // This is arbitrary, but as close to 1337 as I could get.
static uint8_t gzll_channels[CHANNEL_SIZE] = {1, 41, 83, 119};

#define PINCOUNT 25 // Setting up the "allPins" array for debounce checking.  Note that it lacks EN1 and EN2 since we don't debounce them
static uint8_t allPins[PINCOUNT] = {
    S01,
    S02,
    S03,
    S04,
    S05,
    S06,
    S07,
    S08,
    S09,
    S10,
    S11,
    S12,
    S13,
    S14,
    S15,
    S16,
    S17,
    S18,
    S19,
    S20,
    S21,
    S22,
    S23,
    S24,
    SEN
};

// Debounce time (dependent on tick frequency)
#define DEBOUNCE 5

// Mouse activity is how long to wait before putting the mouse to a deep sleep and stop querying it
// It gets queried 125 times/second, so 37500 should be 300 seconds = 5 minutes
#define MOUSE_ACTIVITY 37500

// This is for "waking up" the mouse when the keyboard first boots.
#define MOUSE_AWAKE 3

// These are various register addresses and values for communicating with the optical encoder
#define OPTICAL_ADDRESS_REG 0x00
#define OPTICAL_ADDRESS_VAL 0x30
#define OPTICAL_CONFIG_REG 0x06
#define OPTICAL_MOTION_REG 0x02

// Pin-driving function consts for optical sensor communication
#define FORCE 1
#define SOFT 0
#define HIGH 1
#define LOW 0

// Encoder vals for ease of use
#define EN_ACTIVE 1
#define EN_INACTIVE 0
#define EN_TIMEOUT 20
#define EN1_UP 0
#define EN1_DOWN 1
#define EN2_UP 2
#define EN2_DOWN 3

// These are for setting the dpi on the optical sensor
#define DPI_0600 0x00
#define DPI_0800 0x01
#define DPI_1000 0x02
#define DPI_1200 0x03
#define DPI_1600 0x04

// Boot-magic settings, these will get changed/set according to what is pressed at keyboard boot
static bool disable_mouse = false;

// Key buffers
static uint32_t keys = 0, keys_raw = 0, keys_raw_old = 0;
static uint32_t key_debounce[DEBOUNCE];
static uint32_t sendDelayCount = 0;
static uint16_t mouse_inactivity = 0;
static uint8_t bufferRotation = 0;
static volatile bool mouse_asleep = true, isData = false;

// Encoder/Optical sensor buffers for reading/sending data without losing any
static int8_t preXMovement = 0;
static int8_t preYMovement = 0;
static int8_t preEncMovement = 0;
static int8_t xMovement = 0;
static int8_t yMovement = 0;
static int8_t encMovement = 0;

// Initial settings for proper boot initialization
static uint8_t mouse_wakeup = 0;
static uint8_t encStatus = -1;
static volatile uint8_t mouse_setting = 0x01;

// Decide whether or not to clear analog buffers
static volatile bool lastTransmitFailed;

// Only one "timer" should be active at a time - either maint or deb.  This lets us track which:
#define RTC_MAINT_SET 0
#define RTC_DEB_SET 1
static volatile bool currentTimer = RTC_MAINT_SET;

// Setup switch pins with pullups
static void gpio_config(void) {
    for (uint8_t i=0; i<PINCOUNT; i++){
        nrf_gpio_cfg_sense_input(allPins[i], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    }
    nrf_gpio_cfg_sense_input(EN1, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(EN2, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    for (uint8_t i=0; i<DEBOUNCE; i++){
        key_debounce[i] = 0;
    }
}


/* THE FOLLOWING IS REALLY UGLY Serial protocol stuff for the optical sensor */
static void wait(uint64_t count) {
    nrf_delay_us(count);
}

static void drivePin(uint8_t pin, bool state, bool type) {
	if (type) {
		// Set type is force, set pin to output and drive
		nrf_gpio_cfg_output(pin);
		nrf_gpio_pin_write(pin,state);
    } else {
        // Set type is soft, set pin to input with pull up/down and read
        if (state) {
            nrf_gpio_cfg_input(pin,NRF_GPIO_PIN_PULLUP);
        } else {
            nrf_gpio_cfg_input(pin,NRF_GPIO_PIN_PULLDOWN);
        }
    }
}

static void input(uint8_t pin) {
    nrf_gpio_cfg_input(pin,NRF_GPIO_PIN_NOPULL);
}

static bool read(uint8_t pin) {
    return nrf_gpio_pin_read(pin);
}

static uint8_t getRegister(uint8_t reg) {
    uint8_t returnVal = 0;
    drivePin(DATA_PIN,LOW,FORCE);
    wait(8);
    // First item seems to have different timing
    drivePin(CLOCK_PIN,LOW,FORCE);
    wait(8);
    if (reg & 0x80) {
        drivePin(DATA_PIN,HIGH,FORCE);
    } else {
        drivePin(DATA_PIN,LOW,FORCE);
    }
    drivePin(CLOCK_PIN,HIGH,FORCE);
    wait(8);
	// Items 1-7 have the same timing
    for (uint8_t i=1; i<8; i++) {
        drivePin(CLOCK_PIN,LOW,FORCE);
        if (reg & (0x80 >> i)) {
            drivePin(DATA_PIN,HIGH,FORCE);
        } else {
            drivePin(DATA_PIN,LOW,FORCE);
        }
        wait(1);
        drivePin(CLOCK_PIN,HIGH,FORCE);
        wait(1);
    }
    drivePin(DATA_PIN,HIGH,SOFT);
    wait(9);
    drivePin(CLOCK_PIN,LOW,FORCE);
    wait(6);
    drivePin(CLOCK_PIN,HIGH,FORCE);
    returnVal |= (read(DATA_PIN)<<7);
    wait(10);
    for (uint8_t i=1; i<8; i++) {
        drivePin(CLOCK_PIN,LOW,FORCE);
        wait(2);
        drivePin(CLOCK_PIN,HIGH,FORCE);
        returnVal |= (read(DATA_PIN)<<(7-i));
        wait(1);
    }
    return returnVal;
}

static void setRegister(uint8_t reg, uint8_t val) {
    drivePin(DATA_PIN,LOW,FORCE);
    wait(8);
    drivePin(CLOCK_PIN,LOW,FORCE);
    wait(8);
    drivePin(DATA_PIN,HIGH,FORCE);
    drivePin(CLOCK_PIN,HIGH,FORCE);
    wait(8);
    for (uint8_t i=1; i<8; i++) {
        drivePin(CLOCK_PIN,LOW,FORCE);
        wait(2);
        if (0x80 & (reg<<i)) {
            drivePin(DATA_PIN,HIGH,FORCE);
        } else {
            drivePin(DATA_PIN,LOW,FORCE);
        }
        drivePin(CLOCK_PIN,HIGH,FORCE);
        wait(1);
    }
    //register read, now start writing
    wait(5);
    for (uint8_t i=0; i<8; i++) {
        drivePin(CLOCK_PIN,LOW,FORCE);
        wait(2);
        if (0x80 & (val<<i)) {
            drivePin(DATA_PIN,HIGH,FORCE);
        } else {
            drivePin(DATA_PIN,LOW,FORCE);
        }
        drivePin(CLOCK_PIN,HIGH,FORCE);
        wait(1);
    }
}

static void mousePower(bool state) {
    if (state) {
        mouse_setting = ~0x08 & mouse_setting;
        setRegister(OPTICAL_CONFIG_REG, mouse_setting);
        mouse_asleep = 0;
    } else {
        mouse_setting = 0x08 | mouse_setting;
        setRegister(OPTICAL_CONFIG_REG, mouse_setting);
        mouse_asleep = 1;
    }
}
/* DON'T JUDGE IT, OK? I KNOW IT'S AWFUL */
// (I mean, if you want to clean it up and make it nice, be my guest.)

// Return the key states, masked with valid key pins
static uint32_t read_keys(void) {
    return ~NRF_GPIO->IN & INPUT_MASK;
}

// Assemble packet and send to receiver
static void send_data(void) {
    data_payload[0] = ((keys & 1<<S01) ? 1:0) << 7 | \
              		  ((keys & 1<<S02) ? 1:0) << 6 | \
		              ((keys & 1<<S03) ? 1:0) << 5 | \
		              ((keys & 1<<S04) ? 1:0) << 4 | \
		              ((keys & 1<<S05) ? 1:0) << 3 | \
		              ((keys & 1<<S06) ? 1:0) << 2 | \
		              ((keys & 1<<S07) ? 1:0) << 1 | \
		              ((keys & 1<<S08) ? 1:0) << 0;

    data_payload[1] = ((keys & 1<<S09) ? 1:0) << 7 | \
		              ((keys & 1<<S10) ? 1:0) << 6 | \
		              ((keys & 1<<S11) ? 1:0) << 5 | \
		              ((keys & 1<<S12) ? 1:0) << 4 | \
		              ((keys & 1<<S13) ? 1:0) << 3 | \
		              ((keys & 1<<S14) ? 1:0) << 2 | \
		              ((keys & 1<<S15) ? 1:0) << 1 | \
		              ((keys & 1<<S16) ? 1:0) << 0;

    data_payload[2] = ((keys & 1<<S17) ? 1:0) << 7 | \
		              ((keys & 1<<S18) ? 1:0) << 6 | \
		              ((keys & 1<<S19) ? 1:0) << 5 | \
		              ((keys & 1<<S20) ? 1:0) << 4 | \
		              ((keys & 1<<S21) ? 1:0) << 3 | \
		              ((keys & 1<<S22) ? 1:0) << 2 | \
		              ((keys & 1<<S23) ? 1:0) << 1 | \
		              ((keys & 1<<S24) ? 1:0) << 0;

    data_payload[3] = ((keys & 1<<SEN) ? 1:0) << 7 | \
					                        0 << 6 | \
					                        0 << 5 | \
					                        0 << 4 | \
					                        0 << 3 | \
					                        0 << 2 | \
					                        0 << 1 | \
					                        0 << 0;

    // Adjust "analog" values to add in any new information if the packet hasn't sent
    // 0 it out if the packet did send...
    if (!lastTransmitFailed) {
        xMovement = 0;
        yMovement = 0;
        encMovement = 0;
    } else {
        lastTransmitFailed = false;
    }
    // Protect X from overflow
    int16_t mouseAdd = 0 + xMovement;
    mouseAdd += preXMovement;
    preXMovement = 0;
    if (mouseAdd > 127) {
        mouseAdd = 127;
    } else if (mouseAdd < -127) {
        mouseAdd = -127;
    }
    xMovement = (int8_t) mouseAdd;
    // Now do it for y
    mouseAdd = 0 + yMovement;
    mouseAdd += preYMovement;
    preYMovement = 0;
    if (mouseAdd > 127) {
        mouseAdd = 127;
    } else if (mouseAdd < -127) {
        mouseAdd = -127;
    }
    yMovement = (int8_t) mouseAdd;
    // And I guess we shouldn't leave the encoder out
    mouseAdd = 0 + encMovement;
    mouseAdd += preEncMovement;
    preEncMovement = 0;
    if (mouseAdd > 127) {
        mouseAdd = 127;
    } else if (mouseAdd < -127) {
        mouseAdd = -127;
    }
    encMovement = (int8_t) mouseAdd;
    data_payload[4] = (uint8_t) xMovement;
    data_payload[5] = (uint8_t) yMovement;
    data_payload[6] = (uint8_t) encMovement;
    if (nrf_gzll_ok_to_add_packet_to_tx_fifo(PIPE_NUMBER)){
        nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, data_payload, TX_PAYLOAD_LENGTH);
    }
}

// Re-sync mouse in case SPI/I2C-ish protocol gets out of sync on clock signal.
static void reSyncMouse(void) {
    // First, disable anything that might interrupt.  That means GPIOs and timers:
    NVIC_DisableIRQ(GPIOTE_IRQn);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    nrf_drv_rtc_disable(&rtc_maint);
    nrf_drv_rtc_disable(&rtc_deb);

    // Drive clock signal low for at least 1us, then high for at least 1.7ms top reset communication
    drivePin(CLOCK_PIN,LOW,FORCE);
    wait(2);
    drivePin(CLOCK_PIN,HIGH,FORCE);
    wait(2000);

    // Re-enable interrupts
    NVIC_EnableIRQ(GPIOTE_IRQn);
    if (currentTimer == RTC_MAINT_SET){
        nrf_drv_rtc_enable(&rtc_maint);
    } else {
        nrf_drv_rtc_enable(&rtc_deb);
    }
}

// Helper function to see if the "motion" register reads high
static void checkMouseMovement() {
    bool checkReg = true;
    if (getRegister(OPTICAL_ADDRESS_REG)!=OPTICAL_ADDRESS_VAL) {
        // Serial communication with optical sensor is out of sync.  Re-sync, then try again.
        reSyncMouse();
        if (getRegister(OPTICAL_ADDRESS_REG)!=OPTICAL_ADDRESS_VAL) {
            checkReg = true;
        } else {
            // Failed to get it in sync even after waiting 1.7 ms for sync.  Try again next sync cycle, it's taken too long already.
            checkReg = false;
        }
    }
    if (checkReg) {
        wait(30);
        uint8_t motReg = getRegister(OPTICAL_MOTION_REG);
        isData = (motReg & 0x80);
    }
}

// 125Hz "maintenance" timer to monitor the mouse when keys aren't being pressed
static void handler_maintenance(nrf_drv_rtc_int_type_t int_type) {
    if (!disable_mouse){
        checkMouseMovement();
    } else {
        // If the mouse is disabled, we might not need this at all.
        if (keys == 0) {
            nrf_drv_rtc_disable(&rtc_maint);
        } else {
            sendDelayCount += 8;
            if (sendDelayCount > MAX_SEND_DELAY) {
                send_data();
                sendDelayCount=0;
            }
        }
    }
    if (isData) {
        // If there is data, I want to activate the debounce routine to deal with it.
        mouse_inactivity = 0;
        nrf_drv_rtc_disable(&rtc_maint);
        currentTimer = RTC_DEB_SET;
        nrf_drv_rtc_enable(&rtc_deb);
    } else {
        mouse_inactivity++;
        if (mouse_inactivity>MOUSE_ACTIVITY && keys == 0) {
            nrf_drv_rtc_disable(&rtc_maint);
            mouse_inactivity = 0;
            mousePower(0);
        } else {
            sendDelayCount += 8;
            if (sendDelayCount > MAX_SEND_DELAY) {
		if (keys != 0){
                    send_data();
                }
                sendDelayCount=0;
            }
            // Used to send data here, if it's been waiting long enough.
        }
    }
}

// Helper function for encoder reading - only called when something changes
static uint8_t getEncStatus(bool en1_old, bool en1_new, bool en2_old, bool en2_new) {
    uint8_t returnVal = 0;
    if (en1_old==en1_new) {
        // SOMETHING has changed for this to be called, it must be en2
        if (en2_new) {
            returnVal = EN2_UP;
        } else {
            returnVal = EN2_DOWN;
        }
    } else {
        // SOMETHING has changed for this to be called, it must be en1
        if (en1_new) {
            returnVal = EN1_UP;
        } else {
            returnVal = EN1_DOWN;
        }
    }
    return returnVal;
}

// Encoder Handler is called from handler_debounce, it tracks the encoder movement.
static void handler_encoder(uint32_t newValid) {
    // Check old and new encoder states
    bool en1_old = (keys & 1<<EN1);
    bool en1_new = (newValid & 1<<EN1);
    bool en2_old = (keys & 1<<EN2);
    bool en2_new = (newValid & 1<<EN2);

    if (en1_old==en1_new && en2_old==en2_new){
        // There's no change - and therefore no reason to do anything.
    } else {
        // Something has changed, let's find the new state and figure out what to do with it.
        int8_t newEncStatus = getEncStatus(en1_old, en1_new, en2_old, en2_new);
        if ((encStatus == EN1_UP) && (newEncStatus == EN2_UP)) {
            // 2 is following 1, clockwise.
            if ((en1_new & en2_new) == false) {
            //both are 0, so it's the tick to activate them.
            preEncMovement = preEncMovement +1;
            }
        } else if ((encStatus == EN1_UP) && (newEncStatus == EN2_DOWN)) {
            // 2 is leading 1, counterclockwise
            if ((en1_new & en2_new) == false) {
            preEncMovement = preEncMovement -1;
            }
        } else if ((encStatus == EN2_UP) && (newEncStatus == EN1_UP)) {
            // 2 is leading 1, counterclockwise
            if ((en1_new & en2_new) == false) {
            preEncMovement = preEncMovement -1;
            }
        } else if ((encStatus == EN2_UP) && (newEncStatus == EN1_DOWN)){
            // 2 is following 1, clockwise.
            if ((en1_new & en2_new) == false) {
            preEncMovement = preEncMovement +1;
            }
        }
        encStatus = newEncStatus;
	nrf_gpio_cfg_sense_input(EN1, NRF_GPIO_PIN_PULLUP, (newValid & 1<<EN1)?NRF_GPIO_PIN_SENSE_HIGH:NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_input(EN2, NRF_GPIO_PIN_PULLUP, (newValid & 1<<EN2)?NRF_GPIO_PIN_SENSE_HIGH:NRF_GPIO_PIN_SENSE_LOW);
    }
}

// 1000Hz debounce sampling, activated by a timed interrupt every ms - that interrupt is enabled whenever a key is pressed and disabled when debouncing is finished
static void handler_debounce(nrf_drv_rtc_int_type_t int_type) {
    /** Our debouncing is adaptive, power-saving, minimal-latency, and rather unique/complicated.
        First off, we "pre-bounce" (not a real term) keys - that is to say, rather than waiting
        for a key to be pressed, then waiting AGAIN for the output to stabalize, we begin with a
        list (stored bitwise in a uint32_t) of keys that are "allowed" to activate - that is, they
        haven't changed state for at least DEBOUNCE periods.  As soon as a key changes state, we
        can immediately assume the new condition is the new keystate, then wait until it has been
        stable for DEBOUNCE periods before it's "allowed" to change state again.

        Whenever a key is de/pre-bouncing, we want to minimize current usage (because the pull-up
        resistor is 13Kohm, we waste a (relatively) large amount of power if the key is pressed and
        the pullup is enabled).  Thus, we set each input that is either being debounced OR
        currently LOW to a high-z input state without a pulling resistor, which consumes minimal
        (nA of) current.  When it comes time to check states, we briefly give them a pull-up,
        read the state, and then we need to set the input state back appropriately - if a key is
        still LOW or debouncing, we need to go back to high-z with no pull, but if a key has just
        finished debouncing (and is high), we need to enable their interrupt again.

        Once we have the current, debounced key-state, we send it through gazell to the receiver
        where it will be decoded and passed on to the pro micro running QMK.  The process for all
        this is layed out in comments in the code below.

        Note the 'HIGH' and "LOW" are relative terms, here - physically what is "low" means "the key
        is pressed" and is therefore inverted in the "read_keys" function - so... yeah it's confusing
    **/

    // The "allowedInput" was calculated last cylce, and tells us what has allowed input during this cycle
    // The format of this uint32_t is a 1 on any bit representing a key that is allowed to change, and 0s elsewhere
    // We're going to begin calculating the new "allowedInput" by gathering together any keys that have changed
    // state in the key_debounce buffers - we'll ignore the buffer that's going to be filled this cycle.
    // We do this before we connect pullups to input pins so that we don't have unecessary clock cylces
    // where power is being wasted.

    if (!disable_mouse) {
        if (mouse_asleep){
            mousePower(1);
            mouse_wakeup = 0;
        }
    }

    // Check all the key_debounce buffers to see what has changed in the last (DEBOUNCE) cycles
    uint32_t allowedInput = 0;
    for (uint8_t i=0; i<DEBOUNCE; i++) {
        allowedInput |= key_debounce[i];
    }
    allowedInput = ~allowedInput;

    // Now we can read in the "raw" keystates of the switches that might be bouncing.
    keys_raw = read_keys();

    // We compare it with our last "raw" keystates to see what has changed.
    uint32_t delta = keys_raw^keys_raw_old;
    keys_raw_old = keys_raw;

    // We need to ensure we record changing values in the buffer so they're properly debounced.
    bufferRotation = (bufferRotation+1) % DEBOUNCE; // This just increments the buffer rotation and wraps it properly around DEBOUNCE
    key_debounce[bufferRotation] = delta;

    // We'll now start figuring out what keystates are "valid" (allowed, not bouncing, etc) for this cycle
    uint32_t newValid = ((keys_raw^keys) & allowedInput) ^ keys;

    // The input mask ignores the encoder status, since we don't debounce them - the line below makes sure to
    // 0-out and then record the encoder status as well.
    newValid = (newValid & (~(1<<EN1) & ~(1<<EN2))) | ((keys_raw & (1<<EN1)) | (keys_raw & (1<<EN2)));

    // By comparing the old "allowedInput" from this cylce and the new "allowedInput" for the next cycle,
    // we can determine which keys need to be high-z, and which need to be configured as interrupts ("activated").
    // We COULD just configure everything that's not high-z as interrupts, but some of them
    // already have it, and we don't want to waste clock cycles setting it again.
    uint32_t newSenseLow = (newValid ^ keys) & keys;
    uint32_t newSenseHigh = (newValid ^ keys) & newValid;
    for (uint8_t i = 0; i < PINCOUNT; i++) {
        if (newSenseHigh & (1<<allPins[i])) {
            nrf_gpio_cfg_sense_input(allPins[i], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_HIGH);
        } else if (newSenseLow & (1<<allPins[i])) { // Or activate things that need interrupts
            nrf_gpio_cfg_sense_input(allPins[i], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
        }
    }

    // Make sure we figure out if the encoder is moving.
    handler_encoder(newValid);

    // Now we need to check the mouse for input before sending data.
    bool wasData = false;
    if (mouse_wakeup >= MOUSE_AWAKE){
        if (!isData){
            checkMouseMovement();
        }
        wasData = isData;
        if (isData){
            mouse_inactivity = 0;
            wait(30);
            preYMovement = (int8_t) getRegister(0x03);
            wait(30);
            preXMovement = (int8_t) getRegister(0x04);
            isData = false;
        }
    } else {
        if (!disable_mouse){
            mouse_wakeup++;
        }
    }

    // Finally we can see if we're done
    if (keys == newValid && ~allowedInput == 0 && !wasData) { // If nothing is pressed,nothing is debouncing, and there wasn't any optical sensor data
        nrf_drv_rtc_disable(&rtc_deb); // Then we want to disable the debounce timer interrupt so the nrf can sleep until another key is pressed.
        currentTimer = RTC_MAINT_SET;
        nrf_drv_rtc_enable(&rtc_maint); // Make sure the watchdog for the mouse is running
        mouse_inactivity = 0;
        keys = newValid; // Make sure we update the actual keystate that will be sent
        send_data(); // Let the receiver know that everything is 0
        sendDelayCount = 0; // We can reset this to 0 and it'll figure it out next time this handler is called
    } else {
        // If the key state is different from what it was at the beginning of the cycle...
        // Or we haven't sent an updated keystate in a while...
        // Or there's data from the optical sensor
        if (keys != newValid || sendDelayCount > MAX_SEND_DELAY || wasData) {
            keys = newValid; // Make sure we update the actual keystate that will be sent
            send_data(); // Let the receiver know about it. (if it doesn't get a packet every so often, it assumes everything is 0)
            sendDelayCount = 0; // And make sure we reset our send timeout.
        } else {
            sendDelayCount++; // Increment our send timeout so that the receiver stays current and doesn't 0 everything out
        }
        // Finally, forward-set the keystate variable for the next cycle.
        keys = newValid; // Just in case we didn't do this earlier.
    }
}

// Bootmagic checks for keys held down on keyboard start to set configs
static void bootMagic(void)
{
    // Set the relevant keys as inputs with pullups so properly scan them
    nrf_gpio_cfg_input(SEN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(S01, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(S02, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(S03, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(S04, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(S05, NRF_GPIO_PIN_PULLUP);

    // Initialize their values as false so we can override them with the read-in value
    bool dpi_00 = false, dpi_01 = false, dpi_02 = false, dpi_03 = false, dpi_04 = false, dpi_05 = false;

    // Read in the keystates
    dpi_00 = !read(SEN);
    dpi_01 = !read(S01);
    dpi_02 = !read(S02);
    dpi_03 = !read(S03);
    dpi_04 = !read(S04);
    dpi_05 = !read(S05);

    // Set our default mouse setting
    mouse_setting = DPI_1000;

    // Check to see which DPI setting was selected, lowest has priority
    if (dpi_00) {
        // NO DPI, mouse disable
        disable_mouse = true;
    } else if (dpi_05) {
        // Lowest DPI setting
        mouse_setting = DPI_0600;
    } else if (dpi_04) {
        // Second-lowest DPI setting
        mouse_setting = DPI_0800;
    } else if (dpi_03) {
        // Middle-ground DPI setting
        mouse_setting = DPI_1000;
    } else if (dpi_02) {
        // Second-highest DPI setting
        mouse_setting = DPI_1200;
    } else if (dpi_01) {
        // Highest DPI setting
        mouse_setting = DPI_1600;
    } else {
        // None chosen, go with middle ground
        mouse_setting = DPI_1000;
    }

    mouse_setting &= ~0x08; // This is to have power by default

    setRegister(OPTICAL_CONFIG_REG, mouse_setting);

	// Now we wait until there are no buttons pressed to continue the boot.
	while (!read(S01) || !read(S02) || !read(S03) || !read(S04) || !read(S04) || !read(SEN)) {
		// There' still a button pressed, don't continue with boot
		wait(1000);
	}
}

// Low frequency clock configuration
static void lfclk_config(void)
{
    nrf_drv_clock_init();

    nrf_drv_clock_lfclk_request(NULL);
}

// RTC peripheral configuration
static void rtc_config(void)
{
    // Initialize RTC instance
    nrf_drv_rtc_init(&rtc_maint, NULL, handler_maintenance);
    nrf_drv_rtc_init(&rtc_deb, NULL, handler_debounce);

    // Power on RTC instance
    nrf_drv_rtc_enable(&rtc_maint);
    nrf_drv_rtc_enable(&rtc_deb);

    // Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc_maint,true);
    nrf_drv_rtc_tick_enable(&rtc_deb,true);

    // ... but actually don't enable the debounce handler.
    nrf_drv_rtc_disable(&rtc_deb);
}

int main()
{
    // Initiaize Gazell
    nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    nrf_gzll_set_device_channel_selection_policy(NRF_GZLL_DEVICE_CHANNEL_SELECTION_POLICY_USE_CURRENT);
    nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_4_DBM);
    nrf_gzll_set_channel_table(gzll_channels, CHANNEL_SIZE);
    nrf_gzll_set_datarate(NRF_GZLL_DATARATE_2MBIT);
    nrf_gzll_set_sync_lifetime(8);
    nrf_gzll_set_timeslot_period(600);
    nrf_gzll_set_timeslots_per_channel(2);
    nrf_gzll_set_timeslots_per_channel_when_device_out_of_sync(10);

    // Attempt sending every packemes
    nrf_gzll_set_max_tx_attempts(100);

    // Addressing
    nrf_gzll_set_base_address_0(0x01020304);
    nrf_gzll_set_base_address_1(0x05060708);

    // Enable Gazell to start sending over the air
    nrf_gzll_enable();

    // Configure 32kHz xtal oscillator
    lfclk_config();

    // Check for bootmagic settings, and ensure the mouse is running properly
    wait(5000);
    mousePower(1);
    wait(5000);
    bootMagic();
    wait(5000); // Make sure boot magic settings have time to apply

    // Configure RTC peripherals with ticks
    rtc_config();

    // Configure all keys as inputs with pullups
    gpio_config();

    // The mouse is powered on, but if boot magic has dictated, we need to shut it off.
    if (disable_mouse){
        mousePower(0);
    } else {
        mouse_wakeup = MOUSE_AWAKE;
    }

    // Set the GPIOTE PORT event as interrupt source, and enable interrupts for GPIOTE
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NVIC_SetPriority(GPIOTE_IRQn, 3); //optional: set priority of interrupt
    NVIC_EnableIRQ(GPIOTE_IRQn);


    // Main loop, constantly sleep, waiting for RTC and gpio IRQs
    while(1)
    {
        __SEV();
        __WFE();
        __WFE();
    }
}

// This handler will be run after wakeup from system ON (GPIO wakeup)
void GPIOTE_IRQHandler(void)
{
    if (NRF_GPIOTE->EVENTS_PORT)
    {
        //clear wakeup event
        NRF_GPIOTE->EVENTS_PORT = 0;

        //enable rtc interupt triggers
        nrf_drv_rtc_disable(&rtc_maint);
        currentTimer = RTC_DEB_SET;
        nrf_drv_rtc_enable(&rtc_deb);
    }
}



/*****************************************************************************/
/** Gazell callback function definitions  */
/*****************************************************************************/

void  nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    uint32_t ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    if (tx_info.payload_received_in_ack)
    {
        // Pop packet and write first byte of the payload to the GPIO port.
        nrf_gzll_fetch_packet_from_rx_fifo(pipe, ack_payload, &ack_payload_length);
    }
}

// no action is taken when a packet fails to send, this might need to change
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    lastTransmitFailed = true;
}

// Callbacks not needed
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{}
void nrf_gzll_disabled()
{}
