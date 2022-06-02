
#include "MeAuriga.h"
#include <SoftwareSerial.h>
#include <MeBluetooth.h>
#include <MeRGBLed.h>

#define AURIGARINGLEDNUM  12
#define RINGALLLEDS        0

#include "animate_ring.h"

// on-board LED ring, at PORT0 (onboard), with 12 LEDs
MeRGBLed led_ring(0, 12);
AnimateRing anim(&led_ring);

// initialize an MeBluetooth class from the Makerblock library
MeBluetooth bti(PORT_3); // built-in
MeBluetooth bte(PORT_5); // external

/*Define Ultrasonic Sensor Connection*/
MeUltrasonicSensor ultraSensor(PORT_7);

/*Define Motor Connections*/
MeEncoderOnBoard MOTOR_RIGHT(SLOT1); // right
MeEncoderOnBoard MOTOR_LEFT(SLOT2); // left

/*Define Motor Speed From 0 to 255*/
#define MOTOR_SPEED 50
/* Turn the rover if its within TURN_IF_WITHIN cm of an object (via ultrasonic) */
const double TURN_IF_WITHIN = 15.0;
/* How many milliseconds to run the motors during a movement step */
const int MOTOR_HOLD_MS = 1500;

/*RSSI Variables*/
int rssiRefresh = 0;
int rssi_old = -10000;
int rssi_new = -10000;
unsigned long rssi_last_recv = 0;

double curr_dist_cm = 0.0;

unsigned long last_print = 0; // the ms timestamp of the latest debug print
unsigned long last_dist_poll = 0; // the ms timestamp of the latest ultrasound poll
unsigned long motors_until = 0; // the ms timestamp at/after the motors can be used again/should be cleared
bool motors_stopped = false; // allows turning off the motors without affecting other parts of the program

// prints all bluetooth messages to serial as a hexdump
const bool DEBUG_BLUETOOTH = false;


const unsigned long freq_print = 500; // how often to print internal state to serial
const unsigned long freq_dist_poll = 100; // how often to poll the distance sensor for new info

// functions for initializing the motors/encoders - from Me_Auriga_encoder_callback.ino example
void isr_process_encoder1(void) {
  if(digitalRead(MOTOR_RIGHT.getPortB()) == 0) {
    MOTOR_RIGHT.pulsePosMinus();
  } else {
    MOTOR_RIGHT.pulsePosPlus();;
  }
}
void isr_process_encoder2(void) {
  if(digitalRead(MOTOR_LEFT.getPortB()) == 0) {
    MOTOR_LEFT.pulsePosMinus();
  } else {
    MOTOR_LEFT.pulsePosPlus();
  }
}
void init_motors() {
  attachInterrupt(MOTOR_RIGHT.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(MOTOR_LEFT.getIntNum(), isr_process_encoder2, RISING);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  // unnecessary?
  MOTOR_RIGHT.setPulse(9);
  MOTOR_LEFT.setPulse(9);
  MOTOR_RIGHT.setRatio(39.267);
  MOTOR_LEFT.setRatio(39.267);

  MOTOR_RIGHT.setPosPid(1.8,0,1.2);
  MOTOR_LEFT.setPosPid(1.8,0,1.2);
  MOTOR_RIGHT.setSpeedPid(0.18,0,0);
  MOTOR_LEFT.setSpeedPid(0.18,0,0);
}

// runs on startup of the bot
void setup() {
    // delay a bit so we can usefully detect it initializing/etc
    delay(500);
    Serial.begin(2000000);
    Serial.println("initializing...");

    // LM.setpin(LM_FORWARD, LEFT_MOTOR);
    // RM.setpin(RM_FORWARD, RIGHT_MOTOR);
    // LM.stop();
    // RM.stop();

    // 12 LED Ring controller is on Auriga D44/PWM
    led_ring.setpin( 44 );

    // little startup animation to know if its been rebooted
    led_ring.setColor(10,10,10);
    led_ring.show();
    delay(200);
    led_ring.setColor(0,0,0);
    led_ring.show();
    delay(200);
    led_ring.setColor(10,10,10);
    led_ring.show();
    delay(200);
    led_ring.setColor(0,0,0);
    led_ring.show();
    anim.resume();

    // initialize the bluetooth modules/motors
    // bti.begin(115200);
    bte.begin(115200);
    init_motors();

    Serial.println("initialized.");
    last_print = millis();
}

// line buffers for each bluetooth module, and how much data they hold
// holds partial messages if necessary (none should be over 64 bytes long)
#define BT_BUFFERLEN 64
uint8_t bti_linebuf[BT_BUFFERLEN];
size_t bti_linebuf_len = 0;
uint8_t bte_linebuf[BT_BUFFERLEN];
size_t bte_linebuf_len = 0;

// returns true if the buffer only contains printable ascii
bool is_printable_ascii(const uint8_t *buf, size_t buflen) {
    for(int i = 0; i < buflen; i++) {
        uint8_t c = buf[i];
        if(!isascii(c) || iscntrl(c)) {
            Serial.print("found bad character: ");
            Serial.println(c, 16);
            return false;
        }
    }
    return true;
}

// process incoming bluetooth data, and set global variables if appropriate
void read_bt(const char *label, MeBluetooth *bt, uint8_t *const buf, size_t *buflen) {
    int c; // character we may or may not read from the bluetooth stream

    // read a character from the Bluetooth stream into 'c'
    while((c = bt->read()) != -1) {
        // write the read character into the our buffer
        // note: we always have capacity, so we can always add to it
        buf[*buflen] = (uint8_t) c;
        ++*buflen; // increase the tracked length

        // terminate this one if the next char is gonna go over (eg: adding a char will fill it)
        if(*buflen == BT_BUFFERLEN-1) {
            buf[*buflen] = '\0';
            (*buflen)++;
        }

        // convert newlines to string terminators
        if(buf[*buflen-1] == '\n') buf[*buflen-1] = '\0';

        // if it ends in zero, we have the whole message - process it
        if(buf[*buflen-1] == '\0') {
            if(DEBUG_BLUETOOTH) {
                // print messages as a hexdump
                Serial.print("\nraw msg buffer [l=");
                Serial.print(*buflen);
                Serial.print("]: ");
                for(size_t i = 0; i < *buflen; i++) {
                    Serial.print(buf[i], 16);
                }
                Serial.print(" (");
                Serial.print((char*) buf);
                Serial.println(")");
                // Serial.println();
            }

            size_t msglen = *buflen - 1;
            // check if the buffer (minus our terminator) is ascii
            if(is_printable_ascii(buf, msglen)) {
                char* s = (char*) memchr(buf, ':', *buflen);
                if(s != NULL) {
                    // separate message type and payload data
                    char* payload = s+1;
                    *s = '\0';
                    char* typ = (char*) buf;

                    // detect data packets by type
                    if(strncmp(typ, "msg", 3) == 0 && strncmp(payload, "heartbeat", 9) == 0) {
                        // flash on the LEDs if we have a BT heartbeat
                        anim.set_color(AURIGARINGLEDNUM/2, 0, 0, 5);
                        anim.set_dirty();
                        // Serial.print("+");
                    } else if(strncmp(typ, "rssi", 4) == 0) {
                        // parse int from 'payload' and set it to rssi_new
                        char* end;
                        long value = strtol(payload, &end, 10);
                        if(end == payload || *end != '\0') {
                            Serial.print("error reading new rssi value, not a valid integer: ");
                            Serial.println(payload);
                        } else {
                            int32_t r,g,b;
                            r = g = b = 0;
                            if(value > rssi_new) { // we want a higher (closer to zero) RSSI if we are closer
                                g = 5;
                            } else if(value < rssi_new) {
                                r = 20;
                            } else {
                                r = g = b = 5;
                            }
                            anim.set_color(-1, r, g, b);
                            anim.set_color(0, r, g, b);
                            anim.set_color(1, r, g, b);
                            anim.set_dirty();
                            rssi_new = value;
                            rssi_last_recv = millis();
                        }
                    } else {
                        // print out separated components to serial if unknown
                        char tbuf[64 + BT_BUFFERLEN];
                        int written = snprintf(tbuf, sizeof(tbuf), "\n[%s][t=%06lu][s=%u][%s] %s\n", label, millis(), *buflen, typ, payload);
                        if(written >= sizeof(tbuf)) buf[sizeof(tbuf)-1] = '\0';
                        Serial.print(tbuf);
                    }
                } else {
                    // write out known
                    char tbuf[64 + BT_BUFFERLEN];
                    int written = snprintf(tbuf, sizeof(tbuf), "\n[%s][t=%06lu][s=%u][???] %.*s\n", label, millis(), *buflen, msglen, buf);
                    if(written >= sizeof(tbuf)) buf[sizeof(tbuf)-1] = '\0';
                    Serial.print(tbuf);
                }
            } else {
                // string isn't printable ascii, is unexpected
                // -> print as hex
                
                // print a prefix
                char tbuf[64];
                int written = snprintf(tbuf, sizeof(tbuf), "\n[%s][t=%06lu][s=%u][hex] ", label, millis(), *buflen);
                if(written >= sizeof(tbuf)) buf[sizeof(tbuf)-1] = '\0';
                Serial.print(tbuf);

                // print the data as hex
                for(size_t i = 0; i < *buflen; ++i) {
                    Serial.print(buf[i], 16);
                }
                Serial.println();
            }
            
            // clear out our buffer for the next iteration
            *buflen = 0;
        }
    }
}

// when the movement for (right, left) motors are to be stopped
unsigned long mr_expire = 0;
unsigned long ml_expire = 0;

// called in the main loop() method to drive the motors. must be called every few milliseconds for the motors to run.
void motor_loop() {
    unsigned long now = millis();
    
    if(motors_stopped || (ml_expire != 0 && ml_expire <= now)) {
        MOTOR_LEFT.runSpeed(0.0);
        ml_expire = 0;
    }
    if(motors_stopped || (mr_expire != 0 && mr_expire <= now)) {
        MOTOR_RIGHT.runSpeed(0.0);
        mr_expire = 0;
    }

    MOTOR_LEFT.loop();
    MOTOR_RIGHT.loop();
}

/* Sets motor speeds in RPMs. Negative goes backwards. Motors are moved at that speed for `for_ms` milliseconds,
 after which they stop. This function can be called again to restart the timer. Passing zero for `for_ms` doesn't
 expire the motor movement. */
void set_motors(float left_speed, float right_speed, unsigned long for_ms) {
    // don't execute if we've stopped the motors over serial port
    if(motors_stopped) return;

    char fl_left[16];
    char fl_right[16];
    char upd8buf[128];
    dtostrf(left_speed, 3, 2, fl_left);
    dtostrf(right_speed, 3, 2, fl_right);
    int written = snprintf(upd8buf, sizeof(upd8buf),
        "[%08lu] setting motors to (%s, %s)",
        millis(), fl_left, fl_right
    );
    Serial.println(upd8buf);

    // reverse right_speed to account for mirrored motors (so pos is forward, like how left is)
    right_speed = -right_speed;

    // set the motor speed
    MOTOR_LEFT.runSpeed(left_speed);
    MOTOR_RIGHT.runSpeed(right_speed);

    // get the time if this movement period is bounded
    unsigned long now = 0;
    if(for_ms != 0) now = millis();

    // set the expire timers
    ml_expire = now + for_ms;
    mr_expire = now + for_ms;
}

/*Function For Handling Object Detection*/
bool object_detect(double dist_cm) {
    if (dist_cm <= TURN_IF_WITHIN) {     /*ultraSensor.distanceCm() Limit may need to be increased from 15*/
        Serial.print("triggering object detection: ");
        Serial.println(dist_cm);
        set_motors(-MOTOR_SPEED, MOTOR_SPEED, 1000);
        return true;
    }
    return false;
}

/*Function For Handling RSSI Navigation*/
bool rssi_nav(int *rssi_new, int *rssi_old) {
    // Serial.print("rssi nav: (old, new) = (");
    // Serial.print(*rssi_new);
    // Serial.print(",");
    // Serial.print(*rssi_old);
    // Serial.println(")");

    if (*rssi_new == -10000) {
        // keep motors off if we never received an RSSI value
        // set_motors(0, 0, 0);
        return false;
    } else {
        // otherwise act acccording to that new value
        int diff = *rssi_old - *rssi_new;

        if(diff == 0) { // stays the same
            set_motors(MOTOR_SPEED, MOTOR_SPEED, 500);
        } else if(diff < 0) { // gets closer?
            set_motors(MOTOR_SPEED, MOTOR_SPEED, MOTOR_HOLD_MS);
        } else if(diff > 0) { // gets further?
            set_motors(-MOTOR_SPEED, -MOTOR_SPEED, 500);
        }
        *rssi_old = *rssi_new;
        return true;
    }
}

enum BOT_STATE {
    STATE_START,
    STATE_RSSI_INIT,
    STATE_FORWARD,
    STATE_OBJ_DETECT,
    STATE_ROTATE,
};

enum BOT_STATE state = STATE_START;
bool motor_test = false;

void loop() {
    unsigned long now = millis();
    if(last_print + freq_print < now) {
        // print internal state to serial
        char upd8float[16];
        char upd8buf[256];
        dtostrf(curr_dist_cm, 3, 2, upd8float);
        int written = snprintf(upd8buf, sizeof(upd8buf),
            "[%08lu] state=%d, rssi_old=%+8d, rssi_new=%+8d, rssi_diff=%+3d, dist_cm=%6s (from %lu), motor_expire=(%8lu, %8lu) motors=%s",
            now, state, rssi_old, rssi_new, rssi_old-rssi_new, upd8float, last_dist_poll, ml_expire, mr_expire,
            motors_stopped ? "disabled" : "enabled"
        );
        Serial.println(upd8buf);

        last_print = now;
    }
    if(last_dist_poll + freq_dist_poll < now) {
        // poll distance sensor
        curr_dist_cm = ultraSensor.distanceCm();
        last_dist_poll = now;
    }
    if((char) Serial.read() == 't') {
        // toggle motors if 't' was sent over serial
        // mostly for debugging so the rover doesn't go wild
        Serial.println("motors toggled!");
        motors_stopped = !motors_stopped;
        if(motors_stopped) {
            set_motors(0, 0, 0);
        }
    }

    // let the motors, LED Ring animation, and bluetooth routines update as appropriate
    motor_loop();
    anim.tick();
    read_bt("bt_ext", &bte, bte_linebuf, &bte_linebuf_len);


    if(!motor_test && ml_expire == 0 && mr_expire == 0) {
        // only advance our state machine if our movement has expired
        switch(state) {
            case STATE_START: {
                state = STATE_RSSI_INIT;
                break;
            };
            case STATE_RSSI_INIT: {
                /* wait for both RSSI values to fill. If rssi_new is filled, but not rssi_old, fill it with rssi_new */
                if(rssi_new != -1000) {
                    if(rssi_old == -1000) {
                        rssi_old = rssi_new;
                    } else if(rssi_old != rssi_new) {
                        state = STATE_FORWARD;
                    }
                }
                const unsigned long TWO_MINS = 2UL*60*1000;
                if(millis() > TWO_MINS) {
                    motor_test = true;
                    anim.set_anim_color(30, 30, 0);
                }
                break;
            };
            case STATE_FORWARD: {
                set_motors(50, 50, 2000);
                state = STATE_OBJ_DETECT;
                break;
            };
            case STATE_OBJ_DETECT: {
                if(curr_dist_cm < TURN_IF_WITHIN) {
                    /* turn 60 degrees right */
                    Serial.println("turning 60deg right");
                    set_motors(-50, 50, 1000);
                    state = STATE_FORWARD;
                } else {
                    
                    state = STATE_FORWARD;
                }
                break;
            }
        }
        
        return;
    }


    // only run navigation if the motors are available (eg: not in the middle of another step)
    if(ml_expire == 0 && mr_expire == 0) {
        // we haven't elapsed our hold time yet

        // debug - different cases for motor movement depending on time
        unsigned long state = (millis() / 20000) % 4;
        float l, r;
        switch(state) {
            case 0: l=MOTOR_SPEED; r=MOTOR_SPEED; break;
            case 1: l=MOTOR_SPEED; r=-MOTOR_SPEED; break;
            case 2: l=-MOTOR_SPEED; r=-MOTOR_SPEED; break;
            case 3: l=-MOTOR_SPEED; r=MOTOR_SPEED; break;
        }
        set_motors(l, r, 5000);
        motors_until = millis() + 1000;

        // bool activated = object_detect(curr_dist_cm);
        // if(!activated) {
        //     activated = rssi_nav(&rssi_new, &rssi_old);
        // }
    }

    // artificial delay in loop? can't think of a reason to actually need this
    // delay(100);
}

