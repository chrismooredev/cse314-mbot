#define AURIGARINGLEDNUM  12
#include <MeRGBLed.h>

class AnimateRing {
    MeRGBLed *ring;
    uint8_t current;
    bool active;
    bool flashed;
    uint32_t speed;
    uint32_t last_tick;

public:
    AnimateRing(MeRGBLed *ring);
    AnimateRing(MeRGBLed *ring, bool start);
    AnimateRing(MeRGBLed *ring, bool startActive, uint8_t start_at, uint32_t speed);
    void tick();
    
    void resume();
    void resume(bool reset);

    void stop();
    void stop(bool clear);

    void flash(uint8_t r, uint8_t g, uint8_t b);
};
