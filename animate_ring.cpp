#define AURIGARINGLEDNUM  12
#include <MeRGBLed.h>
#include "animate_ring.h"

AnimateRing::AnimateRing(MeRGBLed *ring) : AnimateRing(ring, false) {}
AnimateRing::AnimateRing(MeRGBLed *ring, bool start) : AnimateRing(ring, start, 0, 500) {}
AnimateRing::AnimateRing(MeRGBLed *ring, bool startActive, uint8_t start_at, uint32_t speed) {
    this->ring = ring;
    this->current = start_at;
    this->active = startActive;
    this->last_tick = 0; // initialized on first run
    this->speed = speed;
}
void AnimateRing::tick() {
    if(! this->active && ! this->flashed) return;
    
    if(this->last_tick == 0) {
        this->last_tick = millis();
        this->ring->setColor(this->current, 50, 0, 0);
        this->ring->show();
        return;
    }

    uint32_t now = millis();

    if(((this->last_tick) + speed) > now) return;
    
    if(this->flashed) {
        this->ring->setColor(0, 0, 0);
    }

    this->last_tick += this->speed;
    this->ring->setColor(this->current, 0, 0, 0);
    this->current++;
    if(this->current > AURIGARINGLEDNUM) this->current = 1;
    this->ring->setColor(this->current, 50, 0, 0);
    this->ring->show();
}

void AnimateRing::resume() { this->resume(true); }
void AnimateRing::resume(bool reset) {
    this->active = true;
    this->tick();
}

void AnimateRing::stop() { this->stop(true); }
void AnimateRing::stop(bool clear) {
    this->active = false;
    if(clear) {
        this->ring->setColor(0, 0, 0);
        this->ring->show();
    }
}

void AnimateRing::flash(uint8_t r, uint8_t g, uint8_t b) {
    this->flashed = true;
    this->last_tick = millis();
    this->ring->setColor(r, g, b);
    this->ring->show();
}
