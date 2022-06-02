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
    this->r = this->g = this->b = 2;
}
void AnimateRing::tick() {
    if(! this->active && ! this->dirty) return;
    
    if(this->last_tick == 0) {
        this->last_tick = millis();
        this->ring->setColor(this->current, this->r, this->g, this->b);
        this->ring->show();
        return;
    }

    uint32_t now = millis();

    if(((this->last_tick) + speed) > now) return;
    
    if(this->dirty) {
        this->ring->setColor(0, 0, 0);
    }

    this->last_tick += this->speed;
    this->ring->setColor(this->current, 0, 0, 0);
    this->current++;
    if(this->current > AURIGARINGLEDNUM) this->current = 1;
    this->ring->setColor(this->current, this->r, this->g, this->b);
    this->ring->show();
}
void AnimateRing::set_anim_color(uint8_t r, uint8_t g, uint8_t b) {
    this->r = r;
    this->g = g;
    this->b = b;
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
    this->dirty = true;
    this->last_tick = millis();
    this->ring->setColor(r, g, b);
    this->ring->show();
}

void AnimateRing::set_color(int8_t ind, uint8_t r, uint8_t g, uint8_t b) {
    while(ind < 0) {
        ind += AURIGARINGLEDNUM;
    }
    // 2 = offset to make LED(0) be forward
    ind = (ind+2) % AURIGARINGLEDNUM;
    ++ind; // conv zero-indexed to one-indexed
    this->ring->setColor(ind, r, g, b);
    this->dirty = true;
}
void AnimateRing::set_dirty() {
    this->dirty = true;
    this->ring->show();
}
