#include "minivan.h"
#include "features/caps_word.h"

#define SFT_TAB LSFT(KC_TAB)
#define LT2_TAB LT(2, KC_TAB)
#define LT3_SPC LT(3, KC_SPC)
#define LT2_SPC LT(2, KC_SPC)
#define LT3_ENT LT(3, KC_ENT)
#define T_SFT_QT RSFT_T(KC_QUOT)

enum custom_keycodes {
  M_IME = SAFE_RANGE,
  M_LSFT
};

enum {
  TD_SFT_L3 = 0
};

qk_tap_dance_action_t tap_dance_actions[] = {
  [TD_SFT_L3] = ACTION_TAP_DANCE_LAYER_TOGGLE(KC_LSFT, 3),
};

uint16_t key_timer;

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  if (!process_caps_word(keycode, record)) {
    return false;
  }

  if (record->event.pressed) {
    switch(keycode) {
      case M_IME:
        SEND_STRING(SS_DOWN(X_LSHIFT)SS_DOWN(X_LALT));
        return false;
      case M_LSFT:
        key_timer = timer_read();
        register_code(KC_LSFT);
        return false;
    }
  } else {
    switch(keycode) {
      case M_IME:
        SEND_STRING(SS_UP(X_LSHIFT)SS_UP(X_LALT));
        return false;
      case M_LSFT:
        unregister_code(KC_LSFT);
        if (timer_elapsed(key_timer) < 125) {
          caps_word_set(!caps_word_get());
        }
        return false;
    }
  }
  return true;
};

bool caps_word_press_user(uint16_t keycode) {
  switch (keycode) {
    // Keycodes that continue Caps Word, with shift applied.
    case KC_A ... KC_Z:
      add_weak_mods(MOD_BIT(KC_LSFT));  // Apply shift to the next key.
      return true;

    // Keycodes that continue Caps Word, without shifting.
    case KC_1 ... KC_0:
    case KC_BSPC:
    case KC_MINS:
    case KC_UNDS:
      return true;

    case M_LSFT:
      return true;

    default:
      return false;  // Deactivate Caps Word.
  }
}

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT(
    KC_ESC ,     KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,  KC_BSPC,
    LT2_TAB,     KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,  KC_SCLN,  KC_MINS,
    M_LSFT,     KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,  KC_COMM,   KC_DOT,  KC_SLSH, T_SFT_QT,
    KC_LCTL,  KC_LGUI,  KC_LALT,    TO(1),       LT3_ENT     ,       LT2_SPC     ,    TG(3),  KC_RSFT,  KC_RCTL,    TG(4)
  ),
  [1] = LAYOUT(
    KC_ESC ,     KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,  KC_BSPC,
    KC_TAB ,     KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,  KC_SCLN,   KC_ENT,
    KC_LSFT,     KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,  KC_COMM,   KC_DOT,  KC_SLSH,  KC_QUOT,
    KC_LCTL,  KC_LGUI,  KC_LALT,    TO(0),       LT3_SPC     ,       LT2_SPC     ,    TG(3),  KC_RSFT,  KC_CAPS,    TG(4)
  ),
  [2] = LAYOUT(
    KC_GRV ,  KC_PIPE,  KC_BSLS,  _______,  _______,  _______,  _______,  SFT_TAB,   KC_TAB,  KC_LBRC,  KC_RBRC,   KC_DEL,
    _______,  KC_PLUS,   KC_EQL,    KC_GT,  _______,  _______,  KC_LEFT,  KC_DOWN,    KC_UP, KC_RIGHT,  _______,  KC_CAPS,
    _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,
    _______,  _______,  _______,  _______,       _______     ,       _______     ,  _______,  _______,  CG_TOGG,    RESET
  ),
  [3] = LAYOUT(
    _______,  _______,  _______,  _______,  _______,  _______,  _______,     KC_7,     KC_8,     KC_9,     KC_0,  _______,
    _______,  _______,  _______,  _______,  _______,  _______,  _______,     KC_4,     KC_5,     KC_6,  _______,  _______,
    _______,  _______,  _______,  _______,  _______,  _______,   KC_DOT,     KC_1,     KC_2,     KC_3, KC_MS_UP,  _______,
    _______,  _______,  _______,  _______,       _______     ,       _______     ,  _______,KC_MS_LEFT,KC_MS_DOWN,  KC_MS_RIGHT
  ),
  [4] = LAYOUT(
    _______,   KC_F14,   KC_F15,  _______,  _______,  _______,  _______,    KC_F1,    KC_F2,    KC_F3,    KC_F4,  _______,
    _______,  KC_VOLD,  KC_VOLU,  KC_MUTE,  _______,  _______,  _______,    KC_F5,    KC_F6,    KC_F7,    KC_F8,   KC_INS,
    _______,  _______,  _______,  _______,  _______,  _______,  _______,    KC_F9,   KC_F10,   KC_F11,   KC_F12,  _______,
    _______,  _______,  _______,  _______,       _______     ,       _______     ,  _______,  _______,  _______,  _______
  )
  /* [TEMPLATE] = LAYOUT( */
  /*   _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______, */
  /*   _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______, */
  /*   _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______, */
  /*   _______,  _______,  _______,  _______,       _______     ,       _______     ,  _______,  _______,  _______,  _______ */
  /* ) */
};

void process_indicator_update(layer_state_t state, uint8_t usb_led) {
  for (int i = 0; i < 3; i++) {
    setrgb(0, 0, 0, (LED_TYPE *)&led[i]);
  }

  setrgb(218, 23, 180, (LED_TYPE *)&led[0]);
  if (state & (1<<1)) {
    setrgb(0, 255, 0, (LED_TYPE *)&led[0]);
  }

  if (state & (1<<3)) {
    setrgb(32, 104, 255, (LED_TYPE *)&led[1]);
  }

  if (state & (1<<2)) {
    setrgb(114, 0, 255, (LED_TYPE *)&led[1]);
  }

  if (state & (1<<4)) {
    setrgb(97, 213, 84, (LED_TYPE *)&led[1]);
  }

  if (usb_led & (1<<USB_LED_CAPS_LOCK)) {
    setrgb(255, 78, 79, (LED_TYPE *)&led[2]);
  }

  rgblight_set();
};

void keyboard_post_init_user(void) {
  process_indicator_update(layer_state, host_keyboard_leds());
};

void led_set_user(uint8_t usb_led) {
  process_indicator_update(layer_state, host_keyboard_leds());
};

layer_state_t layer_state_set_user(layer_state_t state) {
  process_indicator_update(state, host_keyboard_leds());
  return state;
};

