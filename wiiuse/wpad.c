/* SPDX-License-Identifier: Zlib */
/* SPDX-FileCopyrightText: Copyright devkitPro */
/* SPDX-FileCopyrightText: Alberto Mardegan (mardy) */

#include "wpad.h"

#include "bt-embedded/buffer.h"
#include "bt-embedded/hci.h"
#include "embedded-game-controller/egc.h"
#include "embedded-game-controller/egc_audio.h"
#include "embedded-game-controller/egc_message.h"
#include "embedded-game-controller/egc_timer.h"
#include "embedded-game-controller/hid_drivers/wiimote.h"

#include <limits.h>
#include <math.h>
#include <ogc/conf.h>
#include <ogc/irq.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tuxedo/sync.h>
#include <tuxedo/thread.h>
#include <tuxedo/tick.h>

#define WPAD_LOG(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)

#ifdef WITH_DEBUG
#define WPAD_DEBUG(fmt, ...) WPAD_LOG("[D] %s:%d " fmt "\n", __func__, __LINE__, ##__VA_ARGS__)
#else
#define WPAD_DEBUG(fmt, ...)
#endif
#define WPAD_WARNING(fmt, ...) WPAD_LOG("[W] " fmt "\n", ##__VA_ARGS__)

#define BIT(x) (1 << x)
#define RAD_TO_DEGREE(r) (r * 180.0f / (float)M_PI)

#define OBJECT_FROM_MEMBER(object, member, ptr) \
    (object *)((u8*)ptr - offsetof(object, member))

#define ACCEL_ZERO 512
#define ACCEL_ONE (256 * 3)

#define REPORT_QUEUE_SIZE 2

enum ir_state_t {
    IR_STATE_DEAD = 0,
    IR_STATE_GOOD,
    IR_STATE_SINGLE,
    IR_STATE_LOST,
};

typedef struct controller_t Controller;
typedef struct input_report_t InputReport;

typedef struct audio_data_t {
    u8 *samples;
    u16 size;
    u16 played_samples;
} AudioData;

enum worker_command_e {
    WORKER_CMD_CONTROL_SPEAKER,
    WORKER_CMD_PLAY_AUDIO,
    WORKER_CMD_SET_DATA_FORMAT,
    WORKER_CMD_SET_MOTION_PLUS,
    WORKER_CMD_SET_RUMBLE,
    WORKER_CMD_DISCONNECT,

    /* Commands without controller parameter */
    WORKER_CMD_SEARCH_START,
    WORKER_CMD_SEARCH_STOP,
    WORKER_CMD_PAIRING_START,
    WORKER_CMD_PAIRING_STOP,
    WORKER_CMD_WIPE_SAVED,
} ATTRIBUTE_PACKED;

union worker_command_u {
    struct worker_command_t {
        enum worker_command_e opcode;
        s8 controller_index;
        /* Parameters */
        union {
            bool rumble_enabled;
        };
    } ATTRIBUTE_PACKED cmd;
    egc_message_t msg;
};

static struct controller_t {
    egc_input_device_t *egc_device;
    egc_audio_device_t *audio_device;
    s32 audio_format;
    AudioData audio_data;
    egc_timer_t audio_timer;
    KThread audio_thread;
    u8 *audio_thread_stack;
    WPADData data;
    struct input_report_t {
        EgcWiimoteExpType exp_type;
        egc_connection_e connection_type;
        u8 num_accelerometers;
        u8 ir_report_size;
        u8 battery_level;
        bool battery_critical;
        bool has_gyro;
        bool has_touch;
        u32 available_buttons;
        u32 available_axes;
        u32 buttons;
        egc_accelerometer_t accel[2];
        egc_gyroscope_t gyro;
        s16 axes[EGC_GAMEPAD_AXIS_COUNT];
        egc_point_t touch;
        egc_point_t sb_points[2];
        int sb_width;
        float sb_angle;
        u8 ir_report[12];
    } reports[REPORT_QUEUE_SIZE];
    /* If the client does not call WPAD_ReadEvent() often enough, it will
     * miss some updates.
     * These next two members are indices into the above "reports" array, which
     * we use as a ring buffer. */
    s8 report_oldest;
    s8 report_newest;
    s8 num_dropped_reports;
    u32 buttons_last;
    struct state_t {
        u32 buttons;
        bool ir_valid;
        float ir_x;
        float ir_y;
        struct vec3w_t accel;
        int exp_type;
        union {
            struct state_nunchuk_t {
                u8 buttons;
                struct vec2b_t js;
                struct vec3w_t accel;
            } nunchuk;
            struct state_classic_t {
                u32 buttons;
                struct vec2b_t ljs;
                struct vec2b_t rjs;
            } classic;
            struct state_guitar_t {
                u32 buttons;
                struct vec2b_t js;
                u8 whammy_bar;
                u8 touch_bar;
            } guitar;
            struct state_balance_t {
                u16 corners[4];
            } balance;
            struct state_motion_plus_t {
                s16 xyz[3];
            } motion_plus;
        } exp;
    } last_state;
    struct thresholds_t {
        s32 buttons;
        s32 ir;
        s32 accel;
        s32 js;
        s32 wb;
        s32 mp;
    } idle_thresholds;
    u32 last_state_time; /* in seconds */
    u8 data_format;
    bool mp_requested : 1;
    bool accel_requested : 1;
    bool is_wiimote: 1;
    bool speaker_requested : 1;
    bool battery_critical : 1;
} s_controllers[WPAD_MAX_DEVICES];
#ifdef WIIUSE_HAS_PAD_STATUS_API
WPADStatusCallback s_status_cb = NULL;
#endif
static u32 s_idle_timeout = 300;
static conf_pads s_conf_devices;
static conf_pad_guests s_conf_guest_devices;
static egc_bt_stored_link_key_t s_stored_keys[16]; /* The Wii BT controller stores 16 */
static KThread s_egc_event_thread;
#define EGC_EVENT_THREAD_STACK_SIZE (1048 * 4)
#define STACK_SENTINEL 0x3a
static u8 *s_egc_event_thread_stack = NULL;
/* We could have a report mutex for each Controller, but it's probably not
 * worth it, since we are updating them all at the same time. */
static KMutex s_report_mutex;

/* This is coded as a bit flag because one might call WPAD_StartPairing() while
 * WPAD_Search() is active. In that case, we want to return back to guest
 * pairing once the normal pairing operation completes.
 */
static enum {
    PAIRING_MODE_NONE = 0,
    PAIRING_MODE_GUEST = 1 << 0,
    PAIRING_MODE_NORMAL = 1 << 1,
} s_pairing_mode = PAIRING_MODE_NONE;
static u32 s_pairing_start_time;

static void default_host_sync_button_cb(u32 held);
static WPADHostSyncBtnCallback s_host_sync_button_cb = default_host_sync_button_cb;
static WPADDisconnectCallback s_disconnect_cb;

static bool s_initialized = false;
static bool s_terminated = false;

/* EGC private API */
void _egc_bt_on_initialized(void *);

static void debug_print_free_stack(const u8 *stack, int size)
{
#ifdef WPAD_DEBUG
    int i = 0;
    for (i = 0; i < size; i++) {
        if (stack[i] != STACK_SENTINEL) break;
    }
    WPAD_DEBUG("EGC thread free stack: %d bytes", i);
#endif
}

static inline struct worker_command_t worker_command(egc_message_t msg)
{
    union worker_command_u u;
    u.msg = msg;
    return u.cmd;
}

static inline egc_message_t worker_command_to_msg(enum worker_command_e opcode,
                                                  Controller *controller)
{
    union worker_command_u u = { 0 };
    u.cmd.opcode = opcode;
    u.cmd.controller_index = controller ? (controller - s_controllers) : -1;
    return u.msg;
}

static void worker_post_command(enum worker_command_e opcode,
                                Controller *controller)
{
    egc_message_post(worker_command_to_msg(opcode, controller));
}

static u32 gettime_secs()
{
    return PPCGetTickCount() / PPC_TIMER_CLOCK;
}

static Controller *controller_from_egc(egc_input_device_t *device)
{
    for (int i = 0; i < WPAD_MAX_DEVICES; i++) {
        if (s_controllers[i].egc_device == device) {
            return &s_controllers[i];
        }
    }
    return NULL;
}

static Controller *controller_by_channel(int channel)
{
    return channel >= 0 && channel < WPAD_MAX_DEVICES ?
           &s_controllers[channel] : NULL;
}

static void controller_init(Controller *controller)
{
    bool sensor_bar_below = CONF_GetSensorBarPosition() == 0;

    controller->last_state_time = gettime_secs();
    controller->report_oldest = controller->report_newest = -1;
    WPADData *data = &controller->data;
    data->ir.vres[0] = 560;
    data->ir.vres[1] = 420;
    data->ir.aspect = WIIUSE_ASPECT_4_3;
    data->ir.pos = sensor_bar_below ? WIIUSE_IR_BELOW : WIIUSE_IR_ABOVE;
}

static void controller_update_data_format(Controller *controller)
{
    egc_input_device_t *device = controller->egc_device;
    u8 fmt = controller->data_format;
    bool ir_requested = fmt == WPAD_FMT_BTNS_ACC_IR;
    controller->accel_requested =
        fmt == WPAD_FMT_BTNS_ACC || fmt == WPAD_FMT_BTNS_ACC_IR;
    for (int i = 0; i < device->desc->num_accelerometers; i++) {
        egc_input_device_enable_accelerometer(device, i,
                                              controller->accel_requested);
    }
    egc_input_device_enable_touch_point(device, 0, ir_requested);
}

static void controller_update_motion_plus(Controller *controller)
{
    egc_input_device_enable_gyroscope(controller->egc_device, 0,
                                      controller->mp_requested);
}

static inline egc_bt_address_t address_from_conf(const u8 *conf)
{
    egc_bt_address_t addr = {{ conf[5], conf[4], conf[3], conf[2], conf[1], conf[0] }};
    return addr;
}

static inline void address_to_conf(const egc_bt_address_t *address, u8 *conf)
{
    conf[0] = address->bytes[5];
    conf[1] = address->bytes[4];
    conf[2] = address->bytes[3];
    conf[3] = address->bytes[2];
    conf[4] = address->bytes[1];
    conf[5] = address->bytes[0];
}

static int get_active_slot(const egc_bt_address_t *address)
{
    /* Note the "CONF_PAD_MAX_ACTIVE + 1": it's a hack to include the balance
     * board in the loop: the balance board is stored in a conf_pad_device
     * struct right after the array of the active devices. */
    for (int i = 0; i < CONF_PAD_MAX_ACTIVE + 1; i++) {
        egc_bt_address_t bdaddr = address_from_conf(s_conf_devices.active[i].bdaddr);
        if (egc_bt_address_cmp(address, &bdaddr) == 0) {
            WPAD_DEBUG("Device is active");
            return i;
        }
    }
    return -1;
}

static conf_pad_guest_device *get_guest_device(const egc_bt_address_t *address)
{
    for (int i = 0; i < s_conf_guest_devices.num_guests; i++) {
        egc_bt_address_t bdaddr = address_from_conf(s_conf_guest_devices.guests[i].bdaddr);
        if (egc_bt_address_cmp(address, &bdaddr) == 0) {
            return &s_conf_guest_devices.guests[i];
        }
    }
    return NULL;
}

static conf_pad_device *get_registered_device(const egc_bt_address_t *address)
{
    for (int i = 0; i < CONF_PAD_MAX_REGISTERED; i++) {
        egc_bt_address_t bdaddr = address_from_conf(s_conf_devices.registered[i].bdaddr);
        if (egc_bt_address_cmp(address, &bdaddr) == 0) {
            return &s_conf_devices.registered[i];
        }
    }
    return NULL;
}

static bool is_registered(const egc_bt_address_t *address)
{
    return get_registered_device(address) != NULL;
}

static void device_added_cb(egc_input_device_t *device, void *userdata)
{
    int chan = -1;

    egc_bt_address_t address;
    egc_bt_device_get_address(device, &address);
    int slot = get_active_slot(&address);
    /* If the slot is still free (it should!), use it; otherwise, look for a
     * free one */
    if (slot >= 0 && s_controllers[slot].egc_device == NULL) {
        chan = slot;
    } else if (device->desc->type == EGC_DEVICE_TYPE_BALANCE_BOARD) {
        if (!s_controllers[WPAD_BALANCE_BOARD].egc_device) {
            chan = WPAD_BALANCE_BOARD;
        }
    } else {
        for (int i = 0; i < WPAD_MAX_DEVICES; i++) {
            Controller *controller = &s_controllers[i];
            if (!controller->egc_device) {
                /* We've found a free slot */
                chan = i;
                break;
            }
        }
    }

    if (chan < 0) {
        WPAD_WARNING("All controller channels busy");
        return;
    }

    bool conf_changed = false;
    bool guest_conf_changed = false;
    if (conf_pad_device *reg = get_registered_device(&address)) {
        if (reg->name[0] == '\0') {
            strncpy(reg->name, device->desc->name, sizeof(reg->name));
            conf_changed = true;
        }
    } else if (conf_pad_guest_device *guest = get_guest_device(&address)) {
        if (guest->name[0] == '\0') {
            strncpy(guest->name, device->desc->name, sizeof(guest->name));
            guest_conf_changed = true;
        }
    }

    if (chan != slot) {
        /* Need to update the list of active devices; note that while this
         * array has only 4 elements, the struct field after it is another
         * conf_pad_device struct for the balance board, so we can treat them
         * as if it was an array with 5 elements */
        address_to_conf(&address, s_conf_devices.active[chan].bdaddr);
        /* Note: the "name" member of the active structs is not used, so we are
         * not filling it up here */
        conf_changed = true;
    }

    if (conf_changed) {
        CONF_SetPadDevices(&s_conf_devices);
    }
    if (guest_conf_changed) {
        CONF_SetPadGuestDevices(&s_conf_guest_devices);
    }

    Controller *controller = &s_controllers[chan];
    controller_init(controller);
    controller->egc_device = device;
    controller->is_wiimote =
        device->desc->vendor_id == 0x057e &&
        (device->desc->product_id == 0x0306 || device->desc->product_id == 0x0330);
    egc_input_device_set_leds(device, 1 << chan);
    controller_update_data_format(controller);
    controller_update_motion_plus(controller);
}

static void device_removed_cb(egc_input_device_t *device, void *userdata)
{
    Controller *controller = controller_from_egc(device);
    if (controller) {
        if (s_disconnect_cb) {
            int reason = egc_bt_device_get_disconnection_reason(device);
            s_disconnect_cb(controller - s_controllers, reason);
        }

        /* Remember the persistent settings */
        u8 data_format = controller->data_format;
        bool mp_requested = controller->mp_requested;
        struct thresholds_t thresholds = controller->idle_thresholds;

        memset(controller, 0, sizeof(Controller));
        controller_init(controller);

        controller->idle_thresholds = thresholds;
        controller->data_format = data_format;
        controller->mp_requested = mp_requested;
    }
}

static egc_bt_connection_reply_e connection_filter_cb(
    const egc_bt_address_t *address,
    u16 service_class, u8 device_class_major, u8 device_class_minor,
    bool is_incoming, void *userdata)
{
    WPAD_DEBUG("Connection with " EGC_BT_ADDRESS_FMT ", incoming=%d",
               EGC_BT_ADDRESS_DATA(address), is_incoming);

    if (is_incoming) {
        if (get_active_slot(address) >= 0 || is_registered(address)) {
            return EGC_BT_CONNECTION_REPLY_REQ_AUTH;
        }

        WPAD_WARNING("Device " EGC_BT_ADDRESS_FMT " not paired, refusing connection",
                     EGC_BT_ADDRESS_DATA(address));
        return EGC_BT_CONNECTION_REPLY_REFUSE;
    } else {
        if (s_pairing_mode & (PAIRING_MODE_NORMAL | PAIRING_MODE_GUEST)) {
            /* libogc's WPAD code is not doing this, but if we don't remove the
             * old key, and if it's wrong, the Wiimote won't connect. */
            egc_bt_delete_link_key(address);
        }
        /* The connection is initiated from the Wii, meaning that we are
         * pairing to a new device. No matter whether this is a permanent or
         * guest pairing, both require an authenticated connection.
         */
        return EGC_BT_CONNECTION_REPLY_REQ_AUTH;
    }
}

static void link_key_requested_cb(const egc_bt_address_t *address, void *userdata)
{
    WPAD_DEBUG("Address: " EGC_BT_ADDRESS_FMT ", pairing mode %d", EGC_BT_ADDRESS_DATA(address), s_pairing_mode);

    conf_pad_guest_device *guest = get_guest_device(address);
    if (guest && !(s_pairing_mode & PAIRING_MODE_NORMAL)) {
        WPAD_DEBUG("Sending link key");
        egc_bt_send_link_key(address, guest->link_key);
    } else {
        WPAD_DEBUG("No link key");
        egc_bt_send_link_key(address, NULL);
    }
}

static void pin_requested_cb(const egc_bt_address_t *address, void *userdata)
{
    WPAD_DEBUG("Address: " EGC_BT_ADDRESS_FMT ", pairing mode %d", EGC_BT_ADDRESS_DATA(address), s_pairing_mode);
    if (s_pairing_mode & PAIRING_MODE_NORMAL) {
        u8 pin[6];
        egc_bt_get_local_address((egc_bt_address_t *)pin);
        egc_bt_send_pin(address, pin, sizeof(pin));
    } else if (s_pairing_mode & PAIRING_MODE_GUEST) {
        egc_bt_send_pin(address, (u8 *)address, 6);
    } else {
        /* We should not end up here, but the HID spec recommends a simple PIN. */
        egc_bt_send_pin(address, NULL, 0);
    }
}

static void link_key_received_cb(const egc_bt_address_t *address,
                                 const egc_bt_link_key_t *key, void *userdata)
{
    WPAD_DEBUG("Address: " EGC_BT_ADDRESS_FMT, EGC_BT_ADDRESS_DATA(address));

    if (s_pairing_mode & PAIRING_MODE_NORMAL) {
        egc_bt_store_link_key(address, key);
        conf_pad_device *reg = get_registered_device(address);
        if (!reg) {
            WPAD_DEBUG("Adding registered device");
            memmove(&s_conf_devices.registered[1],
                    &s_conf_devices.registered[0],
                    sizeof(conf_pad_device) * (CONF_PAD_MAX_REGISTERED - 1));
            reg = &s_conf_devices.registered[0];
            address_to_conf(address, reg->bdaddr);
            /* Name is set after the connection gets established */
            memset(reg->name, 0, sizeof(reg->name));

            if (s_conf_devices.num_registered < CONF_PAD_MAX_REGISTERED) {
                s_conf_devices.num_registered++;
            }
        }
        WPAD_DEBUG("Registered device is in slot %d, name '%s'", reg - s_conf_devices.registered, reg->name);
        /* Remove the device from the guest list, if present */
        conf_pad_guest_device *r = get_guest_device(address);
        if (r) {
            int index = r - s_conf_guest_devices.guests;
            memmove(r, r + 1, sizeof(conf_pad_guest_device) * (CONF_PAD_ACTIVE_AND_OTHER - index - 1));
            s_conf_guest_devices.num_guests--;
        }
    } else if (s_pairing_mode & PAIRING_MODE_GUEST) {
        conf_pad_guest_device *r = get_guest_device(address);
        if (!r) {
            WPAD_DEBUG("Adding guest device");
            /* Allocate slot 0 */
            memmove(&s_conf_guest_devices.guests[1],
                    &s_conf_guest_devices.guests[0],
                    sizeof(conf_pad_guest_device) * (CONF_PAD_ACTIVE_AND_OTHER - 1));
            r = &s_conf_guest_devices.guests[0];
            if (s_conf_guest_devices.num_guests < CONF_PAD_MAX_ACTIVE) {
                s_conf_guest_devices.num_guests++;
            }
        }
        WPAD_DEBUG("Guest device is in slot %d, name '%s'", r - s_conf_guest_devices.guests, r->name);
        address_to_conf(address, r->bdaddr);
        /* Name is set after the connection gets established */
        memset(r->name, 0, sizeof(r->name));
        memcpy(r->link_key, key, sizeof(r->link_key));
    }
}

static void update_expansion_buttons(WPADData *data)
{
    if (data->exp.type == EXP_NUNCHUK) {
        struct nunchuk_t *exp = &data->exp.nunchuk;
        exp->btns = data->btns_d >> 16;
        exp->btns_last = data->btns_l >> 16;
        exp->btns_held = data->btns_h >> 16;
        exp->btns_released = data->btns_u >> 16;
    } else if (data->exp.type == EXP_CLASSIC) {
        struct classic_ctrl_t *exp = &data->exp.classic;
        /* For historical reasons, the button words are reversed here */
#define WORD_SWAPPED(x) ((x << 16) | (x >> 16))
        exp->btns = WORD_SWAPPED(data->btns_d);
        exp->btns_last = WORD_SWAPPED(data->btns_l);
        exp->btns_held = WORD_SWAPPED(data->btns_h);
        exp->btns_released = WORD_SWAPPED(data->btns_u);
#undef WORD_SWAPPED
    } else if (data->exp.type == EXP_GUITAR_HERO_3) {
        struct guitar_hero_3_t *exp = &data->exp.gh3;
        exp->btns = data->btns_d >> 16;
        exp->btns_last = data->btns_l >> 16;
        exp->btns_held = data->btns_h >> 16;
        exp->btns_released = data->btns_u >> 16;
    }
    /* Motion+ and balance board have no extra buttons */
}

static void setup_joystick(struct joystick_t *js, int max)
{
    js->max.x = js->max.y = max;
    js->min.x = js->min.y = 0;
    js->center.x = js->center.y = max / 2;
}

static void setup_expansion(Controller *controller, InputReport *report)
{
    WPADData *data = &controller->data;

    int type = data->exp.type;
    memset(&data->exp, 0, sizeof(data->exp));
    data->exp.type = type;
    if (data->exp.type == EXP_NUNCHUK) {
        struct nunchuk_t *exp = &data->exp.nunchuk;
        setup_joystick(&exp->js, 255);
    } else if (data->exp.type == EXP_CLASSIC) {
        struct classic_ctrl_t *exp = &data->exp.classic;
        setup_joystick(&exp->ljs, 64);
        setup_joystick(&exp->rjs, 32);

        if (report->connection_type == EGC_CONNECTION_USB) {
            exp->wired = exp->charging = true;
        } else if (report->connection_type == EGC_CONNECTION_SERIAL) {
            exp->wired = true;
            exp->charging = false;
        } else {
            exp->wired = exp->charging = false;
        }
    }
}

static void update_accelerometer_values(InputReport *report, int index,
                                        struct vec3w_t *accel,
                                        struct orient_t *orient,
                                        struct gforce_t *gforce)
{
    const egc_accelerometer_t *a = &report->accel[index];
#define EGC_TO_WPAD(accel) \
    (ACCEL_ZERO + accel * ACCEL_ONE / EGC_ACCELEROMETER_RES_PER_G)
    accel->x = EGC_TO_WPAD(a->x);
    accel->y = EGC_TO_WPAD(-a->z);
    accel->z = EGC_TO_WPAD(a->y);
#undef EGC_TO_WPAD

    gforce->x = (float)a->x / EGC_ACCELEROMETER_RES_PER_G;
    gforce->y = -(float)a->z / EGC_ACCELEROMETER_RES_PER_G;
    gforce->z = (float)a->y / EGC_ACCELEROMETER_RES_PER_G;

    orient->yaw = 0;
    float v = RAD_TO_DEGREE(atan2f(gforce->x, gforce->z));
    if (isfinite(v)) {
        orient->a_roll = orient->roll = v;
    }
    v = RAD_TO_DEGREE(atan2f(gforce->y, gforce->z));
    if (isfinite(v)) {
        orient->a_pitch = orient->pitch = v;
    }
}

static void update_expansion_type(Controller *controller, InputReport *report)
{
    WPADData *data = &controller->data;

    int old_type = data->exp.type;

    switch (report->exp_type) {
    case EGC_WIIMOTE_EXP_NUNCHUCK:
        data->exp.type = EXP_NUNCHUK;
        break;
    case EGC_WIIMOTE_EXP_CLASSIC:
        data->exp.type = EXP_CLASSIC;
        data->exp.classic.type = CLASSIC_TYPE_ORIG;
        break;
    case EGC_WIIMOTE_EXP_CLASSIC_PRO:
        data->exp.type = EXP_CLASSIC;
        data->exp.classic.type = CLASSIC_TYPE_PRO;
        break;
    case EGC_WIIMOTE_EXP_CLASSIC_WIIU_PRO:
        data->exp.type = EXP_CLASSIC;
        data->exp.classic.type = CLASSIC_TYPE_WIIU;
        break;
    case EGC_WIIMOTE_EXP_GUITAR_HERO_3:
        data->exp.type = EXP_GUITAR_HERO_3;
        break;
    case EGC_WIIMOTE_EXP_BALANCE_BOARD:
        data->exp.type = EXP_WII_BOARD;
        break;
    case EGC_WIIMOTE_EXP_MOTION_PLUS:
        data->exp.type = EXP_MOTION_PLUS;
        break;
    default:
        data->exp.type = EXP_NONE;
        break;
    }

    if (data->exp.type != old_type) {
        setup_expansion(controller, report);
    }

    data->data_present |= WPAD_DATA_EXPANSION;
}

static void update_buttons(Controller *controller, const InputReport *report)
{
    WPADData *data = &controller->data;

    data->btns_h = report->buttons;
    data->btns_l = controller->buttons_last;
    data->btns_d = report->buttons & ~controller->buttons_last;
    data->btns_u = ~report->buttons & controller->buttons_last;

    data->data_present |= WPAD_DATA_BUTTONS;
}

static void update_accelerometer(Controller *controller, InputReport *report)
{
    if (controller->accel_requested && report->num_accelerometers > 0) {
        update_accelerometer_values(report, 0,
                                    &controller->data.accel,
                                    &controller->data.orient,
                                    &controller->data.gforce);
        controller->data.data_present |= WPAD_DATA_ACCEL;
    }
}

static void update_joystick(struct joystick_t *js, InputReport *report, int x_axis)
{
    s16 x = report->axes[x_axis];
    s16 y = -1 - report->axes[x_axis + 1];
    int max = js->max.x;
    js->pos.x = (x - INT16_MIN) * max / UINT16_MAX;
    js->pos.y = (y - INT16_MIN) * max / UINT16_MAX;

    float dx = x / (float)INT16_MAX;
    float dy = y / (float)INT16_MAX;
    js->ang = RAD_TO_DEGREE(atan2f(dx, dy));
    js->mag = hypotf(dx, dy);
}

static void update_expansion_data(Controller *controller, InputReport *report)
{
    WPADData *data = &controller->data;

    if (data->exp.type == EXP_NUNCHUK) {
        struct nunchuk_t *exp = &data->exp.nunchuk;
#ifdef WIIUSE_HAS_NUNCHUK_ACCEL_CALIB
        exp->accel_calib.cal_zero =
            (struct vec3w_t){ ACCEL_ZERO, ACCEL_ZERO, ACCEL_ZERO };
        exp->accel_calib.cal_g =
            (struct vec3w_t){ ACCEL_ONE, ACCEL_ONE, ACCEL_ONE };
#endif
        update_joystick(&exp->js, report, EGC_GAMEPAD_AXIS_LEFTX);
#ifdef WIIUSE_HAS_NUNCHUK_FLAGS
        exp->flags = NULL;
#endif
        update_accelerometer_values(report, 1,
                                    &exp->accel, &exp->orient, &exp->gforce);
    } else if (data->exp.type == EXP_CLASSIC) {
        struct classic_ctrl_t *exp = &data->exp.classic;

        update_joystick(&exp->ljs, report, EGC_GAMEPAD_AXIS_LEFTX);
        update_joystick(&exp->rjs, report, EGC_GAMEPAD_AXIS_RIGHTX);

        s16 left_trigger = report->axes[EGC_GAMEPAD_AXIS_LEFT_TRIGGER];
        s16 right_trigger = report->axes[EGC_GAMEPAD_AXIS_RIGHT_TRIGGER];
        exp->l_shoulder = ((float)left_trigger) / INT16_MAX;
        exp->r_shoulder = ((float)right_trigger) / INT16_MAX;

        /* TODO battery https://github.com/embedded-game-controller/embedded-game-controller/issues/71 */
    } else if (data->exp.type == EXP_GUITAR_HERO_3) {
        struct guitar_hero_3_t *exp = &data->exp.gh3;
        s16 *axes = report->axes;
        /* The raw values range between 0 and 0x1f, whereas EGC's max is 0x7fff */
        exp->wb_raw = axes[EGC_GUITAR_AXIS_WHAMMY_BAR] >> 10;
        exp->whammy_bar = axes[EGC_GUITAR_AXIS_WHAMMY_BAR] / (float)INT16_MAX;
        exp->tb_raw = axes[EGC_GUITAR_AXIS_EFFECT] >> 10;
        /* tb_raw ranges between 0 and 31, rest pos being 15 */
        int diff = (exp->tb_raw - 15) / 3;
        int tb;
        switch (diff) {
        case -4: tb = GUITAR_HERO_3_TOUCH_GREEN; break;
        case -3: tb = GUITAR_HERO_3_TOUCH_GREEN | GUITAR_HERO_3_TOUCH_RED; break;
        case -2: tb = GUITAR_HERO_3_TOUCH_RED; break;
        case -1: tb = GUITAR_HERO_3_TOUCH_RED | GUITAR_HERO_3_TOUCH_YELLOW; break;
        case 0: tb = GUITAR_HERO_3_TOUCH_AVAILABLE; break;
        case 1: tb = GUITAR_HERO_3_TOUCH_YELLOW; break;
        case 2: tb = GUITAR_HERO_3_TOUCH_YELLOW | GUITAR_HERO_3_TOUCH_BLUE; break;
        case 3: tb = GUITAR_HERO_3_TOUCH_BLUE; break;
        case 4: tb = GUITAR_HERO_3_TOUCH_BLUE | GUITAR_HERO_3_TOUCH_ORANGE; break;
        case 5: tb = GUITAR_HERO_3_TOUCH_ORANGE; break;
        default: tb = 0; break;
        }
        exp->touch_bar = tb;

        update_joystick(&exp->js, report, EGC_GUITAR_AXIS_STICKX);
    } else if (data->exp.type == EXP_MOTION_PLUS) {
        struct motion_plus_t *exp = &data->exp.mp;
        /* libogc's wiiuse returns the raw values, which are very inconvenient
         * for the client to use. Here we use the same axes and directions as
         * wiiuse, but use 0 as neuter value (instead of 8192) and a unit of
         * 1/256 radians per second.
         * If some clients are expecting the old values, we can add a flag to
         * return the values used by libogc's wiiuse. */
        exp->rx = -report->gyro.x * 256 / EGC_GYROSCOPE_RES;
        exp->ry = report->gyro.z * 256 / EGC_GYROSCOPE_RES;
        exp->rz = report->gyro.y * 256 / EGC_GYROSCOPE_RES;
        exp->status = 0;
        exp->ext = 0; /* We don't support other expansions if the M+ is active */
    }
}

static void wm_parse_ir_points_basic(const u8 *data, struct ir_dot_t *points)
{
    for (int i = 0; i < 2; i++, data += 5) {
        points[i * 2].rx = ((data[2] & 0x30) << 4) | data[0];
        points[i * 2].ry = ((data[2] & 0xc0) << 2) | data[1];
        points[i * 2 + 1].rx = ((data[2] & 0x03) << 8) | data[3];
        points[i * 2 + 1].ry = ((data[2] & 0x0c) << 6) | data[4];
        points[i * 2].size = points[i * 2 + 1].size = 0;
    }
}

static void wm_parse_ir_points_extended(const u8 *data, struct ir_dot_t *points)
{
    for (int i = 0; i < 4; i++, data += 3) {
        points[i].rx = ((data[2] & 0x30) << 4) | data[0];
        points[i].ry = ((data[2] & 0xc0) << 2) | data[1];
        points[i].size = data[2] & 0x0f;
    }
}

static void parse_ir_report(struct ir_t *ir, InputReport *report)
{
    if (report->ir_report_size == 10) {
        wm_parse_ir_points_basic(report->ir_report, ir->dot);
    } else {
        wm_parse_ir_points_extended(report->ir_report, ir->dot);
    }

    /* Update the "visible" field */
    for (int i = 0; i < 4; i++) {
        struct ir_dot_t *dot = &ir->dot[i];
        dot->visible = dot->rx != 0x3ff;
    }
}

static void update_ir(Controller *controller, InputReport *report)
{
    WPADData *data = &controller->data;
    struct ir_t *ir = &data->ir;

    if (report->ir_report_size > 0) {
        parse_ir_report(ir, report);
    }

    egc_point_t p;
    if (!report->has_touch || ((p = report->touch), p.x < 0 || p.y < 0)) {
        ir->num_dots = 0;
        ir->state = IR_STATE_LOST;
        ir->raw_valid = 0;
        ir->smooth_valid = 0;
        ir->valid = 0;
        return;
    }

    if (controller->is_wiimote) {
        float sb_angle = report->sb_angle;
        ir->distance = report->sb_width;
        ir->angle = sb_angle * 180 / M_PI;
#ifdef WIIUSE_HAS_SENSOR_BAR_DATA
        sb_t *sb = &ir->sensorbar;
        sb->angle = sb->off_angle = sb_angle;
        bool valid = true;
        for (int i = 0; i < 2; i++) {
            egc_point_t p = report->sb_points[i];
            if (p.x < 0) {
                valid = false;
                break;
            }

            sb->dots[i].x = (p.x - 512.0f) / 512;
            sb->dots[i].y = (p.y - 384.0f) / 384;
            float sine = sinf(sb_angle);
            float cosine = cosf(sb_angle);
            sb->rot_dots[i].x = cosine * sb->dots[i].x + sine * sb->dots[i].y;
            sb->rot_dots[i].y = -sine * sb->dots[i].x + cosine * sb->dots[i].y;
            sb->acc_dots[i] = sb->rot_dots[i];
        }

        if (!valid) {
            *sb = (sb_t){ 0 };
        }
#endif
    } else {
#ifdef WIIUSE_HAS_SENSOR_BAR_DATA
        ir->sensorbar = (sb_t){ 0 };
#endif
        ir->distance = 100;
        ir->z = 1;
        ir->angle = 0;
    }
    ir->num_dots = 2;
    ir->state = IR_STATE_GOOD;

    ir->raw_valid = 1;
    ir->ax = p.x * 1024 / (float)INT16_MAX;
    ir->ay = p.y * 768 / (float)INT16_MAX;

    ir->valid = 1;
    ir->x = p.x * ir->vres[0] / (float)INT16_MAX;
    ir->y = p.y * ir->vres[1] / (float)INT16_MAX;

    ir->smooth_valid = 1;
    /* EGC performs its own smoothing */
    ir->sx = ir->x;
    ir->sy = ir->y;
    ir->error_cnt = 0;
    ir->glitch_cnt = 0;

    data->data_present |= WPAD_DATA_IR;
}

static void update_status(Controller *controller, InputReport *report)
{
    WPADData *data = &controller->data;

    /* libogc's wiiuse reports the raw battery level, which ranges from 0 to 200. */
    data->battery_level = report->battery_level * 200 / 255;
    controller->battery_critical = report->battery_critical;
}

static void collect_state(const WPADData *data, struct state_t *state)
{
    state->buttons = data->btns_h;
    state->ir_valid = data->ir.valid;
    state->ir_x = data->ir.x;
    state->ir_y = data->ir.y;
    state->accel = data->accel;
    state->exp_type = data->exp.type;

    switch (data->exp.type) {
    case EXP_NUNCHUK:
        {
            struct state_nunchuk_t *s = &state->exp.nunchuk;
            const struct nunchuk_t *exp = &data->exp.nunchuk;
            s->buttons = exp->btns;
            s->js = exp->js.pos;
            s->accel = exp->accel;
        }
        break;
    case EXP_CLASSIC:
        {
            struct state_classic_t *s = &state->exp.classic;
            const struct classic_ctrl_t *exp = &data->exp.classic;
            s->buttons = exp->btns;
            s->ljs = exp->ljs.pos;
            s->rjs = exp->rjs.pos;
        }
        break;
    case EXP_GUITAR_HERO_3:
        {
            struct state_guitar_t *s = &state->exp.guitar;
            const struct guitar_hero_3_t *exp = &data->exp.gh3;
            s->buttons = exp->btns;
            s->js = exp->js.pos;
            s->whammy_bar = exp->wb_raw;
            s->touch_bar = exp->tb_raw;
        }
        break;
    case EXP_WII_BOARD:
        {
            struct state_balance_t *s = &state->exp.balance;
            const struct wii_board_t *exp = &data->exp.wb;
            s->corners[0] = exp->rtl;
            s->corners[1] = exp->rtr;
            s->corners[2] = exp->rbl;
            s->corners[3] = exp->rbr;
        }
        break;
    case EXP_MOTION_PLUS:
        {
            struct state_motion_plus_t *s = &state->exp.motion_plus;
            const struct motion_plus_t *exp = &data->exp.mp;
            s->xyz[0] = exp->rx;
            s->xyz[1] = exp->ry;
            s->xyz[2] = exp->rz;
        }
        break;
    }
}

static bool state_has_changed(const struct state_t *old,
                              const struct state_t *new,
                              const struct thresholds_t *t)
{
#define CHECK_SAME(a, b) \
    if (a != b) return true

#define ABS(x) (x < 0 ? -(x) : (x))
#define CHECK_THRESHOLD(a, b, threshold) \
    if (ABS(a - b) >= threshold) return true

    if (t->buttons >= 0) CHECK_SAME(old->buttons, new->buttons);
    if (t->ir >= 0) {
        CHECK_SAME(old->ir_valid, new->ir_valid);
        if (new->ir_valid) {
            CHECK_THRESHOLD(old->ir_x, new->ir_x, t->ir);
            CHECK_THRESHOLD(old->ir_y, new->ir_y, t->ir);
        }
    }
    if (t->accel >= 0) {
        CHECK_THRESHOLD(old->accel.x, new->accel.x, t->accel);
        CHECK_THRESHOLD(old->accel.y, new->accel.y, t->accel);
        CHECK_THRESHOLD(old->accel.z, new->accel.z, t->accel);
    }

    CHECK_SAME(old->exp_type, new->exp_type);

    switch (new->exp_type) {
    case EXP_NUNCHUK:
        {
            const struct state_nunchuk_t *n = &new->exp.nunchuk;
            const struct state_nunchuk_t *o = &old->exp.nunchuk;
            if (t->buttons >= 0) CHECK_SAME(o->buttons, n->buttons);
            if (t->js >= 0) {
                CHECK_THRESHOLD(o->js.x, n->js.x, t->js);
                CHECK_THRESHOLD(o->js.y, n->js.y, t->js);
            }
            if (t->accel >= 0) {
                CHECK_THRESHOLD(o->accel.x, n->accel.x, t->accel);
                CHECK_THRESHOLD(o->accel.y, n->accel.y, t->accel);
                CHECK_THRESHOLD(o->accel.z, n->accel.z, t->accel);
            }
        }
        break;
    case EXP_CLASSIC:
        {
            const struct state_classic_t *n = &new->exp.classic;
            const struct state_classic_t *o = &old->exp.classic;
            if (t->buttons >= 0) CHECK_SAME(o->buttons, n->buttons);
            if (t->js >= 0) {
                CHECK_THRESHOLD(o->ljs.x, n->ljs.x, t->js);
                CHECK_THRESHOLD(o->ljs.y, n->ljs.y, t->js);
                CHECK_THRESHOLD(o->rjs.x, n->rjs.x, t->js);
                CHECK_THRESHOLD(o->rjs.y, n->rjs.y, t->js);
            }
        }
        break;
    case EXP_GUITAR_HERO_3:
        {
            const struct state_guitar_t *n = &new->exp.guitar;
            const struct state_guitar_t *o = &old->exp.guitar;
            if (t->buttons >= 0) CHECK_SAME(o->buttons, n->buttons);
            if (t->js >= 0) {
                CHECK_THRESHOLD(o->js.x, n->js.x, t->js);
                CHECK_THRESHOLD(o->js.y, n->js.y, t->js);
                CHECK_THRESHOLD(o->whammy_bar, n->whammy_bar, t->js);
                CHECK_THRESHOLD(o->touch_bar, n->touch_bar, t->js);
            }
        }
        break;
    case EXP_WII_BOARD:
        {
            const struct state_balance_t *n = &new->exp.balance;
            const struct state_balance_t *o = &old->exp.balance;
            if (t->wb >= 0) {
                for (int i = 0; i < 4; i++)
                    CHECK_THRESHOLD(o->corners[i], n->corners[i], t->wb);
            }
        }
        break;
    case EXP_MOTION_PLUS:
        {
            const struct state_motion_plus_t *n = &new->exp.motion_plus;
            const struct state_motion_plus_t *o = &old->exp.motion_plus;
            if (t->mp >= 0) {
                for (int i = 0; i < 3; i++)
                    CHECK_THRESHOLD(o->xyz[i], n->xyz[i], t->mp);
            }
        }
        break;
    }
    return false;
#undef CHECK_SAME
#undef CHECK_THRESHOLD
#undef ABS
}

static void check_state_changed(Controller *controller) {
    WPADData *data = &controller->data;

    struct state_t new;
    collect_state(data, &new);
    if (state_has_changed(&controller->last_state, &new,
                          &controller->idle_thresholds)) {
        controller->last_state = new;
        controller->last_state_time = gettime_secs();
    } else {
        if (gettime_secs() > controller->last_state_time + s_idle_timeout) {
            egc_input_device_suspend(controller->egc_device);
        }
    }
}

/* This runs in the egc_event_thread */
static void remap_buttons(struct input_report_t *report)
{
    u32 buttons = report->buttons;

    u32 wpad_btns = 0;
#define PRESSED(x) (buttons & BIT(x))
    if (report->exp_type == EGC_WIIMOTE_EXP_CLASSIC ||
        report->exp_type == EGC_WIIMOTE_EXP_CLASSIC_PRO ||
        report->exp_type == EGC_WIIMOTE_EXP_CLASSIC_WIIU_PRO) {
        if (PRESSED(EGC_GAMEPAD_BUTTON_EAST))
            wpad_btns |= WPAD_CLASSIC_BUTTON_A;
        if (PRESSED(EGC_GAMEPAD_BUTTON_SOUTH))
            wpad_btns |= WPAD_CLASSIC_BUTTON_B;
        if (PRESSED(EGC_GAMEPAD_BUTTON_WEST))
            wpad_btns |= WPAD_CLASSIC_BUTTON_Y;
        if (PRESSED(EGC_GAMEPAD_BUTTON_NORTH))
            wpad_btns |= WPAD_CLASSIC_BUTTON_X;

        if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_UP))
            wpad_btns |= WPAD_CLASSIC_BUTTON_UP;
        if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_DOWN))
            wpad_btns |= WPAD_CLASSIC_BUTTON_DOWN;
        if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_LEFT))
            wpad_btns |= WPAD_CLASSIC_BUTTON_LEFT;
        if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_RIGHT))
            wpad_btns |= WPAD_CLASSIC_BUTTON_RIGHT;

        if (PRESSED(EGC_GAMEPAD_BUTTON_START))
            wpad_btns |= WPAD_CLASSIC_BUTTON_PLUS;
        if (PRESSED(EGC_GAMEPAD_BUTTON_GUIDE))
            wpad_btns |= WPAD_CLASSIC_BUTTON_HOME;
        if (PRESSED(EGC_GAMEPAD_BUTTON_BACK))
            wpad_btns |= WPAD_CLASSIC_BUTTON_MINUS;

        if (PRESSED(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER))
            wpad_btns |= WPAD_CLASSIC_BUTTON_FULL_L;
        if (PRESSED(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER))
            wpad_btns |= WPAD_CLASSIC_BUTTON_FULL_R;

        /* On the Wii classic controller, the analog buttons are L and R,
         * instead of ZL and ZR as in most other controllers.
         */
        s16 left_trigger = report->axes[EGC_GAMEPAD_AXIS_LEFT_TRIGGER];
        s16 right_trigger = report->axes[EGC_GAMEPAD_AXIS_RIGHT_TRIGGER];
        if (report->exp_type == EGC_WIIMOTE_EXP_CLASSIC) {
            if (left_trigger == INT16_MAX)
                wpad_btns |= WPAD_CLASSIC_BUTTON_FULL_L;
            if (right_trigger == INT16_MAX)
                wpad_btns |= WPAD_CLASSIC_BUTTON_FULL_R;
            if (PRESSED(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER))
                wpad_btns |= WPAD_CLASSIC_BUTTON_ZL;
            if (PRESSED(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER))
                wpad_btns |= WPAD_CLASSIC_BUTTON_ZR;
        } else {
            if (left_trigger == INT16_MAX)
                wpad_btns |= WPAD_CLASSIC_BUTTON_ZL;
            if (right_trigger == INT16_MAX)
                wpad_btns |= WPAD_CLASSIC_BUTTON_ZR;
            if (PRESSED(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER))
                wpad_btns |= WPAD_CLASSIC_BUTTON_FULL_L;
            if (PRESSED(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER))
                wpad_btns |= WPAD_CLASSIC_BUTTON_FULL_R;

            if (report->exp_type == EGC_WIIMOTE_EXP_CLASSIC_WIIU_PRO) {
                if (PRESSED(EGC_GAMEPAD_BUTTON_LEFT_STICK))
                    wpad_btns |= WPAD_WII_U_PRO_BUTTON_LSTICK;
                if (PRESSED(EGC_GAMEPAD_BUTTON_RIGHT_STICK))
                    wpad_btns |= WPAD_WII_U_PRO_BUTTON_RSTICK;
            }
        }
    } else if (report->exp_type == EGC_WIIMOTE_EXP_GUITAR_HERO_3) {
        if (PRESSED(EGC_GUITAR_BUTTON_FRET0))
            wpad_btns |= WPAD_GUITAR_HERO_3_BUTTON_GREEN;
        if (PRESSED(EGC_GUITAR_BUTTON_FRET1))
            wpad_btns |= WPAD_GUITAR_HERO_3_BUTTON_RED;
        if (PRESSED(EGC_GUITAR_BUTTON_FRET2))
            wpad_btns |= WPAD_GUITAR_HERO_3_BUTTON_YELLOW;
        if (PRESSED(EGC_GUITAR_BUTTON_FRET3))
            wpad_btns |= WPAD_GUITAR_HERO_3_BUTTON_BLUE;
        if (PRESSED(EGC_GUITAR_BUTTON_FRET4))
            wpad_btns |= WPAD_GUITAR_HERO_3_BUTTON_ORANGE;
        if (PRESSED(EGC_GUITAR_BUTTON_STRUM_UP))
            wpad_btns |= WPAD_GUITAR_HERO_3_BUTTON_STRUM_UP;
        if (PRESSED(EGC_GUITAR_BUTTON_STRUM_DOWN))
            wpad_btns |= WPAD_GUITAR_HERO_3_BUTTON_STRUM_DOWN;
        if (PRESSED(EGC_GUITAR_BUTTON_START))
            wpad_btns |= WPAD_GUITAR_HERO_3_BUTTON_PLUS;
        if (PRESSED(EGC_GUITAR_BUTTON_BACK))
            wpad_btns |= WPAD_GUITAR_HERO_3_BUTTON_MINUS;
    } else {
        if (PRESSED(EGC_GAMEPAD_BUTTON_EAST))
            wpad_btns |= WPAD_BUTTON_A;
        if (PRESSED(EGC_GAMEPAD_BUTTON_SOUTH))
            wpad_btns |= WPAD_BUTTON_B;
        if (PRESSED(EGC_GAMEPAD_BUTTON_WEST))
            wpad_btns |= WPAD_BUTTON_1;
        if (PRESSED(EGC_GAMEPAD_BUTTON_NORTH))
            wpad_btns |= WPAD_BUTTON_2;

        if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_UP))
            wpad_btns |= WPAD_BUTTON_UP;
        if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_DOWN))
            wpad_btns |= WPAD_BUTTON_DOWN;
        if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_LEFT))
            wpad_btns |= WPAD_BUTTON_LEFT;
        if (PRESSED(EGC_GAMEPAD_BUTTON_DPAD_RIGHT))
            wpad_btns |= WPAD_BUTTON_RIGHT;

        if (PRESSED(EGC_GAMEPAD_BUTTON_START))
            wpad_btns |= WPAD_BUTTON_PLUS;
        if (PRESSED(EGC_GAMEPAD_BUTTON_GUIDE))
            wpad_btns |= WPAD_BUTTON_HOME;
        if (PRESSED(EGC_GAMEPAD_BUTTON_BACK))
            wpad_btns |= WPAD_BUTTON_MINUS;
    }

    if (report->exp_type == EGC_WIIMOTE_EXP_NUNCHUCK) {
        if (PRESSED(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER))
            wpad_btns |= WPAD_NUNCHUK_BUTTON_C;
        s16 left_trigger = report->axes[EGC_GAMEPAD_AXIS_LEFT_TRIGGER];
        if (left_trigger == INT16_MAX)
            wpad_btns |= WPAD_NUNCHUK_BUTTON_Z;
    }
#undef PRESSED
    report->buttons = wpad_btns;
}

/* This runs in the egc_event_thread */
static void read_input_report(egc_input_device_t *device,
                              struct input_report_t *report,
                              bool is_wiimote)
{
    if (is_wiimote) {
        report->exp_type = egc_driver_wiimote_get_exp_type(device);
        egc_driver_wiimote_get_ir_data(device, report->sb_points,
                                       &report->sb_width, &report->sb_angle);
    } else if (device->desc->type == EGC_DEVICE_TYPE_GUITAR) {
        report->exp_type = EGC_WIIMOTE_EXP_GUITAR_HERO_3;
    } else if (device->desc->available_buttons & BIT(EGC_GAMEPAD_BUTTON_LEFT_STICK)) {
        report->exp_type = EGC_WIIMOTE_EXP_CLASSIC_WIIU_PRO;
    } else {
        report->exp_type = EGC_WIIMOTE_EXP_CLASSIC_PRO;
    }
    report->connection_type = device->connection;
    report->num_accelerometers = device->desc->num_accelerometers;
    if (report->num_accelerometers > 2) {
        report->num_accelerometers = 2;
    }
    report->has_gyro = device->desc->num_gyroscopes > 0;
    report->has_touch = device->desc->num_touch_points > 0;
    report->available_buttons = device->desc->available_buttons;
    report->available_axes = device->desc->available_axes;
    s16 *axes;
    egc_accelerometer_t *accel;
    egc_gyroscope_t *gyro;
    egc_point_t *points;
    egc_input_device_read_data(device,
                               EGC_READ_BUTTONS |
                               EGC_READ_AXES |
                               EGC_READ_ACCELEROMETERS |
                               EGC_READ_GYROSCOPE |
                               EGC_READ_TOUCH_POINTS,
                               &report->buttons,
                               &axes,
                               &accel,
                               &gyro,
                               &points);
    memcpy(report->axes, axes, sizeof(report->axes));
    memcpy(report->accel, accel,
           sizeof(egc_accelerometer_t) * report->num_accelerometers);
    if (report->has_gyro) report->gyro = *gyro;
    if (report->has_touch) report->touch = *points;

    remap_buttons(report);
    report->battery_level = egc_input_device_get_battery_level(device);
    report->battery_critical = egc_input_device_is_battery_critical(device);
}

static void save_ir_report(InputReport *report, const u8 *data, u8 size)
{
    memcpy(report->ir_report, data, size);
    report->ir_report_size = size;
}

/**
 * Handler for raw input reports from EGC. We use to access some data that EGC
 * doesn't have an API for, such as the IR dot positions.
 * This is called in the EGC event thread.
 */
static void read_raw_report(InputReport *report, const void *data, u16 size)
{
    const u8 *report_data = data;
    switch (report_data[0]) {
    case 0x33: /* Buttons, accelerometer and IR */
        save_ir_report(report, report_data + 6, 12);
        break;
    case 0x36: /* Buttons, IR and expansion */
        save_ir_report(report, report_data + 2, 10);
        break;
    case 0x37: /* Buttons, accelerometer, IR and expansion */
        save_ir_report(report, report_data + 6, 10);
        break;
    default:
        break;
    }
}

static void device_input_cb(egc_input_device_t *device, void *unused)
{
    Controller *controller = controller_from_egc(device);
    if (!controller) return;

    KMutexLock(&s_report_mutex);

    controller->report_newest = (controller->report_newest + 1) % REPORT_QUEUE_SIZE;
    InputReport *current = &controller->reports[controller->report_newest];
    if (controller->report_newest == controller->report_oldest) {
        controller->report_oldest =
            (controller->report_oldest + 1) % REPORT_QUEUE_SIZE;
        controller->num_dropped_reports++;
    } else if (controller->report_oldest < 0) {
        controller->report_oldest = controller->report_newest;
    }
    read_input_report(device, current, controller->is_wiimote);

    const void *raw_report;
    u16 raw_report_size;
    egc_input_device_get_raw_report(device, &raw_report, &raw_report_size);
    if (raw_report) {
        read_raw_report(current, raw_report, raw_report_size);
    }

    KMutexUnlock(&s_report_mutex);
}

static sptr egc_event_thread_func(void *userdata)
{
    u32 time_until_next_timer = UINT_MAX;
    while (!s_terminated) {
        /* Events are processed by the device_input_cb callback */
        egc_wait_events(time_until_next_timer);
        time_until_next_timer = egc_timer_process();
    }

    return 0;
}

static void audio_timer_cb(egc_timer_t *timer)
{
    Controller *controller = OBJECT_FROM_MEMBER(Controller, audio_timer, timer);
    egc_audio_device_t *device = controller->audio_device;
    AudioData *audio_data = &controller->audio_data;

    int max_stream_size = egc_audio_device_get_max_stream_size(device);
    const u8 *samples = audio_data->samples;
    if (audio_data->played_samples < audio_data->size) {
        u16 size = audio_data->size - audio_data->played_samples;
        if (size > max_stream_size) size = max_stream_size;
        int rc = egc_audio_device_stream(device, samples + audio_data->played_samples, size);
        if (rc < 0) {
            /* Error, we'll stop playing. */
            audio_data->played_samples = audio_data->size;
        } else {
            audio_data->played_samples += size;
        }
    } else {
        audio_data->samples = NULL;
        egc_timer_stop(timer);
    }
}

static void worker_play_audio(Controller *controller)
{
    egc_audio_device_t *device = controller->audio_device;

    const egc_audio_spec_t *spec = egc_audio_device_get_spec(device);
    int max_stream_size = egc_audio_device_get_max_stream_size(device);

    int bytes_per_seconds;
    if (spec->format == EGC_AUDIO_FORMAT_ADPCM_YAMAHA) {
        /* divide by 2 since a sample in this format is only half a byte */
        bytes_per_seconds = spec->frequency / 2;
    } else { /* PCM S8 */
        bytes_per_seconds = spec->frequency;
    }
    u64 sleep_us = 1000000 * max_stream_size / bytes_per_seconds;
    WPAD_DEBUG("Time sleep_us: %lldus", sleep_us);

    controller->audio_data.played_samples = 0;
    egc_timer_start(&controller->audio_timer, sleep_us, audio_timer_cb);
    /* Start playing the sound immediately */
    audio_timer_cb(&controller->audio_timer);
}

static void worker_setup_audio(Controller *controller)
{
    bool enabled = controller->audio_format != 0;
    if (enabled) {
        egc_audio_format_e format =
            (controller->audio_format & WPAD_SPEAKER_FMT_MASK) == WPAD_SPEAKER_FMT_PCM ?
            EGC_AUDIO_FORMAT_S8 : EGC_AUDIO_FORMAT_ADPCM_YAMAHA;
        u16 frequency = controller->audio_format & WPAD_SPEAKER_FREQ_MASK;
        egc_audio_options_t options = {
            .spec = { format, 1, frequency },
        };
        controller->audio_device =
            egc_input_device_open_audio(controller->egc_device, &options);
    } else if (controller->audio_device) {
        egc_timer_stop(&controller->audio_timer);
        memset(&controller->audio_data, 0, sizeof(controller->audio_data));
        egc_audio_device_close(controller->audio_device);
        controller->audio_device = NULL;
    }
}

static const u16 yamaha_indexscale[] = {
    230, 230, 230, 230, 307, 409, 512, 614,
    230, 230, 230, 230, 307, 409, 512, 614
};

static const s8 yamaha_difflookup[] = {
    1, 3, 5, 7, 9, 11, 13, 15,
    -1, -3, -5, -7, -9, -11, -13, -15
};

static inline int clip_int(int x, int a, int b)
{
    if (x < a) return a;
    else if (x > b) return b;
    else return x;
}

static inline int min_int(int a, int b)
{
    return a < b ? a : b;
}

typedef struct {
    s32 predictor;
    s32 step;
} YamahaEncoderStatus;

u8 encode_sample(YamahaEncoderStatus *info, s16 sample)
{
    if (!info->step) {
        info->predictor = 0;
        info->step = 127;
    }

    int delta = sample - info->predictor;
    u8 nibble = delta >= 0 ?
        min_int(7, delta * 4 / info->step) :
        min_int(7, -delta * 4 / info->step) + 8;

    info->predictor += info->step * yamaha_difflookup[nibble] / 8;
    info->predictor = clip_int(info->predictor, INT16_MIN, INT16_MAX);
    info->step = info->step * yamaha_indexscale[nibble] >> 8;
    info->step = clip_int(info->step, 127, 24576);

    return nibble;
}

static void default_host_sync_button_cb(u32 held)
{
    if (held) {
        WPAD_WipeSavedControllers();
    } else {
        WPAD_StartPairing();
    }
}

static bool bt_vendor_event_cb(BteHci *hci, BteBuffer *event_data, void *)
{
    if (event_data->size < 3)
        return false;

    const uint8_t *data = event_data->data + 2;

    /* Known event codes:
     * - 0x08: Short press of the sync button
     * - 0x09: Long press of the sync button
     */
    if (data[0] == 0x08 || data[0] == 0x09) {
        s_host_sync_button_cb(data[0] == 0x09);
        return true;
    }

    return false;
}

static void install_vendor_event_handler(BteHci *hci)
{
    bte_hci_on_vendor_event(hci, bt_vendor_event_cb);
}

static void worker_message_cb(egc_message_t message)
{
    struct worker_command_t c = worker_command(message);

    WPAD_DEBUG("opcode %d, index %d", c.opcode, c.controller_index);
    Controller *controller = c.controller_index >= 0 ?
        &s_controllers[c.controller_index] : NULL;
    /* The device was disconnected, ignore the command */
    if (controller && !controller->egc_device) return;

    switch (c.opcode) {
    case WORKER_CMD_PLAY_AUDIO:
        worker_play_audio(controller);
        break;
    case WORKER_CMD_CONTROL_SPEAKER:
        worker_setup_audio(controller);
        break;
    case WORKER_CMD_SET_DATA_FORMAT:
        controller_update_data_format(controller);
        break;
    case WORKER_CMD_SET_MOTION_PLUS:
        controller_update_motion_plus(controller);
        break;
    case WORKER_CMD_SET_RUMBLE:
        u16 frequency = c.rumble_enabled ? UINT16_MAX : 0;
        egc_input_device_set_rumble(controller->egc_device, frequency, frequency);
        break;
    case WORKER_CMD_DISCONNECT:
        egc_input_device_suspend(controller->egc_device);
        break;
    case WORKER_CMD_SEARCH_START:
    case WORKER_CMD_SEARCH_STOP:
    case WORKER_CMD_PAIRING_START:
    case WORKER_CMD_PAIRING_STOP:
        {
            bool was_scanning = s_pairing_mode != PAIRING_MODE_NONE;
            if (c.opcode == WORKER_CMD_SEARCH_START) {
                s_pairing_mode |= PAIRING_MODE_GUEST;
            } else if (c.opcode == WORKER_CMD_SEARCH_STOP) {
                s_pairing_mode &= ~PAIRING_MODE_GUEST;
            } else if (c.opcode == WORKER_CMD_PAIRING_START) {
                s_pairing_mode |= PAIRING_MODE_NORMAL;
            } else { /* WORKER_CMD_PAIRING_STOP */
                s_pairing_mode &= ~PAIRING_MODE_NORMAL;
            }

            bool is_scanning = s_pairing_mode != PAIRING_MODE_NONE;
            if (!was_scanning && is_scanning) {
                egc_bt_start_scan();
            } else if (was_scanning && !is_scanning) {
                egc_bt_stop_scan();
            }
        }
        break;
    case WORKER_CMD_WIPE_SAVED:
        memset(&s_stored_keys, 0, sizeof(s_stored_keys));
        memset(&s_conf_devices, 0, sizeof(s_conf_devices));
        memset(&s_conf_guest_devices, 0, sizeof(s_conf_guest_devices));
        egc_bt_delete_link_key(NULL);
        break;
    }
}

static void debug_print_registered_devices()
{
#ifdef WITH_DEBUG
    WPAD_DEBUG("Registered devices:");
    for (int i = 0; i < s_conf_devices.num_registered; i++) {
        conf_pad_device *reg = &s_conf_devices.registered[i];
        WPAD_DEBUG("- " EGC_BT_ADDRESS_FMT ": name '%s'",
                   EGC_BT_ADDRESS_DATA_REVERSED((egc_bt_address_t *)reg->bdaddr), reg->name);
    }

    WPAD_DEBUG("Guest devices:");
    for (int i = 0; i < s_conf_guest_devices.num_guests; i++) {
        conf_pad_guest_device *reg = &s_conf_guest_devices.guests[i];
        WPAD_DEBUG("- " EGC_BT_ADDRESS_FMT ": name '%s'",
                   EGC_BT_ADDRESS_DATA_REVERSED((egc_bt_address_t *)reg->bdaddr), reg->name);
    }

    WPAD_DEBUG("Active devices:");
    for (int i = 0; i < CONF_PAD_MAX_ACTIVE; i++) {
        conf_pad_device *reg = &s_conf_devices.active[i];
        WPAD_DEBUG("- " EGC_BT_ADDRESS_FMT ": name '%s'",
                   EGC_BT_ADDRESS_DATA_REVERSED((egc_bt_address_t *)reg->bdaddr), reg->name);
    }
#endif
}

s32 WPAD_Init(void)
{
    bool sensor_bar_below = CONF_GetSensorBarPosition() == 0;

    for (int i = 0; i < WPAD_MAX_DEVICES; i++) {
        Controller *controller = controller_by_channel(i);
        controller_init(controller);
        controller->data_format = WPAD_FMT_BTNS;
        controller->mp_requested = false;

        struct thresholds_t *t = &controller->idle_thresholds;
        t->buttons = WPAD_THRESH_DEFAULT_BUTTONS;
        t->ir = WPAD_THRESH_DEFAULT_IR;
        t->accel = WPAD_THRESH_DEFAULT_ACCEL;
        t->js = WPAD_THRESH_DEFAULT_JOYSTICK;
        t->wb = WPAD_THRESH_DEFAULT_BALANCEBOARD;
        t->mp = WPAD_THRESH_DEFAULT_MOTION_PLUS;
    }

    if (CONF_GetPadDevices(&s_conf_devices) < 0 ||
        s_conf_devices.num_registered > CONF_PAD_MAX_REGISTERED) {
        goto err_badconf;
    }

    if (CONF_GetPadGuestDevices(&s_conf_guest_devices) < 0 ||
        s_conf_guest_devices.num_guests > CONF_PAD_MAX_ACTIVE) {
        goto err_badconf;
    }

    debug_print_registered_devices();

    /* Clear the list of guest devices in case of a program crash. The list is
     * being properly restored in WPAD_Shutdown() */
    if (CONF_SetPadGuestDevices(NULL) < 0)
        goto err_badconf;

    egc_initialize(device_added_cb, device_removed_cb, NULL);
    egc_message_set_handler(worker_message_cb);
    egc_input_device_enable_gyroscope_default(false);
    egc_bt_set_connection_filter(connection_filter_cb);
    egc_bt_enable_link_keys_storage(s_stored_keys, sizeof(s_stored_keys) / sizeof(s_stored_keys[0]));
    egc_bt_on_link_key_requested(link_key_requested_cb);
    egc_bt_on_pin_requested(pin_requested_cb);
    egc_bt_on_link_key_received(link_key_received_cb);
    egc_bt_enter_page_mode();
    egc_driver_wiimote_set_bar_position(sensor_bar_below ?
                                        EGC_DRIVER_WIIMOTE_BAR_POS_BOTTOM :
                                        EGC_DRIVER_WIIMOTE_BAR_POS_TOP);
    egc_register_input_handler(device_input_cb);

    _egc_bt_on_initialized(install_vendor_event_handler);

    s_egc_event_thread_stack = malloc(EGC_EVENT_THREAD_STACK_SIZE);
#ifdef WITH_DEBUG
    memset(s_egc_event_thread_stack, STACK_SENTINEL, EGC_EVENT_THREAD_STACK_SIZE);
#endif
    KThreadPrepare(&s_egc_event_thread, egc_event_thread_func, NULL,
                   s_egc_event_thread_stack + EGC_EVENT_THREAD_STACK_SIZE,
                   /* Higher than main thread */
                   (KTHR_MAIN_PRIO + KTHR_MAX_PRIO) / 2);
    KThreadResume(&s_egc_event_thread);

    s_initialized = true;
    return WPAD_ERR_NONE;

err_badconf:
    WPAD_Shutdown();
    return WPAD_ERR_BADCONF;
}

s32 WPAD_ReadEvent(s32 chan, WPADData *data)
{
    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;

    if (s_pairing_mode & PAIRING_MODE_NORMAL) {
        /* Stop pairing after 20 seconds */
        u32 time_now = gettime_secs();
        if (time_now > s_pairing_start_time + 20) {
            worker_post_command(WORKER_CMD_PAIRING_STOP, NULL);
        }
    }

    if (!controller->egc_device) return WPAD_ERR_NO_CONTROLLER;

    InputReport report;
    bool is_valid = false;

    KMutexLock(&s_report_mutex);
    if (controller->report_oldest >= 0) {
        InputReport *src = &controller->reports[controller->report_oldest];
        memcpy(&report, src, sizeof(report));
        memset(src, 0, sizeof(*src));
        is_valid = true;
        if (controller->report_oldest == controller->report_newest) {
            /* We have read all the queued reports */
            controller->report_oldest = -1;
        } else {
            controller->report_oldest =
                (controller->report_oldest + 1) % REPORT_QUEUE_SIZE;
        }
    }
    KMutexUnlock(&s_report_mutex);

    if (!is_valid) return WPAD_ERR_QUEUE_EMPTY;

    controller->data.data_present = 0;
    update_expansion_type(controller, &report);
    update_buttons(controller, &report);
    update_accelerometer(controller, &report);
    update_expansion_data(controller, &report);
    update_expansion_buttons(&controller->data);
    update_ir(controller, &report);
    update_status(controller, &report);
    check_state_changed(controller);
    controller->buttons_last = report.buttons;
    if (data != &controller->data) {
        memcpy(data, &controller->data, sizeof(*data));
    }
    return 0;
}

s32 WPAD_DroppedEvents(s32 chan)
{
    s32 ret, count = 0;
    if (chan == WPAD_CHAN_ALL) {
        for (int i = WPAD_CHAN_0; i < WPAD_MAX_DEVICES; i++) {
            if ((ret = WPAD_DroppedEvents(i)) >= WPAD_ERR_NONE) {
                count += ret;
            } else {
                return ret;
            }
        }
        return count;
    }

    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;

    count = controller->num_dropped_reports;
    controller->num_dropped_reports = 0;
    return count;
}

s32 WPAD_Flush(s32 chan)
{
    return 0;
}

s32 WPAD_ReadPending(s32 chan, WPADDataCallback datacb)
{
    s32 ret;
    int count = 0;
    if (chan == WPAD_CHAN_ALL) {
        for (int i = WPAD_CHAN_0; i < WPAD_MAX_DEVICES; i++) {
            if ((ret = WPAD_ReadPending(i, datacb)) >= WPAD_ERR_NONE) {
                count += ret;
            }
        }
        return count;
    }

    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;

    WPADData *data = &controller->data;
    u32 btns_prev, btns_down_coalesced, btns_last;
    u32 btns_changed_coalesced = 0;
    btns_prev = btns_down_coalesced = btns_last = controller->buttons_last;
    while (ret = WPAD_ReadEvent(chan, data), ret >= WPAD_ERR_NONE) {
        if (datacb) {
            datacb(chan, data);
        }

        /* controller->buttons_last now holds the latest button state */
        u32 btns_down_now = controller->buttons_last;

        /* buttons that changed that haven't changed before */
        u32 btns_changed = (btns_down_now ^ btns_prev) & ~btns_changed_coalesced;
        btns_prev = btns_down_now;

        /* Update the coalesced state of down buttons: clear all the buttons
         * which have changed in this round and set them to their latest
         * status. */
        btns_down_coalesced = (btns_down_coalesced & ~btns_changed) |
            (btns_down_now & btns_changed);

        /* Remember the buttons changed, not to change them again */
        btns_changed_coalesced |= btns_changed;
        count++;
    }

    data->btns_h = btns_down_coalesced;
    data->btns_l = btns_last;
    data->btns_d = btns_down_coalesced & ~btns_last;
    data->btns_u = ~btns_down_coalesced & btns_last;
    update_expansion_buttons(data);

    return ret == WPAD_ERR_QUEUE_EMPTY ? count : ret;
}

s32 WPAD_SetDataFormat(s32 chan, s32 fmt)
{
    int ret;
    if (chan == WPAD_CHAN_ALL) {
        for (int i = WPAD_CHAN_0; i < WPAD_MAX_DEVICES; i++) {
            if ((ret = WPAD_SetDataFormat(i, fmt)) < WPAD_ERR_NONE) {
                return ret;
            }
        }
        return WPAD_ERR_NONE;
    }

    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;

    controller->data_format = fmt;
    if (controller->egc_device) {
        worker_post_command(WORKER_CMD_SET_DATA_FORMAT, controller);
    }

    return WPAD_ERR_NONE;
}

s32 WPAD_SetMotionPlus(s32 chan, u8 enable)
{
    int ret;
    if (chan == WPAD_CHAN_ALL) {
        for (int i = WPAD_CHAN_0; i < WPAD_MAX_DEVICES; i++) {
            if ((ret = WPAD_SetMotionPlus(i, enable)) < WPAD_ERR_NONE) {
                return ret;
            }
        }
        return WPAD_ERR_NONE;
    }

    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;

    controller->mp_requested = enable;
    if (controller->egc_device) {
        worker_post_command(WORKER_CMD_SET_MOTION_PLUS, controller);
    }

    return WPAD_ERR_NONE;
}

s32 WPAD_SetVRes(s32 chan, u32 xres, u32 yres)
{
    int ret;
    if (chan == WPAD_CHAN_ALL) {
        for (int i = WPAD_CHAN_0; i < WPAD_MAX_DEVICES; i++) {
            if ((ret = WPAD_SetVRes(i, xres, yres)) < WPAD_ERR_NONE) {
                return ret;
            }
        }
        return WPAD_ERR_NONE;
    }

    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;

    controller->data.ir.vres[0] = xres - 1;
    controller->data.ir.vres[1] = yres - 1;

    return WPAD_ERR_NONE;
}

s32 WPAD_GetStatus(void)
{
    /* 1 means initialized */
    return 1;
}

s32 WPAD_Probe(s32 chan, u32 *type)
{
    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;
    if (!controller->egc_device) return WPAD_ERR_NO_CONTROLLER;

    if (type) {
        *type = controller->data.exp.type;
    }
    return WPAD_ERR_NONE;
}

s32 WPAD_SetEventBufs(s32 chan, WPADData *bufs, u32 cnt)
{
    if (chan < WPAD_CHAN_0 || chan >= WPAD_MAX_DEVICES) return WPAD_ERR_BAD_CHANNEL;

    return WPAD_ERR_NONE;
}

s32 WPAD_Disconnect(s32 chan)
{
    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;
    if (!controller->egc_device) return WPAD_ERR_NO_CONTROLLER;

    worker_post_command(WORKER_CMD_DISCONNECT, controller);
    return WPAD_ERR_NONE;
}

s32 WPAD_ControlSpeaker(s32 chan, s32 enable)
{
    s32 ret;

    if (chan == WPAD_CHAN_ALL) {
        for (int i = WPAD_CHAN_0; i < WPAD_MAX_DEVICES; i++)
            if ((ret = WPAD_ControlSpeaker(i, enable)) < WPAD_ERR_NONE)
                return ret;
        return WPAD_ERR_NONE;
    }

    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;
    if (!controller->egc_device) return WPAD_ERR_NO_CONTROLLER;

    s32 audio_format = enable != 1 ? enable : WPAD_SPEAKER_FMT_ADPCM_6000;
    if (audio_format == controller->audio_format) return WPAD_ERR_NONE;

    controller->audio_format = audio_format;
    worker_post_command(WORKER_CMD_CONTROL_SPEAKER, controller);
    return WPAD_ERR_NONE;
}

s32 WPAD_IsSpeakerEnabled(s32 chan)
{
    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;
    if (!controller->egc_device) return WPAD_ERR_NO_CONTROLLER;

    return controller->speaker_requested ? WPAD_ERR_NONE : WPAD_ERR_NOT_READY;
}

/* Note: this function takes PCM signed 16-bit samples *in big endian format* */
void WPAD_EncodeData(WPADEncStatus *info, u32 flag, const s16 *samples, s32 num_samples, u8 *encData)
{
    YamahaEncoderStatus *status = (YamahaEncoderStatus *)info;

    if (!(flag & WPAD_ENC_CONT)) status->step = 0;

    for (int i = 0; i < num_samples; i += 2) {
        u8 nibble;
        nibble = encode_sample(status, samples[0]) << 4;
        nibble |= encode_sample(status, samples[1]);
        *encData++ = nibble;
        samples += 2;
    }
}

s32 WPAD_SendStreamData(s32 chan, void *buf, u32 len)
{
    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;
    if (!controller->egc_device) return WPAD_ERR_NO_CONTROLLER;
    if (!controller->audio_device) return WPAD_ERR_BADVALUE;

    if (controller->audio_data.samples) {
        return WPAD_ERR_BADVALUE;
    }

    controller->audio_data.samples = buf;
    controller->audio_data.size = len;

    worker_post_command(WORKER_CMD_PLAY_AUDIO, controller);

    return WPAD_ERR_NONE;
}

s32 WPAD_Search(void)
{
    /* The WPAD library in libogc here clears also the guest controllers,
     * which does not seem a good idea. */
    memset(&s_conf_devices.active, 0, sizeof(s_conf_devices.active));

    worker_post_command(WORKER_CMD_SEARCH_START, NULL);
    return WPAD_ERR_NONE;
}

s32 WPAD_StopSearch(void)
{
    worker_post_command(WORKER_CMD_SEARCH_STOP, NULL);
    return WPAD_ERR_NONE;
}

s32 WPAD_StartPairing(void)
{
    worker_post_command(WORKER_CMD_PAIRING_START, NULL);
    s_pairing_start_time = gettime_secs();
    return WPAD_ERR_NONE;
}

s32 WPAD_WipeSavedControllers(void)
{
    worker_post_command(WORKER_CMD_WIPE_SAVED, NULL);
    return WPAD_ERR_NONE;
}

/* Not a public symbol, but called by libogc/system.c when the Wii gets
 * switched off. */
u32 __WPADClearActiveList(void)
{
    memset(&s_conf_devices.active, 0, sizeof(s_conf_devices.active));
    memset(&s_conf_guest_devices, 0, sizeof(s_conf_guest_devices));
    u32 ret;
    u32 level = IRQ_Disable();
    if (CONF_SetPadGuestDevices(&s_conf_guest_devices) >= 0 &&
        CONF_SetPadDevices(&s_conf_devices) >= 0) {
        ret = CONF_SaveChanges();
    } else {
        ret = 1;
    }
    IRQ_Restore(level);
    return ret;
}

s32 WPAD_Shutdown(void)
{
    if (!s_initialized) return WPAD_ERR_NONE;

    s_terminated = true;

    WPAD_DEBUG("Saving CONF");
    debug_print_registered_devices();
    CONF_SetPadGuestDevices(&s_conf_guest_devices);
    CONF_SetPadDevices(&s_conf_devices);
    CONF_SaveChanges();

    KThreadJoin(&s_egc_event_thread);
    if (s_egc_event_thread_stack) {
        debug_print_free_stack(s_egc_event_thread_stack, EGC_EVENT_THREAD_STACK_SIZE);
        free(s_egc_event_thread_stack);
    }
    return WPAD_ERR_NONE;
}

[[deprecated]]
void WPAD_SetPowerButtonCallback(WPADShutdownCallback cb)
{
}

[[deprecated]]
void WPAD_SetBatteryDeadCallback(WPADShutdownCallback cb)
{
}

void WPAD_SetDisconnectCallback(WPADDisconnectCallback cb)
{
    s_disconnect_cb = cb;
}

void WPAD_SetHostSyncButtonCallback(WPADHostSyncBtnCallback cb)
{
    s_host_sync_button_cb = cb ? cb : default_host_sync_button_cb;
}

s32 WPAD_ScanPads(void)
{
    return WPAD_ReadPending(WPAD_CHAN_ALL, NULL);
}

s32 WPAD_Rumble(s32 chan, int status)
{
    s32 ret;

    if (chan == WPAD_CHAN_ALL) {
        for (int i = WPAD_CHAN_0; i < WPAD_MAX_DEVICES; i++)
            if ((ret = WPAD_Rumble(i, status)) < WPAD_ERR_NONE)
                return ret;
        return WPAD_ERR_NONE;
    }

    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;

    union worker_command_u u = { 0 };
    u.cmd.opcode = WORKER_CMD_SET_RUMBLE;
    u.cmd.controller_index = chan;
    u.cmd.rumble_enabled = status != 0;
    egc_message_post(u.msg);
    return WPAD_ERR_NONE;
}

void WPAD_SetIdleTimeout(u32 seconds)
{
    s_idle_timeout = seconds;
}

s32 WPAD_SetIdleThresholds(s32 chan, s32 btns, s32 ir, s32 accel, s32 js, s32 wb, s32 mp)
{
    s32 ret;

    if (chan == WPAD_CHAN_ALL) {
        for (int i = WPAD_CHAN_0; i < WPAD_MAX_DEVICES; i++)
            if ((ret = WPAD_SetIdleThresholds(i, btns, ir, accel, js, wb, mp)) < WPAD_ERR_NONE)
                return ret;
        return WPAD_ERR_NONE;
    }

    Controller *controller = controller_by_channel(chan);
    if (!controller) return WPAD_ERR_BAD_CHANNEL;

    struct thresholds_t *t = &controller->idle_thresholds;
    t->buttons = btns < 0 ? -1 : 0;
    t->ir = ir;
    t->accel = accel;
    t->js = js;
    t->wb = wb;
    t->mp = mp;
    return WPAD_ERR_NONE;
}

WPADData *WPAD_Data(int chan)
{
    if (chan < 0 || chan >= WPAD_MAX_DEVICES) return NULL;
    return &s_controllers[chan].data;
}

u8 WPAD_BatteryLevel(int chan)
{
    if (chan < 0 || chan >= WPAD_MAX_DEVICES) return 0;
    return s_controllers[chan].data.battery_level;
}

u32 WPAD_ButtonsUp(int chan)
{
    if (chan < 0 || chan >= WPAD_MAX_DEVICES) return 0;
    return s_controllers[chan].data.btns_u;
}

u32 WPAD_ButtonsDown(int chan)
{
    if (chan < 0 || chan >= WPAD_MAX_DEVICES) return 0;
    return s_controllers[chan].data.btns_d;
}

u32 WPAD_ButtonsHeld(int chan)
{
    if (chan < 0 || chan >= WPAD_MAX_DEVICES) return 0;
    return s_controllers[chan].data.btns_h;
}

void WPAD_IR(int chan, struct ir_t *ir)
{
    if (chan < 0 || chan >= WPAD_MAX_DEVICES || !ir) return;
    *ir = s_controllers[chan].data.ir;
}

void WPAD_Orientation(int chan, struct orient_t *orient)
{
    if (chan < 0 || chan >= WPAD_MAX_DEVICES || !orient) return;
    *orient = s_controllers[chan].data.orient;
}

void WPAD_GForce(int chan, struct gforce_t *gforce)
{
    if (chan < 0 || chan >= WPAD_MAX_DEVICES || !gforce) return;
    *gforce = s_controllers[chan].data.gforce;
}

void WPAD_Accel(int chan, struct vec3w_t *accel)
{
    if (chan < 0 || chan >= WPAD_MAX_DEVICES || !accel) return;
    *accel = s_controllers[chan].data.accel;
}

void WPAD_Expansion(int chan, struct expansion_t *exp)
{
    if (chan < 0 || chan >= WPAD_MAX_DEVICES || !exp) return;
    *exp = s_controllers[chan].data.exp;
}

bool WPAD_IsBatteryCritical(int chan)
{
    Controller *controller = controller_by_channel(chan);
    if (!controller) return false;

    return controller->battery_critical;
}

#ifdef WIIUSE_HAS_PAD_STATUS_API
void WPAD_SetStatusCallback(WPADStatusCallback cb)
{
    s_status_cb = cb;
}

void WPAD_PadStatus(int chan)
{
    /* Our status is always updated, we can invoke the callback right away */
    s_status_cb(chan);
}
#endif
