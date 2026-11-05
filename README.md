# wiiuse-bte

A reimplementation of libogc's WPAD API, as a drop-in replacement of the wiiuse
and bte libraries.

## Description

This project produces the libbte.a and libwiiuse.a static libraries, named
after their libogc's counterparts, in order to make it possible for application
developers to switch between the two implementations without having to change
their application's code.

At the moment only the `WPAD_*` APIs are implemented, and there are no plans to
implement `wiiuse_*` and `BTE_*` functions unless they are necessary to some
application.

## Changes from libogc's wiiuse

### Improvement: working speaker

The wiimote speaker works fine, also on old wiimotes (RVL-CNT-01). All the
input functionality runs into its own separate thread with a higher priority
than the main thread, so sound is quite stable.


### Improvement: WPAD_Disconnect() works

WPAD_Disconnect() doesn't seem to be working in libogc's wiiuse. This is fixed
in wiiuse-bte.


### Improvement: support for USB and bluetooth controllers

Since this library uses
[embedded-game-controller](https://github.com/embedded-game-controller/embedded-game-controller)
for talking to the wiimote, it automatically gets support for other controllers
supported by EGC for free. The only controller that has been explicitly
disabled is the GameCube one, because Wii homebrew applications typically use
the PAD library for it; however, if desired, it's possible to enable support
for GameCube controllers by changing a couple of lines in the CMakeLists.txt
file.

A couple of notes about non-Wii controllers:

1. You need to be in search mode (call WPAD_Search() to enable it) in order to
   discover them.
2. They are currently reported as a Wiimote with a classic controller pro
   expansion (if the controller's sticks can act as buttons, then as a Wii U
   Pro), and **they don't report any button on the Wiimote**.


### API: speaker formats

A new enumeration has been added, which can be used as an argument for
WPAD_ControlSpeaker():

| Enum name                   | Requested format |
|-----------------------------|------------------|
| WPAD_SPEAKER_FMT_PCM_1500   | PCM, 1500 Hz     |
| WPAD_SPEAKER_FMT_PCM_2000   | PCM, 2000 Hz     |
| WPAD_SPEAKER_FMT_ADPCM_3000 | ADPCM, 3000 Hz   |
| WPAD_SPEAKER_FMT_ADPCM_6000 | ADPCM, 6000 Hz   |

If the `enable` parameter of WPAD_ControlSpeaker() is simply set to `1`, then
ADPCM at 6000 Hz is selected.

Note: newer wiimotes (RVL-CNT-01-TR) seem to only support the ADPCM format.

### Motion Plus report is different

libogc's wiiuse reports the gyroscope values in a strange format: the neutral
value seems to be 8192, and it's unclear what the unit of measure is.
wiiuse-bte instead uses 0 as the neutral value, and the unit of measure for the
rotation speed is in 1/256 radians per second; that is, a value of 256 means
that the controller is rotating at the speed of one radian per second.


## Compatibility features

The WPAD public structures expose a lot of information, some of which might not
be used at all. Those fields and functions whose utility is dubious have been
wrapped with preprocessor macros to exclude them by default; for example:

```c
typedef struct nunchuk_t {
#ifdef WIIUSE_HAS_NUNCHUK_ACCEL_CALIB
    struct accel_t accel_calib;     /**< nunchuk accelerometer calibration      */
#endif
    struct joystick_t js;           /**< joystick calibration                   */

#ifdef WIIUSE_HAS_NUNCHUK_FLAGS
    int *flags;                     /**< options flag (points to wiimote_t.flags) */
#endif
    ...
```

If an application written for libogc's wiiuse breaks when build against
wiiuse-bte, it's likely that the reason is these missing fields. If so, please
file a bug, so that we can restore the disabled functionality.
