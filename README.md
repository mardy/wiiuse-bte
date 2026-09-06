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
