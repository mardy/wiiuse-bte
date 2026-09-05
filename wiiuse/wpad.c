// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: Copyright devkitPro
// SPDX-FileCopyrightText: Alberto Mardegan (mardy)

#include "wpad.h"

#include "embedded-game-controller/egc.h"

s32 WPAD_Init(void)
{
    egc_initialize(NULL, NULL, NULL);
    return WPAD_ERR_NONE;
}
