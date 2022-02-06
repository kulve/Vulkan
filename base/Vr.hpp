/*
* OpenHMD functions
*
* Copyright(C) by Tuomas Kulve <tuomas@kulve.fi>
*
* This code is licensed under the MIT license(MIT) (http://opensource.org/licenses/MIT)
*/
/*
 * Partly copied from:
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2013 Fredrik Hultin.
 * Copyright (C) 2013 Jakob Bornecrantz.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */
#pragma once

#include <openhmd.h>
#include <stdint.h>

namespace vr
{
    class Vr
    {
      public:
        Vr();
        ~Vr();
        bool initVr();
        void update();
        const char* getNameVendor();
        const char* getNameProduct();
        uint32_t getWidth();
        uint32_t getHeight();
        float getEyeSeparation();
        float getFov();
        float getRoll();
        float getPitch();
        float getYaw();
      private:
        ohmd_context *ohmd_ctx;
        ohmd_device* hmd;
        float sep;
        float fov;
        uint32_t hmd_w;
        uint32_t hmd_h;
        float roll;
        float pitch;
        float yaw;
        const char* name_product;
        const char* name_vendor;
    };
}// namespace vr
