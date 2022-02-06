/*
* OpenHMD functions
*
* Copyright(C) by Tuomas Kulve <tuomas@kulve.fi>
*
* This code is licensed under the MIT license(MIT) (http://opensource.org/licenses/MIT)
*/
/*
 * Partly copied from: https://github.com/OpenHMD/OpenHMD
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2013 Fredrik Hultin.
 * Copyright (C) 2013 Jakob Bornecrantz.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

#include <Vr.hpp>
#include <iostream>

#include <cmath>

#define DEG2RAD(a) ((a) * (180/M_PI))

namespace vr
{
    Vr::Vr()
    {
        roll = 0;
        pitch = 0;
        yaw = 0;
    }

    Vr::~Vr()
    {
        // TODO
    }


    bool Vr::initVr()
    {
        ohmd_ctx = ohmd_ctx_create();
        int num_devices = ohmd_ctx_probe(ohmd_ctx);
        if (num_devices < 0){
            std::cout << "Failed to probe devices: " << ohmd_ctx_get_error(ohmd_ctx) << std::endl;
            return false;
        }

        std::cout << "OpenHMD devices found: " << num_devices << std::endl;

        // Print device information
        for(int i = 0; i < num_devices; i++){
            int device_class = 0, device_flags = 0;
            const char* device_class_s[] = {"HMD", "Controller", "Generic Tracker", "Unknown"};

            if (i > 0) {
                std::cout << "Skipping device " << i << ": " << ohmd_list_gets(ohmd_ctx, i, OHMD_PRODUCT) << std::endl;
                continue;
            }

            ohmd_list_geti(ohmd_ctx, i, OHMD_DEVICE_CLASS, &device_class);
            ohmd_list_geti(ohmd_ctx, i, OHMD_DEVICE_FLAGS, &device_flags);

            printf("device %d\n", i);
            printf("  vendor:  %s\n", ohmd_list_gets(ohmd_ctx, i, OHMD_VENDOR));
            printf("  product: %s\n", ohmd_list_gets(ohmd_ctx, i, OHMD_PRODUCT));
            printf("  path:    %s\n", ohmd_list_gets(ohmd_ctx, i, OHMD_PATH));
            printf("  class:   %s\n", device_class_s[device_class > OHMD_DEVICE_CLASS_GENERIC_TRACKER ? 4 : device_class]);
            printf("  flags:   %02x\n",  device_flags);
            printf("    null device:         %s\n", device_flags & OHMD_DEVICE_FLAGS_NULL_DEVICE ? "yes" : "no");
            printf("    rotational tracking: %s\n", device_flags & OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING ? "yes" : "no");
            printf("    positional tracking: %s\n", device_flags & OHMD_DEVICE_FLAGS_POSITIONAL_TRACKING ? "yes" : "no");
            printf("    left controller:     %s\n", device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER ? "yes" : "no");
            printf("    right controller:    %s\n\n", device_flags & OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER ? "yes" : "no");
        }


        ohmd_device_settings* settings = ohmd_device_settings_create(ohmd_ctx);
        // If OHMD_IDS_AUTOMATIC_UPDATE is set to 0, ohmd_ctx_update() must be called at least 10 times per second.
        // It is enabled by default.
        int auto_update = 1;
        ohmd_device_settings_seti(settings, OHMD_IDS_AUTOMATIC_UPDATE, &auto_update);

        hmd = ohmd_list_open_device_s(ohmd_ctx, 0, settings);
        if(!hmd){
            std::cout << "failed to open device: " << ohmd_ctx_get_error(ohmd_ctx) << std::endl;
            return false;
        }

        int w, h;
        ohmd_device_geti(hmd, OHMD_SCREEN_HORIZONTAL_RESOLUTION, &w);
        ohmd_device_geti(hmd, OHMD_SCREEN_VERTICAL_RESOLUTION, &h);

        if (w < 0 || h < 0) {
            std::cerr << "failed to get HMD resolution" << std::endl;
            return false;
        }
        hmd_w = static_cast<uint32_t>(w);
        hmd_h = static_cast<uint32_t>(h);

        ohmd_device_getf(hmd, OHMD_LEFT_EYE_FOV, &fov);
        fov = DEG2RAD(fov);

        ohmd_device_getf(hmd, OHMD_LENS_HORIZONTAL_SEPARATION, &sep);

        name_vendor = ohmd_list_gets(ohmd_ctx, 0, OHMD_VENDOR);
        name_product = ohmd_list_gets(ohmd_ctx, 0, OHMD_PRODUCT);

        std::cout << "OpenHDM device: " << name_vendor << ": " << name_product << std::endl;
        std::cout << "  Res: " << hmd_w << "x" << hmd_h << std::endl;
        std::cout << "  Sep: " << sep << std::endl;
        std::cout << "  FOV: " << fov << std::endl;

        return true;
    }

    void Vr::update()
    {
        ohmd_ctx_update(ohmd_ctx);

        float q[4];
        ohmd_device_getf(hmd, OHMD_ROTATION_QUAT, q);

        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q[3] *q[0] + q[1] * q[2]);
        double cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
        roll = std::atan2(sinr_cosp, cosr_cosp);
        roll = DEG2RAD(roll);

        // pitch (y-axis rotation)
        double sinp = 2 * (q[3] * q[1]- q[2] * q[0]);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // flip pitch
        pitch *= -1;
        pitch = DEG2RAD(pitch);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1]);
        double cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
        yaw = std::atan2(siny_cosp, cosy_cosp);
        yaw = DEG2RAD(yaw);
    }

    const char* Vr::getNameVendor()
    {
        return name_vendor;
    }
    const char* Vr::getNameProduct()
    {
        return name_product;
    }
    uint32_t Vr::getWidth()
    {
        return hmd_w;
    }
    uint32_t Vr::getHeight()
    {
        return hmd_h;
    }
    float Vr::getEyeSeparation()
    {
        return sep;
    }
    float Vr::getFov()
    {
        return fov;
    }

    float Vr::getRoll()
    {
        return roll;
    }

    float Vr::getPitch()
    {
        return pitch;
    }

    float Vr::getYaw()
    {
        return yaw;
    }
}
