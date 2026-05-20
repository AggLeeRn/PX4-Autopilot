#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>

#include <drivers/drv_hrt.h>

extern "C" __EXPORT int fake_pin_sensor_main(int argc, char *argv[]);

int fake_pin_sensor_main(int argc, char *argv[])
{
    PX4_INFO("Fake pin sensor started");

    vehicle_acceleration_s accel{};

    orb_advert_t accel_pub =
        orb_advertise(ORB_ID(vehicle_acceleration), &accel);

    for (int i = 0; i < 20; i++) {

        accel.timestamp = hrt_absolute_time();

        /*
         * Fake data
         */
        accel.xyz[0] = (float)i;
        accel.xyz[1] = (float)(i * 2);
        accel.xyz[2] = 9.81f;

        orb_publish(
            ORB_ID(vehicle_acceleration),
            accel_pub,
            &accel
        );

        PX4_INFO(
            "published accel: %.2f %.2f %.2f",
            (double)accel.xyz[0],
            (double)accel.xyz[1],
            (double)accel.xyz[2]
        );

        px4_usleep(500000);
    }

    PX4_INFO("Fake pin sensor finished");

    return 0;
}
