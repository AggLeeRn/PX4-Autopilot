#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/airspeed.h>

using namespace time_literals;

class Core : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static ModuleBase::Descriptor desc;

	Core() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
	~Core() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return 0; }
	static int print_usage(const char *reason = nullptr)
	{
		PX4_INFO("usage: core {start|stop|status}");
		return 0;
	}

	bool init()
	{
		ScheduleOnInterval(1_s);
		PX4_INFO("Core started");
		return true;
	}

private:
	void Run() override;

	static float mag_heading(float x, float y)
	{
		float h = atan2f(-x, y) * 180.0f / M_PI_F;
		if (h < 0) h += 360.0f;
		return h;
	}

	static const char *direction_str(float deg)
	{
		if      (deg <  22.5f || deg >= 337.5f) return "Sever";
		else if (deg <  67.5f)                  return "Severovychod";
		else if (deg < 112.5f)                  return "Vychod";
		else if (deg < 157.5f)                  return "Jihovychod";
		else if (deg < 202.5f)                  return "Jih";
		else if (deg < 247.5f)                  return "Jihozapad";
		else if (deg < 292.5f)                  return "Zapad";
		else                                     return "Severozapad";
	}

	static float heading_diff(float a, float b)
	{
		float d = fabsf(a - b);
		if (d > 180.0f) d = 360.0f - d;
		return d;
	}

	uORB::Subscription _gps_sub{ORB_ID(sensor_gps), 0};
	uORB::Subscription _gps2_sub{ORB_ID(sensor_gps), 1};
	uORB::Subscription _mag_sub{ORB_ID(sensor_mag), 0};
	uORB::Subscription _mag1_sub{ORB_ID(sensor_mag), 1};
	uORB::Subscription _mag2_sub{ORB_ID(sensor_mag), 2};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};

	float       _last_speed{0.0f};
	float       _last_heading{-1.0f};
	int         _last_fix_type{-1};
	hrt_abstime _last_status_us{0};

	static constexpr float       SPEED_THRESHOLD   = 1.0f;
	static constexpr float       HEADING_THRESHOLD = 10.0f;
	static constexpr hrt_abstime STATUS_INTERVAL   = 5_s;
};

ModuleBase::Descriptor Core::desc{task_spawn, custom_command, print_usage};

void Core::Run()
{
	hrt_abstime now = hrt_absolute_time();
	bool do_status = (now - _last_status_us) >= STATUS_INTERVAL;

	// --- GPS ---
	sensor_gps_s gps{};
	bool has_gps = _gps_sub.update(&gps) || _gps2_sub.update(&gps);

	if (has_gps) {
		int fix = (int)gps.fix_type;
		if (_last_fix_type < 0) {
			_last_fix_type = fix;
		} else if (fix != _last_fix_type) {
			if (fix >= 3 && _last_fix_type < 3) {
				PX4_INFO("[GPS] FIX ZISKAN fix=%d sats=%d", fix, gps.satellites_used);
			} else if (fix < 3 && _last_fix_type >= 3) {
				PX4_INFO("[GPS] FIX ZTRACEN fix=%d sats=%d", fix, gps.satellites_used);
			}
			_last_fix_type = fix;
		}
	}

	// --- Lokalni poloha + rychlost ---
	vehicle_local_position_s lpos{};
	float speed_2d = 0.0f;
	bool has_lpos = _lpos_sub.update(&lpos);

	if (has_lpos) {
		speed_2d = sqrtf(lpos.vx * lpos.vx + lpos.vy * lpos.vy);
		if (fabsf(speed_2d - _last_speed) >= SPEED_THRESHOLD) {
			PX4_INFO("[ZMENA] Rychlost %.1f -> %.1f m/s",
				(double)_last_speed, (double)speed_2d);
			_last_speed = speed_2d;
		}
	}

	// --- Magnetometry + kurz ---
	sensor_mag_s mag0{}, mag1{}, mag2{};
	bool has_mag0 = _mag_sub.advertised()  && _mag_sub.update(&mag0);
	bool has_mag1 = _mag1_sub.advertised() && _mag1_sub.update(&mag1);
	bool has_mag2 = _mag2_sub.advertised() && _mag2_sub.update(&mag2);

	float avg_heading = -1.0f;
	int mag_count = (int)has_mag0 + (int)has_mag1 + (int)has_mag2;

	if (mag_count > 0) {
		float sum = 0.0f;
		if (has_mag0) sum += mag_heading(mag0.x, mag0.y);
		if (has_mag1) sum += mag_heading(mag1.x, mag1.y);
		if (has_mag2) sum += mag_heading(mag2.x, mag2.y);
		avg_heading = sum / (float)mag_count;

		if (_last_heading < 0.0f) {
			_last_heading = avg_heading;
		} else if (heading_diff(avg_heading, _last_heading) >= HEADING_THRESHOLD) {
			PX4_INFO("[ZMENA] Kurz %.1f -> %.1f deg (%s)",
				(double)_last_heading, (double)avg_heading, direction_str(avg_heading));
			_last_heading = avg_heading;
		}
	}

	// --- Pitot ---
	airspeed_s airspeed{};
	bool has_airspeed = _airspeed_sub.update(&airspeed);

	// --- Globalni poloha ---
	vehicle_global_position_s gpos{};
	bool has_gpos = _gpos_sub.update(&gpos);

	// --- Periodicky status (kazdych 5s) ---
	if (do_status) {
		_last_status_us = now;
		PX4_INFO("=== STATUS ===");

		if (has_lpos) {
			PX4_INFO("[POS] x=%.1f y=%.1f z=%.1fm spd=%.1fm/s",
				(double)lpos.x, (double)lpos.y, (double)lpos.z, (double)speed_2d);
		}

		if (avg_heading >= 0.0f) {
			PX4_INFO("[KRZ] %.1f deg -> %s",
				(double)avg_heading, direction_str(avg_heading));
		}

		if (has_gps) {
			if (gps.fix_type >= 3) {
				PX4_INFO("[GPS] fix=%d sats=%d alt=%.1fm",
					gps.fix_type, gps.satellites_used, (double)gps.altitude_msl_m);
				PX4_INFO("[GPS] lat=%.6f lon=%.6f",
					(double)gps.latitude_deg, (double)gps.longitude_deg);
			} else {
				PX4_INFO("[GPS] no fix | fix=%d sats=%d",
					gps.fix_type, gps.satellites_used);
			}
		}

		if (has_gpos) {
			PX4_INFO("[GPOS] lat=%.6f lon=%.6f alt=%.1fm",
				(double)gpos.lat, (double)gpos.lon, (double)gpos.alt);
		}

		if (has_airspeed) {
			PX4_INFO("[PITOT] %.1f m/s (true %.1f m/s)",
				(double)airspeed.indicated_airspeed_m_s,
				(double)airspeed.true_airspeed_m_s);
		}

		if (has_mag0) {
			PX4_INFO("[MAG0 RM3100] %.1f deg", (double)mag_heading(mag0.x, mag0.y));
		}
		if (has_mag1) {
			PX4_INFO("[MAG1 GPS]    %.1f deg", (double)mag_heading(mag1.x, mag1.y));
		}
		if (has_mag2) {
			PX4_INFO("[MAG2 ICM]    %.1f deg", (double)mag_heading(mag2.x, mag2.y));
		}

		PX4_INFO("==============");
	}
}

int Core::task_spawn(int argc, char *argv[])
{
	Core *instance = new Core();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	PX4_ERR("alloc failed");
	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

extern "C" __EXPORT int core_main(int argc, char *argv[])
{
	return ModuleBase::main(Core::desc, argc, argv);
}
