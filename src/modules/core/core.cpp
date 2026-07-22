// MicoAir H743-V2 — monitoring module
// Logy jen kdyz armed. Sleduje GPS, rychlost, kurz, naklon, lidar, baterii.

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
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/vehicle_optical_flow.h>

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
		PX4_INFO("Core started (MicoAir H743-V2)");
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
		if      (deg <  22.5f || deg >= 337.5f) return "S";
		else if (deg <  67.5f)                  return "SV";
		else if (deg < 112.5f)                  return "V";
		else if (deg < 157.5f)                  return "JV";
		else if (deg < 202.5f)                  return "J";
		else if (deg < 247.5f)                  return "JZ";
		else if (deg < 292.5f)                  return "Z";
		else                                     return "SZ";
	}

	static float heading_diff(float a, float b)
	{
		float d = fabsf(a - b);
		if (d > 180.0f) d = 360.0f - d;
		return d;
	}

	static const char *flight_mode_str(uint8_t nav_state)
	{
		switch (nav_state) {
		case 0:  return "MANUAL";
		case 1:  return "ALTCTL";
		case 2:  return "POSCTL";
		case 3:  return "MISSION";
		case 4:  return "LOITER";
		case 5:  return "RTL";
		case 10: return "ACRO";
		case 14: return "OFFBOARD";
		case 15: return "STABILIZED";
		case 17: return "TAKEOFF";
		case 18: return "LAND";
		default: return "?";
		}
	}

	// Quaternion -> roll a pitch ve stupnich (NED konvence)
	static void quat_to_rp(const float q[4], float &roll_deg, float &pitch_deg)
	{
		float w = q[0], x = q[1], y = q[2], z = q[3];
		roll_deg  = atan2f(2.0f * (w * x + y * z),
				   1.0f - 2.0f * (x * x + y * y)) * (180.0f / M_PI_F);
		float sp  = 2.0f * (w * y - z * x);
		sp = (sp >  1.0f) ?  1.0f : sp;
		sp = (sp < -1.0f) ? -1.0f : sp;
		pitch_deg = asinf(sp) * (180.0f / M_PI_F);
	}

	uORB::Subscription _gps_sub{ORB_ID(sensor_gps), 0};
	uORB::Subscription _gps2_sub{ORB_ID(sensor_gps), 1};
	uORB::Subscription _mag_sub{ORB_ID(sensor_mag), 0};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _bat_sub{ORB_ID(battery_status)};
	uORB::Subscription _dist_sub{ORB_ID(distance_sensor)};
	uORB::Subscription _esc_sub{ORB_ID(esc_status)};
	uORB::Subscription _flow_sub{ORB_ID(vehicle_optical_flow)};

	float       _last_speed{0.0f};
	float       _last_heading{-1.0f};
	int         _last_fix_type{-1};
	bool        _was_armed{false};
	hrt_abstime _last_status_us{0};
	hrt_abstime _last_bat_warn_us{0};

	static constexpr float       SPEED_THRESHOLD   = 1.0f;
	static constexpr float       HEADING_THRESHOLD = 10.0f;
	static constexpr hrt_abstime STATUS_INTERVAL   = 1_s;
	static constexpr hrt_abstime BAT_WARN_INTERVAL = 30_s;
	static constexpr float       BAT_WARN_LEVEL    = 0.20f;
	static constexpr float       BAT_CRIT_LEVEL    = 0.10f;
};

ModuleBase::Descriptor Core::desc{task_spawn, custom_command, print_usage};

void Core::Run()
{
	hrt_abstime now = hrt_absolute_time();

	// --- Stav vozidla ---
	vehicle_status_s status{};
	_status_sub.update(&status);
	bool is_armed = (status.arming_state == 2); // ARMING_STATE_ARMED

	if (is_armed && !_was_armed) {
		PX4_INFO("[CORE] === ARMED === mod: %s", flight_mode_str(status.nav_state));
		_last_status_us = now;
	} else if (!is_armed && _was_armed) {
		PX4_INFO("[CORE] === DISARMED ===");
	}
	_was_armed = is_armed;

	// podminka pro armed mode
	// if (!is_armed) {
	// 	return;
	// }

	// --- GPS ---
	sensor_gps_s gps{};
	bool has_gps = _gps_sub.update(&gps) || _gps2_sub.update(&gps);

	if (has_gps) {
		int fix = (int)gps.fix_type;
		if (_last_fix_type < 0) {
			_last_fix_type = fix;
		} else if (fix != _last_fix_type) {
			if (fix >= 3 && _last_fix_type < 3) {
				PX4_INFO("[GPS] FIX fix=%d sats=%d", fix, gps.satellites_used);
			} else if (fix < 3 && _last_fix_type >= 3) {
				PX4_WARN("[GPS] FIX ZTRACEN fix=%d sats=%d", fix, gps.satellites_used);
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

	// --- Magnetometr + kurz ---
	sensor_mag_s mag{};
	float heading = -1.0f;

	if (_mag_sub.advertised() && _mag_sub.update(&mag)) {
		heading = mag_heading(mag.x, mag.y);
		if (_last_heading < 0.0f) {
			_last_heading = heading;
		} else if (heading_diff(heading, _last_heading) >= HEADING_THRESHOLD) {
			PX4_INFO("[ZMENA] Kurz %.0f -> %.0f deg (%s)",
				(double)_last_heading, (double)heading, direction_str(heading));
			_last_heading = heading;
		}
	}

	// --- Naklon (attitude) ---
	vehicle_attitude_s att{};
	float roll_deg = 0.0f, pitch_deg = 0.0f;
	bool has_att = _att_sub.update(&att);

	if (has_att) {
		quat_to_rp(att.q, roll_deg, pitch_deg);
	}

	// --- Lidar (MTF-01P) ---
	distance_sensor_s dist{};
	bool has_dist = _dist_sub.update(&dist);

	// --- ESC / RPM (BDShot300 telemetrie) ---
	esc_status_s esc{};
	bool has_esc = _esc_sub.update(&esc);

	// --- Optical flow (MTF-01P kamera) ---
	vehicle_optical_flow_s flow{};
	bool has_flow = _flow_sub.update(&flow);

	// --- Baterie ---
	battery_status_s bat{};
	bool has_bat = _bat_sub.update(&bat);

	if (has_bat && bat.connected) {
		bool crit = bat.remaining < BAT_CRIT_LEVEL;
		bool warn = bat.remaining < BAT_WARN_LEVEL;
		hrt_abstime interval = crit ? 10_s : BAT_WARN_INTERVAL;

		if ((warn || crit) && (now - _last_bat_warn_us) >= interval) {
			_last_bat_warn_us = now;
			if (crit) {
				PX4_ERR("[BAT] KRITICKA! %.1fV %.0f%%",
					(double)bat.voltage_v, (double)(bat.remaining * 100.0f));
			} else {
				PX4_WARN("[BAT] Nizka baterie %.1fV %.0f%%",
					(double)bat.voltage_v, (double)(bat.remaining * 100.0f));
			}
		}
	}

	// --- Periodicky status (kazdou 1s) ---
	if ((now - _last_status_us) >= STATUS_INTERVAL) {
		_last_status_us = now;
		PX4_INFO("=== STATUS [%s] ===", flight_mode_str(status.nav_state));

		if (has_lpos) {
			PX4_INFO("[POS] x=%.1f y=%.1f z=%.1fm spd=%.1fm/s",
				(double)lpos.x, (double)lpos.y, (double)lpos.z, (double)speed_2d);
		}

		if (heading >= 0.0f) {
			PX4_INFO("[KRZ] %.0f deg -> %s", (double)heading, direction_str(heading));
		}

		if (has_att) {
			const char *pitch_dir = (pitch_deg > 1.0f) ? "DOZADU" : (pitch_deg < -1.0f) ? "DOPREDU" : "ROVNE";
			const char *roll_dir  = (roll_deg  > 1.0f) ? "DOPRAVA" : (roll_deg  < -1.0f) ? "DOLEVA"  : "ROVNE";
			PX4_INFO("[ATT] pitch=%.1f(%s) roll=%.1f(%s)",
				(double)pitch_deg, pitch_dir,
				(double)roll_deg,  roll_dir);
		}

		if (has_gps) {
			if (gps.fix_type >= 3) {
				PX4_INFO("[GPS] fix=%d sats=%d lat=%.5f lon=%.5f alt=%.1fm",
					gps.fix_type, gps.satellites_used,
					(double)gps.latitude_deg, (double)gps.longitude_deg,
					(double)gps.altitude_msl_m);
			} else {
				PX4_INFO("[GPS] no fix | fix=%d sats=%d", gps.fix_type, gps.satellites_used);
			}
		}

		if (has_dist) {
			PX4_INFO("[LIDAR] dist=%.2fm", (double)dist.current_distance);
		}

		if (has_flow) {
			PX4_INFO("[FLOW]  qual=%d flow=[%.4f, %.4f] dist=%.2fm",
				(int)flow.quality,
				(double)flow.pixel_flow[0], (double)flow.pixel_flow[1],
				(double)flow.distance_m);
		}

		if (has_lpos && lpos.dist_bottom_valid) {
			PX4_INFO("[EKF]   h_ground=%.2fm vz=%.2fm/s",
				(double)lpos.dist_bottom, (double)lpos.vz);
		}

		if (has_esc && esc.esc_count > 0) {
			uint8_t n = esc.esc_count < 12 ? esc.esc_count : 12;
			for (uint8_t i = 0; i < n; i++) {
				bool online = (esc.esc_online_flags >> i) & 1u;
				PX4_INFO("[ESC%u] %s rpm=%d",
					(unsigned)(i + 1),
					online ? "OK" : "OFF",
					(int)esc.esc[i].esc_rpm);
			}
		}

		if (has_bat && bat.connected) {
			PX4_INFO("[BAT] %.2fV %.0f%%",
				(double)bat.voltage_v, (double)(bat.remaining * 100.0f));
		}

		PX4_INFO("==================");
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
