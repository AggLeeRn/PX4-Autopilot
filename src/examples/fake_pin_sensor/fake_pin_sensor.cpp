#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_acceleration.h>

#include <math.h>

using namespace time_literals;

class FakePinSensor : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static ModuleBase::Descriptor desc;

	FakePinSensor() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1) {}

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return 0; }
	static int print_usage(const char *reason = nullptr) { return 0; }

	int init();

private:
	void Run() override;

	uORB::Subscription _accel_sub{ORB_ID(vehicle_acceleration)};

	static constexpr float MOVING_THRESHOLD = 2.0f;
};

ModuleBase::Descriptor FakePinSensor::desc{task_spawn, custom_command, print_usage};

int FakePinSensor::init()
{
	ScheduleOnInterval(100_ms);
	return 0;
}

void FakePinSensor::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	vehicle_acceleration_s accel;

	if (_accel_sub.update(&accel)) {
		float horiz = sqrtf(accel.xyz[0] * accel.xyz[0] + accel.xyz[1] * accel.xyz[1]);

		if (horiz > MOVING_THRESHOLD) {
			PX4_INFO("POHYB: %.2f m/s^2", (double)horiz);
		} else {
			PX4_INFO("STOJI: %.2f m/s^2", (double)horiz);
		}
	}
}

int FakePinSensor::task_spawn(int argc, char *argv[])
{
	FakePinSensor *instance = new FakePinSensor();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}
	}

	PX4_ERR("alloc failed");
	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return -1;
}

extern "C" __EXPORT int fake_pin_sensor_main(int argc, char *argv[])
{
	return ModuleBase::main(FakePinSensor::desc, argc, argv);
}
