#include "FluidFlowSimulator.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>

FluidFlowSimulator::FluidFlowSimulator() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
    _random_generator(std::random_device{}()),
    _flow_distribution(1.0f, 2.0f)
{
    _param_min_flow.set(1.0f);
    _param_max_flow.set(2.0f);
    _param_change_interval.set(10000000);

    update_parameters();
    _loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
}

FluidFlowSimulator::~FluidFlowSimulator()
{
    ScheduleClear();
    perf_free(_loop_perf);
}

bool FluidFlowSimulator::init()
{
    ScheduleOnInterval(100000); // 100 ms = 10 Hz
    return true;
}

void FluidFlowSimulator::Run()
{
    perf_begin(_loop_perf);

    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // Check for parameter updates
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        update_parameters();
    }

    // Update flow rate if it's time
    update_flow_rate();
    
    // Publish data
    publish_flow_data();

    perf_end(_loop_perf);
}

void FluidFlowSimulator::update_flow_rate()
{
    hrt_abstime now = hrt_absolute_time();
    
    if (now - _last_flow_change > _param_change_interval.get()) {
        _current_flow_rate = _flow_distribution(_random_generator);
        PX4_INFO("Flow rate changed to: %.2f l/min", (double)_current_flow_rate);
        _last_flow_change = now;
    }
}

void FluidFlowSimulator::publish_flow_data()
{
    FluidFlow_s fluid_flow{};
    fluid_flow.timestamp = hrt_absolute_time();
    fluid_flow.flow_rate = _current_flow_rate;
    fluid_flow.sensor_healthy = 1;

    _fluid_flow_pub.publish(fluid_flow);
}

void FluidFlowSimulator::update_parameters()
{
    _flow_distribution = std::uniform_real_distribution<float>(
        _param_min_flow.get(), 
        _param_max_flow.get()
    );
}

int FluidFlowSimulator::task_spawn(int argc, char *argv[])
{
    FluidFlowSimulator *instance = new FluidFlowSimulator();

    if (!instance) {
        PX4_ERR("alloc failed");
        return -1;
    }

    if (instance->init()) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;
        return 0;
    }

    delete instance;
    return -1;
}

int FluidFlowSimulator::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int FluidFlowSimulator::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Fluid flow rate simulator module.

This module simulates fluid flow rate measurements and publishes them in the FluidFlow topic.
The flow rate changes randomly within configured bounds at regular intervals.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("fluidflow_simulator", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("stop");
    PRINT_MODULE_USAGE_COMMAND("status");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int fluidflow_simulator_main(int argc, char *argv[])
{
    return FluidFlowSimulator::main(argc, argv);
}
