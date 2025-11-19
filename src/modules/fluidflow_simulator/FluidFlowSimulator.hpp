#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/FluidFlow.h>
#include <uORB/topics/parameter_update.h>

#include <lib/perf/perf_counter.h>

#include <random>

class FluidFlowSimulator : public ModuleBase<FluidFlowSimulator>, public px4::ScheduledWorkItem, public ModuleParams
{
public:
    FluidFlowSimulator();
    ~FluidFlowSimulator() override;

    / @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    / @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    / @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    / @see ModuleBase::run() */
    void Run() override;

    bool init();

private:
    void update_flow_rate();
    void publish_flow_data();
    void update_parameters();

    uORB::Publication<FluidFlow_s> _fluid_flow_pub{ORB_ID(FluidFlow)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

    float _current_flow_rate{1.5f}; // текущий расход в л/мин
    hrt_abstime _last_flow_change{0}; // время последнего изменения расхода
    
    // Параметры
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::FFS_MIN_FLOW>) _param_min_flow,
        (ParamFloat<px4::params::FFS_MAX_FLOW>) _param_max_flow,
        (ParamInt<px4::params::FFS_CHANGE_INTERVAL>) _param_change_interval
    );

    std::default_random_engine _random_generator;
    std::uniform_real_distribution<float> _flow_distribution;

    perf_counter_t _loop_perf{nullptr};
};
