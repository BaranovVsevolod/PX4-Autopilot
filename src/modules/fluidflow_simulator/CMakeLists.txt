px4_add_module(
	MODULE modules__fluidflow_simulator
	MAIN fluidflow_simulator
	STACK_MAIN 1200
	STACK_MAX 2000
	SRCS
		FluidFlowSimulator.cpp
	DEPENDS
	PARAMETERS
		fluidflow_simulator.params
	)
