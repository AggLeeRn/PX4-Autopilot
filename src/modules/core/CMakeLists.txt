px4_add_module(
	MODULE modules__core
	MAIN core
	SRCS
		core.cpp
	DEPENDS
		px4_work_queue
)
