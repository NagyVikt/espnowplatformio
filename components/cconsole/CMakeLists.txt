idf_component_register(SRCS "cconsole.c" INCLUDE_DIRS "."
	REQUIRES console fatfs nvs_flash hub stats
)