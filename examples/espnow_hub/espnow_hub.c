#include "freertos/idf_additions.h"
#include "hub.h"
#include "stats.h"

void app_main(void)
{
	//stats_run();
	hub_init();
}
