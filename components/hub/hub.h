/*******************************BEGIN: INCLUDE GUARD*******************************/
#ifndef HUB_H_
#define HUB_H_


/*******************************BEGIN: STANDARD LIBRARY INCLUDES*******************************/
#include <stdint.h>
/*******************************END: STANDARD LIBRARY INCLUDES*******************************/



/*******************************BEGIN: HEADER FILE INCLUDES OF DIFFERENT MODULES*******************************/

/*******************************END: HEADER FILE INCLUDES OF DIFFERENT MODULES*******************************/

/*******************************BEGIN: STRUCTS, ENUMS, UNIONS, DEFINES (PUBLIC)*******************************/

typedef struct {
	uint8_t mac_addr[6];
}PEER_t;

/*******************************END: END: STRUCTS, ENUMS, UNIONS, DEFINES (PUBLIC)*******************************/

/*******************************BEGIN: GLOBAL VARIABLES EXPOSED BY THIS MODULE*******************************/

/*******************************END: GLOBAL VARIABLES EXPOSED BY THIS MODULE*******************************/



/*******************************BEGIN: API PROTOTYPES EXPOSED BY THIS MODULE*******************************/
void hub_init(void);
void hub_spawn_comm_task(PEER_t	peer);
void hub_spawn_comm_loop_task(PEER_t peer);
void hub_spawn_comm_all_loop_task(int loop);
void hub_spawn_measurement_task(void);
void hub_delete_measurement_task(void);


/*******************************END: API PROTOTYPES EXPOSED BY THIS MODULE*******************************/


#endif
/*******************************END: INCLUDE GUARD*******************************/