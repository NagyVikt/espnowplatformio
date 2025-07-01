/*******************************BEGIN: INCLUDE GUARD*******************************/
#ifndef CCONSOLE_H_
#define CCONSOLE_H_


/*******************************BEGIN: STANDARD LIBRARY INCLUDES*******************************/

/*******************************END: STANDARD LIBRARY INCLUDES*******************************/



/*******************************BEGIN: HEADER FILE INCLUDES OF DIFFERENT MODULES*******************************/

/*******************************END: HEADER FILE INCLUDES OF DIFFERENT MODULES*******************************/


/*******************************BEGIN: PUBLIC STRUCTS, ENUMS, UNIONS, DEFINES*******************************/
enum e_device_mode {
	MODE_HUB,
	MODE_STA
};
/*******************************END: PUBLIC STRUCTS, ENUMS, UNIONS, DEFINES*******************************/


/*******************************BEGIN: GLOBAL VARIABLES EXPOSED BY THIS MODULE*******************************/

/*******************************END: GLOBAL VARIABLES EXPOSED BY THIS MODULE*******************************/



/*******************************BEGIN: API PROTOTYPES EXPOSED BY THIS MODULE*******************************/
void cconsole_init(enum e_device_mode device_mode);
/*******************************END: API PROTOTYPES EXPOSED BY THIS MODULE*******************************/


#endif
/*******************************END: INCLUDE GUARD*******************************/