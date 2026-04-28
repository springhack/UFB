#include "model/filament_state.h"
_ams ams[ams_max_number];

void ams_init()
{
    for(uint8_t i=0;i<ams_max_number;i++)
    {
        ams[i].init();
    }
}


