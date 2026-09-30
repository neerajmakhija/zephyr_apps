#ifndef ENVNODE_H
#define ENVNODE_H

#include <stdint.h>

struct env_reading {
	uint32_t uptime_ms;
	int32_t  temp_cdeg;      /* corrected die temp, 0.01 °C */
	int32_t  vdda_mv;
};

#endif  