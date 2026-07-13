#pragma once

#include <stddef.h>

#include "esp_err.h"
#include "keemash_mesh_node.h"

void mesh_v2_link_require(void);
esp_err_t mesh_v2_link_send(const uint8_t dst[6], const void *packet,
			    size_t packet_len, uint8_t priority);
