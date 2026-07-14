#pragma once

#include "keemash_mesh_time.h"

#define mesh_time_sync_handle_rx keemash_mesh_time_handle_v1
#define mesh_time_sync_apply_v2 keemash_mesh_time_apply_v2

static inline void mesh_time_sync_init(void)
{
	keemash_mesh_time_init("CET-1CEST,M3.5.0/2,M10.5.0/3");
}
