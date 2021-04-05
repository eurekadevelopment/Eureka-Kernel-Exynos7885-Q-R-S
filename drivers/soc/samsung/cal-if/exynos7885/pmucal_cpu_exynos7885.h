/* individual sequence descriptor for each core - on, off, status */
struct pmucal_seq core00_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_SHORTSTOP", 0x10900000, 0x0820, (0xffffffff << 0), (0xFFFFFFFF << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPUSEQUENCER_OPTION", 0x11C80000, 0x2488, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPUSEQUENCER_OPTION", 0x11C80000, 0x2488, (0x1 << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPU0_CONFIGURATION", 0x11C80000, 0x2000, (0xf << 0), (0xF << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core00_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPU0_CONFIGURATION", 0x11C80000, 0x2000, (0xf << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core00_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x11C80000, 0x2004, (0xf << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core01_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_SHORTSTOP", 0x10900000, 0x0820, (0xffffffff << 0), (0xFFFFFFFF << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPUSEQUENCER_OPTION", 0x11C80000, 0x2488, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPUSEQUENCER_OPTION", 0x11C80000, 0x2488, (0x1 << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPU1_CONFIGURATION", 0x11C80000, 0x2080, (0xf << 0), (0xF << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core01_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPU1_CONFIGURATION", 0x11C80000, 0x2080, (0xf << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core01_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU1_STATUS", 0x11C80000, 0x2084, (0xf << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core10_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPU0_CONFIGURATION", 0x11C80000, 0x2200, (0xf << 0), (0xF << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core10_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPU0_CONFIGURATION", 0x11C80000, 0x2200, (0xf << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core10_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_CPU0_STATUS", 0x11C80000, 0x2204, (0xf << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core11_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPU1_CONFIGURATION", 0x11C80000, 0x2280, (0xf << 0), (0xF << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core11_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPU1_CONFIGURATION", 0x11C80000, 0x2280, (0xf << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core11_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_CPU1_STATUS", 0x11C80000, 0x2284, (0xf << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core12_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPU2_CONFIGURATION", 0x11C80000, 0x2300, (0xf << 0), (0xF << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core12_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPU2_CONFIGURATION", 0x11C80000, 0x2300, (0xf << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core12_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_CPU2_STATUS", 0x11C80000, 0x2304, (0xf << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core13_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPU3_CONFIGURATION", 0x11C80000, 0x2380, (0xf << 0), (0xF << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core13_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPU3_CONFIGURATION", 0x11C80000, 0x2380, (0xf << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core13_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_CPU3_STATUS", 0x11C80000, 0x2384, (0xf << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core20_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_SHORTSTOP", 0x10A00000, 0x0820, (0xffffffff << 0), (0xFFFFFFFF << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPUSEQUENCER_OPTION", 0x11C80000, 0x24C8, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPUSEQUENCER_OPTION", 0x11C80000, 0x24C8, (0x1 << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPU0_CONFIGURATION", 0x11C80000, 0x2100, (0xf << 0), (0xF << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core20_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPU0_CONFIGURATION", 0x11C80000, 0x2100, (0xf << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core20_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER2_CPU0_STATUS", 0x11C80000, 0x2104, (0xf << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core21_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_SHORTSTOP", 0x10A00000, 0x0820, (0xffffffff << 0), (0xFFFFFFFF << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPUSEQUENCER_OPTION", 0x11C80000, 0x24C8, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPUSEQUENCER_OPTION", 0x11C80000, 0x24C8, (0x1 << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPU1_CONFIGURATION", 0x11C80000, 0x2180, (0xf << 0), (0xF << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core21_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPU1_CONFIGURATION", 0x11C80000, 0x2180, (0xf << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core21_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER2_CPU1_STATUS", 0x11C80000, 0x2184, (0xf << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq cluster0_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPUSEQUENCER_OPTION", 0x11C80000, 0x2488, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPUSEQUENCER_OPTION", 0x11C80000, 0x2488, (0x1 << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq cluster0_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPUSEQUENCER_OPTION", 0x11C80000, 0x2488, (0x1 << 0), (0x1 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER0_CPUSEQUENCER_OPTION", 0x11C80000, 0x2488, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq cluster0_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPUSEQUENCER_STATUS", 0x11C80000, 0x2484, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq cluster1_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPUSEQUENCER_OPTION", 0x11C80000, 0x24A8, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPUSEQUENCER_OPTION", 0x11C80000, 0x24A8, (0x1 << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_CPUSEQUENCER_STATUS", 0x11C80000, 0x24A4, (0x1 << 0), (0x1 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq cluster1_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPUSEQUENCER_OPTION", 0x11C80000, 0x24A8, (0x1 << 0), (0x1 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER1_CPUSEQUENCER_OPTION", 0x11C80000, 0x24A8, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_CPUSEQUENCER_STATUS", 0x11C80000, 0x24A4, (0x1 << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq cluster1_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_CPUSEQUENCER_STATUS", 0x11C80000, 0x24A4, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq cluster2_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPUSEQUENCER_OPTION", 0x11C80000, 0x24C8, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPUSEQUENCER_OPTION", 0x11C80000, 0x24C8, (0x1 << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER2_CPUSEQUENCER_STATUS", 0x11C80000, 0x24C4, (0x1 << 0), (0x1 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq cluster2_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPUSEQUENCER_OPTION", 0x11C80000, 0x24C8, (0x1 << 0), (0x1 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CLUSTER2_CPUSEQUENCER_OPTION", 0x11C80000, 0x24C8, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER2_CPUSEQUENCER_STATUS", 0x11C80000, 0x24C4, (0x1 << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
};
struct pmucal_seq cluster2_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER2_CPUSEQUENCER_STATUS", 0x11C80000, 0x24C4, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
enum pmucal_cpu_corenum {
	CPU_CORE00,
	CPU_CORE01,
	CPU_CORE10,
	CPU_CORE11,
	CPU_CORE12,
	CPU_CORE13,
	CPU_CORE20,
	CPU_CORE21,
	PMUCAL_NUM_CORES,
};

struct pmucal_cpu pmucal_cpu_list[PMUCAL_NUM_CORES] = {
	[CPU_CORE00] = {
		.id = CPU_CORE00,
		.on = core00_on,
		.off = core00_off,
		.status = core00_status,
		.num_on = ARRAY_SIZE(core00_on),
		.num_off = ARRAY_SIZE(core00_off),
		.num_status = ARRAY_SIZE(core00_status),
	},
	[CPU_CORE01] = {
		.id = CPU_CORE01,
		.on = core01_on,
		.off = core01_off,
		.status = core01_status,
		.num_on = ARRAY_SIZE(core01_on),
		.num_off = ARRAY_SIZE(core01_off),
		.num_status = ARRAY_SIZE(core01_status),
	},
	[CPU_CORE10] = {
		.id = CPU_CORE10,
		.on = core10_on,
		.off = core10_off,
		.status = core10_status,
		.num_on = ARRAY_SIZE(core10_on),
		.num_off = ARRAY_SIZE(core10_off),
		.num_status = ARRAY_SIZE(core10_status),
	},
	[CPU_CORE11] = {
		.id = CPU_CORE11,
		.on = core11_on,
		.off = core11_off,
		.status = core11_status,
		.num_on = ARRAY_SIZE(core11_on),
		.num_off = ARRAY_SIZE(core11_off),
		.num_status = ARRAY_SIZE(core11_status),
	},
	[CPU_CORE12] = {
		.id = CPU_CORE12,
		.on = core12_on,
		.off = core12_off,
		.status = core12_status,
		.num_on = ARRAY_SIZE(core12_on),
		.num_off = ARRAY_SIZE(core12_off),
		.num_status = ARRAY_SIZE(core12_status),
	},
	[CPU_CORE13] = {
		.id = CPU_CORE13,
		.on = core13_on,
		.off = core13_off,
		.status = core13_status,
		.num_on = ARRAY_SIZE(core13_on),
		.num_off = ARRAY_SIZE(core13_off),
		.num_status = ARRAY_SIZE(core13_status),
	},
	[CPU_CORE20] = {
		.id = CPU_CORE20,
		.on = core20_on,
		.off = core20_off,
		.status = core20_status,
		.num_on = ARRAY_SIZE(core20_on),
		.num_off = ARRAY_SIZE(core20_off),
		.num_status = ARRAY_SIZE(core20_status),
	},
	[CPU_CORE21] = {
		.id = CPU_CORE21,
		.on = core21_on,
		.off = core21_off,
		.status = core21_status,
		.num_on = ARRAY_SIZE(core21_on),
		.num_off = ARRAY_SIZE(core21_off),
		.num_status = ARRAY_SIZE(core21_status),
	},
};
unsigned int pmucal_cpu_list_size = 8;
enum pmucal_cpu_clusternum {
	CPU_CLUSTER0,
	CPU_CLUSTER1,
	CPU_CLUSTER2,
	PMUCAL_NUM_CLUSTERS,
};

struct pmucal_cpu pmucal_cluster_list[PMUCAL_NUM_CLUSTERS] = {
	[CPU_CLUSTER0] = {
		.id = CPU_CLUSTER0,
		.on = cluster0_on,
		.off = cluster0_off,
		.status = cluster0_status,
		.num_on = ARRAY_SIZE(cluster0_on),
		.num_off = ARRAY_SIZE(cluster0_off),
		.num_status = ARRAY_SIZE(cluster0_status),
	},
	[CPU_CLUSTER1] = {
		.id = CPU_CLUSTER1,
		.on = cluster1_on,
		.off = cluster1_off,
		.status = cluster1_status,
		.num_on = ARRAY_SIZE(cluster1_on),
		.num_off = ARRAY_SIZE(cluster1_off),
		.num_status = ARRAY_SIZE(cluster1_status),
	},
	[CPU_CLUSTER2] = {
		.id = CPU_CLUSTER2,
		.on = cluster2_on,
		.off = cluster2_off,
		.status = cluster2_status,
		.num_on = ARRAY_SIZE(cluster2_on),
		.num_off = ARRAY_SIZE(cluster2_off),
		.num_status = ARRAY_SIZE(cluster2_status),
	},
};
unsigned int pmucal_cluster_list_size = 3;
