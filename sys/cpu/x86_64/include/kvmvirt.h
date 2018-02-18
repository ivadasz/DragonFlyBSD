#ifndef _CPU_KVMVIRT_H_
#define _CPU_KVMVIRT_H_

struct pvclock_vcpu_time_info {
	uint32_t version;
	uint32_t pad0;
	uint64_t tsc_timestamp;
	uint64_t system_time;
	uint32_t tsc_to_system_mul;
	int8_t tsc_shift;
	uint8_t flags;
	uint8_t pad[2];
} __attribute__((packed));

#endif /* !_CPU_KVMVIRT_H_ */
