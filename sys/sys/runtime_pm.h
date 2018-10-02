#ifndef _RUNTIME_PM_H_
#define _RUNTIME_PM_H_

#ifdef _KERNEL

struct rpm_ops {
	int (*runtime_suspend) (device_t dev);
	int (*runtime_resume) (device_t dev);
};

void pm_runtime_register(device_t dev, struct rpm_ops *ops);
void pm_runtime_unregister(device_t dev);
void pm_runtime_mark_last_busy(device_t dev);
void pm_runtime_put_autosuspend(device_t dev);
void pm_runtime_set_autosuspend_delay(device_t dev, int delay);
void pm_runtime_get_sync(device_t dev);
void pm_runtime_get_noresume(device_t dev);
int pm_runtime_get_if_in_use(device_t dev);

#endif /* _KERNEL */

#endif /* _RUNTIME_PM_H_ */
