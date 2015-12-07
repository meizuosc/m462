/*
 *  linux/include/linux/qos_cooling.h
 *
 */
#ifndef __QOS_COOLING_H__
#define __QOS_COOLING_H__

#include <linux/thermal.h>

#ifdef CONFIG_EXYNOS_THERMAL

struct thermal_cooling_device *qos_cooling_register(void);
void qos_cooling_unregister(struct thermal_cooling_device *cdev);
int get_qos_level(void);
#else /* !CONFIG_CPU_THERMAL */
static inline struct thermal_cooling_device *qos_cooling_register(void)
{
	return NULL;
}
static inline
void qos_cooling_unregister(struct thermal_cooling_device *cdev)
{
	return;
}
static inline void get_qos_level(void)
{
	return;
}
#endif	/* CONFIG_EXYNOS_THERMAL */
#endif /* __QOS_COOLING_H__ */
