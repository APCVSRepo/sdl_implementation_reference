/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2009-2010, 2012 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file rk.c
 * implementation of platform_specific_code on rk platforms, such as rk3328h.
 *
 * mali_device_driver(MDD) includes 2 parts :
 *	.DP : platform_dependent_part :
 *		located in <mdd_src_dir>/mali/platform/<platform_name>/
 *	.DP : common_part :
 *		common part implemented by ARM.
 */

#define ENABLE_DEBUG_LOG
#include "custom_log.h"

#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_PM
#include <linux/pm_runtime.h>
#endif
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/rockchip/cpu.h>

#include <linux/mali/mali_utgard.h>
#include "mali_kernel_common.h"
#include "../../common/mali_osk_mali.h"

/*---------------------------------------------------------------------------*/

u32 mali_group_error;

/*---------------------------------------------------------------------------*/

/*
 * rk_platform_context_of_mali_device.
 */
struct rk_context {
	/* mali device. */
	struct device *dev;
	/* is the GPU powered on?  */
	bool is_powered;
};

struct rk_context *s_rk_context;

/*-------------------------------------------------------*/

static int rk_context_create_sysfs_files(struct device *dev)
{
	int ret = 0;

	return ret;
}

static void rk_context_remove_sysfs_files(struct device *dev)
{
}

/*
 * Init rk_platform_context of mali_device.
 */
static int rk_context_init(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct rk_context *platform; /* platform_context */

	platform = kzalloc(sizeof(*platform), GFP_KERNEL);
	if (!platform) {
		E("no mem.");
		return _MALI_OSK_ERR_NOMEM;
	}

	platform->dev = dev;
	platform->is_powered = false;

	ret = rk_context_create_sysfs_files(dev);
	if (ret) {
		E("fail to create sysfs files, ret = %d", ret);
		goto EXIT;
	}

	s_rk_context = platform;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

EXIT:
	return ret;
}

static void rk_context_deinit(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk_context *platform = s_rk_context;

	pm_runtime_disable(dev);

	s_rk_context = NULL;

	rk_context_remove_sysfs_files(dev);

	if (platform) {
		platform->is_powered = false;
		platform->dev = NULL;
		kfree(platform);
	}
}

/*---------------------------------------------------------------------------*/
/* for devfreq cooling. */

#if defined(CONFIG_MALI_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)

#define FALLBACK_STATIC_TEMPERATURE 55000

static struct thermal_zone_device *gpu_tz;

/* Calculate gpu static power example for reference */
static unsigned long rk_model_static_power(unsigned long voltage)
{
	int temperature, temp;
	int temp_squared, temp_cubed, temp_scaling_factor;
	const unsigned long coefficient = (410UL << 20) / (729000000UL >> 10);
	const unsigned long voltage_cubed = (voltage * voltage * voltage) >> 10;
	unsigned long static_power;

	if (gpu_tz) {
		int ret;

		ret = gpu_tz->ops->get_temp(gpu_tz, &temperature);
		if (ret) {
			MALI_DEBUG_PRINT(2, ("fail to read temp: %d\n", ret));
			temperature = FALLBACK_STATIC_TEMPERATURE;
		}
	} else {
		temperature = FALLBACK_STATIC_TEMPERATURE;
	}

	/* Calculate the temperature scaling factor. To be applied to the
	 * voltage scaled power.
	 */
	temp = temperature / 1000;
	temp_squared = temp * temp;
	temp_cubed = temp_squared * temp;
	temp_scaling_factor =
		(2 * temp_cubed)
		- (80 * temp_squared)
		+ (4700 * temp)
		+ 32000;

	static_power = (((coefficient * voltage_cubed) >> 20)
			* temp_scaling_factor)
		       / 1000000;

	return static_power;
}

/* Calculate gpu dynamic power example for reference */
static unsigned long rk_model_dynamic_power(unsigned long freq,
					    unsigned long voltage)
{
	/* The inputs: freq (f) is in Hz, and voltage (v) in mV.
	 * The coefficient (c) is in mW/(MHz mV mV).
	 *
	 * This function calculates the dynamic power after this formula:
	 * Pdyn (mW) = c (mW/(MHz*mV*mV)) * v (mV) * v (mV) * f (MHz)
	 */
	const unsigned long v2 = (voltage * voltage) / 1000; /* m*(V*V) */
	const unsigned long f_mhz = freq / 1000000; /* MHz */
	const unsigned long coefficient = 3600; /* mW/(MHz*mV*mV) */
	unsigned long dynamic_power;

	dynamic_power = (coefficient * v2 * f_mhz) / 1000000; /* mW */

	return dynamic_power;
}

struct devfreq_cooling_power rk_cooling_ops = {
	.get_static_power = rk_model_static_power,
	.get_dynamic_power = rk_model_dynamic_power,
};
#endif

/*---------------------------------------------------------------------------*/

#ifdef CONFIG_PM

static int rk_platform_enable_clk_gpu(struct device *dev)
{
	int ret = 0;
#if defined(CONFIG_MALI_DEVFREQ) && defined(CONFIG_HAVE_CLK)
	struct mali_device *mdev = dev_get_drvdata(dev);

	if (mdev->clock)
		ret = clk_enable(mdev->clock);
#endif
	return ret;
}

static void rk_platform_disable_clk_gpu(struct device *dev)
{
#if defined(CONFIG_MALI_DEVFREQ) && defined(CONFIG_HAVE_CLK)
	struct mali_device *mdev = dev_get_drvdata(dev);

	if (mdev->clock)
		clk_disable(mdev->clock);
#endif
}

static int rk_platform_enable_gpu_regulator(struct device *dev)
{
	int ret = 0;
#if defined(CONFIG_MALI_DEVFREQ) && defined(CONFIG_REGULATOR)
	struct mali_device *mdev = dev_get_drvdata(dev);

	if (mdev->regulator)
		ret = regulator_enable(mdev->regulator);
#endif
	return ret;
}

static void rk_platform_disable_gpu_regulator(struct device *dev)
{
#if defined(CONFIG_MALI_DEVFREQ) && defined(CONFIG_REGULATOR)
	struct mali_device *mdev = dev_get_drvdata(dev);

	if (mdev->regulator)
		regulator_disable(mdev->regulator);
#endif
}

static int rk_platform_power_on_gpu(struct device *dev)
{
	int ret = 0;

	ret = rk_platform_enable_clk_gpu(dev);
	if (ret) {
		E("fail to enable clk_gpu, ret : %d.", ret);
		goto fail_to_enable_clk;
	}

	ret = rk_platform_enable_gpu_regulator(dev);
	if (ret) {
		E("fail to enable vdd_gpu, ret : %d.", ret);
		goto fail_to_enable_regulator;
	}

	return 0;

fail_to_enable_regulator:
	rk_platform_disable_clk_gpu(dev);

fail_to_enable_clk:
	return ret;
}

static void rk_platform_power_off_gpu(struct device *dev)
{
	rk_platform_disable_clk_gpu(dev);
	rk_platform_disable_gpu_regulator(dev);
}

static int mali_runtime_suspend(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(4, ("mali_runtime_suspend() called\n"));

	if (device->driver &&
	    device->driver->pm &&
	    device->driver->pm->runtime_suspend) {
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->runtime_suspend(device);
	}

	rk_platform_power_off_gpu(device);

	return ret;
}

static int mali_runtime_resume(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(4, ("mali_runtime_resume() called\n"));

	rk_platform_power_on_gpu(device);

	if (device->driver &&
	    device->driver->pm &&
	    device->driver->pm->runtime_resume) {
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->runtime_resume(device);
	}

	return ret;
}

static int mali_runtime_idle(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(4, ("mali_runtime_idle() called\n"));

	if (device->driver &&
	    device->driver->pm &&
	    device->driver->pm->runtime_idle) {
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->runtime_idle(device);
		if (ret)
			return ret;
	}

	pm_runtime_suspend(device);

	return 0;
}
#endif

static int mali_os_suspend(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(4, ("mali_os_suspend() called\n"));

	if (device->driver &&
	    device->driver->pm &&
	    device->driver->pm->suspend) {
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->suspend(device);
	}

	rk_platform_power_off_gpu(device);

	return ret;
}

static int mali_os_resume(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(4, ("mali_os_resume() called\n"));

	rk_platform_power_on_gpu(device);

	if (device->driver &&
	    device->driver->pm &&
	    device->driver->pm->resume) {
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->resume(device);
	}

	return ret;
}

static int mali_os_freeze(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(4, ("mali_os_freeze() called\n"));

	if (device->driver &&
	    device->driver->pm &&
	    device->driver->pm->freeze) {
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->freeze(device);
	}

	return ret;
}

static int mali_os_thaw(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(4, ("mali_os_thaw() called\n"));

	if (device->driver &&
	    device->driver->pm &&
	    device->driver->pm->thaw) {
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->thaw(device);
	}

	return ret;
}

static const struct dev_pm_ops mali_gpu_device_type_pm_ops = {
	.suspend = mali_os_suspend,
	.resume = mali_os_resume,
	.freeze = mali_os_freeze,
	.thaw = mali_os_thaw,
#ifdef CONFIG_PM
	.runtime_suspend = mali_runtime_suspend,
	.runtime_resume = mali_runtime_resume,
	.runtime_idle = mali_runtime_idle,
#endif
};

static const struct device_type mali_gpu_device_device_type = {
	.pm = &mali_gpu_device_type_pm_ops,
};

/*
 * platform_specific_data of platform_device of mali_gpu.
 */
static const struct mali_gpu_device_data mali_gpu_data = {
	.shared_mem_size = 1024 * 1024 * 1024, /* 1GB */
	.max_job_runtime = 100, /* 100 ms */
#if defined(CONFIG_MALI_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
	.gpu_cooling_ops = &rk_cooling_ops,
#endif
};

static void mali_platform_device_add_config(struct platform_device *pdev)
{
	pdev->name = MALI_GPU_NAME_UTGARD,
	pdev->id = 0;
	pdev->dev.type = &mali_gpu_device_device_type;
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask,
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
}

/*---------------------------------------------------------------------------*/
/* platform_device_functions called by common_part. */

int mali_platform_device_init(struct platform_device *pdev)
{
	int err = 0;

	mali_platform_device_add_config(pdev);

	D("to add platform_specific_data to platform_device_of_mali.");
	err = platform_device_add_data(pdev,
				       &mali_gpu_data,
				       sizeof(mali_gpu_data));
	if (err) {
		E("fail to add platform_specific_data. err : %d.", err);
		goto EXIT;
	}

	err = rk_context_init(pdev);
	if (err) {
		E("fail to init rk_context. err : %d.", err);
		goto EXIT;
	}

#if defined(CONFIG_MALI_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
	/* Get thermal zone */
	gpu_tz = thermal_zone_get_zone_by_name("gpu_thermal");
	if (IS_ERR(gpu_tz)) {
		W("Error getting gpu thermal zone (%ld), not yet ready?",
		  PTR_ERR(gpu_tz));
		gpu_tz = NULL;
		/* err =  -EPROBE_DEFER; */
	}
#endif

EXIT:
	return err;
}

void mali_platform_device_deinit(struct platform_device *pdev)
{
	MALI_DEBUG_PRINT(4, ("mali_platform_device_unregister() called\n"));

	rk_context_deinit(pdev);
}
