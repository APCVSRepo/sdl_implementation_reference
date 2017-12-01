/*
 *************************************************************************
 * Rockchip driver for CIF ISP 1.0
 * (Based on Intel driver for sofiaxxx)
 *
 * Copyright (C) 2015 Intel Mobile Communications GmbH
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *************************************************************************
 */

#ifndef CONFIG_OF
#error "this driver requires a kernel with device tree support"
#endif

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include "cif_isp10.h"
#include <linux/platform_data/rk_isp10_platform.h>
#include "cif_isp10_regs.h"
#ifndef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <media/v4l2-controls_rockchip.h>
#ifdef CONFIG_CIF_ISP10_REG_TRACE
#include <stdarg.h>

static struct {
	char *reg_trace;
	loff_t reg_trace_read_pos;
	loff_t reg_trace_write_pos;
	size_t reg_trace_max_size;
	void __iomem *base_addr;
	bool rtrace;
	bool ftrace;
	bool internal;
	spinlock_t lock;/* spin lock */
} cif_isp10_reg_trace;
#endif
#endif

struct cif_isp10_pltfrm_csi_config {
	struct list_head list;
	u32 pps;
	struct cif_isp10_csi_config csi_config;
};

struct cif_isp10_pltfrm_data {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	void __iomem *base_addr;
	int irq;
	struct {
		int mis;
		int (*isr)(unsigned int mis, void *cntxt);
	} irq_handlers[4];
	struct list_head csi0_configs;
	struct list_head csi1_configs;
	s32 exp_time;
	u16 gain;

#ifndef CONFIG_DEBUG_FS
	struct {
		struct dentry *dir;
		struct dentry *cif_isp10_file;
		struct dentry *csi0_file;
		struct dentry *csi1_file;
#ifdef CONFIG_CIF_ISP10_REG_TRACE
		struct dentry *reg_trace_file;
#endif
		void (*print_func)(void *cntxt, const char *block_name);
		void *print_cntxt;
	} dbgfs;
#endif
};

void cif_isp10_pltfrm_debug_register_print_cb(
	struct device *dev,
	void (*print)(void *cntxt, const char *block),
	void *cntxt) {
#ifndef CONFIG_DEBUG_FS
	struct cif_isp10_pltfrm_data *pdata = dev->platform_data;

	pdata->dbgfs.print_cntxt = cntxt;
	pdata->dbgfs.print_func = print;
#endif
}

#ifndef CONFIG_DEBUG_FS
#define CIF_ISP10_DBGFS_BUF_SIZE 1024
static char cif_isp10_dbgfs_buf[CIF_ISP10_DBGFS_BUF_SIZE];

static int cif_isp10_dbgfs_fill_csi_config_from_string(
	struct device *dev,
	struct cif_isp10_csi_config *csi_config,
	char *strp)
{
	char *token;

	token = strsep(&strp, " ");
	if (IS_ERR_OR_NULL(token))
		goto missing_token;
	if (IS_ERR_VALUE(kstrtou32(token, 10,
			&csi_config->vc)))
		goto wrong_token_format;
	token = strsep(&strp, " ");
	if (IS_ERR_OR_NULL(token))
		goto missing_token;
	if (IS_ERR_VALUE(kstrtou32(token, 10,
			&csi_config->nb_lanes)))
		goto wrong_token_format;
	token = strsep(&strp, " ");
	if (IS_ERR_OR_NULL(token))
		goto missing_token;
	if (IS_ERR_VALUE(kstrtou32(token, 16,
			&csi_config->dphy1)))
		goto wrong_token_format;
	token = strsep(&strp, " ");
	if (IS_ERR_OR_NULL(token))
		goto missing_token;
	if (IS_ERR_VALUE(kstrtou32(token, 16,
			&csi_config->dphy2)))
		goto wrong_token_format;
	token = strsep(&strp, " ");
	if (!IS_ERR_OR_NULL(token)) {
		if (IS_ERR_VALUE(kstrtou32(token, 10,
				&csi_config->ana_bandgab_bias)))
			goto wrong_token_format;
	} else {
		csi_config->ana_bandgab_bias = (u32)-1;
	}

	return 0;
missing_token:
	cif_isp10_pltfrm_pr_err(dev,
		"missing token, command format is 'push <pps> <vc> <#lanes> <HEX dphy1> <HEX dphy2> <analog bandgap bias>'\n");
	return -EINVAL;
wrong_token_format:
	cif_isp10_pltfrm_pr_err(dev,
		"wrong token format, command format is 'push <pps> <vc> <#lanes> <HEX dphy1> <HEX dphy2> <analog bandgap bias>'\n");
	return -EINVAL;
}

static int cif_isp10_dbgfs_csi_configs_init(
	struct device *dev,
	enum cif_isp10_inp inp,
	struct list_head *csi_configs)
{
	int ret = 0;
	struct device *img_src_dev = NULL;
	struct device_node *parent_node = NULL;
	struct device_node *child_node = NULL, *prev_node = NULL;
	struct cif_isp10_pltfrm_csi_config *cfg = NULL;
	u32 pps;

	img_src_dev = cif_isp10_pltfrm_get_img_src_device(dev, inp);
	if (IS_ERR_OR_NULL(img_src_dev)) {
		ret = -EFAULT;
		goto err;
	}
	parent_node = of_node_get(img_src_dev->of_node);
	put_device(img_src_dev);
	img_src_dev = NULL;

	while (!IS_ERR_OR_NULL(child_node =
		of_get_next_child(parent_node, prev_node))) {
		if (!strncasecmp(child_node->name,
			"intel,camera-module-csi-config",
			strlen("intel,camera-module-csi-config"))) {
			ret = of_property_read_u32(child_node,
				"intel,csi-pixels-per-second", &pps);
			if (IS_ERR_VALUE(ret)) {
				cif_isp10_pltfrm_pr_err(dev,
					"reading property 'intel,csi-pixels-per-second'\n");
				goto err;
			}
			cfg = kmalloc(
				sizeof(struct cif_isp10_pltfrm_csi_config),
			GFP_KERNEL);
			if (!cfg) {
				cif_isp10_pltfrm_pr_err(dev,
					"memory allocation failed\n");
				ret = -ENOMEM;
				goto err;
			}
			cfg->pps = pps;
			ret = cif_isp10_pltfrm_fill_csi_config_from_node(
					dev, &cfg->csi_config, child_node);
			if (IS_ERR_VALUE(ret))
				goto err;
			list_add_tail(&cfg->list, csi_configs);
			cfg = NULL;
		}
		of_node_put(prev_node);
		prev_node = child_node;
	}
	of_node_put(prev_node);
	of_node_put(parent_node);

	return 0;
err:
	of_node_put(prev_node);
	of_node_put(child_node);
	of_node_put(parent_node);
	kfree(cfg);
	if (!IS_ERR_OR_NULL(img_src_dev))
		put_device(img_src_dev);
	return ret;
}

static ssize_t cif_isp10_dbgfs_csi_read(
	struct file *f,
	char __user *out,
	size_t count,
	loff_t *pos)
{
	u32 out_size = 0;
	u32 str_len;
	struct cif_isp10_pltfrm_csi_config *cfg;
	u32 index = 0;
	struct list_head *list_pos;
	struct device *dev = f->f_inode->i_private;
	struct cif_isp10_pltfrm_data *pdata = dev->platform_data;
	struct list_head *csi_configs;
	enum cif_isp10_inp inp;

	if (f->f_inode == pdata->dbgfs.csi0_file->d_inode) {
		csi_configs = &pdata->csi0_configs;
		inp = CIF_ISP10_INP_CSI_0;
	} else if (f->f_inode == pdata->dbgfs.csi1_file->d_inode) {
		csi_configs = &pdata->csi1_configs;
		inp = CIF_ISP10_INP_CSI_1;
	} else {
		cif_isp10_pltfrm_pr_err(dev, "wrong file handle\n");
		return -EINVAL;
	}

	if (list_empty(csi_configs))
		if (IS_ERR_VALUE(cif_isp10_dbgfs_csi_configs_init(
		dev, inp, csi_configs)))
			return -EFAULT;

	if (*pos)
		return 0;

	list_for_each(list_pos, csi_configs) {
		cfg = list_entry(list_pos,
			struct cif_isp10_pltfrm_csi_config, list);
		sprintf(cif_isp10_dbgfs_buf,
			"csi-config-%d:\n"
			"   pps = %d\n"
			"   vc = %d\n"
			"   nb_lanes = %d\n"
			"   dphy1 = 0x%08x\n"
			"   dphy2 = 0x%08x\n"
			"   ana_bandgap_bias = %d\n",
			index,
			cfg->pps, cfg->csi_config.vc, cfg->csi_config.nb_lanes,
			cfg->csi_config.dphy1, cfg->csi_config.dphy2,
			cfg->csi_config.ana_bandgab_bias);
		index++;
		str_len = strnlen(cif_isp10_dbgfs_buf,
			CIF_ISP10_DBGFS_BUF_SIZE);
		if (str_len > count) {
			*pos += out_size;
			return 0;
		}
		*pos = 0;
		if (IS_ERR_VALUE(simple_read_from_buffer(
			out + out_size, str_len, pos,
			cif_isp10_dbgfs_buf, str_len)))
			break;
		out_size += strnlen(cif_isp10_dbgfs_buf,
			CIF_ISP10_DBGFS_BUF_SIZE);
		count -= str_len;
	}

	*pos += out_size;
	return out_size;
}

static ssize_t cif_isp10_dbgfs_csi_write(
	struct file *f,
	const char __user *in,
	size_t count,
	loff_t *pos)
{
	ssize_t ret;
	char *strp = cif_isp10_dbgfs_buf;
	char *token;
	struct device *dev = f->f_inode->i_private;
	struct cif_isp10_pltfrm_data *pdata = dev->platform_data;
	struct list_head *csi_configs;
	enum cif_isp10_inp inp;

	if (count > CIF_ISP10_DBGFS_BUF_SIZE) {
		cif_isp10_pltfrm_pr_err(dev, "command line too large\n");
		return -EINVAL;
	}

	if (f->f_inode == pdata->dbgfs.csi0_file->d_inode) {
		csi_configs = &pdata->csi0_configs;
		inp = CIF_ISP10_INP_CSI_0;
	} else if (f->f_inode == pdata->dbgfs.csi1_file->d_inode) {
		csi_configs = &pdata->csi1_configs;
		inp = CIF_ISP10_INP_CSI_1;
	} else {
		cif_isp10_pltfrm_pr_err(dev, "wrong file handle\n");
		return -EINVAL;
	}

	if (list_empty(csi_configs))
		if (IS_ERR_VALUE(cif_isp10_dbgfs_csi_configs_init(
		dev, inp, csi_configs)))
			return -EFAULT;

	memset(cif_isp10_dbgfs_buf, 0, CIF_ISP10_DBGFS_BUF_SIZE);
	ret = simple_write_to_buffer(strp,
		CIF_ISP10_DBGFS_BUF_SIZE, pos, in, count);
	if (IS_ERR_VALUE(ret))
		return ret;

	token = strsep(&strp, " ");
	if (!strcmp(token, "push")) {
		struct cif_isp10_pltfrm_csi_config cfg;

		token = strsep(&strp, " ");
		if (IS_ERR_OR_NULL(token)) {
			cif_isp10_pltfrm_pr_err(dev,
				"missing token, command format is 'push <pps> <vc> <#lanes> <HEX dphy1> <HEX dphy2> <analog bandgap bias>'\n");
			return -EINVAL;
		}
		if (IS_ERR_VALUE(kstrtou32(token, 10,
				&cfg.pps))) {
			cif_isp10_pltfrm_pr_err(dev,
				"missing token, command format is 'push <pps> <vc> <#lanes> <HEX dphy1> <HEX dphy2> <analog bandgap bias>'\n");
			return -EINVAL;
		}
		ret = cif_isp10_dbgfs_fill_csi_config_from_string(
			dev, &cfg.csi_config, strp);
		if (IS_ERR_VALUE(ret))
			return ret;
		ret = cif_isp10_pltfrm_l_s_csi_config(
			dev, inp, cfg.pps, &cfg.csi_config);
		if (IS_ERR_VALUE(ret))
			return ret;
	} else if (!strncmp(token, "reset", 5)) {
	} else {
		cif_isp10_pltfrm_pr_err(dev, "unknown command %s\n", token);
		return -EINVAL;
	}

	return count;
}

void cif_isp10_dbgfs_fill_sensor_aec_para(
	struct cif_isp10_device *cif_isp10_dev,
	s32 exp_time,
	u16 gain)
{
	struct cif_isp10_pltfrm_data *pdata;

	pdata = (struct cif_isp10_pltfrm_data *)
			cif_isp10_dev->dev->platform_data;
	pdata->exp_time = exp_time;
	pdata->gain = gain;
}

static ssize_t cif_isp10_dbgfs_sensor_read(
	struct file *f,
	char __user *out,
	size_t count,
	loff_t *pos)
{
	u32 out_size = 0;
	u32 str_len;
	struct device *dev = f->f_inode->i_private;
	struct cif_isp10_pltfrm_data *pdata = dev->platform_data;

	if (*pos)
		return 0;

	sprintf(cif_isp10_dbgfs_buf,
		"sensor current exp_time: %d\n"
		"               gain = %d\n",
		pdata->exp_time,
		pdata->gain);
	str_len = strnlen(cif_isp10_dbgfs_buf,
			  CIF_ISP10_DBGFS_BUF_SIZE);
	if (str_len > count) {
		*pos += out_size;
		return 0;
	}
	*pos = 0;
	if (IS_ERR_VALUE(simple_read_from_buffer(
		out + out_size, str_len, pos,
		cif_isp10_dbgfs_buf, str_len)))
		goto ERR;
	out_size += strnlen(cif_isp10_dbgfs_buf,
		CIF_ISP10_DBGFS_BUF_SIZE);
	count -= str_len;

	*pos += out_size;
ERR:
	return out_size;
}

static ssize_t cif_isp10_dbgfs_sensor_write(
	struct file *f,
	const char __user *in,
	size_t count,
	loff_t *pos)
{
	return 0;
}

static ssize_t cif_isp10_dbgfs_write(
	struct file *f,
	const char __user *in,
	size_t count,
	loff_t *pos)
{
	ssize_t ret;
	char *strp = cif_isp10_dbgfs_buf;
	char *token;
	struct device *dev = f->f_inode->i_private;
	struct cif_isp10_pltfrm_data *pdata = dev->platform_data;

	if (count > CIF_ISP10_DBGFS_BUF_SIZE) {
		cif_isp10_pltfrm_pr_err(dev, "command line too large\n");
		return -EINVAL;
	}

	memset(cif_isp10_dbgfs_buf, 0, CIF_ISP10_DBGFS_BUF_SIZE);
	ret = simple_write_to_buffer(strp,
		CIF_ISP10_DBGFS_BUF_SIZE, pos, in, count);
	if (IS_ERR_VALUE(ret))
		return ret;

	token = strsep(&strp, " ");
	if (!strncmp(token, "print", 5)) {
		token = strsep(&strp, " ");
		if (IS_ERR_OR_NULL(token)) {
			cif_isp10_pltfrm_pr_err(dev,
				"missing token, command format is 'print all|<list of block name>'\n");
			return -EINVAL;
		}
		if (!strncmp(token, "register", 8)) {
			u32 addr;
			struct cif_isp10_pltfrm_data *pdata =
				dev->platform_data;
			token = strsep(&strp, " ");
			while (token) {
				if (IS_ERR_VALUE(kstrtou32(token,
					16, &addr))) {
					cif_isp10_pltfrm_pr_err(dev,
						"malformed token, must be a hexadecimal register address\n");
					return -EINVAL;
				}
				pr_info("0x%04x: 0x%08x\n",
					addr,
					ioread32(pdata->base_addr +
						addr));
				token = strsep(&strp, " ");
			}
		} else if (pdata->dbgfs.print_func) {
			unsigned long flags;

			local_irq_save(flags);
			while (token) {
				pdata->dbgfs.print_func(
					pdata->dbgfs.print_cntxt,
					token);
				token = strsep(&strp, " ");
			}
			local_irq_restore(flags);
		}
	} else if (!strncmp(token, "power", 5)) {
		token = strsep(&strp, " ");
		if (IS_ERR_OR_NULL(token)) {
			cif_isp10_pltfrm_pr_err(dev,
				"missing token, command format is 'power [off|on]'\n");
			return -EINVAL;
		}
		if (!strncmp(token, "on", 2)) {
			if (IS_ERR_VALUE(cif_isp10_pltfrm_pm_set_state(dev,
				CIF_ISP10_PM_STATE_SW_STNDBY, NULL)))
				cif_isp10_pltfrm_pr_err(dev,
					"power on failed\n");
			else
				cif_isp10_pltfrm_pr_info(dev,
					"switched on\n");
		} else if (!strncmp(token, "off", 3)) {
			if (IS_ERR_VALUE(cif_isp10_pltfrm_pm_set_state(dev,
				CIF_ISP10_PM_STATE_OFF, NULL)))
				cif_isp10_pltfrm_pr_err(dev,
					"power off failed\n");
			else
				cif_isp10_pltfrm_pr_info(dev,
					"switched off\n");
		} else {
			cif_isp10_pltfrm_pr_err(dev,
				"missing token, command format is 'power [off|on]'\n");
			return -EINVAL;
		}
	} else if (!strncmp(token, "set", 3)) {
		token = strsep(&strp, " ");
		if (IS_ERR_OR_NULL(token)) {
			cif_isp10_pltfrm_pr_err(dev,
				"missing token, command format is 'set register <hex addr>=<hex val>'\n");
			return -EINVAL;
		}
		if (!strncmp(token, "register", 8)) {
			u32 addr;
			u32 val;
			struct cif_isp10_pltfrm_data *pdata =
				dev->platform_data;
			token = strsep(&strp, "=");
			if (IS_ERR_VALUE(kstrtou32(token,
				16, &addr))) {
				cif_isp10_pltfrm_pr_err(dev,
					"malformed token, address must be a hexadecimal register address\n");
				return -EINVAL;
			}
			token = strp;
			if (IS_ERR_VALUE(kstrtou32(token,
				16, &val))) {
				cif_isp10_pltfrm_pr_err(dev,
					"malformed token, value must be a hexadecimal value\n");
				return -EINVAL;
			}
			iowrite32(val, pdata->base_addr + addr);
		} else {
			cif_isp10_pltfrm_pr_err(dev,
				"unknown command %s\n", token);
			return -EINVAL;
		}
	} else {
		cif_isp10_pltfrm_pr_err(dev,
			"unknown command %s\n", token);
		return -EINVAL;
	}
	return count;
}

static const struct file_operations cif_isp10_dbgfs_csi_fops = {
	.read = cif_isp10_dbgfs_csi_read,
	.write = cif_isp10_dbgfs_csi_write
};

static const struct file_operations cif_isp10_dbgfs_fops = {
	.write = cif_isp10_dbgfs_write
};

static const struct file_operations cif_isp10_dbgfs_sensor_fops = {
	.read = cif_isp10_dbgfs_sensor_read,
	.write = cif_isp10_dbgfs_sensor_write
};

#ifdef CONFIG_CIF_ISP10_REG_TRACE

static inline int cif_isp10_pltfrm_trace_printf(
	struct device *dev,
	const char *fmt,
	va_list args)
{
	int i;
	u32 rem_size;
	unsigned long flags = 0;

	if (!in_irq())
		spin_lock_irqsave(&cif_isp10_reg_trace.lock, flags);
	cif_isp10_reg_trace.internal = true;

	rem_size = cif_isp10_reg_trace.reg_trace_max_size -
		cif_isp10_reg_trace.reg_trace_write_pos;

	if (rem_size <= 0) {
		if (!in_irq())
			spin_unlock_irqrestore(
				&cif_isp10_reg_trace.lock, flags);
		cif_isp10_reg_trace.internal = false;
		return 0;
	}

	i = vsnprintf(cif_isp10_reg_trace.reg_trace +
		cif_isp10_reg_trace.reg_trace_write_pos,
		rem_size,
		fmt, args);
	if (i == rem_size) /* buffer full */
		i = 0;
	else if (i < 0)
		cif_isp10_pltfrm_pr_err(dev,
			"error writing trace buffer, error %d\n", i);
	else
		cif_isp10_reg_trace.reg_trace_write_pos += i;
	cif_isp10_reg_trace.internal = false;
	if (!in_irq())
		spin_unlock_irqrestore(&cif_isp10_reg_trace.lock, flags);

	return i;
}

inline int cif_isp10_pltfrm_rtrace_printf(
	struct device *dev,
	const char *fmt,
	...)
{
	va_list args;
	int i;

	va_start(args, fmt);
	i = cif_isp10_pltfrm_trace_printf(dev, fmt, args);
	va_end(args);

	return i;
}

inline int cif_isp10_pltfrm_ftrace_printf(
	struct device *dev,
	const char *fmt,
	...)
{
	va_list args;
	int i;

	if (!cif_isp10_reg_trace.ftrace ||
		cif_isp10_reg_trace.internal)
		return 0;

	va_start(args, fmt);
	i = cif_isp10_pltfrm_trace_printf(dev, fmt, args);
	va_end(args);

	return i;
}

static void cif_isp10_dbgfs_reg_trace_clear(
	struct device *dev)
{
	cif_isp10_reg_trace.reg_trace_write_pos = 0;
	cif_isp10_reg_trace.reg_trace_read_pos = 0;
}

static ssize_t cif_isp10_dbgfs_reg_trace_read(
	struct file *f,
	char __user *out,
	size_t count,
	loff_t *pos)
{
	ssize_t bytes;
	size_t available = cif_isp10_reg_trace.reg_trace_write_pos -
		cif_isp10_reg_trace.reg_trace_read_pos;
	size_t rem = count;

	cif_isp10_reg_trace.internal = true;

	if (!available)
		cif_isp10_reg_trace.reg_trace_read_pos = 0;

	while (rem && available) {
		bytes = simple_read_from_buffer(
			out + (count - rem), count,
			&cif_isp10_reg_trace.reg_trace_read_pos,
			cif_isp10_reg_trace.reg_trace,
			cif_isp10_reg_trace.reg_trace_write_pos);
		if (bytes < 0) {
			cif_isp10_pltfrm_pr_err(NULL,
				"buffer read failed with error %d\n",
				bytes);
			cif_isp10_reg_trace.internal = false;
			return bytes;
		}
		rem -= bytes;
		available = cif_isp10_reg_trace.reg_trace_write_pos -
			cif_isp10_reg_trace.reg_trace_read_pos;
	}

	cif_isp10_reg_trace.internal = false;
	return count - rem;
}

static ssize_t cif_isp10_dbgfs_reg_trace_write(
	struct file *f,
	const char __user *in,
	size_t count,
	loff_t *pos)
{
	ssize_t ret;
	char *strp = cif_isp10_dbgfs_buf;
	char *token;
	struct device *dev = f->f_inode->i_private;
	u32 max_size;
	unsigned long flags = 0;

	if (!in_irq())
		spin_lock_irqsave(&cif_isp10_reg_trace.lock, flags);

	cif_isp10_reg_trace.internal = true;

	if (count > CIF_ISP10_DBGFS_BUF_SIZE) {
		cif_isp10_pltfrm_pr_err(dev, "command line too long\n");
		return -EINVAL;
	}

	memset(cif_isp10_dbgfs_buf, 0, CIF_ISP10_DBGFS_BUF_SIZE);
	ret = simple_write_to_buffer(strp,
		CIF_ISP10_DBGFS_BUF_SIZE, pos, in, count);
	if (IS_ERR_VALUE(ret))
		goto err;

	token = strsep(&strp, " ");
	if (!strncmp(token, "clear", 5)) {
		cif_isp10_dbgfs_reg_trace_clear(dev);
		cif_isp10_pltfrm_pr_info(dev,
			"register trace buffer cleared\n");
	} else if (!strcmp(token, "size")) {
		token = strsep(&strp, " ");
		if (IS_ERR_OR_NULL(token)) {
			cif_isp10_pltfrm_pr_err(dev,
				"missing token, command format is 'size <num entries>'\n");
			ret = -EINVAL;
			goto err;
		}
		if (IS_ERR_VALUE(kstrtou32(token, 10,
				&max_size))) {
			cif_isp10_pltfrm_pr_err(dev,
				"wrong token format, <num entries> must be positive integer>'\n");
			ret = -EINVAL;
			goto err;
		}
		if (cif_isp10_reg_trace.reg_trace) {
			devm_kfree(dev, cif_isp10_reg_trace.reg_trace);
			cif_isp10_reg_trace.reg_trace = NULL;
		}
		cif_isp10_dbgfs_reg_trace_clear(dev);
		if (max_size > 0) {
			cif_isp10_reg_trace.reg_trace = devm_kzalloc(dev,
				max_size, GFP_KERNEL);
			if (!cif_isp10_reg_trace.reg_trace) {
				cif_isp10_pltfrm_pr_err(dev,
					"memory allocation failed\n");
				ret = -ENOMEM;
				goto err;
			}
			cif_isp10_reg_trace.reg_trace_max_size = max_size;
			cif_isp10_pltfrm_pr_info(dev,
				"register trace buffer size set to %d Byte\n",
				max_size);
		}
	} else if (!strncmp(token, "rtrace", 6)) {
		token = strsep(&strp, " ");
		if (IS_ERR_OR_NULL(token)) {
			cif_isp10_pltfrm_pr_err(dev,
				"missing token, command format is 'rtrace [off|on]'\n");
			ret = -EINVAL;
			goto err;
		}
		if (!strncmp(token, "on", 2)) {
			cif_isp10_reg_trace.rtrace = true;
			cif_isp10_pltfrm_pr_info(dev,
				"register trace enabled\n");
		} else if (!strncmp(token, "off", 3)) {
			cif_isp10_reg_trace.rtrace = false;
			cif_isp10_pltfrm_pr_info(dev,
				"register trace disabled\n");
		} else {
			cif_isp10_pltfrm_pr_err(dev,
				"missing token, command format is 'rtrace [off|on]'\n");
			ret = -EINVAL;
			goto err;
		}
	} else if (!strncmp(token, "ftrace", 6)) {
		token = strsep(&strp, " ");
		if (IS_ERR_OR_NULL(token)) {
			cif_isp10_pltfrm_pr_err(dev,
				"missing token, command format is 'ftrace [off|on]'\n");
			ret = -EINVAL;
			goto err;
		}
		if (!strncmp(token, "on", 2)) {
			cif_isp10_reg_trace.ftrace = true;
			cif_isp10_pltfrm_pr_info(dev,
				"function trace enabled\n");
		} else if (!strncmp(token, "off", 3)) {
			cif_isp10_reg_trace.ftrace = false;
			cif_isp10_pltfrm_pr_info(dev,
				"function trace disabled\n");
		} else {
			cif_isp10_pltfrm_pr_err(dev,
				"missing token, command format is 'ftrace [off|on]'\n");
			ret = -EINVAL;
			goto err;
		}
	} else {
		cif_isp10_pltfrm_pr_err(dev, "unknown command %s\n", token);
		ret = -EINVAL;
		goto err;
	}
	cif_isp10_reg_trace.internal = false;
	if (!in_irq())
		spin_unlock_irqrestore(&cif_isp10_reg_trace.lock, flags);
	return count;
err:
	cif_isp10_reg_trace.internal = false;
	if (!in_irq())
		spin_unlock_irqrestore(&cif_isp10_reg_trace.lock, flags);
	return ret;
}

static const struct file_operations
cif_isp10_dbgfs_reg_trace_fops = {
	.read = cif_isp10_dbgfs_reg_trace_read,
	.write = cif_isp10_dbgfs_reg_trace_write
};

#endif
#endif

static irqreturn_t cif_isp10_pltfrm_irq_handler(int irq, void *cntxt)
{
	unsigned int i, mis_val;
	int ret;
	struct device *dev = cntxt;
	struct cif_isp10_pltfrm_data *pdata =
		dev_get_platdata(dev);
	void *cif_isp10_dev = dev_get_drvdata(dev);

	if (irq != pdata->irq)
		return IRQ_NONE;

	for (i = 0; i < ARRAY_SIZE(pdata->irq_handlers); i++) {
		if (IS_ERR_VALUE(pdata->irq_handlers[i].mis))
			break;

		if (IS_ERR_OR_NULL(pdata->irq_handlers[i].isr)) {
			cif_isp10_pltfrm_pr_err(NULL,
				"ISR for IRQ #%d not set\n", irq);
			break;
		}

		mis_val = cif_ioread32(pdata->base_addr +
			pdata->irq_handlers[i].mis);
		if (mis_val == 0)
			continue;

		ret = pdata->irq_handlers[i].isr(mis_val, cif_isp10_dev);
		if (IS_ERR_VALUE(ret)) {
			cif_isp10_pltfrm_pr_err(NULL,
				"ISR for IRQ #%d failed with error %d\n",
				irq, ret);
		}
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

const char *cif_isp10_pltfrm_pm_state_string(
	enum cif_isp10_pm_state pm_state)
{
	switch (pm_state) {
	case CIF_ISP10_PM_STATE_OFF:
		return "CIF_ISP10_PM_STATE_OFF";
	case CIF_ISP10_PM_STATE_SUSPENDED:
		return "CIF_ISP10_PM_STATE_SUSPENDED";
	case CIF_ISP10_PM_STATE_SW_STNDBY:
		return "CIF_ISP10_PM_STATE_SW_STNDBY";
	case CIF_ISP10_PM_STATE_STREAMING:
		return "CIF_ISP10_PM_STATE_STREAMING";
	default:
		return "PM_STATE_UNKNOWN";
	}
}

inline void cif_isp10_pltfrm_write_reg(
	struct device *dev,
	u32 data,
	CIF_ISP10_PLTFRM_MEM_IO_ADDR addr)
{
	iowrite32(data, addr);
#ifdef CONFIG_CIF_ISP10_REG_TRACE
	{
		unsigned long flags = 0;

		if (!in_irq())
			spin_lock_irqsave(&cif_isp10_reg_trace.lock, flags);
		cif_isp10_reg_trace.internal = true;
		if (((cif_isp10_reg_trace.reg_trace_write_pos +
			(20 * sizeof(char))) <
			cif_isp10_reg_trace.reg_trace_max_size) &&
			cif_isp10_reg_trace.rtrace) {
			int bytes =
				sprintf(cif_isp10_reg_trace.reg_trace +
					cif_isp10_reg_trace.reg_trace_write_pos,
					"%04x %08x\n",
					addr - cif_isp10_reg_trace.base_addr,
					data);
			if (bytes > 0)
				cif_isp10_reg_trace.reg_trace_write_pos +=
					bytes;
			else
				cif_isp10_pltfrm_pr_err(dev,
					"error writing trace buffer, error %d\n",
					bytes);
		}
		cif_isp10_reg_trace.internal = false;
		if (!in_irq())
			spin_unlock_irqrestore(
				&cif_isp10_reg_trace.lock, flags);
	}
#endif
}

inline void cif_isp10_pltfrm_write_reg_OR(
	struct device *dev,
	u32 data,
	CIF_ISP10_PLTFRM_MEM_IO_ADDR addr)
{
	cif_isp10_pltfrm_write_reg(dev,
		(ioread32(addr) | data), addr);
}

inline void cif_isp10_pltfrm_write_reg_AND(
	struct device *dev,
	u32 data,
	CIF_ISP10_PLTFRM_MEM_IO_ADDR addr)
{
	cif_isp10_pltfrm_write_reg(dev,
		(ioread32(addr) & data), addr);
}

inline u32 cif_isp10_pltfrm_read_reg(
	struct device *dev,
	CIF_ISP10_PLTFRM_MEM_IO_ADDR addr)
{
	return ioread32(addr);
}

int cif_isp10_pltfrm_dev_init(
	struct cif_isp10_device *cif_isp10_dev,
	struct device **_dev,
	void __iomem **reg_base_addr)
{
	int ret;
	struct cif_isp10_pltfrm_data *pdata;
	struct device *dev = *_dev;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct resource *res;
	void __iomem *base_addr;
	unsigned int i, irq;

	dev_set_drvdata(dev, cif_isp10_dev);
	cif_isp10_dev->dev = dev;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		cif_isp10_pltfrm_pr_err(dev,
			"could not allocate memory for platform data\n");
		ret = -ENOMEM;
		goto err;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "register");
	if (!res) {
		cif_isp10_pltfrm_pr_err(NULL,
			"platform_get_resource_byname failed\n");
		ret = -ENODEV;
		goto err;
	}
	base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(base_addr)) {
		cif_isp10_pltfrm_pr_err(NULL, "devm_ioremap_resource failed\n");
		if (IS_ERR(base_addr))
			ret = PTR_ERR(base_addr);
		else
			ret = -ENODEV;
	}
	*reg_base_addr = base_addr;
	pdata->base_addr = base_addr;

	irq = platform_get_irq_byname(pdev, "cif_isp10_irq");
	if (IS_ERR_VALUE(irq)) {
		ret = irq;
		cif_isp10_pltfrm_pr_err(NULL,
		"platform_get_irq_byname failed\n");
		goto err;
	}

	ret = devm_request_threaded_irq(dev,
			irq,
			cif_isp10_pltfrm_irq_handler,
			NULL,
			0,
			dev_driver_string(dev),
			dev);
	if (IS_ERR_VALUE(ret)) {
		cif_isp10_pltfrm_pr_err(NULL,
		"devm_request_threaded_irq failed\n");
		goto err;
	}
	pdata->irq = irq;

	pdata->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR_OR_NULL(pdata->pinctrl)) {
		pdata->pins_default = pinctrl_lookup_state(pdata->pinctrl,
			PINCTRL_STATE_DEFAULT);
		if (IS_ERR(pdata->pins_default))
			cif_isp10_pltfrm_pr_err(dev,
						"could not get default pinstate\n");

		pdata->pins_sleep = pinctrl_lookup_state(pdata->pinctrl,
			PINCTRL_STATE_SLEEP);
		if (IS_ERR(pdata->pins_sleep))
			cif_isp10_pltfrm_pr_warn(dev,
						"could not get pins_sleep pinstate\n");

		pdata->pins_inactive = pinctrl_lookup_state(pdata->pinctrl,
			"inactive");
		if (IS_ERR(pdata->pins_inactive))
			cif_isp10_pltfrm_pr_warn(dev,
						"could not get pins_inactive pinstate\n");

		if (!IS_ERR_OR_NULL(pdata->pins_default))
			pinctrl_select_state(pdata->pinctrl,
				pdata->pins_default);
	}

	for (i = 0; i < ARRAY_SIZE(pdata->irq_handlers); i++)
		pdata->irq_handlers[i].mis = -EINVAL;

	dev->platform_data = pdata;

	INIT_LIST_HEAD(&pdata->csi0_configs);
	INIT_LIST_HEAD(&pdata->csi1_configs);

#ifndef CONFIG_DEBUG_FS
	pdata->dbgfs.dir = debugfs_create_dir("cif_isp10", NULL);
	pdata->dbgfs.csi0_file = debugfs_create_file(
		"csi-0",
		0644,
		pdata->dbgfs.dir,
		dev,
		&cif_isp10_dbgfs_csi_fops);
	pdata->dbgfs.csi1_file = debugfs_create_file(
		"csi-1",
		0644,
		pdata->dbgfs.dir,
		dev,
		&cif_isp10_dbgfs_csi_fops);
	pdata->dbgfs.cif_isp10_file = debugfs_create_file(
		"cif_isp20",
		0200,
		pdata->dbgfs.dir,
		dev,
		&cif_isp10_dbgfs_fops);
	pdata->dbgfs.cif_isp10_file = debugfs_create_file(
		"sensor",
		0644,
		pdata->dbgfs.dir,
		dev,
		&cif_isp10_dbgfs_sensor_fops);
#ifdef CONFIG_CIF_ISP10_REG_TRACE
	pdata->dbgfs.reg_trace_file = debugfs_create_file(
		"reg_trace",
		0644,
		pdata->dbgfs.dir,
		dev,
		&cif_isp10_dbgfs_reg_trace_fops);
	spin_lock_init(&cif_isp10_reg_trace.lock);
	cif_isp10_reg_trace.reg_trace = NULL;
	cif_isp10_dbgfs_reg_trace_clear(dev);
	cif_isp10_reg_trace.reg_trace_max_size = 0;
	cif_isp10_reg_trace.base_addr = base_addr;
	cif_isp10_reg_trace.rtrace = true;
	cif_isp10_reg_trace.ftrace = false;
	cif_isp10_reg_trace.internal = false;
#endif
#endif

	return 0;
err:
	cif_isp10_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	if (!IS_ERR_OR_NULL(pdata))
		devm_kfree(dev, pdata);
	return ret;
}

int cif_isp10_pltfrm_soc_init(
	struct cif_isp10_device *cif_isp10_dev,
	struct pltfrm_soc_cfg *soc_cfg)
{
	struct pltfrm_soc_cfg_para cfg_para;
	struct device *dev = cif_isp10_dev->dev;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct pltfrm_soc_init_para init_para;
	int ret = 0;

	if (!IS_ERR_OR_NULL(soc_cfg) && !IS_ERR_OR_NULL(soc_cfg->soc_cfg)) {
		cfg_para.cmd = PLTFRM_SOC_INIT;
		cfg_para.cfg_para = &init_para;
		init_para.pdev = pdev;
		init_para.isp_base = cif_isp10_dev->config.base_addr;
		ret = soc_cfg->soc_cfg(&cfg_para);
		if (ret == 0)
			cif_isp10_dev->soc_cfg = soc_cfg;
	}

	return ret;
}

int cif_isp10_pltfrm_mipi_dphy_config(
	struct cif_isp10_device *cif_isp10_dev)
{
	struct pltfrm_soc_cfg_para cfg_para;
	struct pltfrm_soc_cfg *soc_cfg;
	int ret = 0;

	soc_cfg = cif_isp10_dev->soc_cfg;
	if (!IS_ERR_OR_NULL(soc_cfg) &&
		!IS_ERR_OR_NULL(soc_cfg->soc_cfg)) {
		cfg_para.cmd =
			PLTFRM_MIPI_DPHY_CFG;
		cfg_para.cfg_para =
			(void *)(&cif_isp10_dev->config.cam_itf.cfg.mipi);
		ret = soc_cfg->soc_cfg(&cfg_para);
	}

	return ret;
}

int cif_isp10_pltfrm_pm_set_state(
	struct device *dev,
	enum cif_isp10_pm_state pm_state)
{
	int ret;
	struct cif_isp10_device *cif_isp10_dev = dev_get_drvdata(dev);
	struct pltfrm_soc_cfg *soc_cfg = cif_isp10_dev->soc_cfg;
	struct pltfrm_soc_cfg_para cfg_para;

	switch (pm_state) {
	case CIF_ISP10_PM_STATE_OFF:
	case CIF_ISP10_PM_STATE_SUSPENDED:
		cfg_para.cmd = PLTFRM_CLKDIS;
		cfg_para.cfg_para = NULL;
		ret = soc_cfg->soc_cfg(&cfg_para);
		break;
	case CIF_ISP10_PM_STATE_SW_STNDBY:
	case CIF_ISP10_PM_STATE_STREAMING:
		cfg_para.cmd = PLTFRM_CLKEN;
		cfg_para.cfg_para = NULL;
		ret = soc_cfg->soc_cfg(&cfg_para);
		break;
	default:
		cif_isp10_pltfrm_pr_err(dev,
			"unknown or unsupported PM state %d\n", pm_state);
		return -EINVAL;
	}

	if (IS_ERR_VALUE(ret))
		cif_isp10_pltfrm_pr_err(dev,
			"setting pm state to %s failed with error %d\n",
			cif_isp10_pltfrm_pm_state_string(pm_state), ret);
	else
		cif_isp10_pltfrm_pr_dbg(dev,
			"successfully changed pm state to %s\n",
			cif_isp10_pltfrm_pm_state_string(pm_state));
	return ret;
}

int cif_isp10_pltfrm_g_interface_config(
	struct cif_isp10_img_src *img_src,
	struct pltfrm_cam_itf *cam_itf)
{
	int ret = 0;

	ret = cif_isp10_img_src_ioctl(img_src,
			PLTFRM_CIFCAM_G_ITF_CFG, (void *)cam_itf);
	if (IS_ERR_VALUE(ret)) {
		cif_isp10_pltfrm_pr_err(
			dev,
			"cif_isp10_img_src_ioctl PLTFRM_CIFCAM_G_ITF_CFG failed!\n");
		return ret;
	}
	return 0;
}

int cif_isp10_pltfrm_pinctrl_set_state(
	struct device *dev,
	enum cif_isp10_pinctrl_state pinctrl_state)
{
	int ret = 0;
	struct cif_isp10_pltfrm_data *pdata = dev_get_platdata(dev);

	cif_isp10_pltfrm_pr_dbg(dev,
		"set pinctrl state to %d\n", pinctrl_state);

	if (!pdata) {
		cif_isp10_pltfrm_pr_err(dev,
			"unable to retrieve CIF platform data\n");
		ret = -EINVAL;
		goto err;
	}
	if (IS_ERR_OR_NULL(pdata->pinctrl))
		return 0;

	switch (pinctrl_state) {
	case CIF_ISP10_PINCTRL_STATE_SLEEP:
		if (!IS_ERR_OR_NULL(pdata->pins_sleep))
			ret = pinctrl_select_state(pdata->pinctrl,
				pdata->pins_sleep);
		break;
	case CIF_ISP10_PINCTRL_STATE_ACTIVE:
	case CIF_ISP10_PINCTRL_STATE_DEFAULT:
		if (!IS_ERR_OR_NULL(pdata->pins_default))
			ret = pinctrl_select_state(pdata->pinctrl,
				pdata->pins_default);
		break;
	case CIF_ISP10_PINCTRL_STATE_INACTIVE:
		if (!IS_ERR_OR_NULL(pdata->pins_inactive))
			ret = pinctrl_select_state(pdata->pinctrl,
				pdata->pins_inactive);
		break;
	default:
		cif_isp10_pltfrm_pr_err(dev,
			"unknown or unsupported pinctrl state %d\n",
			pinctrl_state);
		ret = -EINVAL;
		goto err;
	}

	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_isp10_pltfrm_pr_err(dev, "failed with error %d\n", ret);
	return ret;
}

int cif_isp10_pltfrm_irq_register_isr(
	struct device *dev,
	unsigned int mis,
	int (*isr)(unsigned int mis, void *cntxt),
	void *cntxt)
{
	int ret = 0;
	unsigned int i;
	int slot = -EINVAL;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct cif_isp10_pltfrm_data *pdata =
		dev_get_platdata(&pdev->dev);
	bool skip_request_irq = false;
	const char *irq_name;

	switch (mis) {
	case CIF_MIPI_MIS:
		irq_name = "CIF_ISP10_MIPI_IRQ";
		break;
	case CIF_ISP_MIS:
		irq_name = "CIF_ISP10_ISP_IRQ";
		break;
	case CIF_MI_MIS:
		irq_name = "CIF_ISP10_MI_IRQ";
		break;
	default:
		cif_isp10_pltfrm_pr_err(dev,
			"unknown or unsupported IRQ %d\n", mis);
		ret = -EINVAL;
		goto err;
	}
	cif_isp10_pltfrm_pr_dbg(dev,
		"registering ISR for IRQ %s\n", irq_name);

	for (i = 0; i < ARRAY_SIZE(pdata->irq_handlers); i++) {
		if (IS_ERR_VALUE(slot) &&
			IS_ERR_VALUE(pdata->irq_handlers[i].mis))
			slot = (int)i;
		if (pdata->irq_handlers[i].mis == mis) {
			cif_isp10_pltfrm_pr_dbg(dev,
				"overwriting ISR for IRQ %s\n", irq_name);
			slot = (int)i;
			skip_request_irq = true;
			break;
		}
	}
	if (IS_ERR_VALUE(slot)) {
		if (!isr)
			return 0;
		cif_isp10_pltfrm_pr_err(dev,
			"cannot register ISR for IRQ %s, too many ISRs already registered\n",
			irq_name);
		ret = -EFAULT;
		goto err;
	}
	pdata->irq_handlers[slot].isr = isr;
	if (!isr) {
		pdata->irq_handlers[slot].mis = -EINVAL;
		skip_request_irq = true;
	} else {
		pdata->irq_handlers[slot].mis = mis;
	}

	return 0;
err:
	cif_isp10_pltfrm_pr_err(dev, "failed with error %d\n", ret);
	return ret;
}

const char *cif_isp10_pltfrm_get_device_type(
	struct device *dev)
{
	return dev->of_node->type;
}

const char *cif_isp10_pltfrm_dev_string(
	struct device *dev)
{
	return dev_driver_string(dev);
}

int cif_isp10_pltfrm_get_img_src_device(
	struct device *dev,
	struct cif_isp10_img_src **img_src_array,
	unsigned int array_len)
{
	struct device_node *node = NULL;
	struct device_node *camera_list_node = NULL;
	struct i2c_client *client = NULL;
	int ret = 0;
	int index, size = 0;
	const __be32 *phandle;
	int num_cameras = 0;
	struct cif_isp10_device *cif_isp10_dev = dev_get_drvdata(dev);

	node = of_node_get(dev->of_node);
	if (IS_ERR_OR_NULL(node)) {
		dev_err(dev, "Unable to obtain CIF device node\n");
		ret = -EEXIST;
		goto err;
	}

	phandle = of_get_property(node,
		"rockchip,camera-modules-attached", &size);
	if (IS_ERR_OR_NULL(phandle)) {
		cif_isp10_pltfrm_pr_err(dev,
			"no camera-modules-attached'\n");
			ret = -EINVAL;
			goto err;
	}

	for (index = 0; index < size / sizeof(*phandle); index++) {
		camera_list_node = of_parse_phandle(node,
			"rockchip,camera-modules-attached", index);
		of_node_put(node);
		if (IS_ERR_OR_NULL(camera_list_node)) {
			cif_isp10_pltfrm_pr_err(dev,
				"invalid index %d for property 'rockchip,camera-modules-attached'\n",
				index);
				ret = -EINVAL;
				goto err;
		}

		if (!strcmp(camera_list_node->type,
					"v4l2-i2c-subdev")) {
			client = of_find_i2c_device_by_node(
				camera_list_node);
			of_node_put(camera_list_node);
			if (IS_ERR_OR_NULL(client)) {
				cif_isp10_pltfrm_pr_err(dev,
					"could not get camera i2c client, maybe not yet created, deferring device probing...\n");
				continue;
			}
		} else {
			cif_isp10_pltfrm_pr_dbg(dev,
				"device of type %s not supported\n",
				camera_list_node->type);
			of_node_put(camera_list_node);
			continue;
		}

		img_src_array[num_cameras] =
			cif_isp10_img_src_to_img_src(
				&client->dev,
				cif_isp10_dev->soc_cfg);
		if (!IS_ERR_OR_NULL(img_src_array[num_cameras])) {
			cif_isp10_pltfrm_pr_info(dev,
				"%s attach to cif isp10 img_src_array[%d]\n",
				cif_isp10_img_src_g_name(
					img_src_array[num_cameras]),
				num_cameras);
			num_cameras++;
			if (num_cameras >= array_len) {
				cif_isp10_pltfrm_pr_err(dev,
					"cif isp10 isn't support > %d 'camera modules attached'\n",
					array_len);
				break;
			}
		} else {
			continue;
		}
	}

	return num_cameras;
err:
	dev_err(dev, "failed with error %d\n", ret);
	if (!IS_ERR_OR_NULL(client))
		put_device(&client->dev);
	if (!IS_ERR_OR_NULL(camera_list_node))
		of_node_put(camera_list_node);
	return ret;
}

void cif_isp10_pltfrm_dev_release(
	struct device *dev)
{
#ifndef CONFIG_DEBUG_FS
	{
		struct cif_isp10_pltfrm_data *pdata =
			dev->platform_data;
		debugfs_remove(pdata->dbgfs.csi0_file);
		debugfs_remove(pdata->dbgfs.csi1_file);
		debugfs_remove_recursive(pdata->dbgfs.dir);
	}
#endif
}

