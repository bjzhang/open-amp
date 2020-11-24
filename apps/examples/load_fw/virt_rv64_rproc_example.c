/*
 * RV64 Virt APU life cycle management remoteproc example implementation
 *
 * Copyright(c) 2020 Bamvor Jian ZHANG
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <sysfs/libsysfs.h>
#include <common.h>
#include <metal/device.h>
#include <string.h>		//for strerror

#define DEV_BUS_NAME	"platform"
#define RPROC_DEV_NAME  "90200000.rproc"
static struct metal_io_region *rproc_io;	/**< pointer to rproc i/o region */
static struct metal_device *rproc_dev;		/**< pointer to rproc device */
struct remoteproc_mem rproc_mem;		/**< rproc memory */

static struct remoteproc *apu_rproc_init(struct remoteproc *rproc,
				 struct remoteproc_ops *ops, void *arg)
{
	struct rproc_priv *priv;
	int cpu_id = *((int *)arg);
	struct metal_device *dev;
	metal_phys_addr_t rproc_pa;
	int ret = -1;

	if(cpu_id < RV_NODE_APU_0 || cpu_id > RV_NODE_APU_N) {
		LPERROR("%s: invalid node id: %d \n\r",__func__, cpu_id);
		return NULL;
	}

	LPRINTF("%s: node id: %d\n\r", __func__, cpu_id);
	priv = metal_allocate_memory(sizeof(*priv));
	if (!priv)
		return NULL;

	memset(priv, 0, sizeof(*priv));
	priv->rproc = rproc;
	priv->cpu_id = cpu_id;
	priv->rproc->ops = ops;
	metal_list_init(&priv->rproc->mems);
	priv->rproc->priv = priv;
	rproc->state = RPROC_READY;
	/* Get remoteproc device */
	ret = metal_device_open(DEV_BUS_NAME, RPROC_DEV_NAME,
				&dev);
	if (ret) {
		LPERROR("ERROR: failed to open shm device: %d.\r\n",
			ret);
		goto err_open;
	}
	LPRINTF("Successfully open rproc device.\r\n");
	rproc_dev = dev;
	rproc_io = metal_device_io_region(dev, 0);
	if (!rproc_io)
		goto err_get_io;

	rproc_pa = metal_io_phys(rproc_io, 0);
	remoteproc_init_mem(&rproc_mem, "rproc", rproc_pa, rproc_pa,
			    metal_io_region_size(rproc_io),
			    rproc_io);
	remoteproc_add_mem(rproc, &rproc_mem);
	LPRINTF("Successfully added rproc shared memory\r\n");
	return priv->rproc;

err_get_io:
	metal_device_close(rproc_dev);
err_open:
	return NULL;
}

static void apu_rproc_remove(struct remoteproc *rproc)
{
	struct rproc_priv *priv;

	priv = (struct rproc_priv *)rproc->priv;
	metal_free_memory(priv);
}

static void *apu_rproc_mmap(struct remoteproc *rproc,
		    metal_phys_addr_t *pa, metal_phys_addr_t *da,
		    size_t size, unsigned int attribute,
		    struct metal_io_region **io)
{
	metal_phys_addr_t lpa, lda;
	struct metal_io_region *tmpio;

	(void)attribute;
	(void)size;
	(void)rproc;

	lpa = *pa;
	lda = *da;

	if (lpa == METAL_BAD_PHYS && lda == METAL_BAD_PHYS)
		return NULL;
	if (lpa == METAL_BAD_PHYS)
		lpa = lda;
	if (lda == METAL_BAD_PHYS)
		lda = lpa;
	tmpio = rproc_io;
	if (!tmpio)
		return NULL;

	*pa = lpa;
	*da = lda;
	if (io)
		*io = tmpio;
	return metal_io_phys_to_virt(tmpio, lpa);
}

/**
  * "echo 0 >   /sys/devices/system/cpu/cpu1/online"
  * name = cpu1/online
  * value = 0
  */
static int write_cpuhp_cfg(char *name, char *value)
{
	char sysfs_path[SYSFS_PATH_MAX];
	char path[SYSFS_PATH_MAX];
	struct sysfs_attribute *attr = NULL;
	int ret;

	if (!name || !value)
		return -EINVAL;

	ret = sysfs_get_mnt_path(sysfs_path, sizeof(sysfs_path));
	if (ret) {
		LPERROR("Failed to get sysfs path\n");
		return ret;
	}
	ret = snprintf(path, sizeof(path), "%s/devices/system/cpu/%s",
		       sysfs_path, name);
	if (ret >= (int)sizeof(path)) {
		return -EOVERFLOW;
	}

	attr = sysfs_open_attribute(path);
	if (!attr) {
		LPERROR("open cpu hotplug attribute %s failed, %s", path,
			strerror(errno));
		return -errno;
	}

#ifdef DEBUG
	ret = sysfs_read_attribute(attr);
	if (ret)
		goto exit_close;

	LPRINTF("Current %s is %s\n", name, attr->value);
#endif /* #ifdef DEBUG */
	ret = sysfs_write_attribute(attr, value, strlen(value) + 1);
	if (ret) {
		LPERROR("write cpu hotplug attribute %s as %s failed, %s\n",
			path, value, strerror(errno));
		goto exit_close;
	}

	LPRINTF("Set %s to %s.\n", name, value);
exit_close:
	sysfs_close_attribute(attr);
	return ret;
}

static int write_openamp_cfg(char *name, char *value)
{
	char sysfs_path[SYSFS_PATH_MAX];
	char path[SYSFS_PATH_MAX];
	struct sysfs_attribute *attr = NULL;
	int ret;

	if (!name || !value)
		return -EINVAL;

	ret = sysfs_get_mnt_path(sysfs_path, sizeof(sysfs_path));
	if (ret) {
		LPERROR("Failed to get sysfs path\n");
		return ret;
	}
	ret = snprintf(path, sizeof(path), "%s/firmware/openamp/%s",
		       sysfs_path, name);
	if (ret >= (int)sizeof(path))
		return -EOVERFLOW;

	attr = sysfs_open_attribute(path);
	if (!attr)
		return -errno;

#ifdef DEBUG
	ret = sysfs_read_attribute(attr);
	if (ret)
		goto exit_close;

	LPRINTF("Current %s is %s\n", name, attr->value);
#endif /* #ifdef DEBUG */
	ret = sysfs_write_attribute(attr, value, strlen(value) + 1);
	if (ret)
		goto exit_close;

	LPRINTF("Set %s to %s.\n", name, value);
exit_close:
	sysfs_close_attribute(attr);
	return ret;
}

static int apu_rproc_start(struct remoteproc *rproc)
{
	struct rproc_priv *priv;
	char cpuhp[] = "cpuAA/online";
	char hartid[3];		//The max cores in qemu is 8
	char paddr[17];		//64bit address + \0
	int ret;

	priv = rproc->priv;
	ret = snprintf(cpuhp, sizeof(cpuhp), "cpu%d/online", priv->cpu_id);
	if (ret >= (int)sizeof(cpuhp)) {
		ret = -EOVERFLOW;
		goto err;
	}
	//It will failed if such cpu is not managed by Linux
	ret = write_cpuhp_cfg(cpuhp, "0");

	ret = snprintf(hartid, sizeof(hartid), "%d", priv->cpu_id);
	if (ret >= (int)sizeof(hartid)) {
		ret = -EOVERFLOW;
		goto err;
	}
	ret = write_openamp_cfg("hartid", hartid);
	if (ret)
		goto err;

	ret = snprintf(paddr, sizeof(paddr), "0x%lx", rproc->bootaddr);
	if (ret >= (int)sizeof(paddr)) {
		ret = -EOVERFLOW;
		goto err;
	}

	ret = write_openamp_cfg("paddr", paddr);
	if (ret)
		goto err;

	ret = write_openamp_cfg("state", "start");
	if (ret)
		goto err;

err:
	if (ret < 0)
		LPERROR("remote proc start failed with error: %s\n",
			strerror(ret));

	return ret;
}

static int apu_rproc_stop(struct remoteproc *rproc)
{
	int ret;

	(void)rproc;
	ret = write_openamp_cfg("state", "stop");
	if (ret)
		return ret;

	return 0;
}

static int apu_rproc_shutdown(struct remoteproc *rproc)
{
	struct remoteproc_mem *mem;
	struct metal_list *node;

	/* Delete all the registered remoteproc memories */
	metal_list_for_each(&rproc->mems, node) {
		struct metal_list *tmpnode;

		mem = metal_container_of(node, struct remoteproc_mem, node);
		tmpnode = node;

		node = tmpnode->prev;
		metal_list_del(tmpnode);
	}
	return 0;
}

struct remoteproc_ops virt_rv64_rproc_ops = {
    .init = apu_rproc_init,
    .remove = apu_rproc_remove,
    .start = apu_rproc_start,
    .stop = apu_rproc_stop,
    .shutdown = apu_rproc_shutdown,
    .mmap = apu_rproc_mmap,
};
