/*
 * Load firmware example
 *
 * Copyright(c) 2018 Xilinx Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_info.h>
#include <common.h>
#ifdef WITH_RV64
#include <stdlib.h>
#include <unistd.h>		//for sleep()
#include <signal.h>		//for sigaction()
#include <string.h>		//for strerror()
#include <errno.h>		//for errno
#endif /* #ifdef WITH_RV64 */

extern struct image_store_ops mem_image_store_ops;

struct mem_file {
	const void *base;
};

static struct mem_file image = {
	.base = (void *)0x3ED00000,
};

static volatile int exit_request = 0;

int load_exectuable_block(struct remoteproc *rproc,
			  struct image_store_ops *store_ops, void *store,
			  const char *img_path)
{
	int ret;

	if (rproc == NULL)
		return -EINVAL;
	/* Configure remoteproc to get ready to load executable */
	remoteproc_config(rproc, NULL);

	/* Load remoteproc executable */
	LPRINTF("Start to load executable with remoteproc_load() \r\n");
	ret = remoteproc_load(rproc, img_path, store, store_ops, NULL);
	if (ret) {
		LPRINTF("failed to load firmware\r\n");
		return ret;
	}
	/* Start the processor */
	ret = remoteproc_start(rproc);
	if (ret) {
		LPRINTF("failed to start processor\r\n");
		return ret;
	}
	LPRINTF("successfully started the processor\r\n");
#if !(defined(RPU_BOOT_LINUX) || defined(WITH_RV64))
#error should not compile!
#ifdef WITH_WFI
	/* ... */
	asm volatile("wfi");
#endif /* #ifdef WITH_WFI */
	LPRINTF("going to stop the processor\r\n");
	remoteproc_stop(rproc);
	/* application may want to do some cleanup before shutdown */
	LPRINTF("going to shutdown the processor\r\n");
	remoteproc_shutdown(rproc);
#endif /* #if !defined(RPU_BOOT_LINUX) || !defined(WITH_RV64) */
	return 0;
}

#ifndef RPU_BOOT_LINUX
int load_exectuable_noblock(struct remoteproc *rproc,
			     struct image_store_ops *store_ops, void *store,
			     const char *img_path)
{
	int ret;
	const void *img_data;
	void *img_info = NULL;
	metal_phys_addr_t pa;
	struct metal_io_region *io;
	size_t offset, noffset;
	size_t len, nlen, nmlen;
	unsigned char padding;

	if (rproc == NULL)
		return -EINVAL;
	/* Configure remoteproc to get ready to load executable */
	remoteproc_config(rproc, NULL);
	/* Load remoteproc executable */
	LPRINTF("Start to load executable with remoteproc_load() \r\n");
	ret = store_ops->open(store, img_path, &img_data);
	if (ret <= 0)
		return -EINVAL;
	offset = 0;
	len = (size_t)ret;
	do {
		nlen = 0;
		pa = METAL_BAD_PHYS;
		io = NULL;
		nmlen = 0;
		LPRINTF("%s, loading 0x%lx,0x%lx\r\n",
			 __func__, offset, len);
		ret = remoteproc_load_noblock(rproc, img_data, offset, len,
					      &img_info, &pa, &io, &noffset,
					      &nlen, &nmlen, &padding);
		if (ret) {
			LPERROR("failed to load executable, 0x%lx,0x%lx\r\n",
				offset, len);
			return ret;
		}
		if (nlen == 0)
			break;
		offset = noffset;
		len = nlen;
		ret = store_ops->load(store, noffset, nlen, &img_data, pa,
				      io, 1);
		if (ret != (int)nlen) {
			LPERROR("failed to load data to memory, 0x%lx,0x%lx\r\n",
				noffset, nlen);
			return ret;
		}
		if (nmlen > nlen && io != NULL) {
			/* pad the rest of the memory with 0 */
			size_t tmpoffset;

			tmpoffset = metal_io_phys_to_offset(io, pa + nlen);
			metal_io_block_set(io, tmpoffset, padding,
					   (nmlen - nlen));

		}
	} while(1);

	/* Start the processor */
	ret = remoteproc_start(rproc);
	if (ret) {
		LPRINTF("failed to start processor\r\n");
		return ret;
	}
	LPRINTF("successfully started the processor\r\n");
#ifdef WITH_WFI
	/* ... */
	asm volatile("wfi");
#endif /* `#ifdef WITH_WFI */
	LPRINTF("going to stop the processor\r\n");
	remoteproc_stop(rproc);
	/* application may want to do some cleanup before shutdown */
	LPRINTF("going to shutdown the processor\r\n");
	remoteproc_shutdown(rproc);
	return 0;
}
#endif /* RPU_BOOT_LINUX */

static void rproc_shutdown(int sig, siginfo_t *info, void *ucontext)
{
	(void)sig;
	(void)info;
	(void)ucontext;
	LPRINTF("shutdown reqeusted\n");
	exit_request = 1;
}

int install_sig(void)
{
	int ret = 0;

	struct sigaction sig = {
		.sa_sigaction = rproc_shutdown,
		.sa_flags = SA_RESTART || SA_SIGINFO
	};
	struct sigaction sigterm_old;
	struct sigaction sigint_old;

	sigemptyset(&sig.sa_mask);
	if (sigaction(SIGTERM, &sig, &sigterm_old) == -1) {
		ret = -errno;
		goto err;
	}
	if (sigaction(SIGINT, &sig, &sigint_old) == -1) {
		ret = -errno;
		goto err;
	}

err:
	return ret;
}

void waiting_for_exit(struct remoteproc *rproc)
{
	if (!rproc)
		return;

	while(!exit_request) {
		LPRINTF("waiting for remote proc shutdown request\n");
		sleep(1);
	}
	LPRINTF("going to stop the processor\r\n");
	remoteproc_stop(rproc);
	/* application may want to do some cleanup before shutdown */
	LPRINTF("going to shutdown the processor\r\n");
	remoteproc_shutdown(rproc);
}

int main(int argc, char *argv[])
{
	struct remoteproc *rproc = NULL;
	void *store = &image;
	unsigned int cpu_id;
	int ret;
	char *path = NULL;
	int i = 0;

#ifdef WITH_ZYNQMP
	(void)argc;
	(void)argv;
	cpu_id = LOAD_FW_TARGET;
#elif defined(WITH_RV64)
	if (argc > 2) {
		i++;
		path = argv[i];
	}
	if (argc > 1) {
		i++;
		cpu_id = atoi(argv[i]);
	} else {
		return -1;
	}
#endif /* #ifdef WITH_ZYNQMP */
	LPRINTF("Loading Exectuable Demo\n");
	rproc = app_init(cpu_id);
	if (!rproc) {
		LPERROR("app_init failed\r\n");
		return -1;
	}
	ret = load_exectuable_block(rproc, &mem_image_store_ops, store, path);
	if (ret < 0) {
		LPERROR("load_exectuable_block failed\r\n");
		/* Make sure the remote is shut down */
		remoteproc_shutdown(rproc);
		return -1;
	}
	ret = install_sig();
	if ( ret < 0) {
		LPERROR("install signal handler failed: %s\r\n", strerror(ret));
		/* Make sure the remote is shut down */
		remoteproc_shutdown(rproc);
		return -1;
	}
	waiting_for_exit(rproc);
#if !(defined(RPU_BOOT_LINUX) || defined(WITH_RV64))
#error should not compile!
	ret = load_exectuable_noblock(rproc, &mem_image_store_ops, store,
				      NULL);
	if (ret < 0) {
		LPERROR("load_exectuable_noblock failed\r\n");
		/* Make sure the remote is shut down */
		remoteproc_shutdown(rproc);
		return -1;
	}
#endif /* #if !defined(RPU_BOOT_LINUX) || !defined(WITH_RV64) */
	remoteproc_remove(rproc);
	return ret;
}
