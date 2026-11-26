/* SPDX-License-Identifier: GPL-2.0-only */
/* Compatibility between the kernel/mac80211 versions the driver supports.
 * The minimum is 6.18 (OpenWrt builds against mac80211 backports, where
 * LINUX_VERSION_CODE is the base kernel's, e.g. 6.12 with backports 6.18).
 */

#ifndef _MWL_COMPAT_H_
#define _MWL_COMPAT_H_

#include <linux/slab.h>
#include <linux/version.h>

/* kmalloc_obj() and kzalloc_obj() appeared in 7.0 (OpenWrt also carries
 * them in its 6.12 tree); only the form with the default GFP_KERNEL is
 * provided here.
 */
#ifndef kmalloc_obj
#define kmalloc_obj(P)	kmalloc(sizeof(P), GFP_KERNEL)
#endif
#ifndef kzalloc_obj
#define kzalloc_obj(P)	kzalloc(sizeof(P), GFP_KERNEL)
#endif

/* 7.1 moved action_code out of the per-action structs of ieee80211_mgmt */
#if LINUX_VERSION_CODE < KERNEL_VERSION(7, 1, 0)
#define MWL_ACTION_CODE(mgmt)	((mgmt)->u.action.u.addba_req.action_code)
#define MWL_ADDBA_REQ(mgmt)	((mgmt)->u.action.u.addba_req)
#else
#define MWL_ACTION_CODE(mgmt)	((mgmt)->u.action.action_code)
#define MWL_ADDBA_REQ(mgmt)	((mgmt)->u.action.addba_req)
#endif

#endif /* _MWL_COMPAT_H_ */
