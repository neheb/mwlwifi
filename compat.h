/* SPDX-License-Identifier: GPL-2.0-only */
/* Compatibility between the kernel/mac80211 versions the driver supports.
 * The minimum is 6.18 (OpenWrt builds against mac80211 backports, where
 * LINUX_VERSION_CODE is the base kernel's, e.g. 6.12 with backports 6.18).
 */

#ifndef _MWL_COMPAT_H_
#define _MWL_COMPAT_H_

#include <linux/version.h>

/* 7.1 moved action_code out of the per-action structs of ieee80211_mgmt */
#if LINUX_VERSION_CODE < KERNEL_VERSION(7, 1, 0)
#define MWL_ACTION_CODE(mgmt)	((mgmt)->u.action.u.addba_req.action_code)
#define MWL_ADDBA_REQ(mgmt)	((mgmt)->u.action.u.addba_req)
#else
#define MWL_ACTION_CODE(mgmt)	((mgmt)->u.action.action_code)
#define MWL_ADDBA_REQ(mgmt)	((mgmt)->u.action.addba_req)
#endif

#endif /* _MWL_COMPAT_H_ */
