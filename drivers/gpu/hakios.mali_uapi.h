// SPDX-License-Identifier: BSD-2-Clause-Patent
// The interface between the Mali-G610 driver of this repository (Mali, an IOService with an IOUserClient) and its only user, the
// libdrm stand-in the Vulkan driver (PanVK) is linked against on iOS (MaliMetal's darwin-drm). Plain C, shared by both sides.
//
// It is Panthor's uAPI (include/uapi/drm/panthor_drm.h) taken apart into calls that carry no user pointers: the shim walks the arrays of
// DRM_IOCTL_PANTHOR_VM_BIND / GROUP_CREATE / GROUP_SUBMIT itself and makes one call per element. Every call is IOConnectCallStructMethod
// with the "in" structure as the input and the "out" structure as the output (either may be absent).
//
// Memory is mapped with IOConnectMapMemory64(connection, type, task, &address, &size, kIOMapAnywhere | kIOMapWriteCombineCache):
//   type = a buffer object handle        the buffer, read/write, write-combined (the GPU is not cache coherent)
//   type = MALI_MAP_SYNCOBJ | group  the group's syncobj page: one { uint64_t seqno; uint32_t status, pad; } per queue, 16 bytes each,
//                                        read-only for the client; queue q's job number n is done when seqno >= n.
#ifndef MALI_UAPI_H
#define MALI_UAPI_H

#include <stdint.h>

// IOServiceMatching() matches the class name of the service (IOProviderClass), not the registry entry's name: the class is MaliService.
#define MALI_SERVICE "MaliService"
#define MALI_MAP_SYNCOBJ 0x80000000u
// IOConnectMapMemory64 type: the display's scanout buffer, writable (write-combined), for the GPU hand-off presenter (MALI_GOP). Only exists
// while the display driver runs in GPU mode (boot-arg sheng-fb-gpu=1); no process but the render server is granted it.
#define MALI_MAP_SCANOUT 0x40000000u
#define MALI_MAX_QUEUES 4

enum mali_selector {
  MALI_DEV_QUERY = 0,     // out: mali_dev_info
  MALI_VM_CREATE,         // in: mali_vm_create_in, out: mali_vm_create_out
  MALI_VM_DESTROY,        // in: mali_vm_destroy_in
  MALI_BO_CREATE,         // in: mali_bo_create_in, out: mali_bo_create_out
  MALI_BO_DESTROY,        // in: mali_bo_destroy_in
  MALI_VM_BIND,           // in: mali_vm_bind_in (one operation)
  MALI_GROUP_CREATE,      // in: mali_group_create_in, out: mali_group_create_out
  MALI_GROUP_DESTROY,     // in: mali_group_destroy_in
  MALI_GROUP_SUBMIT,      // in: mali_group_submit_in, out: mali_group_submit_out
  MALI_FENCE_WAIT,        // in: mali_fence_wait_in, out: mali_fence_wait_out
  MALI_HEAP_CREATE,       // in: mali_heap_create_in, out: mali_heap_create_out (a tiler heap in a VM)
  MALI_HEAP_DESTROY,      // in: mali_heap_destroy_in
  MALI_BO_CREATE_HOST,    // in: mali_bo_create_host_in, out: mali_bo_create_out (a buffer object over the caller's own pages)
  MALI_LOG,               // in: mali_log_in (a line for the kernel's console: the Metal plugin cannot reach a console from a sandboxed process)
  MALI_GOP,               // in: mali_gop_in, out: mali_gop_out (the display driver's GPU hand-off: mode, wait for a finished frame, complete it)
  MALI_SELECTOR_COUNT
};

// Panthor's drm_panthor_gpu_info followed by drm_panthor_csif_info, field for field, then what the DRM core and the driver version add.
struct mali_dev_info {
  uint32_t gpu_id, gpu_rev, csf_id, l2_features, tiler_features, mem_features, mmu_features, thread_features;
  uint32_t max_threads, thread_max_workgroup_size, thread_max_barrier_size, coherency_features;
  uint32_t texture_features[4];
  uint32_t as_present, pad0;
  uint64_t shader_present, l2_present, tiler_present;
  uint32_t core_features, pad1;
  uint32_t csg_slot_count, cs_slot_count, cs_reg_count, scoreboard_slot_count, unpreserved_cs_reg_count, pad2;
  uint64_t timestamp_frequency;
  uint32_t fw_version, abi_version;
};

struct mali_vm_create_in { uint64_t user_va_range; };    // 0: everything below the kernel's region
struct mali_vm_create_out { uint32_t vm, pad; uint64_t user_va_range; };
struct mali_vm_destroy_in { uint32_t vm, pad; };

struct mali_bo_create_in { uint64_t size; uint32_t flags, pad; };
struct mali_bo_create_out { uint32_t handle, pad; uint64_t size; };
struct mali_bo_destroy_in { uint32_t handle, pad; };
// VK_EXT_external_memory_host: the pages [ptr, ptr + size) of the caller (whole 4 KiB pages) are wired and become a buffer object that can be
// bound into a VM like any other. The CPU side stays the caller's own mapping (an IOSurface's, whatever its cache attributes); the GPU is not
// cache coherent, so the importer cleans and invalidates its CPU caches around the GPU's use. Such a BO has no IOConnectMapMemory64 view.
struct mali_log_in { char text[200]; };
// The display driver's GPU hand-off (GopFramebuffer NativeGOPWire.h, NGP_METHOD_*): instead of copying a finished frame into the scanout buffer with
// the CPU, it offers it to a privileged client, which has the GPU copy it (the scanout mapped writable: MALI_MAP_SCANOUT) and completes it.
//   op 0 (mode)     out.mode
//   op 2 (wait)     arg = timeout in ms (at most 1000); out.swap: the frame's pages are mapped into the caller at swap.address, bytes long;
//                   kIOReturnTimeout when there was none (the normal answer of an idle display)
//   op 3 (complete) arg = swap.sequence, status 0 = the copy is in the scanout (anything else: the display driver falls back to its own copy); no out
enum { MALI_GOP_MODE = 0, MALI_GOP_WAIT = 2, MALI_GOP_COMPLETE = 3 };
struct mali_gop_in { uint32_t op, status; uint64_t arg; };
struct mali_gop_mode { uint32_t version, format; uint64_t width, height, stride, bytes; };   // format 1: BGRX8888
struct mali_gop_swap { uint64_t sequence, address, bytes, stride; uint32_t width, height, format, reserved; };
union mali_gop_out { struct mali_gop_mode mode; struct mali_gop_swap swap; };
struct mali_bo_create_host_in { uint64_t ptr, size; uint32_t flags, pad; };

enum { MALI_BIND_MAP = 0, MALI_BIND_UNMAP = 1 };
enum { MALI_BIND_READONLY = 1u << 0, MALI_BIND_NOEXEC = 1u << 1, MALI_BIND_UNCACHED = 1u << 2 };   // as DRM_PANTHOR_VM_BIND_OP_MAP_*
struct mali_vm_bind_in {
  uint32_t vm, op;
  uint32_t bo, flags;               // map only
  uint64_t bo_offset, va, size;     // unmap uses va and size only
};

struct mali_group_create_in {
  uint32_t vm, queue_count;
  uint32_t ring_size[MALI_MAX_QUEUES];        // bytes, power of two, at least 4096
  uint8_t priority[MALI_MAX_QUEUES];
  uint64_t compute_core_mask, fragment_core_mask, tiler_core_mask;
  uint8_t max_compute_cores, max_fragment_cores, max_tiler_cores, group_priority;
  uint32_t pad;
};
struct mali_group_create_out { uint32_t group, csg_slot; };
struct mali_group_destroy_in { uint32_t group, pad; };

struct mali_group_submit_in { uint32_t group, queue; uint64_t cs_va; uint32_t cs_size, latest_flush; };
struct mali_group_submit_out { uint64_t seqno; };   // the job's number on that queue, counted from 1

struct mali_fence_wait_in { uint32_t group, queue; uint64_t seqno, timeout_us; };   // timeout_us 0: just look
struct mali_fence_wait_out { uint64_t done_seqno; uint32_t signaled, pad; };

// Panthor's TILER_HEAP_CREATE: chunks of chunk_size (a multiple of 4 KiB, 128 KiB..8 MiB) chained newest first in the VM's kernel region; the heap
// context (32 bytes in a 64 byte slot) is where the firmware keeps its state. No chunk is ever added: when the initial ones are used up the
// firmware's out-of-memory exception goes unanswered (the driver has no interrupts yet).
struct mali_heap_create_in { uint32_t vm, initial_chunk_count, chunk_size, max_chunks, target_in_flight, pad; };
struct mali_heap_create_out { uint32_t handle, pad; uint64_t ctx_va, first_chunk_va; };
struct mali_heap_destroy_in { uint32_t handle, pad; };

#endif
