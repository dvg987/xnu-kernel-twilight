// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// hakios.mali: the Mali-G310 (Valhall, CSF, arch 10.12) of the Amlogic S905X5M (S7D), the twilight box's GPU.
//
// This is Ken-u's Mali driver for the RK3588's Mali-G610 (github.com/Ken-u/Mali, Mali.cpp at 9dd949a, BSD-2-Clause-Patent; LICENSE
// there), whose CSF firmware loader, MMU, command-stream groups and Panthor-style user client are the same for every v10 GPU, with the
// RK3588's own parts replaced by the S7D's (Amlogic's common_drivers: meson-s7d.dtsi, s7d-clkc.h, drivers/clk/meson/s7d.c,
// reset-meson.c; gpu-aml's mali_kbase_runtime_pm.c):
//
//   power    no power domain: the GPU is on the always-on VDD_EE rail (s7d-pd.h has none for it)
//   clocks   CLKCTRL 0xfe000000: MALI_CLK_CTRL +0x15c and MALI_STACK_CLK_CTRL +0x160 (mali_0: sel 11:9, gate 8, div 6:0; mali_1 the
//            same at 27:25, 24, 22:16; bit 31 picks mali_1), sources 3 fclk_div2p5 800, 4 fclk_div3 666, 5 fclk_div4 500, 6 fclk_div5
//            400, 7 fclk_div7 285.7 MHz (the PLLs are the secure firmware's, running); the bus gate SYS_CLK_EN0_REG0 +0x44 bit 6
//   reset    RESET 0xfe002000, level register +0x44 (active low): bit 2 the GPU, bit 1 its APB; the bootloader leaves both released
//   GPU      0xfd000000 (8 MiB window); SPI 144 "GPU", 145 "MMU", 146 "JOB" (polled here, as in the original)
//   quirk    Amlogic's mali_hw_init(): PWR_KEY (0x50) = 0x2968a819, PWR_OVERRIDE1 (0x58) = 0x00200fff after the soft reset
//   firmware fw/mali_csffw.bin of the small image (CoreELEC's arm/mali/arch10.8/mali_csffw.bin, the v10 image), not built in
//
// Boot-args: mali=1  power, clocks, GPU_ID, L2 (M0);  mali=2  and the firmware and MCU (M1);  3.. as in the original
//            mali-clk=S  the clock source (3..7 above; default 6: 400 MHz, what Amlogic's Linux runs at)
#include <IOKit/IOLib.h>
#include <kern/thread_call.h>
#include <kern/clock.h>
#include <kern/cpu_number.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOService.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOUserClient.h>
#include <libkern/c++/OSMetaClass.h>
#include <libkern/OSKextLib.h>
#include <mach/kmod.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <stdint.h>
#include <string.h>

#include "hakios.mali_uapi.h"

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern "C" const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);   // (hakios.image.c)
// The display driver's GPU hand-off (drivers/GopFramebuffer/NativeGOPPresenter.cpp, linked with the display archive only): the render server's
// presenter reaches it through its connection to this driver, because the sandbox lets it open the accelerator and not the presenter's service.
// Weak fallbacks keep a kernel that links this driver without the display archive linkable; the display driver's strong definitions replace them.
extern "C" __attribute__((weak)) IOReturn native_gop_handoff_call(uint32_t, task_t, const void *, size_t, void *, size_t) { return kIOReturnUnsupported; }
extern "C" __attribute__((weak)) IOMemoryDescriptor *native_gop_handoff_memory(void) { return nullptr; }
// patches/xnu/evb10-thread-sample.patch (osfmk/kern/bsd_kern.c): whether a task's address range belongs to a file (a vnode pager). The weak fallback is for
// a kernel without that patch: nothing is then copied.
extern "C" __attribute__((weak)) boolean_t evb10_range_is_file_backed(task_t, uint64_t, uint64_t) { return 0; }

namespace {

constexpr uint64_t kClkPhys = 0xfe000000ULL, kResetPhys = 0xfe002000ULL;
constexpr uint64_t kGpuPhys = 0xfd000000ULL;
constexpr uint32_t kGpuMapSize = 0x200000;     // control block, job control, MMU address spaces, the doorbells (0x80000 + 0x10000 * id: slot 7 uses id 8 = 0x100000)
enum : uint32_t { kClkSysEn0 = 0x44, kClkMali = 0x15c, kClkMaliStack = 0x160, kResetLevel1 = 0x44 };
constexpr uint32_t kSysMaliGate = 1u << 6, kResetMali = 1u << 2, kResetMaliApb = 1u << 1;

// GPU registers (Panthor's panthor_regs.h)
enum : uint32_t {
  kGpuId = 0x0, kL2Features = 0x4, kCoreFeatures = 0x8, kTilerFeatures = 0xc, kMemFeatures = 0x10, kMmuFeatures = 0x14, kAsPresent = 0x18,
  kCsfId = 0x1c, kIntRaw = 0x20, kIntClear = 0x24, kIntMask = 0x28, kIntStat = 0x2c, kCmd = 0x30, kStatus = 0x34,
  kThreadMax = 0xa0, kThreadFeatures = 0xac, kShaderPresent = 0x100, kTilerPresent = 0x110, kL2Present = 0x120,
  kShaderReady = 0x140, kTilerReady = 0x150, kL2Ready = 0x160, kL2PwrOn = 0x1a0, kL2PwrTrans = 0x220,
  kCoherencyFeatures = 0x300, kStackPresent = 0xe00, kMcuControl = 0x700, kMcuStatus = 0x704,
  kPwrKey = 0x50, kPwrOverride1 = 0x58,
};
constexpr uint32_t kSoftReset = 1u | (1u << 8), kIrqResetCompleted = 1u << 8;

volatile uint32_t *gClk, *gReset, *gGpu;
unsigned gLevel;   // mali

inline uint32_t gpu(uint32_t off) { return gGpu[off / 4]; }
inline void gpuWrite(uint32_t off, uint32_t value) { gGpu[off / 4] = value; __asm__ volatile("dsb sy" ::: "memory"); }

// Poll with a time limit; true when the condition came true. Each step 10 microseconds.
template <typename Condition> bool waitUs(unsigned limit, Condition done) {
  for (unsigned t = 0; t <= limit; t += 10) {
    if (done()) return true;
    IODelay(10);
  }
  return false;
}

const char *clockName(uint32_t sel) {
  static const char *const names[8] = {"xtal", "gp1_pll", "?", "fclk_div2p5 (800 MHz)", "fclk_div3 (666 MHz)", "fclk_div4 (500 MHz)",
                                       "fclk_div5 (400 MHz)", "fclk_div7 (285 MHz)"};
  return names[sel & 7];
}

// The GPU's clock and its shader stacks' from one of the fixed PLL's dividers (mali_0, bit 31 clear), ungated; the bus clock ungated; both
// resets released. The vendor's Linux sets the rate first and then enables (its glue says the other order may not be ready).
bool setClocks() {
  unsigned source = 6;
  PE_parse_boot_argn("mali-clk", &source, sizeof source);
  if (source < 3 || source > 7) source = 6;
  printf("mali: clocks before: mali %#x stack %#x sys_en0 %#x; reset level %#x\n", gClk[kClkMali / 4], gClk[kClkMaliStack / 4], gClk[kClkSysEn0 / 4],
         gReset[kResetLevel1 / 4]);
  const uint32_t value = (source << 9) | (1u << 8);   // div 0: the source's own rate
  gClk[kClkMali / 4] = value;
  gClk[kClkMaliStack / 4] = value;
  gClk[kClkSysEn0 / 4] = gClk[kClkSysEn0 / 4] | kSysMaliGate;
  if ((gReset[kResetLevel1 / 4] & (kResetMali | kResetMaliApb)) != (kResetMali | kResetMaliApb)) {
    printf("mali: the GPU was held in reset: releasing it\n");
    gReset[kResetLevel1 / 4] = gReset[kResetLevel1 / 4] | kResetMali | kResetMaliApb;
  }
  __asm__ volatile("dsb sy" ::: "memory");
  IODelay(100);
  printf("mali: clocks now:    mali %#x stack %#x sys_en0 %#x (%s); reset level %#x\n", gClk[kClkMali / 4], gClk[kClkMaliStack / 4],
         gClk[kClkSysEn0 / 4], clockName(source), gReset[kResetLevel1 / 4]);
  return true;
}

bool softReset() {
  gpuWrite(kIntClear, 0xffffffffu);
  gpuWrite(kCmd, kSoftReset);
  if (!waitUs(100000, [] { return (gpu(kIntRaw) & kIrqResetCompleted) != 0; })) {
    printf("mali: soft reset did not complete (raw %#x status %#x)\n", gpu(kIntRaw), gpu(kStatus));
    return false;
  }
  gpuWrite(kIntClear, kIrqResetCompleted);
  gpuWrite(kPwrKey, 0x2968a819u);          // Amlogic's mali_hw_init(): the override register unlocked, then its power-transition timing
  gpuWrite(kPwrOverride1, 0x00200fffu);
  return true;
}

void identify() {
  const uint32_t id = gpu(kGpuId);
  printf("mali: GPU_ID %#x = arch %u.%u.%u product %u version %u.%u status %u%s\n", id, id >> 28, (id >> 24) & 0xf, (id >> 20) & 0xf,
         (id >> 16) & 0xf, (id >> 12) & 0xf, (id >> 4) & 0xff, id & 0xf, (id >> 24) == 0xac && ((id >> 16) & 0xf) == 4 ? " (Mali-G310)" : " (NOT the expected Mali-G310, arch 10.12 product 4)");
  printf("mali: L2 features %#x core %#x tiler %#x mem %#x mmu %#x (va %u bits, pa %u bits) as present %#x csf id %#x\n", gpu(kL2Features),
         gpu(kCoreFeatures), gpu(kTilerFeatures), gpu(kMemFeatures), gpu(kMmuFeatures), gpu(kMmuFeatures) & 0xff, (gpu(kMmuFeatures) >> 8) & 0xff,
         gpu(kAsPresent), gpu(kCsfId));
  printf("mali: present: shader %#x tiler %#x l2 %#x stack %#x; threads %u features %#x; coherency features %#x\n", gpu(kShaderPresent),
         gpu(kTilerPresent), gpu(kL2Present), gpu(kStackPresent), gpu(kThreadMax), gpu(kThreadFeatures), gpu(kCoherencyFeatures));
  printf("mali: MCU control %#x status %#x, GPU status %#x, irq raw %#x\n", gpu(kMcuControl), gpu(kMcuStatus), gpu(kStatus), gpu(kIntRaw));
}

// mali-poke=ADDR/VALUE[,ADDR/VALUE...]: 32-bit writes to physical addresses (hex, no 0x; boot-args take no ':') once, after the GPU domain is on, for
// trying a register setting (NoC QoS, say) without rebuilding the kernel. Up to four, each reported before and after.
void pokeRegisters() {
  char text[96] = {};
  if (!PE_parse_boot_argn("mali-poke", text, sizeof text) || !text[0]) return;
  const char *c = text;
  for (unsigned n = 0; n < 4 && *c; ++n) {
    auto hex = [&c](uint64_t &out) {
      out = 0;
      bool any = false;
      for (;; ++c) {
        const char d = *c;
        const unsigned v = d >= '0' && d <= '9' ? d - '0' : d >= 'a' && d <= 'f' ? d - 'a' + 10 : d >= 'A' && d <= 'F' ? d - 'A' + 10 : 99;
        if (v == 99) break;
        out = (out << 4) | v;
        any = true;
      }
      return any;
    };
    uint64_t addr = 0, value = 0;
    if (!hex(addr) || *c != '/') { printf("mali: poke: cannot read an address at \"%s\"\n", c); return; }
    ++c;
    if (!hex(value)) { printf("mali: poke: cannot read a value after %#llx\n", static_cast<unsigned long long>(addr)); return; }
    if (*c == ',') ++c;
    volatile uint32_t *page = reinterpret_cast<volatile uint32_t *>(ml_io_map(static_cast<vm_offset_t>(addr & ~0xfffULL), 0x1000));
    if (!page || (addr & 3)) { printf("mali: poke: cannot map or align %#llx\n", static_cast<unsigned long long>(addr)); continue; }
    volatile uint32_t *r = page + ((addr & 0xfff) >> 2);
    const uint32_t before = *r;
    *r = static_cast<uint32_t>(value);
    __asm__ volatile("dsb sy" ::: "memory");
    printf("mali: poke %#llx: %#x -> wrote %#x -> reads %#x\n", static_cast<unsigned long long>(addr), before, static_cast<uint32_t>(value), *r);
  }
}

// Panthor's panthor_gpu_power_on(L2): ask for power, wait for READY. It proves that the GPU's own power state machine and clocks work.
void l2Test() {
  const uint32_t mask = gpu(kL2Present);
  gpuWrite(kIntClear, 0xffffffffu);
  gpuWrite(kL2PwrOn, mask);
  const bool ok = waitUs(20000, [mask] { return (gpu(kL2Ready) & mask) == mask; });
  printf("mali: L2 power-up %s (ready %#x trans %#x, irq raw %#x)\n", ok ? "worked" : "FAILED", gpu(kL2Ready), gpu(kL2PwrTrans), gpu(kIntRaw));
}


// ========================================================================================================== M1: firmware and MCU

extern "C" void pmap_set_cache_attributes(unsigned int pn, unsigned int attributes);

const unsigned char *kFirmware;   // fw/mali_csffw.bin of the small image (mali_start)
size_t kFirmwareSize;

// GPU-visible memory: physically contiguous below 2^40 (the GPU's output address width), 4 KiB aligned, Normal non-cacheable for the CPU
// (VM_WIMG_WCOMB = 0x6; the GPU of this SoC is not cache coherent: coherency features read 0). Unaligned accesses, which memcpy does,
// are not allowed on device memory: the buffer is made write-combining (Normal non-cacheable) from the start, its kernel mapping too (on
// this XNU the later attribute change does not reach the descriptor's own mapping, and bzero's DC ZVA faulted on it), and the pages'
// attribute set to the same for the mappings made later.
struct Mem {
  IOBufferMemoryDescriptor *md;
  uint8_t *kva;
  uint64_t pa;
  size_t size;
};

bool memAlloc(Mem &m, size_t size) {
  size = (size + 0xfff) & ~static_cast<size_t>(0xfff);
  IOBufferMemoryDescriptor *md = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
      kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryMapperNone) | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache,
      size, 0x000000fffffff000ULL);
  if (!md) {
    static unsigned failures;
    if (failures++ < 16) printf("mali: no physically contiguous memory for %lu bytes\n", static_cast<unsigned long>(size));
    return false;
  }
  if (md->prepare() != kIOReturnSuccess) { md->release(); return false; }
  IOByteCount length = 0;
  const uint64_t pa = md->getPhysicalSegment(0, &length, kIOMemoryMapperNone);
  uint8_t *kva = static_cast<uint8_t *>(md->getBytesNoCopy());
  if (!kva || !pa || length < size || (pa & 0xfff)) { (void)md->complete(); md->release(); return false; }
  for (uint64_t off = 0; off < size; off += 0x1000) pmap_set_cache_attributes(static_cast<unsigned int>((pa + off) >> 12), 0x6U);
  __asm__ volatile("dsb sy\n\tisb" ::: "memory");
  memset(kva, 0, size);
  __asm__ volatile("dsb sy" ::: "memory");
  m.md = md; m.kva = kva; m.pa = pa; m.size = size;
  return true;
}

// ---- the MCU's page table: AArch64 stage 1, 4 KiB granule, 48 bit, four levels, as Linux's io-pgtable builds it for Panthor.
// Leaf: valid|page (3), AttrIndx[4:2], AP[7:6] (bit 6 = EL0 accessible, bit 7 = read-only), SH[9:8], AF (bit 10), nG (bit 11), XN (bit 54).
constexpr uint64_t kPteTable = 3, kPtePage = 3, kPteUnpriv = 1ull << 6, kPteRdOnly = 1ull << 7, kPteShOs = 2ull << 8, kPteShIs = 3ull << 8, kPteAf = 1ull << 10,
                   kPteNg = 1ull << 11, kPteXn = 1ull << 54;
constexpr unsigned kIdxNc = 0, kIdxCache = 1;   // the MAIR order of io-pgtable-arm: NC 0x44, WBRWA 0xff
// AS_MEMATTR is that MAIR translated by Panthor's mair_to_memattr(): NC entries 0x4c (inner/outer NC, explicit alloc 0), WB 0x9f, INC_OWBRWA 0x9c.
constexpr uint64_t kMemAttr = 0x4c4c4c4c9c4c9f4cULL;

// The tables are handed out of physically contiguous chunks: `pool` first (page 0 is the root table), then `more`, 64 KiB each when it is full.
// Small on purpose: a lot of processes come and go, and after a few minutes the physical memory has no 1 MiB hole left (gpu33: "no memory for the page tables"
// from the fifth client on, with plenty of free pages).
constexpr unsigned kPtMore = 255;
constexpr size_t kPtChunk = 0x10000;
struct Vm {
  Mem pool;
  size_t used;                 // bytes handed out of the newest chunk
  Mem more[kPtMore];
  unsigned moreCount;
};
Vm gMcuVm, gGpuVm;   // address space 0 (the MCU's) and the one GPU jobs run in

bool vmInit(Vm &vm, size_t pages) {
  if (!memAlloc(vm.pool, pages * 0x1000)) return false;
  vm.used = 0x1000;
  return true;
}

uint8_t *tableKva(Vm &vm, uint64_t pa) {
  if (pa >= vm.pool.pa && pa < vm.pool.pa + vm.pool.size) return vm.pool.kva + (pa - vm.pool.pa);
  for (unsigned i = 0; i < vm.moreCount; ++i)
    if (pa >= vm.more[i].pa && pa < vm.more[i].pa + vm.more[i].size) return vm.more[i].kva + (pa - vm.more[i].pa);
  return nullptr;
}

uint64_t *ptNew(Vm &vm, uint64_t *pa) {
  Mem *chunk = vm.moreCount ? &vm.more[vm.moreCount - 1] : &vm.pool;
  if (vm.used + 0x1000 > chunk->size) {
    if (vm.moreCount >= kPtMore || !memAlloc(vm.more[vm.moreCount], kPtChunk)) return nullptr;
    chunk = &vm.more[vm.moreCount++];
    vm.used = 0;
  }
  uint64_t *t = reinterpret_cast<uint64_t *>(chunk->kva + vm.used);
  *pa = chunk->pa + vm.used;
  vm.used += 0x1000;
  return t;
}

bool ptMap(Vm &vm, uint64_t va, uint64_t pa, uint64_t leaf) {
  uint64_t *table = reinterpret_cast<uint64_t *>(vm.pool.kva);
  for (int level = 0; level < 3; ++level) {
    const unsigned index = (va >> (39 - 9 * level)) & 0x1ff;
    if (level == 2 && (table[index] & 3) == 1) return false;   // a 2 MiB block maps this window: its pages are not ours to map one by one
    if (!(table[index] & 1)) {
      uint64_t childPa;
      if (!ptNew(vm, &childPa)) return false;
      table[index] = childPa | kPteTable;
    }
    table = reinterpret_cast<uint64_t *>(tableKva(vm, table[index] & 0x0000fffffffff000ULL));
    if (!table) return false;
  }
  table[(va >> 12) & 0x1ff] = pa | kPtePage | leaf;
  return true;
}

// A 2 MiB block at level 2 (descriptor type 1, the same attribute bits as a page's): one TLB entry where 512 pages took 512, which is what a GPU that
// samples a 93 MiB texture needs (docs/notes.zh-CN.md, "2 MiB blocks"). va and pa are 2 MiB aligned, the window's 512 pages one run of physical memory.
bool ptMapBlock(Vm &vm, uint64_t va, uint64_t pa, uint64_t leaf) {
  uint64_t *table = reinterpret_cast<uint64_t *>(vm.pool.kva);
  for (int level = 0; level < 2; ++level) {
    const unsigned index = (va >> (39 - 9 * level)) & 0x1ff;
    if (!(table[index] & 1)) {
      uint64_t childPa;
      if (!ptNew(vm, &childPa)) return false;
      table[index] = childPa | kPteTable;
    }
    table = reinterpret_cast<uint64_t *>(tableKva(vm, table[index] & 0x0000fffffffff000ULL));
    if (!table) return false;
  }
  // The window holds no mapping (the caller checked the records): a table of cleared pages there is dropped, its page of memory stays with the VM.
  table[(va >> 21) & 0x1ff] = pa | 1 | leaf;
  return true;
}

// A leaf for a buffer the CPU shares with the GPU: not cached on either side (the GPU is not coherent), readable, writable unless told, never executable.
constexpr uint64_t kLeafCached = kPteAf | kPteNg | kPteUnpriv | kPteShIs | (static_cast<uint64_t>(kIdxCache) << 2) | kPteXn;   // write-back, inner shareable
constexpr uint64_t kLeafData = kPteAf | kPteNg | kPteUnpriv | kPteShOs | (static_cast<uint64_t>(kIdxNc) << 2) | kPteXn;

bool vmMap(Vm &vm, uint64_t va, const Mem &mem, uint64_t leaf) {
  for (uint64_t off = 0; off < mem.size; off += 0x1000)
    if (!ptMap(vm, va + off, mem.pa + off, leaf)) return false;
  return true;
}

// ---- firmware sections (the CSF binary format, Panthor's panthor_fw.c): header, then entries; interface entries describe a section
enum : uint32_t {
  kFwMagic = 0xc3f13a6e, kEntryIface = 0,
  kSecRd = 1u << 0, kSecWr = 1u << 1, kSecEx = 1u << 2, kSecCacheMask = 3u << 3, kSecCached = 1u << 3, kSecProt = 1u << 5, kSecShared = 1u << 30, kSecZero = 1u << 31,
};
constexpr uint32_t kSharedVa = 0x04000000;
constexpr unsigned kMaxSections = 16;

struct Section {
  uint32_t flags, vaStart, vaEnd;
  Mem mem;
};
Section gSections[kMaxSections];
unsigned gSectionCount;
Section *gShared;

inline uint32_t le32(const unsigned char *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24); }

bool loadFirmware() {
  const size_t total = kFirmwareSize;
  if (total < 20 || le32(kFirmware) != kFwMagic) { printf("mali: the firmware image has the wrong magic\n"); return false; }
  const uint32_t size = le32(kFirmware + 16);
  printf("mali: firmware: header %u.%u, %zu bytes, entries end at %u\n", kFirmware[5], kFirmware[4], total, size);
  if (kFirmware[5] != 0 || size > total) { printf("mali: unsupported firmware header\n"); return false; }
  // page table pool: a few dozen pages (root + 3 levels for every 2 MiB the sections touch)
  if (!vmInit(gMcuVm, 64)) { printf("mali: no page table memory\n"); return false; }
  uint32_t offset = 20;
  while (offset + 4 <= size) {
    const uint32_t ehdr = le32(kFirmware + offset);
    const uint32_t type = ehdr & 0xff, esize = (ehdr >> 8) & 0xff;
    if (esize < 4 || (esize & 3) || offset + esize > size) { printf("mali: bad firmware entry at %u (header %#x)\n", offset, ehdr); return false; }
    if (type == kEntryIface) {
      if (esize < 24) { printf("mali: short interface entry at %u\n", offset); return false; }
      const unsigned char *e = kFirmware + offset + 4;
      const uint32_t flags = le32(e), vaStart = le32(e + 4), vaEnd = le32(e + 8), dataStart = le32(e + 12), dataEnd = le32(e + 16);
      if (vaEnd < vaStart || dataEnd < dataStart || dataEnd > total || (vaStart & 0xfff) || (vaEnd & 0xfff)) { printf("mali: corrupt section at %u\n", offset); return false; }
      if (flags & kSecProt) {   // protected mode is not supported: Panthor ignores these too
        printf("mali: section %#x-%#x is protected-mode only, skipped\n", vaStart, vaEnd);
      } else if (vaEnd > vaStart) {
        if (gSectionCount == kMaxSections) { printf("mali: too many firmware sections\n"); return false; }
        Section &s = gSections[gSectionCount];
        s.flags = flags; s.vaStart = vaStart; s.vaEnd = vaEnd;
        if (!memAlloc(s.mem, vaEnd - vaStart)) { printf("mali: no memory for section %#x-%#x\n", vaStart, vaEnd); return false; }
        memcpy(s.mem.kva, kFirmware + dataStart, dataEnd - dataStart);   // the rest is zero (memAlloc cleared it; kSecZero is the same thing)
        const bool cached = (flags & kSecCacheMask) == kSecCached;
        uint64_t leaf = kPteAf | kPteNg | kPteUnpriv | (cached ? kPteShIs : kPteShOs) | (static_cast<uint64_t>(cached ? kIdxCache : kIdxNc) << 2);
        if (!(flags & kSecWr)) leaf |= kPteRdOnly;
        if (!(flags & kSecEx)) leaf |= kPteXn;
        for (uint32_t va = vaStart; va < vaEnd; va += 0x1000)
          if (!ptMap(gMcuVm, va, s.mem.pa + (va - vaStart), leaf)) { printf("mali: out of page table memory\n"); return false; }
        if (vaStart == kSharedVa) {
          if (!(flags & kSecShared)) { printf("mali: the interface section is not marked shared\n"); return false; }
          gShared = &s;
        }
        printf("mali: section %2u: va %#09x-%#09x flags %#010x (%s%s%s%s%s) data %u B -> pa %#llx\n", gSectionCount, vaStart, vaEnd, flags,
               (flags & kSecRd) ? "r" : "-", (flags & kSecWr) ? "w" : "-", (flags & kSecEx) ? "x" : "-", cached ? " cached" : " nc", (flags & kSecShared) ? " shared" : "",
               dataEnd - dataStart, static_cast<unsigned long long>(s.mem.pa));
        ++gSectionCount;
      }
    }
    offset += esize;
  }
  if (!gShared) { printf("mali: no shared interface section (va %#x)\n", kSharedVa); return false; }
  __asm__ volatile("dsb sy" ::: "memory");
  printf("mali: %u sections loaded, %zu bytes of page tables, root %#llx\n", gSectionCount, gMcuVm.used, static_cast<unsigned long long>(gMcuVm.pool.pa));
  return true;
}

// ---- address space 0 belongs to the MCU
enum : uint32_t { kMmuIntRaw = 0x2000, kMmuIntClear = 0x2004, kMmuIntMask = 0x2008, kJobIntRaw = 0x1000, kJobIntClear = 0x1004, kJobIntMask = 0x1008,
                  kDoorbell0 = 0x80000, kGpuFaultStatus = 0x3c, kGpuFaultAddr = 0x40 };
constexpr uint32_t kJobGlobalIf = 1u << 31;
constexpr uint32_t asReg(unsigned as, uint32_t off) { return 0x2400 + (as << 6) + off; }
enum : uint32_t { kAsTranstab = 0x0, kAsMemattr = 0x8, kAsLockaddr = 0x10, kAsCommand = 0x18, kAsFaultStatus = 0x1c, kAsFaultAddress = 0x20, kAsStatus = 0x28, kAsTranscfg = 0x30 };
enum : uint32_t { kAsCmdUpdate = 1, kAsCmdLock = 2, kAsCmdFlushMem = 5 };

bool asReady(unsigned as) { return waitUs(100000, [as] { return (gpu(asReg(as, kAsStatus)) & 1) == 0; }); }
bool asCommand(unsigned as, uint32_t cmd) {
  if (!asReady(as)) return false;
  gpuWrite(asReg(as, kAsCommand), cmd);
  return true;
}
void gpuWrite64(uint32_t off, uint64_t value) { gpuWrite(off, static_cast<uint32_t>(value)); gpuWrite(off + 4, static_cast<uint32_t>(value >> 32)); }

bool asEnable(unsigned as, uint64_t transtab) {
  const unsigned vaBits = gpu(kMmuFeatures) & 0xff;
  // Panthor's panthor_mmu_as_enable(): lock the whole range, flush, program, UPDATE.
  gpuWrite64(asReg(as, kAsLockaddr), 63);   // lock_region(0, ~0): width 64 - 1, start 0
  if (!asCommand(as, kAsCmdLock) || !asCommand(as, kAsCmdFlushMem) || !asReady(as)) { printf("mali: AS%u stuck before enable\n", as); return false; }
  const uint64_t transcfg = (2ull << 24) /* PTW_MEMATTR_WB */ | (1ull << 30) /* PTW_RA */ | 6 /* AARCH64_4K */ | (static_cast<uint64_t>(55 - vaBits) << 6);
  gpuWrite64(asReg(as, kAsTranstab), transtab);
  gpuWrite64(asReg(as, kAsMemattr), kMemAttr);
  gpuWrite64(asReg(as, kAsTranscfg), transcfg);
  if (!asCommand(as, kAsCmdUpdate) || !asReady(as)) { printf("mali: AS%u did not take the update\n", as); return false; }
  static unsigned shown;
  if (shown++ < 24)
    printf("mali: AS%u enabled: transtab %#llx transcfg %#llx memattr %#llx\n", as, static_cast<unsigned long long>(transtab), static_cast<unsigned long long>(transcfg),
           static_cast<unsigned long long>(kMemAttr));
  return true;
}

void faultReport() {
  printf("mali: MCU status %u, job irq raw %#x, mmu irq raw %#x, gpu irq raw %#x, gpu fault %#x at %#x%08x\n", gpu(kMcuStatus), gpu(kJobIntRaw), gpu(kMmuIntRaw), gpu(kIntRaw),
         gpu(kGpuFaultStatus), gpu(kGpuFaultAddr + 4), gpu(kGpuFaultAddr));
  for (unsigned as = 0; as < 8; ++as)
    printf("mali: AS%u fault status %#x address %#x%08x, status %#x\n", as, gpu(asReg(as, kAsFaultStatus)), gpu(asReg(as, kAsFaultAddress) + 4), gpu(asReg(as, kAsFaultAddress)),
           gpu(asReg(as, kAsStatus)));
}

// Make the CPU's page table edits visible to the MMU of an address space that is running: lock everything, flush (which also unlocks).
bool asFlushAll(unsigned as) {
  gpuWrite64(asReg(as, kAsLockaddr), 63);
  return asCommand(as, kAsCmdLock) && asCommand(as, kAsCmdFlushMem) && asReady(as);
}

bool startMcu() {
  gpuWrite(kJobIntClear, 0xffffffffu);
  gpuWrite(kMmuIntClear, 0xffffffffu);
  gpuWrite(kJobIntMask, 0);   // polled
  gpuWrite(kMcuControl, 2);   // MCU_CONTROL_AUTO
  const bool booted = waitUs(1000000, [] { return (gpu(kJobIntRaw) & kJobGlobalIf) != 0; });
  printf("mali: MCU %s (MCU status %u, job irq raw %#x)\n", booted ? "booted: the global interface raised its interrupt" : "did NOT boot", gpu(kMcuStatus), gpu(kJobIntRaw));
  if (!booted) faultReport();
  return booted;
}

// ---- the global interface in the shared section (Panthor's panthor_fw.h): control block at its start, input / output blocks at the addresses it names
struct GlbControl { uint32_t version, features, inputVa, outputVa, groupNum, groupStride, perfcntSize, instrFeatures; };
struct GlbInput { uint32_t req, ackIrqMask, doorbellReq, reserved1, progressTimer, poweroffTimer; uint64_t coreEnMask; };
struct GlbOutput { uint32_t ack, reserved1, doorbellAck, reserved2, haltStatus, perfcntStatus, perfcntInsert; };
struct CsgControl { uint32_t features, inputVa, outputVa, suspendSize, protmSuspendSize, streamNum, streamStride; };
enum : uint32_t { kGlbCfgProgress = 1u << 1, kGlbCfgAlloc = 1u << 2, kGlbCfgPoweroff = 1u << 3, kGlbPing = 1u << 8, kGlbIdleEn = 1u << 10, kGlbIdle = 1u << 26 };

volatile GlbControl *gGlbControl;
volatile GlbInput *gGlbInput;
volatile GlbOutput *gGlbOutput;

void *sharedAt(uint32_t va) {
  if (va < kSharedVa || va >= kSharedVa + gShared->mem.size) return nullptr;
  return gShared->mem.kva + (va - kSharedVa);
}

bool globalInterface() {
  volatile GlbControl *control = reinterpret_cast<volatile GlbControl *>(gShared->mem.kva);
  gGlbControl = control;
  const uint32_t version = control->version;
  printf("mali: CSF interface v%u.%u.%u, features %#x, %u groups (stride %#x), instrumentation features %#x, input %#x output %#x\n", version >> 24, (version >> 16) & 0xff,
         version & 0xffff, control->features, control->groupNum, control->groupStride, control->instrFeatures, control->inputVa, control->outputVa);
  if (!version) { printf("mali: the firmware reports version 0: it did not start\n"); return false; }
  volatile GlbInput *input = static_cast<volatile GlbInput *>(sharedAt(control->inputVa));
  volatile GlbOutput *output = static_cast<volatile GlbOutput *>(sharedAt(control->outputVa));
  if (!input || !output) { printf("mali: the global input / output interface is outside the shared section\n"); return false; }
  gGlbInput = input;
  gGlbOutput = output;
  if (control->groupNum) {
    volatile CsgControl *csg = reinterpret_cast<volatile CsgControl *>(gShared->mem.kva + 0x1000);
    printf("mali: group 0: %u streams (stride %#x), features %#x, suspend buffer %#x\n", csg->streamNum, csg->streamStride, csg->features, csg->suspendSize);
  }
  // Panthor's panthor_fw_init_global_iface(): cores, timers, the acks we want to hear about, three configuration toggles, ring the global doorbell.
  input->coreEnMask = gpu(kShaderPresent);
  input->poweroffTimer = 235;               // 10 ms at the arch timer's 24 MHz, in units of 1024 cycles
  input->progressTimer = (5u * 500 * 1024 * 1024) >> 10;
  input->ackIrqMask = kGlbCfgAlloc | kGlbPing | kGlbCfgProgress | kGlbCfgPoweroff | kGlbIdleEn | kGlbIdle;
  const uint32_t toggles = kGlbCfgAlloc | kGlbCfgPoweroff | kGlbCfgProgress | kGlbPing;
  input->req = (input->req & ~toggles) | ((output->ack ^ toggles) & toggles);
  __asm__ volatile("dsb sy" ::: "memory");
  gpuWrite(kDoorbell0, 1);
  const bool acked = waitUs(1000000, [input, output] { return ((input->req ^ output->ack) & toggles) == 0; });
  printf("mali: global interface config + ping %s (req %#x ack %#x, job irq raw %#x, MCU status %u)\n", acked ? "ACKNOWLEDGED" : "NOT acknowledged", input->req, output->ack,
         gpu(kJobIntRaw), gpu(kMcuStatus));
  if (!acked) faultReport();
  return acked;
}

// ---- M2: a group with one queue, a command stream that stores two words, and the CPU reading them back.
// Per-slot interface blocks (Panthor's panthor_fw.h). The firmware reads the inputs and writes the outputs; toggled request bits are answered by toggling the ack.
struct CsControl { uint32_t features, inputVa, outputVa; };
struct CsInput { uint32_t req, config, reserved1, ackIrqMask; uint64_t ringbufBase; uint32_t ringbufSize, reserved2; uint64_t heapStart, heapEnd, ringbufInput, ringbufOutput; };
struct CsOutput {
  uint32_t ack, reserved1[15];
  uint64_t statusCmdPtr;
  uint32_t statusWait, statusReqResource;
  uint64_t statusWaitSyncPtr;
  uint32_t statusWaitSyncValue, statusScoreboards, statusBlockedReason, statusWaitSyncValueHi, reserved2[6], fault, fatal;
  uint64_t faultInfo, fatalInfo;
};
struct CsgInput {
  uint32_t req, ackIrqMask, doorbellReq, csIrqAck, reserved1[4];
  uint64_t allowCompute, allowFragment;
  uint32_t allowOther, endpointReq, reserved2[2];
  uint64_t suspendBuf, protmSuspendBuf;
  uint32_t config, iterTraceConfig;
};
struct CsgOutput { uint32_t ack, reserved1, doorbellAck, csIrqReq, statusEndpointCurrent, statusEndpointReq, statusState, resourceDep; };
struct RingInput { uint64_t insert, extract; };         // page 0 of the queue's interface memory
struct RingOutput { uint64_t extract; uint32_t active; };  // page 1

enum : uint32_t {
  kCsStateMask = 7, kCsStateStart = 1, kCsExtractEvent = 1u << 4, kCsIdleSyncWait = 1u << 8, kCsIdleEmpty = 1u << 10,
  kCsgStateMask = 7, kCsgStateStart = 1, kCsgEndpointConfig = 1u << 4,
};

constexpr uint64_t kRingVa = 0x20000000ULL, kDataVa = 0x20010000ULL, kCodeVa = 0x20020000ULL, kDescVa = 0x20030000ULL, kData2Va = 0x20040000ULL;   // in the GPU's address space (AS1)
// The MCU's AS0: queue interfaces and suspend buffers must sit in the shared region (0x04000000..0x08000000, Panthor's CSF_MCU_SHARED_REGION_*):
// anything above it was seen as address & 0x0fffffff by the firmware (0x10001000 -> a write fault at 0x1000).
constexpr uint32_t kQueueVa = 0x04100000, kSuspendVa = 0x04110000, kProtmVa = 0x04200000;

// Command stream instructions, 64 bit each, opcode in the top byte (Mesa's v10.xml). r0/r1 hold an address, r2.. data.
constexpr uint64_t csMove48(unsigned dst, uint64_t imm) { return (1ull << 56) | (static_cast<uint64_t>(dst) << 48) | (imm & 0xffffffffffffull); }
constexpr uint64_t csMove32(unsigned dst, uint32_t imm) { return (2ull << 56) | (static_cast<uint64_t>(dst) << 48) | imm; }
constexpr uint64_t csWait(unsigned mask) { return (3ull << 56) | (static_cast<uint64_t>(mask) << 16); }
constexpr uint64_t csStore(unsigned base, unsigned addr, unsigned mask, unsigned offset) {
  return (21ull << 56) | (static_cast<uint64_t>(base) << 48) | (static_cast<uint64_t>(addr) << 40) | (static_cast<uint64_t>(mask) << 16) | (offset & 0xffff);
}
constexpr uint64_t csSetSb(unsigned endpoint, unsigned other) { return (23ull << 56) | endpoint | (other << 4); }
constexpr uint64_t csReqResource(unsigned mask) { return (34ull << 56) | mask; }   // bit 0 compute, 1 fragment, 2 tiler, 3 idvs
// RUN_COMPUTE: task increment [13:0], task axis [15:14] (0 = X), the four resource selects at 40/42/44/46 (all 0 = the ones just set).
// FLUSH_CACHE2: L2 [3:0], load/store cache [7:4] (1 clean, 3 clean + invalidate), others [11:8] (2 invalidate), latest flush id register [47:40] (a register holding 0 = always flush), signal slot [51:48].
constexpr uint64_t csFlushCaches(unsigned l2, unsigned lsc, unsigned others, unsigned idReg, unsigned signalSlot) {
  return (36ull << 56) | l2 | (lsc << 4) | (others << 8) | (static_cast<uint64_t>(idReg) << 40) | (static_cast<uint64_t>(signalSlot) << 48);
}
constexpr uint64_t csRunCompute(unsigned increment, unsigned axis) { return (4ull << 56) | (increment & 0x3fff) | (static_cast<uint64_t>(axis) << 14); }

// The shader, assembled from shader/store.asm (Mesa's valhall/asm.py): r0 = a value, r2:r3 = an address, store it, end.
// IADD_IMM.i32 rD, 0x0, #imm: byte 0 0xc0 (source 0 = constant 0), bytes 1..4 the immediate, byte 5 0xc0 + D (destination), 0x10, 0x01.
constexpr uint64_t vaIaddImm(unsigned dst, uint32_t imm) { return 0xc0ull | (static_cast<uint64_t>(imm) << 8) | (static_cast<uint64_t>(0xc0 + dst) << 40) | (0x10ull << 48) | (0x01ull << 56); }
constexpr uint64_t kVaStoreEnd = 0x7861400218000042ULL;   // STORE.i32.slot0.end @r0, ^r2, offset:0

void *sharedPtr(uint32_t va) { return sharedAt(va); }

bool m2() {
  const unsigned csg = 0, cs = 0;
  volatile GlbControl *glb = gGlbControl;
  if (glb->groupNum == 0) { printf("mali: m2: the firmware has no group slots\n"); return false; }
  uint8_t *csgBase = gShared->mem.kva + 0x1000 + csg * glb->groupStride;
  volatile CsgControl *csgCtl = reinterpret_cast<volatile CsgControl *>(csgBase);
  volatile CsgInput *csgIn = static_cast<volatile CsgInput *>(sharedPtr(csgCtl->inputVa));
  volatile CsgOutput *csgOut = static_cast<volatile CsgOutput *>(sharedPtr(csgCtl->outputVa));
  volatile CsControl *csCtl = reinterpret_cast<volatile CsControl *>(csgBase + 0x40 + cs * csgCtl->streamStride);
  volatile CsInput *csIn = static_cast<volatile CsInput *>(sharedPtr(csCtl->inputVa));
  volatile CsOutput *csOut = static_cast<volatile CsOutput *>(sharedPtr(csCtl->outputVa));
  if (!csgIn || !csgOut || !csIn || !csOut) { printf("mali: m2: group / stream interface outside the shared section\n"); return false; }
  printf("mali: m2: group %u at %p, stream %u input %#x output %#x\n", csg, (const void *)csgBase, cs, csCtl->inputVa, csCtl->outputVa);

  // memory: ring buffer + a data page in the GPU's address space, the queue's input/output pair and the suspend buffer in the MCU's
  Mem ring, data, data2, queue, suspend, protm, shaderMem, desc;
  const size_t suspendSize = csgCtl->suspendSize ? csgCtl->suspendSize : 0x1000;
  const size_t protmSize = csgCtl->protmSuspendSize;
  printf("mali: m2: suspend buffer %zu B, protected-mode suspend buffer %zu B\n", suspendSize, protmSize);
  if (!vmInit(gGpuVm, 32) || !memAlloc(ring, 0x1000) || !memAlloc(data, 0x1000) || !memAlloc(data2, 0x1000) || !memAlloc(shaderMem, 0x1000) || !memAlloc(desc, 0x1000) || !memAlloc(queue, 0x2000) || !memAlloc(suspend, suspendSize) || (protmSize && !memAlloc(protm, protmSize))) {
    printf("mali: m2: out of memory\n");
    return false;
  }
  if (!vmMap(gGpuVm, kRingVa, ring, kLeafData) || !vmMap(gGpuVm, kDataVa, data, kLeafData) || !vmMap(gGpuVm, kData2Va, data2, kLeafCached) || !vmMap(gGpuVm, kCodeVa, shaderMem, kLeafData & ~kPteXn) || !vmMap(gGpuVm, kDescVa, desc, kLeafData) ||
      !vmMap(gMcuVm, kQueueVa, queue, kLeafData) || !vmMap(gMcuVm, kSuspendVa, suspend, kLeafData) || (protmSize && !vmMap(gMcuVm, kProtmVa, protm, kLeafData))) {
    printf("mali: m2: out of page table memory\n");
    return false;
  }
  volatile uint32_t *words = reinterpret_cast<volatile uint32_t *>(data.kva);
  for (unsigned i = 0; i < 1024; ++i) words[i] = 0xdeadbeef;
  __asm__ volatile("dsb sy" ::: "memory");
  if (!asFlushAll(0)) { printf("mali: m2: AS0 would not flush\n"); return false; }
  gpuWrite(kMmuIntClear, 0xffffffffu);
  if (!asEnable(1, gGpuVm.pool.pa)) return false;
  printf("mali: m2: ring va %#llx pa %#llx, data va %#llx pa %#llx, queue va %#x pa %#llx, suspend %zu B va %#x pa %#llx\n", static_cast<unsigned long long>(kRingVa),
         static_cast<unsigned long long>(ring.pa), static_cast<unsigned long long>(kDataVa), static_cast<unsigned long long>(data.pa), kQueueVa, static_cast<unsigned long long>(queue.pa),
         suspendSize, kSuspendVa, static_cast<unsigned long long>(suspend.pa));

  volatile RingInput *ringIn = reinterpret_cast<volatile RingInput *>(queue.kva);
  volatile RingOutput *ringOut = reinterpret_cast<volatile RingOutput *>(queue.kva + 0x1000);
  ringIn->insert = 0;
  ringIn->extract = ringOut->extract;

  // cs_slot_prog_locked()
  csIn->ringbufBase = kRingVa;
  csIn->ringbufSize = 0x1000;
  csIn->ringbufInput = kQueueVa;
  csIn->ringbufOutput = kQueueVa + 0x1000;
  csIn->config = (0u /* priority */) | ((csg + 1) << 8) /* doorbell id */;
  csIn->ackIrqMask = ~0u;
  const uint32_t csMask = kCsIdleSyncWait | kCsIdleEmpty | kCsStateMask | kCsExtractEvent;
  csIn->req = (csIn->req & ~csMask) | ((kCsIdleSyncWait | kCsIdleEmpty | kCsStateStart | kCsExtractEvent) & csMask);
  // csg_slot_prog_locked()
  csgIn->allowCompute = gpu(kShaderPresent);
  csgIn->allowFragment = 0;
  csgIn->allowOther = gpu(kTilerPresent);
  csgIn->endpointReq = 4u /* compute cores */ | (0u << 8) | (1u << 16) /* tiler cores */ | (15u << 28) /* priority: Panthor hands the first group the highest */;
  csgIn->config = 1;   // the address space
  csgIn->suspendBuf = kSuspendVa;
  csgIn->protmSuspendBuf = protmSize ? kProtmVa : 0;
  csgIn->ackIrqMask = ~0u;
  csgIn->doorbellReq = (csgIn->doorbellReq & ~1u) | ((~csgOut->doorbellAck) & 1u);
  __asm__ volatile("dsb sy" ::: "memory");
  // start the group, ring the global doorbell for this slot
  printf("mali: m2: before the start: mmu irq raw %#x, job irq raw %#x, AS0 fault %#x\n", gpu(kMmuIntRaw), gpu(kJobIntRaw), gpu(asReg(0, kAsFaultStatus)));
  csgIn->req = (csgIn->req & ~(kCsgStateMask | kCsgEndpointConfig)) | kCsgStateStart | ((csgOut->ack ^ kCsgEndpointConfig) & kCsgEndpointConfig);
  gGlbInput->doorbellReq = (gGlbInput->doorbellReq & ~(1u << csg)) | ((~gGlbOutput->doorbellAck) & (1u << csg));
  __asm__ volatile("dsb sy" ::: "memory");
  gpuWrite(kDoorbell0, 1);
  const bool started = waitUs(1000000, [csgOut, csgIn] { return (csgOut->ack & (kCsgStateMask | kCsgEndpointConfig)) == (kCsgStateStart | (csgIn->req & kCsgEndpointConfig)); });
  printf("mali: m2: group %s (csg req %#x ack %#x, cs req %#x ack %#x, job irq raw %#x)\n", started ? "STARTED" : "did NOT start", csgIn->req, csgOut->ack, csIn->req, csOut->ack,
         gpu(kJobIntRaw));
  if (!started) { faultReport(); return false; }
  const bool csStarted = waitUs(300000, [csOut] { return (csOut->ack & kCsStateMask) == kCsStateStart; });
  printf("mali: m2: stream %s (cs req %#x ack %#x, csg ack %#x, csg irq req %#x, mmu irq raw %#x, job irq raw %#x)\n", csStarted ? "STARTED" : "did NOT start", csIn->req, csOut->ack,
         csgOut->ack, csgOut->csIrqReq, gpu(kMmuIntRaw), gpu(kJobIntRaw));
  if (!csStarted) faultReport();

  // Two submissions. Each: pick the scoreboard entries, load the data address and two values, store them, wait for the stores, pad to 64 bytes with NOPs.
  volatile uint64_t *code = reinterpret_cast<volatile uint64_t *>(ring.kva);
  bool allOk = true;
  for (unsigned round = 0; round < 2; ++round) {
    const uint64_t at = round * 64;
    const uint64_t target = kDataVa + round * 16;
    const uint32_t v0 = 0x600dcafe + round, v1 = 0x12345678 + round * 0x11111111;
    const uint64_t program[8] = {csSetSb(3, 0), csMove48(0, target), csMove32(2, v0), csMove32(3, v1), csStore(2, 0, 3, 0), csWait(1u << 0), 0, 0};
    for (unsigned i = 0; i < 8; ++i) code[at / 8 + i] = program[i];
    __asm__ volatile("dsb sy" ::: "memory");
    ringIn->extract = ringOut->extract;
    ringIn->insert = at + 64;
    __asm__ volatile("dsb sy" ::: "memory");
    gpuWrite(kDoorbell0 + 0x10000 * (csg + 1), 1);
    const bool consumed = waitUs(1000000, [ringOut, at] { return ringOut->extract >= at + 64; });
    const uint32_t g0 = words[round * 4], g1 = words[round * 4 + 1];
    const bool ok = consumed && g0 == v0 && g1 == v1;
    printf("mali: m2: job %u: %s: extract %llu of %llu, data %#x %#x (wanted %#x %#x), cs ack %#x fault %#x fatal %#x, job irq raw %#x mmu raw %#x\n", round,
           ok ? "OK" : "FAILED", static_cast<unsigned long long>(ringOut->extract), static_cast<unsigned long long>(at + 64), g0, g1, v0, v1, csOut->ack, csOut->fault, csOut->fatal,
           gpu(kJobIntRaw), gpu(kMmuIntRaw));
    if (!ok) { faultReport(); allOk = false; break; }
  }
  printf("mali: m2 %s\n", allOk ? "PASSED: the GPU ran command streams and wrote memory the CPU can read" : "FAILED");
  if (!allOk || gLevel < 4) return allOk;

  // ---- M2b: RUN_COMPUTE with a one-thread shader that stores a word. Shader program descriptor and thread storage descriptor in one page.
  const uint32_t kMagic = 0xc0ffee42, kDone = 0xd0de0001;
  volatile uint64_t *shader = reinterpret_cast<volatile uint64_t *>(shaderMem.kva);
  const uint64_t target = kData2Va + 0x100;   // the write-back mapped page
  shader[0] = vaIaddImm(0, kMagic);                                   // r0: the data (the staging register of the STORE)
  shader[1] = vaIaddImm(2, static_cast<uint32_t>(target));            // r2:r3: the address
  shader[2] = vaIaddImm(3, static_cast<uint32_t>(target >> 32));
  shader[3] = kVaStoreEnd;
  volatile uint32_t *spd = reinterpret_cast<volatile uint32_t *>(desc.kva);          // Shader Program, 8 words: type 8, stage 1 (compute), 64 registers, binary
  volatile uint32_t *tsd = reinterpret_cast<volatile uint32_t *>(desc.kva + 0x40);   // Local Storage: TLS size 0, no workgroup memory (WLS instances log2 field = 31)
  spd[0] = 8u | (1u << 4); spd[1] = 0; spd[2] = static_cast<uint32_t>(kCodeVa); spd[3] = static_cast<uint32_t>(kCodeVa >> 32);
  for (unsigned i = 4; i < 8; ++i) spd[i] = 0;
  tsd[0] = 0; tsd[1] = 0x1f;
  for (unsigned i = 2; i < 8; ++i) tsd[i] = 0;
  volatile uint32_t *words2 = reinterpret_cast<volatile uint32_t *>(data2.kva);
  for (unsigned i = 0; i < 1024; ++i) words2[i] = 0xdeadbeef;
  words[128] = 0xdeadbeef;
  __asm__ volatile("dsb sy" ::: "memory");
  const uint64_t at = 128;
  const uint64_t program[24] = {
      csReqResource(1), csSetSb(3, 0),
      csMove48(16, kDescVa),            // SPD_0
      csMove48(24, kDescVa + 0x40),     // TSD_0
      csMove48(0, 0), csMove48(8, 0),   // SRT_0, FAU_0: none
      csMove32(32, 0), csMove32(33, 0) /* WG_SIZE: 1x1x1 (minus one each) */, csMove32(34, 0), csMove32(35, 0), csMove32(36, 0),
      csMove32(37, 1), csMove32(38, 1), csMove32(39, 1),
      csRunCompute(1, 2 /* Z, as Mesa picks it for a 1x1x1 grid */),
      csWait(1u << 3),                  // the endpoint scoreboard entry: the dispatch is done
      csMove32(44, 0), csFlushCaches(3, 3, 2, 44, 0), csWait(1u << 0),   // push the shader's stores out of the GPU's caches
      csMove48(40, kDataVa + 0x200), csMove32(42, kDone), csStore(42, 40, 1, 0), csWait(1u << 0),
      0};
  for (unsigned i = 0; i < 24; ++i) reinterpret_cast<volatile uint64_t *>(ring.kva)[at / 8 + i] = program[i];
  __asm__ volatile("dsb sy" ::: "memory");
  printf("mali: m2b: shader ready ready %#x present %#x; dispatching\n", gpu(kShaderReady), gpu(kShaderPresent));
  ringIn->extract = ringOut->extract;
  ringIn->insert = at + 192;
  __asm__ volatile("dsb sy" ::: "memory");
  gpuWrite(kDoorbell0 + 0x10000 * (csg + 1), 1);
  const bool done = waitUs(1000000, [ringOut] { return ringOut->extract >= at + 192; });
  printf("mali: m2b: dispatch %s: extract %llu of %llu, shader wrote %#x (wanted %#x), end marker %#x (wanted %#x), shader ready %#x, cs ack %#x fault %#x fatal %#x (info %#llx), job irq raw %#x mmu raw %#x\n",
         done ? "finished" : "did NOT finish", static_cast<unsigned long long>(ringOut->extract), static_cast<unsigned long long>(at + 192), words2[64], kMagic, words[128], kDone, gpu(kShaderReady),
         csOut->ack, csOut->fault, csOut->fatal, static_cast<unsigned long long>(csOut->fatalInfo), gpu(kJobIntRaw), gpu(kMmuIntRaw));
  // From the host side too: GPU_CMD flush caches (type 4; L2 clean + invalidate, load/store cache the same, other caches invalidate), wait for CLEAN_CACHES_COMPLETED.
  gpuWrite(kIntClear, 1u << 17);
  gpuWrite(kCmd, 4u | ((3u | (3u << 4) | (2u << 8)) << 8));
  const bool flushed = waitUs(1000000, [] { return (gpu(kIntRaw) & (1u << 17)) != 0; });
  printf("mali: m2b: GPU_CMD cache flush %s; shader word now %#x\n", flushed ? "completed" : "did NOT complete", words2[64]);
  IODelay(200000);   // in case the store was only slow
  printf("mali: m2b: 200 ms later: shader word %#x; other words of the data page that changed:", words2[64]);
  unsigned changed = 0;
  for (unsigned i = 0; i < 1024 && changed < 12; ++i)
    if (words2[i] != 0xdeadbeef) { printf(" [%u]=%#x", i, words2[i]); ++changed; }
  printf("%s\n", changed ? "" : " none");
  printf("mali: m2b: cs status: scoreboards %#x blocked reason %#x wait %#x req resource %#x cmd ptr %#llx, csg status state %#x endpoint current %#x\n", csOut->statusScoreboards,
         csOut->statusBlockedReason, csOut->statusWait, csOut->statusReqResource, static_cast<unsigned long long>(csOut->statusCmdPtr), csgOut->statusState, csgOut->statusEndpointCurrent);
  const bool computeOk = done && words2[64] == kMagic && words[128] == kDone;
  if (!computeOk && gLevel >= 5) {
    // Diagnostic: does a dispatch fetch its shader at all? Point the program at an unmapped address; a thread that starts would fault in AS1.
    spd[2] = 0x30000000u;
    __asm__ volatile("dsb sy" ::: "memory");
    const uint64_t at2 = at + 192;
    volatile uint64_t *rb = reinterpret_cast<volatile uint64_t *>(ring.kva);
    for (unsigned i = 0; i < 24; ++i) rb[at2 / 8 + i] = program[i];
    __asm__ volatile("dsb sy" ::: "memory");
    gpuWrite(kMmuIntClear, 0xffffffffu);
    ringIn->extract = ringOut->extract;
    ringIn->insert = at2 + 192;
    __asm__ volatile("dsb sy" ::: "memory");
    gpuWrite(kDoorbell0 + 0x10000 * (csg + 1), 1);
    const bool done2 = waitUs(1000000, [ringOut] { return ringOut->extract >= at2 + 192; });
    IODelay(100000);
    printf("mali: m2b: dispatch with an unmapped shader: %s (extract %llu), mmu raw %#x, AS1 fault status %#x address %#x%08x, cs fault %#x fatal %#x (info %#llx), gpu irq raw %#x\n",
           done2 ? "finished" : "did NOT finish", static_cast<unsigned long long>(ringOut->extract), gpu(kMmuIntRaw), gpu(asReg(1, kAsFaultStatus)), gpu(asReg(1, kAsFaultAddress) + 4),
           gpu(asReg(1, kAsFaultAddress)), csOut->fault, csOut->fatal, static_cast<unsigned long long>(csOut->fatalInfo), gpu(kIntRaw));
  }
  if (!computeOk) faultReport();
  printf("mali: m2b %s\n", computeOk ? "PASSED: a shader ran on the shader cores and wrote memory" : "FAILED");
  return computeOk;
}

// ========================================================================================================== M3a: contexts, groups, jobs
// The shape of Panthor's uAPI (VM_CREATE / VM_BIND / GROUP_CREATE / GROUP_SUBMIT / syncobj waits), written so the IOKit user client can call it
// as it is: a context owns one address space, a group owns one CSG slot with up to kMaxQueues queues, a job is a CALL to a user command stream
// wrapped in Panthor's own instruction sequence, and it is done when the queue's syncobj counter reaches the job's sequence number.

constexpr unsigned kAsCount = 8, kMaxGroups = 8, kMaxQueues = 4, kCtxPtPages = 16;   // address spaces and CSG slots the hardware has
// What a client sees is not bound to those: a context (Panthor's VM) holds an address space only while one of its groups sits in a CSG slot, and a
// group sits in a slot only while it has work (like Panthor's scheduler, minus the time slicing: a group that is idle gives its slot up when
// another one needs it). So the number of clients is bounded by memory, the number of busy ones by the hardware.
constexpr unsigned kMaxCtx = 48, kMaxGroupObjs = 64, kNoSlot = ~0u;
constexpr uint64_t kKernelVaBase = 0x7ff000000000ULL;   // kernel-chosen objects (rings, syncobjs) live below 2^47, away from what a client maps
constexpr uint32_t kMcuSharedEnd = kSharedVa + 0x04000000;
constexpr unsigned kJobSlots = 16, kJobBytes = kJobSlots * 8;   // Panthor pads every job to two cache lines

void memFree(Mem &m) {
  if (!m.md) return;
  for (uint64_t off = 0; off < m.size; off += 0x1000) pmap_set_cache_attributes(static_cast<unsigned int>((m.pa + off) >> 12), 0x2U);   // VM_WIMG_DEFAULT
  __asm__ volatile("dsb sy\n\tisb" ::: "memory");
  (void)m.md->complete();
  m.md->release();
  m.md = nullptr; m.kva = nullptr; m.pa = 0; m.size = 0;
}

// Turn the 2 MiB block at `slot` of a level-2 table into a table of its 512 pages (same attributes), so a part of it can go. Without memory for the table
// the whole block goes (what stays of the buffer then faults for the GPU: better than a stale mapping of pages the buffer no longer owns).
void blockSplit(Vm &vm, uint64_t *l2, unsigned slot) {
  const uint64_t e = l2[slot];
  uint64_t tablePa;
  uint64_t *t = ptNew(vm, &tablePa);
  if (!t) { l2[slot] = 0; return; }
  const uint64_t base = e & 0x0000ffffffe00000ULL, attrs = (e & ~0x0000ffffffe00000ULL & ~3ULL) | kPtePage;
  for (unsigned i = 0; i < 512; ++i) t[i] = (base + static_cast<uint64_t>(i) * 0x1000) | attrs;
  l2[slot] = tablePa | kPteTable;
}

void vmUnmap(Vm &vm, uint64_t va, size_t size) {
  for (uint64_t off = 0; off < size;) {
    const uint64_t v = va + off;
    uint64_t *table = reinterpret_cast<uint64_t *>(vm.pool.kva);
    uint64_t step = 0x1000;
    for (int level = 0; table && level < 3; ++level) {
      const unsigned slot = (v >> (39 - 9 * level)) & 0x1ff;
      uint64_t e = table[slot];
      if (level == 2 && (e & 3) == 1) {   // a 2 MiB block: all of it goes when the range covers it, else it becomes pages first
        if (!(v & 0x1fffff) && size - off >= 0x200000) { table[slot] = 0; step = 0x200000; table = nullptr; break; }
        blockSplit(vm, table, slot);
        e = table[slot];
      }
      table = (e & 1) ? reinterpret_cast<uint64_t *>(tableKva(vm, e & 0x0000fffffffff000ULL)) : nullptr;
    }
    if (table) table[(v >> 12) & 0x1ff] = 0;
    off += step;
  }
}

void vmFree(Vm &vm) {
  for (unsigned i = 0; i < vm.moreCount; ++i) memFree(vm.more[i]);
  vm.moreCount = 0;
  memFree(vm.pool);
}

// ---- buffer objects the CPU and the GPU share: any physical pages below 2^40 (the page table needs no contiguity), write-combined for the CPU
struct Bo {
  IOMemoryDescriptor *md;
  uint8_t *kva;
  size_t size;
  uint64_t *pages;       // physical address of every 4 KiB page
  unsigned refs;         // the handle, plus one per mapping record
  void *owner;           // the client that made it
  IOMemoryDescriptor *host;   // BO_CREATE_HOST: the caller's wired pages (md, kva stay empty; the caches and the contents are the caller's)
  bool pooled;           // its pages are granules of gBoPool (no memory descriptor of its own to free, no cache attributes to give back)
  unsigned *poolIdx;     // the granules it holds (any of the pool's, not necessarily next to each other)
  unsigned poolCount;
  bool readOnly;         // BO_CREATE_HOST over pages the task maps read-only (a mapped file, a constant segment): the GPU maps them read-only too
  bool hostCopy;         // BO_CREATE_HOST over a file's pages: a copy in pages of its own (md, kva), not a pin of the caller's (see boCreateHost)
};
unsigned gBoLive;
uint64_t gBoContig[2];    // buffer objects of 2 MiB or more: [0] made of one 2 MiB aligned run of physical memory, [1] of whatever pages the allocator had

// ---- a pool of 2 MiB aligned physical memory for the big buffer objects, taken once at start. By the time the first client asks for a texture
// (about 100 s after boot) physical memory is so fragmented that not even one 2 MiB run is left (blk2: every contiguous try failed from 115 s on, the
// 91 MiB wallpaper's too), and the page table could not map 2 MiB blocks. The pool is a set of separate 2 MiB granules (blk3: no run of 128 MiB was to
// be had even at 23 s, 64 MiB was). A client buffer of 2 MiB or more takes free granules of the pool, wherever they are: the CPU side maps them as one
// range list, the GPU side as one 2 MiB block per granule. What the pool has no room for comes from the allocator as before.
constexpr size_t kGranule = 0x200000;
constexpr unsigned kPoolMax = 512;
struct PoolGranule { IOBufferMemoryDescriptor *md; uint8_t *kva; uint64_t pa; uint8_t used; };
struct BoPool { PoolGranule *g; unsigned granules, inUse, peak; } gBoPool;
uint64_t gPoolHits, gPoolMisses;

void boPoolInit() {
  unsigned mib = 128;   // the box has 2 GiB
  PE_parse_boot_argn("mali-pool", &mib, sizeof mib);
  unsigned want = mib / 2;
  if (want > kPoolMax) want = kPoolMax;
  if (!want) { printf("mali: buffer pool off\n"); return; }
  PoolGranule *g = IONewZeroData(PoolGranule, want);
  if (!g) return;
  unsigned n = 0, fails = 0;
  while (n < want && fails < 8) {
    IOBufferMemoryDescriptor *md = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryMapperNone) | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache,
        kGranule, 0x000000ffffe00000ULL);
    if (!md) { ++fails; continue; }
    if (md->prepare() != kIOReturnSuccess) { md->release(); ++fails; continue; }
    IOByteCount length = 0;
    const uint64_t pa = md->getPhysicalSegment(0, &length, kIOMemoryMapperNone);
    uint8_t *kva = static_cast<uint8_t *>(md->getBytesNoCopy());
    if (!kva || !pa || length < kGranule || (pa & (kGranule - 1))) { (void)md->complete(); md->release(); ++fails; continue; }
    for (size_t off = 0; off < kGranule; off += 0x1000) pmap_set_cache_attributes(static_cast<unsigned int>((pa + off) >> 12), 0x6U);
    g[n++] = PoolGranule{md, kva, pa, 0};
  }
  __asm__ volatile("dsb sy\n\tisb" ::: "memory");
  if (!n) { IODeleteData(g, PoolGranule, want); printf("mali: buffer pool: no 2 MiB aligned run to be had, big buffers use scattered pages\n"); return; }
  gBoPool = BoPool{g, n, 0, 0};
  printf("mali: buffer pool: %u granules of 2 MiB (%u MiB) of 2 MiB aligned physical memory\n", n, n * 2);
}

// `count` free granules (any), marked used and listed in `out`; false if the pool has fewer.
bool boPoolTake(unsigned count, unsigned *out) {
  BoPool &p = gBoPool;
  if (!p.g || p.granules - p.inUse < count) return false;
  unsigned got = 0;
  for (unsigned i = 0; i < p.granules && got < count; ++i)
    if (!p.g[i].used) { p.g[i].used = 1; out[got++] = i; }
  p.inUse += count;
  if (p.inUse > p.peak) p.peak = p.inUse;
  return true;
}

void boFree(Bo *bo) {
  if (!bo) return;
  if (bo->pooled) { for (unsigned i = 0; i < bo->poolCount; ++i) gBoPool.g[bo->poolIdx[i]].used = 0; gBoPool.inUse -= bo->poolCount; IODeleteData(bo->poolIdx, unsigned, bo->poolCount); }
  if (!bo->host && !bo->pooled) {
    for (size_t off = 0; bo->pages && off < bo->size; off += 0x1000)
      if (bo->pages[off >> 12]) pmap_set_cache_attributes(static_cast<unsigned int>(bo->pages[off >> 12] >> 12), 0x2U);
    __asm__ volatile("dsb sy\n\tisb" ::: "memory");
  }
  if (bo->host) { (void)bo->host->complete(); bo->host->release(); }
  if (bo->md) { (void)bo->md->complete(); bo->md->release(); }
  if (bo->pages) IODeleteData(bo->pages, uint64_t, bo->size >> 12);
  IODelete(bo, Bo, 1);
  --gBoLive;
}
void boRelease(Bo *bo) { if (bo && --bo->refs == 0) boFree(bo); }

Bo *boCreate(size_t size, void *owner) {
  size = (size + 0xfff) & ~static_cast<size_t>(0xfff);
  if (!size || size > (1ull << 32)) return nullptr;
  Bo *bo = IONewZero(Bo, 1);
  if (!bo) return nullptr;
  ++gBoLive;
  bo->size = size; bo->refs = 1; bo->owner = owner;
  bo->pages = IONewZeroData(uint64_t, size >> 12);
  // A buffer of 2 MiB or more first tries one 2 MiB aligned run of physical memory, which the page table maps as 2 MiB blocks (ctxMapBo): the GPU's
  // translation of a big texture is then a few entries, not one per 4 KiB. Fragmented memory has no such run: the pages the allocator has will do.
  // mali-contig=0 skips the try.
  static unsigned contigOn = 99;
  if (contigOn == 99) { unsigned v = 1; PE_parse_boot_argn("mali-contig", &v, sizeof v); contigOn = v ? 1 : 0; }
  bool prepared = false;
  if (bo->pages && contigOn && owner && size >= kGranule && gBoPool.g) {
    const unsigned count = static_cast<unsigned>((size + kGranule - 1) / kGranule);
    unsigned *idx = IONewData(unsigned, count);
    IOAddressRange *ranges = IONewData(IOAddressRange, count);
    if (idx && ranges && boPoolTake(count, idx)) {
      size_t left = size;
      for (unsigned i = 0; i < count; ++i, left -= kGranule) {
        const PoolGranule &g = gBoPool.g[idx[i]];
        ranges[i].address = reinterpret_cast<mach_vm_address_t>(g.kva);
        ranges[i].length = left < kGranule ? left : kGranule;
        for (size_t off = 0; off < ranges[i].length; off += 0x1000) bo->pages[(static_cast<size_t>(i) * kGranule + off) >> 12] = g.pa + off;
        memset(g.kva, 0, kGranule);
      }
      bo->md = IOMemoryDescriptor::withAddressRanges(ranges, count, static_cast<IOOptionBits>(kIODirectionInOut), kernel_task);
      if (bo->md && bo->md->prepare() == kIOReturnSuccess) {
        bo->pooled = true; bo->poolIdx = idx; bo->poolCount = count;
        __asm__ volatile("dsb sy" ::: "memory");
        IODeleteData(ranges, IOAddressRange, count);
        ++gPoolHits;
        return bo;
      }
      if (bo->md) { bo->md->release(); bo->md = nullptr; }
      for (unsigned i = 0; i < count; ++i) gBoPool.g[idx[i]].used = 0;
      gBoPool.inUse -= count;
    }
    if (idx) IODeleteData(idx, unsigned, count);
    if (ranges) IODeleteData(ranges, IOAddressRange, count);
    ++gPoolMisses;
  }
  if (bo->pages && contigOn && size >= 0x200000 && !gBoPool.g) {
    bo->md = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryMapperNone) | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache,
        size, 0x000000ffffe00000ULL);
    prepared = bo->md && bo->md->prepare() == kIOReturnSuccess;
    if (bo->md && !prepared) { bo->md->release(); bo->md = nullptr; }
    ++gBoContig[prepared ? 0 : 1];
    static unsigned reported;
    if ((!prepared || size >= 0x1000000) && reported < 40) { ++reported; printf("mali: bo of %lu KiB: %s\n", static_cast<unsigned long>(size >> 10), prepared ? "one aligned run" : "no aligned run, scattered pages"); }
  }
  if (!bo->md && bo->pages) bo->md = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
      kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryMapperNone) | kIOMapWriteCombineCache, size, 0x000000fffffff000ULL);
  if (!bo->md || (!prepared && bo->md->prepare() != kIOReturnSuccess)) {
    if (bo->md) { bo->md->release(); bo->md = nullptr; }
    boFree(bo);
    return nullptr;
  }
  bo->kva = static_cast<uint8_t *>(static_cast<IOBufferMemoryDescriptor *>(bo->md)->getBytesNoCopy());
  for (size_t off = 0; off < size;) {
    IOByteCount length = 0;
    const uint64_t pa = bo->md->getPhysicalSegment(off, &length, kIOMemoryMapperNone);
    if (!pa || (pa & 0xfff) || length < 0x1000) { boFree(bo); return nullptr; }
    for (IOByteCount in = 0; in + 0x1000 <= length && off < size; in += 0x1000, off += 0x1000) bo->pages[off >> 12] = pa + in;
  }
  if (!bo->kva) { boFree(bo); return nullptr; }
  for (size_t off = 0; off < size; off += 0x1000) pmap_set_cache_attributes(static_cast<unsigned int>(bo->pages[off >> 12] >> 12), 0x6U);
  __asm__ volatile("dsb sy\n\tisb" ::: "memory");
  memset(bo->kva, 0, size);
  __asm__ volatile("dsb sy" ::: "memory");
  return bo;
}

// A buffer object over pages of a user task (VK_EXT_external_memory_host): wired for as long as the BO lives, bound by physical address like
// the others. The pages keep whatever cache attributes the task's mapping gave them; nothing is zeroed or remapped.
Bo *boCreateHost(task_t task, uint64_t ptr, uint64_t size, void *owner) {
  static unsigned reported;   // the first few refusals say why (the userland only learns "not importable")
  auto refuse = [&](const char *why, uint64_t a = 0, uint64_t b = 0) -> Bo * {
    if (reported < 24) { ++reported; printf("mali: host bo refused: %s (ptr %#llx size %#llx, %#llx %#llx, live bos %d)\n", why, ptr, size, a, b, (int)gBoLive); }
    return nullptr;
  };
  if (!task || !size || (ptr | size) & 0xfff || size > (1ull << 30) || ptr + size < ptr) return refuse("bad range");
  // Pages of a file (an application's mapped assets, which the render server shares with the application) are copied, not pinned: a pin holds the
  // object's activity count, and the unmap of the file's last mapping waits for it (vm_object_destroy) in the thread that is to retire the GPU work
  // that holds the pin: the render server stopped for good at the first application launch (xnu-probe/runs/t17). What the GPU reads of them is what
  // the file held when the buffer was made; the memory of such a mapping does not change under a reader (the application's own, writable, memory
  // and every IOSurface are not a vnode pager's and are pinned as before).
  if (evb10_range_is_file_backed(task, ptr, size)) {
    Bo *copy = boCreate(static_cast<size_t>(size), owner);
    if (!copy) return refuse("no memory for the copy of a file's pages");
    IOMemoryDescriptor *source = IOMemoryDescriptor::withAddressRange(ptr, size, static_cast<IOOptionBits>(kIODirectionOut), task);
    IOByteCount got = 0;
    if (source && source->prepare() == kIOReturnSuccess) { got = source->readBytes(0, copy->kva, static_cast<IOByteCount>(size)); (void)source->complete(); }
    if (source) source->release();
    if (got != size) { boFree(copy); return refuse("a file's pages not read", got, size); }
    __asm__ volatile("dsb sy" ::: "memory");
    copy->hostCopy = true;
    static unsigned copied;
    if (copied++ < 16) printf("mali: host bo of %#llx bytes at %#llx is a file's: copied, not pinned (%u so far, live bos %d)\n", size, ptr, copied, (int)gBoLive);
    return copy;
  }
  Bo *bo = IONewZero(Bo, 1);
  if (!bo) return refuse("no memory");
  ++gBoLive;
  bo->size = static_cast<size_t>(size); bo->refs = 1; bo->owner = owner;
  bo->pages = IONewZeroData(uint64_t, bo->size >> 12);
  bo->host = bo->pages ? IOMemoryDescriptor::withAddressRange(ptr, size, static_cast<IOOptionBits>(kIODirectionInOut), task) : nullptr;
  if (!bo->host) { const char *why = bo->pages ? "no descriptor" : "no page table"; boFree(bo); return refuse(why); }
  IOReturn prepared = bo->host->prepare();
  if (prepared != kIOReturnSuccess) {
    // Pages the task cannot write (KERN_PROTECTION_FAILURE) are wired for reading, and the GPU gets them read-only.
    const IOReturn first = prepared;
    bo->host->release();
    bo->host = IOMemoryDescriptor::withAddressRange(ptr, size, static_cast<IOOptionBits>(kIODirectionOut), task);
    prepared = bo->host ? bo->host->prepare() : kIOReturnNoMemory;
    if (prepared != kIOReturnSuccess) {
      if (bo->host) bo->host->release();
      bo->host = nullptr;
      boFree(bo);
      return refuse("prepare failed", first, prepared);
    }
    bo->readOnly = true;
  }
  for (size_t off = 0; off < bo->size;) {
    IOByteCount length = 0;
    const uint64_t pa = bo->host->getPhysicalSegment(off, &length, kIOMemoryMapperNone);
    if (!pa || (pa & 0xfff) || length < 0x1000 || pa + length > (1ull << 40)) { boFree(bo); return refuse("physical segment", pa, length); }
    for (IOByteCount in = 0; in + 0x1000 <= length && off < bo->size; in += 0x1000, off += 0x1000) bo->pages[off >> 12] = pa + in;
  }
  return bo;
}

// Panthor's panthor_mmu_as_disable(): flush, then an UPDATE with the translation mode "unmapped".
void asDisable(unsigned as) {
  gpuWrite64(asReg(as, kAsLockaddr), 63);
  (void)(asCommand(as, kAsCmdLock) && asCommand(as, kAsCmdFlushMem) && asReady(as));
  gpuWrite64(asReg(as, kAsTranstab), 0);
  gpuWrite64(asReg(as, kAsMemattr), 0);
  gpuWrite64(asReg(as, kAsTranscfg), 1);
  (void)(asCommand(as, kAsCmdUpdate) && asReady(as));
}

struct MapRec {
  uint64_t va, size, boOffset;
  Bo *bo;
};
constexpr unsigned kMaxHeapsPerCtx = 128, kHeapCtxStride = 64;   // Panthor: MAX_HEAPS_PER_POOL, ALIGN(HEAP_CONTEXT_SIZE 32, the L2 line of 64 bytes)
struct Ctx {
  bool used;
  unsigned id;              // the client's handle: index + 1
  unsigned as;              // the address space it holds now; 0: none (its page tables are all it has)
  unsigned residentGroups;  // groups of it that sit in a CSG slot: the address space stays with it while there are any
  uint64_t lastUse;
  Vm pt;
  uint64_t userVaRange;     // what a client may map: [0, userVaRange); the kernel's own objects live from there up
  uint64_t kernelNext;
  void *owner;              // the client that made it (null: the kernel's own tests)
  MapRec *maps;             // what a client mapped, in no order
  unsigned mapCount, mapCap;
  Bo *heapPool;             // the tiler heap contexts of this address space (Panthor's heap pool): kMaxHeapsPerCtx of kHeapCtxStride bytes
  uint64_t heapPoolVa;
};
Ctx gCtx[kMaxCtx];
Ctx *gAsCtx[kAsCount];     // who holds which address space (0 is the MCU's and never handed out)
uint64_t gUseClock;

// userVaRange 0: the default, everything below the kernel's region.
Ctx *ctxCreate(uint64_t userVaRange = 0, void *owner = nullptr) {
  if (userVaRange > (1ull << 47) || (userVaRange & 0xfff)) return nullptr;
  for (unsigned i = 0; i < kMaxCtx; ++i) {
    Ctx &c = gCtx[i];
    if (c.used) continue;
    memset(&c, 0, sizeof c);
    if (!vmInit(c.pt, kCtxPtPages)) { printf("mali: ctx: no memory for the page tables\n"); return nullptr; }
    c.used = true; c.id = i + 1; c.owner = owner;
    c.userVaRange = userVaRange ? userVaRange : kKernelVaBase;
    c.kernelNext = c.userVaRange;
    return &c;
  }
  printf("mali: ctx: no free context\n");
  return nullptr;
}

// Give the context an address space (a free one, else the one of the least recently used context that has no group in a slot). False: all of
// them belong to contexts with a group in a slot.
bool ctxActivate(Ctx &c) {
  c.lastUse = ++gUseClock;
  if (c.as) return true;
  unsigned as = 0;
  for (unsigned a = 1; a < kAsCount && !as; ++a)
    if (!gAsCtx[a] && (gpu(kAsPresent) & (1u << a))) as = a;
  if (!as) {
    Ctx *victim = nullptr;
    for (unsigned a = 1; a < kAsCount; ++a) {
      Ctx *x = gAsCtx[a];
      if (x && !x->residentGroups && (!victim || x->lastUse < victim->lastUse)) victim = x;
    }
    if (!victim) return false;
    as = victim->as;
    asDisable(as);
    gAsCtx[as] = nullptr;
    victim->as = 0;
  }
  gpuWrite(kMmuIntClear, 0xffffffffu);
  if (!asEnable(as, c.pt.pool.pa)) return false;
  c.as = as;
  gAsCtx[as] = &c;
  return true;
}

// The CPU's page table edits are visible to the MMU of an address space that is running; a context without one is read afresh when it gets one.
bool ctxFlush(Ctx &c) { return !c.as || asFlushAll(c.as); }

void heapsFree(Ctx *c);   // below, with the heaps

void ctxDestroy(Ctx *c) {
  if (!c || !c->used) return;
  heapsFree(c);
  if (c->as) { asDisable(c->as); gAsCtx[c->as] = nullptr; }
  for (unsigned i = 0; i < c->mapCount; ++i) boRelease(c->maps[i].bo);
  if (c->maps) IODelete(c->maps, MapRec, c->mapCap);
  vmFree(c->pt);
  memset(c, 0, sizeof *c);
}

bool ctxBind(Ctx &c, uint64_t va, const Mem &m, uint64_t leaf) { return vmMap(c.pt, va, m, leaf) && ctxFlush(c); }
uint64_t ctxKernelVa(Ctx &c, size_t size) {   // one guard page after every object
  const uint64_t va = c.kernelNext;
  c.kernelNext += ((size + 0xfff) & ~static_cast<size_t>(0xfff)) + 0x1000;
  return va;
}

uint64_t gBindPages[2];   // pages mapped for clients: [0] cached, [1] uncached
uint64_t gBindBlocks;     // of those, the ones mapped as 2 MiB blocks (in units of blocks)
// ---- what a client maps (Panthor's VM_BIND): the records say which buffer object backs which range, and keep that object alive
bool mapAdd(Ctx &c, uint64_t va, uint64_t size, uint64_t boOffset, Bo *bo) {
  if (c.mapCount == c.mapCap) {
    const unsigned cap = c.mapCap ? c.mapCap * 2 : 64;
    MapRec *fresh = IONewZero(MapRec, cap);
    if (!fresh) return false;
    if (c.maps) { memcpy(fresh, c.maps, c.mapCount * sizeof(MapRec)); IODelete(c.maps, MapRec, c.mapCap); }
    c.maps = fresh; c.mapCap = cap;
  }
  c.maps[c.mapCount++] = MapRec{va, size, boOffset, bo};
  ++bo->refs;
  return true;
}

IOReturn ctxMapBo(Ctx &c, Bo *bo, uint64_t boOffset, uint64_t va, uint64_t size, uint32_t flags) {
  if (!bo || !size || ((va | size | boOffset) & 0xfff) || boOffset > bo->size || size > bo->size - boOffset) return kIOReturnBadArgument;
  if (va >= c.userVaRange || size > c.userVaRange - va) return kIOReturnBadArgument;
  for (unsigned i = 0; i < c.mapCount; ++i)
    if (va < c.maps[i].va + c.maps[i].size && c.maps[i].va < va + size) return kIOReturnNoSpace;   // overlaps something mapped
  // mali-leaf=1 maps every client buffer write-back cached, =2 uncached (whatever the client asked for): the frames take the same time at
  // 198 and 594 MHz, so what the GPU's memory accesses cost is the next suspect.
  static unsigned leafForce = 99;
  if (leafForce == 99) { leafForce = 0; PE_parse_boot_argn("mali-leaf", &leafForce, sizeof leafForce); }
  const bool uncached = leafForce == 1 ? false : leafForce == 2 ? true : (flags & MALI_BIND_UNCACHED) != 0;
  gBindPages[uncached ? 1 : 0] += size >> 12;
  uint64_t leaf = uncached ? kLeafData : kLeafCached;
  if ((flags & MALI_BIND_READONLY) || bo->readOnly) leaf |= kPteRdOnly;
  if (!(flags & MALI_BIND_NOEXEC)) leaf &= ~kPteXn;
  // mali-blocks=0 maps pages only (the default maps a 2 MiB block wherever the virtual window and the physical run are both 2 MiB aligned).
  static unsigned blocksOn = 99;
  if (blocksOn == 99) { unsigned v = 1; PE_parse_boot_argn("mali-blocks", &v, sizeof v); blocksOn = v ? 1 : 0; }
  for (uint64_t off = 0; off < size;) {
    const uint64_t *run = &bo->pages[(boOffset + off) >> 12];
    bool block = blocksOn && !((va + off) & 0x1fffff) && size - off >= 0x200000 && !(run[0] & 0x1fffff);
    for (unsigned i = 1; block && i < 512; ++i) block = run[i] == run[0] + static_cast<uint64_t>(i) * 0x1000;
    if (block) {
      if (!ptMapBlock(c.pt, va + off, run[0], leaf)) { vmUnmap(c.pt, va, off); return kIOReturnNoMemory; }
      off += 0x200000;
      ++gBindBlocks;
    } else {
      if (!ptMap(c.pt, va + off, run[0], leaf)) { vmUnmap(c.pt, va, off); return kIOReturnNoMemory; }
      off += 0x1000;
    }
  }
  if (!mapAdd(c, va, size, boOffset, bo)) { vmUnmap(c.pt, va, size); return kIOReturnNoMemory; }
  return ctxFlush(c) ? kIOReturnSuccess : kIOReturnTimeout;
}

// Remove [va, va + size) from the records, splitting the ones that stick out on either side, and clear the page table entries.
IOReturn ctxUnmap(Ctx &c, uint64_t va, uint64_t size) {
  if (!size || ((va | size) & 0xfff) || va >= c.userVaRange || size > c.userVaRange - va) return kIOReturnBadArgument;
  const uint64_t end = va + size;
  bool touched = false;
  for (unsigned i = 0; i < c.mapCount;) {
    MapRec r = c.maps[i];
    if (!(va < r.va + r.size && r.va < end)) { ++i; continue; }
    touched = true;
    const bool left = r.va < va, right = r.va + r.size > end;
    if (right && !left) {            // only the right part stays
      c.maps[i] = MapRec{end, r.va + r.size - end, r.boOffset + (end - r.va), r.bo};
      ++i;
    } else if (left) {               // the left part stays; a right part becomes a record of its own (one more reference to the object)
      c.maps[i].size = va - r.va;
      ++i;
      if (right && !mapAdd(c, end, r.va + r.size - end, r.boOffset + (end - r.va), r.bo)) return kIOReturnNoMemory;
    } else {                         // gone altogether
      boRelease(r.bo);
      c.maps[i] = c.maps[--c.mapCount];
    }
  }
  if (!touched) return kIOReturnNotFound;
  vmUnmap(c.pt, va, size);
  return ctxFlush(c) ? kIOReturnSuccess : kIOReturnTimeout;
}

// ---- tiler heaps (Panthor's panthor_heap.c): the firmware's tiler takes chunks from a chain whose head the heap context names, and each chunk
// starts with a 64 byte header whose first word is the next chunk's address (page aligned) ORed with its size in 4 KiB pages. Only the chunks
// made at creation exist: when they run out the firmware reports an out-of-memory exception that nobody handles yet (needs interrupts).
// The context pool and the chunks live in the address space's kernel region, mapped write-back like Panthor's kernel buffer objects.
struct Heap {
  bool used;
  Ctx *ctx;
  void *owner;
  uint32_t chunkSize, chunkCount, maxChunks;
  Bo **chunks;
  uint64_t *chunkVa;
  uint64_t ctxVa;
};
constexpr unsigned kMaxHeaps = 64;
Heap gHeap[kMaxHeaps];

uint64_t ctxKernelVaAligned(Ctx &c, size_t size, uint64_t align) {
  c.kernelNext = (c.kernelNext + align - 1) & ~(align - 1);
  return ctxKernelVa(c, size);
}

// Map a buffer object in the kernel's region (the client-facing ctxMapBo refuses it).
bool ctxMapKernelBo(Ctx &c, Bo *bo, uint64_t va, uint64_t leaf) {
  for (uint64_t off = 0; off < bo->size; off += 0x1000)
    if (!ptMap(c.pt, va + off, bo->pages[off >> 12], leaf)) { vmUnmap(c.pt, va, off); return false; }
  return ctxFlush(c);
}

void heapFree(Heap &h) {
  if (!h.used) return;
  for (uint32_t i = 0; i < h.chunkCount; ++i) {
    if (!h.chunks[i]) continue;
    vmUnmap(h.ctx->pt, h.chunkVa[i], h.chunks[i]->size);
    boRelease(h.chunks[i]);
  }
  if (h.chunks) IODelete(h.chunks, Bo *, h.chunkCount);
  if (h.chunkVa) IODeleteData(h.chunkVa, uint64_t, h.chunkCount);
  memset(&h, 0, sizeof h);
}

void heapsFree(Ctx *c) {
  for (unsigned i = 0; i < kMaxHeaps; ++i)
    if (gHeap[i].used && gHeap[i].ctx == c) heapFree(gHeap[i]);
  if (c->heapPool) {
    vmUnmap(c->pt, c->heapPoolVa, c->heapPool->size);
    boRelease(c->heapPool);
    c->heapPool = nullptr;
  }
}

// initialChunks chunks of chunkSize (a multiple of 4 KiB, 128 KiB .. 8 MiB), chained newest first. Returns the heap's index or -1.
int heapCreate(Ctx &c, uint32_t initialChunks, uint32_t chunkSize, uint32_t maxChunks, void *owner, uint64_t *ctxVa, uint64_t *firstChunkVa) {
  if (!initialChunks || initialChunks > maxChunks || (chunkSize & 0xfff) || chunkSize < 0x20000 || chunkSize > 0x800000) return -1;
  if (!c.heapPool) {
    Bo *pool = boCreate(kMaxHeapsPerCtx * kHeapCtxStride, nullptr);
    if (!pool) return -1;
    const uint64_t va = ctxKernelVaAligned(c, pool->size, 0x1000);
    if (!ctxMapKernelBo(c, pool, va, kLeafCached)) { boRelease(pool); return -1; }
    c.heapPool = pool;
    c.heapPoolVa = va;
  }
  unsigned slot = 0;
  while (slot < kMaxHeaps && gHeap[slot].used) ++slot;
  if (slot == kMaxHeaps) return -1;
  // the context index inside this address space's pool: the lowest one no heap of this space has
  unsigned id = 0;
  for (; id < kMaxHeapsPerCtx; ++id) {
    bool taken = false;
    for (unsigned i = 0; i < kMaxHeaps; ++i) taken = taken || (gHeap[i].used && gHeap[i].ctx == &c && gHeap[i].ctxVa == c.heapPoolVa + id * kHeapCtxStride);
    if (!taken) break;
  }
  if (id == kMaxHeapsPerCtx) return -1;

  Heap &h = gHeap[slot];
  memset(&h, 0, sizeof h);
  h.ctx = &c; h.owner = owner; h.chunkSize = chunkSize; h.maxChunks = maxChunks;
  h.chunks = IONewZero(Bo *, initialChunks);
  h.chunkVa = IONewZeroData(uint64_t, initialChunks);
  h.used = true;
  if (!h.chunks || !h.chunkVa) { h.chunkCount = initialChunks; heapFree(h); return -1; }
  h.chunkCount = initialChunks;
  uint64_t previous = 0;
  for (uint32_t i = 0; i < initialChunks; ++i) {
    Bo *bo = boCreate(chunkSize, nullptr);
    uint64_t va = 0;
    if (bo) {
      va = ctxKernelVaAligned(c, chunkSize, chunkSize >= 0x200000 ? 0x200000 : 0x1000);
      if (!ctxMapKernelBo(c, bo, va, kLeafCached)) { boRelease(bo); bo = nullptr; }
    }
    if (!bo) { heapFree(h); return -1; }
    h.chunks[i] = bo; h.chunkVa[i] = va;
    uint64_t *header = reinterpret_cast<uint64_t *>(bo->kva);   // memory comes back zeroed: only the link
    if (previous) header[0] = (previous & ~0xfffull) | (chunkSize >> 12);
    previous = va;
  }
  __asm__ volatile("dsb sy" ::: "memory");
  memset(c.heapPool->kva + id * kHeapCtxStride, 0, kHeapCtxStride);
  h.ctxVa = c.heapPoolVa + id * kHeapCtxStride;
  *ctxVa = h.ctxVa;
  *firstChunkVa = previous;
  printf("mali: tiler heap %u in address space %u: %u chunks of %#x at %#llx.. (head %#llx), context at %#llx\n", slot, c.as, initialChunks, chunkSize,
         static_cast<unsigned long long>(h.chunkVa[0]), static_cast<unsigned long long>(previous), static_cast<unsigned long long>(h.ctxVa));
  return static_cast<int>(slot);
}

// What lives in the MCU's address space for a CSG slot: the queues' input / output pages and the suspend buffers. Allocated on first use, kept.
struct SlotRes {
  bool ready;
  Mem iface[kMaxQueues];
  uint32_t ifaceVa[kMaxQueues];
  Mem suspend, protm;
  uint32_t suspendVa, protmVa;
};
SlotRes gSlotRes[kMaxGroups];
uint32_t gMcuNext = 0x04300000;

bool mcuAlloc(Mem &m, size_t size, uint32_t &va) {
  if (!memAlloc(m, size)) return false;
  if (gMcuNext + m.size > kMcuSharedEnd) { memFree(m); return false; }
  va = gMcuNext;
  gMcuNext += static_cast<uint32_t>(m.size);
  if (!vmMap(gMcuVm, va, m, kLeafData)) return false;
  return true;
}

struct Queue {
  Mem ring;
  uint64_t ringVa;
  uint32_t ringSize;
  uint64_t insert, seqno;   // byte offset ever written / jobs ever submitted
};
struct GroupParams {
  unsigned queues;
  uint32_t ringSize[kMaxQueues];
  uint8_t queuePriority[kMaxQueues];
  uint64_t computeMask, fragmentMask, tilerMask;
  unsigned maxCompute, maxFragment, maxTiler, priority;
};
struct Group {
  bool used;
  unsigned id;            // the client's handle: index + 1
  Ctx *ctx;
  void *owner;
  unsigned csg, queues;   // csg: the CSG slot it sits in now, kNoSlot when in none (the rings and syncobjs are all it has then)
  uint64_t lastUse;
  GroupParams gp;
  Queue q[kMaxQueues];
  Mem sync;               // one { seqno, status } pair of 16 bytes per queue (Panthor's panthor_syncobj_64b)
  uint64_t syncVa;
  uint64_t serial;        // never repeats: tells a group from the one that had its place before (the display's fence)
};
uint64_t gGroupSerial;
Group gGroup[kMaxGroupObjs];
Group *gSlotGroup[kMaxGroups];   // who sits in which slot
uint32_t gSkipSlots;   // boot-arg mali-skipslot: a mask of CSG slots no group is put into (an experiment: a slot that never runs its jobs)

struct SlotView {
  uint8_t *base;
  volatile CsgControl *ctl;
  volatile CsgInput *in;
  volatile CsgOutput *out;
};
struct CsView {
  volatile CsControl *ctl;
  volatile CsInput *in;
  volatile CsOutput *out;
};

bool slotView(unsigned csg, SlotView &s) {
  s.base = gShared->mem.kva + 0x1000 + csg * gGlbControl->groupStride;
  s.ctl = reinterpret_cast<volatile CsgControl *>(s.base);
  s.in = static_cast<volatile CsgInput *>(sharedAt(s.ctl->inputVa));
  s.out = static_cast<volatile CsgOutput *>(sharedAt(s.ctl->outputVa));
  return s.in && s.out;
}
bool csView(const SlotView &s, unsigned cs, CsView &c) {
  c.ctl = reinterpret_cast<volatile CsControl *>(s.base + 0x40 + cs * s.ctl->streamStride);
  c.in = static_cast<volatile CsInput *>(sharedAt(c.ctl->inputVa));
  c.out = static_cast<volatile CsOutput *>(sharedAt(c.ctl->outputVa));
  return c.in && c.out;
}
void ringGlobalDoorbell(unsigned csg) {
  gGlbInput->doorbellReq = (gGlbInput->doorbellReq & ~(1u << csg)) | ((~gGlbOutput->doorbellAck) & (1u << csg));
  __asm__ volatile("dsb sy" ::: "memory");
  gpuWrite(kDoorbell0, 1);
}

void groupRelease(Group &g) {
  for (unsigned q = 0; q < kMaxQueues; ++q) memFree(g.q[q].ring);
  memFree(g.sync);
  g.used = false;
  g.ctx = nullptr;
}

GroupParams defaultParams(unsigned queues, uint32_t ringSize) {
  GroupParams p{};
  p.queues = queues;
  for (unsigned q = 0; q < kMaxQueues; ++q) p.ringSize[q] = ringSize;
  p.computeMask = p.fragmentMask = gpu(kShaderPresent);
  p.tilerMask = gpu(kTilerPresent);
  p.maxCompute = p.maxFragment = 4;
  p.maxTiler = 1;
  p.priority = 15;
  return p;
}

// A group in a slot that has nothing left to run: the firmware read the whole ring and the syncobj reached every job's number.
bool groupIdle(Group &g) {
  if (g.csg == kNoSlot) return true;
  for (unsigned q = 0; q < g.queues; ++q) {
    const volatile uint64_t *done = reinterpret_cast<volatile uint64_t *>(g.sync.kva + q * 16);
    const volatile RingOutput *ro = reinterpret_cast<volatile RingOutput *>(gSlotRes[g.csg].iface[q].kva + 0x1000);
    if (*done < g.q[q].seqno || ro->extract != g.q[q].insert) return false;
  }
  return true;
}

// Take the group out of its slot (what it has run is done, or it is being destroyed).
void groupStop(Group &g) {
  if (g.csg == kNoSlot) return;
  SlotView s;
  if (slotView(g.csg, s)) {
    s.in->req = s.in->req & ~kCsgStateMask;   // CSG_STATE_TERMINATE = 0
    ringGlobalDoorbell(g.csg);
    const bool gone = waitUs(1000000, [&s] { return (s.out->ack & kCsgStateMask) == 0; });
    if (!gone) { printf("mali: group %u did NOT terminate (req %#x ack %#x)\n", g.csg, s.in->req, s.out->ack); faultReport(); }
    for (unsigned q = 0; q < g.queues; ++q) {
      CsView c;
      if (csView(s, q, c)) c.in->req = c.in->req & ~kCsStateMask;   // CS_STATE_STOP
    }
    __asm__ volatile("dsb sy" ::: "memory");
  }
  gSlotGroup[g.csg] = nullptr;
  if (g.ctx && g.ctx->residentGroups) --g.ctx->residentGroups;
  g.csg = kNoSlot;
}

// Put the group into the given slot (its context already holds an address space). The queues take up where they left off: insert = extract = what has been written so far.
bool groupStart(Group &g, unsigned csg) {
  Ctx &ctx = *g.ctx;
  const GroupParams &gp = g.gp;
  const unsigned queues = g.queues;
  SlotView s;
  if (!slotView(csg, s)) { printf("mali: group: slot %u interface outside the shared section\n", csg); return false; }
  if (queues > s.ctl->streamNum) { printf("mali: group: %u queues wanted, the slot has %u\n", queues, s.ctl->streamNum); return false; }

  SlotRes &sr = gSlotRes[csg];
  if (!sr.ready) {
    const size_t suspendSize = s.ctl->suspendSize ? s.ctl->suspendSize : 0x1000, protmSize = s.ctl->protmSuspendSize;
    bool ok = true;
    for (unsigned q = 0; ok && q < kMaxQueues; ++q) ok = mcuAlloc(sr.iface[q], 0x2000, sr.ifaceVa[q]);
    ok = ok && mcuAlloc(sr.suspend, suspendSize, sr.suspendVa);
    if (ok && protmSize) ok = mcuAlloc(sr.protm, protmSize, sr.protmVa);
    if (!ok || !asFlushAll(0)) { printf("mali: group: no memory for the slot %u interface pages\n", csg); return false; }
    sr.ready = true;
  }
  memset(sr.suspend.kva, 0, sr.suspend.size);
  if (sr.protm.md) memset(sr.protm.kva, 0, sr.protm.size);
  for (unsigned q = 0; q < kMaxQueues; ++q) memset(sr.iface[q].kva, 0, sr.iface[q].size);
  __asm__ volatile("dsb sy" ::: "memory");

  // cs_slot_prog_locked() for every queue, then csg_slot_prog_locked()
  unsigned mask = 0;
  const uint32_t csMask = kCsIdleSyncWait | kCsIdleEmpty | kCsStateMask | kCsExtractEvent;
  for (unsigned q = 0; q < queues; ++q) {
    CsView c;
    if (!csView(s, q, c)) { printf("mali: group: stream %u interface outside the shared section\n", q); return false; }
    volatile RingInput *ri = reinterpret_cast<volatile RingInput *>(sr.iface[q].kva);
    volatile RingOutput *ro = reinterpret_cast<volatile RingOutput *>(sr.iface[q].kva + 0x1000);
    ro->extract = g.q[q].insert;
    ri->insert = g.q[q].insert;
    ri->extract = ro->extract;
    c.in->ringbufBase = g.q[q].ringVa;
    c.in->ringbufSize = g.q[q].ringSize;
    c.in->ringbufInput = sr.ifaceVa[q];
    c.in->ringbufOutput = sr.ifaceVa[q] + 0x1000;
    c.in->config = (gp.queuePriority[q] & 0xfu) | ((csg + 1) << 8);
    c.in->ackIrqMask = ~0u;
    c.in->req = (c.in->req & ~csMask) | ((kCsIdleSyncWait | kCsIdleEmpty | kCsStateStart | kCsExtractEvent) & csMask);
    mask |= 1u << q;
  }
  s.in->allowCompute = gp.computeMask;
  s.in->allowFragment = gp.fragmentMask;
  s.in->allowOther = gp.tilerMask;
  s.in->endpointReq = (gp.maxCompute & 0xffu) | ((gp.maxFragment & 0xffu) << 8) | ((gp.maxTiler & 0xffu) << 16) | ((gp.priority & 0xfu) << 28);
  s.in->config = ctx.as;
  s.in->suspendBuf = sr.suspendVa;
  s.in->protmSuspendBuf = sr.protm.md ? sr.protmVa : 0;
  s.in->ackIrqMask = ~0u;
  s.in->doorbellReq = (s.in->doorbellReq & ~mask) | ((~s.out->doorbellAck) & mask);
  __asm__ volatile("dsb sy" ::: "memory");
  s.in->req = (s.in->req & ~(kCsgStateMask | kCsgEndpointConfig)) | kCsgStateStart | ((s.out->ack ^ kCsgEndpointConfig) & kCsgEndpointConfig);
  ringGlobalDoorbell(csg);
  const bool started = waitUs(1000000, [&s] { return (s.out->ack & (kCsgStateMask | kCsgEndpointConfig)) == (kCsgStateStart | (s.in->req & kCsgEndpointConfig)); });
  if (!started) {
    printf("mali: group: slot %u did NOT start (req %#x ack %#x)\n", csg, s.in->req, s.out->ack);
    faultReport();
    return false;
  }
  for (unsigned q = 0; q < queues; ++q) {
    CsView c;
    (void)csView(s, q, c);
    if (!waitUs(300000, [&c] { return (c.out->ack & kCsStateMask) == kCsStateStart; })) {
      printf("mali: group: slot %u stream %u did NOT start (req %#x ack %#x)\n", csg, q, c.in->req, c.out->ack);
      faultReport();
    }
  }
  gSlotGroup[csg] = &g;
  g.csg = csg;
  ++ctx.residentGroups;
  g.lastUse = ++gUseClock;
  static unsigned shown;
  CsView c0;
  if (shown++ < 32)
    printf("mali: group %u in slot %u: address space %u, %u queues, rings %#x bytes at %#llx.., syncobjs at %#llx, cs features %#x\n", g.id, csg, ctx.as, queues, g.q[0].ringSize,
           static_cast<unsigned long long>(g.q[0].ringVa), static_cast<unsigned long long>(g.syncVa), csView(s, 0, c0) ? c0.ctl->features : 0u);
  return true;
}

// Make room: take the least recently used group that sits idle in a slot out of it.
bool groupEvictIdle(const Group *except) {
  Group *victim = nullptr;
  for (unsigned i = 0; i < kMaxGroups; ++i) {
    Group *x = gSlotGroup[i];
    if (!x || x == except || !groupIdle(*x)) continue;
    if (!victim || x->lastUse < victim->lastUse) victim = x;
  }
  if (!victim) return false;
  groupStop(*victim);
  return true;
}

// 0: the group sits in a slot now; 1: every slot or address space is held by a group with work in flight (try again later); -1: it would not start.
int groupActivate(Group &g) {
  g.lastUse = ++gUseClock;
  if (g.csg != kNoSlot) return 0;
  Ctx &ctx = *g.ctx;
  ++ctx.residentGroups;   // keeps the address space with the context while this goes on
  while (!ctxActivate(ctx))
    if (!groupEvictIdle(&g)) { --ctx.residentGroups; return 1; }
  unsigned csg = kNoSlot;
  for (;;) {
    for (unsigned i = 0; i < gGlbControl->groupNum && i < kMaxGroups && csg == kNoSlot; ++i)
      if (!gSlotGroup[i] && !(gSkipSlots & (1u << i))) csg = i;
    if (csg != kNoSlot) break;
    if (!groupEvictIdle(&g)) { --ctx.residentGroups; return 1; }
  }
  --ctx.residentGroups;
  return groupStart(g, csg) ? 0 : -1;
}

void groupDestroy(Group *g) {
  if (!g || !g->used) return;
  groupStop(*g);
  static unsigned shown;
  if (shown++ < 32) printf("mali: group %u destroyed\n", g->id);
  groupRelease(*g);
}

// A group's rings and syncobjs are made here; activate: put it into a slot at once (the kernel's own tests), else it gets one when it first has work.
Group *groupCreate(Ctx &ctx, const GroupParams &gp, void *owner = nullptr, bool activate = true) {
  const unsigned queues = gp.queues;
  if (!queues || queues > kMaxQueues) return nullptr;
  for (unsigned q = 0; q < queues; ++q)
    if (gp.ringSize[q] < 0x1000 || gp.ringSize[q] > 0x100000 || (gp.ringSize[q] & (gp.ringSize[q] - 1))) return nullptr;
  unsigned idx = 0;
  while (idx < kMaxGroupObjs && gGroup[idx].used) ++idx;
  if (idx == kMaxGroupObjs) { printf("mali: group: no free group\n"); return nullptr; }

  Group &g = gGroup[idx];
  memset(&g, 0, sizeof g);
  g.serial = ++gGroupSerial;
  g.used = true; g.id = idx + 1; g.ctx = &ctx; g.owner = owner; g.csg = kNoSlot; g.queues = queues; g.gp = gp; g.lastUse = ++gUseClock;
  bool ok = memAlloc(g.sync, 0x1000);
  if (ok) { g.syncVa = ctxKernelVa(ctx, 0x1000); ok = ctxBind(ctx, g.syncVa, g.sync, kLeafData); }
  for (unsigned q = 0; ok && q < queues; ++q) {
    Queue &qu = g.q[q];
    qu.ringSize = gp.ringSize[q];
    ok = memAlloc(qu.ring, qu.ringSize);
    if (ok) { qu.ringVa = ctxKernelVa(ctx, qu.ringSize); ok = ctxBind(ctx, qu.ringVa, qu.ring, kLeafData); }
  }
  if (!ok) { printf("mali: group: no memory for the rings\n"); groupRelease(g); return nullptr; }
  if (activate && groupActivate(g) != 0) {
    printf("mali: group: no free slot or address space\n");
    groupDestroy(&g);
    return nullptr;
  }
  return &g;
}

// Panthor's prepare_job_instrs() without profiling: flush, CALL the user's stream, wait for everything, bump the syncobj, error barrier.
// Returns the job's sequence number in *seqno, or false when the ring has no room (the caller waits and retries).
bool groupSubmit(Group &g, unsigned q, uint64_t csVa, uint32_t csSize, uint32_t latestFlush, uint64_t *seqno) {
  if (q >= g.queues) return false;
  Queue &qu = g.q[q];
  SlotView s;
  CsView c;
  if (g.csg == kNoSlot || !slotView(g.csg, s) || !csView(s, q, c)) return false;
  volatile RingInput *ri = reinterpret_cast<volatile RingInput *>(gSlotRes[g.csg].iface[q].kva);
  volatile RingOutput *ro = reinterpret_cast<volatile RingOutput *>(gSlotRes[g.csg].iface[q].kva + 0x1000);
  if (qu.insert + kJobBytes - ro->extract > qu.ringSize) return false;

  const unsigned regs = (c.ctl->features & 0xff) + 1;
  if (regs < 8) return false;
  const uint64_t addrReg = regs - 4, valReg = addrReg + 2;
  const unsigned waitAll = (1u << ((c.ctl->features >> 8) & 0xff)) - 1;
  const uint64_t syncVa = g.syncVa + q * 16;
  // mali-postflush=1: after the user's stream has finished (WAIT all), clean the GPU's L2 and load/store caches into memory before the
  // syncobj says the job is done. The panel's scanout (the VOP) and the CPU read memory, not the GPU's caches, and Panthor's own job end has no
  // flush of its own: the last megabyte or two of a render target can still sit in the L2 when the job "completes".
  static unsigned postFlush = 99;
  if (postFlush == 99) { postFlush = 0; PE_parse_boot_argn("mali-postflush", &postFlush, sizeof postFlush); }
  const uint64_t zeroReg = addrReg - 1;
  uint64_t instrs[kJobSlots] = {};
  unsigned n = 0;
  instrs[n++] = (2ull << 56) | (valReg << 48) | latestFlush;                       // MOV32 val, latest_flush
  instrs[n++] = (36ull << 56) | (valReg << 40) | 0x233;                            // FLUSH_CACHE2.clean_inv_all.no_wait.signal(0) val
  instrs[n++] = (1ull << 56) | (addrReg << 48) | csVa;                             // MOV48 addr, cs.start
  instrs[n++] = (2ull << 56) | (valReg << 48) | csSize;                            // MOV32 val, cs.size
  instrs[n++] = (3ull << 56) | (1u << 16);                                         // WAIT(0): the flush
  instrs[n++] = (32ull << 56) | (addrReg << 40) | (valReg << 32);                  // CALL addr, val
  instrs[n++] = (1ull << 56) | (addrReg << 48) | syncVa;                           // MOV48 addr, sync_addr
  instrs[n++] = (1ull << 56) | (valReg << 48) | 1;                                 // MOV48 val, 1
  instrs[n++] = (3ull << 56) | (static_cast<uint64_t>(waitAll) << 16);             // WAIT(all)
  if (postFlush) {
    instrs[n++] = (2ull << 56) | (zeroReg << 48);                                  // MOV32 zero, 0 (a flush id register holding 0 always flushes)
    instrs[n++] = csFlushCaches(1, 1, 0, static_cast<unsigned>(zeroReg), 0);       // FLUSH_CACHE2: L2 clean, load/store cache clean, signal(0)
    instrs[n++] = (3ull << 56) | (1u << 16);                                       // WAIT(0): the flush
  }
  instrs[n++] = (51ull << 56) | (addrReg << 40) | (valReg << 32) | 1;              // SYNC_ADD64.system_scope.nowait addr, val
  instrs[n++] = (47ull << 56);                                                     // ERROR_BARRIER
  const uint64_t start = qu.insert & (qu.ringSize - 1);
  const size_t first = (qu.ringSize - start) < kJobBytes ? (qu.ringSize - start) : kJobBytes;
  memcpy(qu.ring.kva + start, instrs, first);
  if (first < kJobBytes) memcpy(qu.ring.kva, reinterpret_cast<const uint8_t *>(instrs) + first, kJobBytes - first);
  __asm__ volatile("dsb sy" ::: "memory");
  qu.insert += kJobBytes;
  *seqno = ++qu.seqno;
  ri->extract = ro->extract;
  ri->insert = qu.insert;
  __asm__ volatile("dsb sy" ::: "memory");
  gpuWrite(kDoorbell0 + 0x10000 * (g.csg + 1), 1);
  return true;
}

bool groupWait(Group &g, unsigned q, uint64_t seqno, unsigned timeoutUs) {
  volatile uint64_t *done = reinterpret_cast<volatile uint64_t *>(g.sync.kva + q * 16);
  return waitUs(timeoutUs, [done, seqno] { return *done >= seqno; });
}

// Say what went wrong on the GPU as it happens: an MMU fault (which address space, which address) and a queue that faulted or died (what the firmware says).
// Polled, like everything else of this driver: after the calls that touch the groups and when a fence wait ran out.
void gpuFaultScan(const char *when) {
  static unsigned shown;
  static uint32_t lastFault[kMaxGroups][kMaxQueues], lastFatal[kMaxGroups][kMaxQueues];
  if (shown >= 80) return;
  const uint32_t raw = gpu(kMmuIntRaw);
  if (raw) {
    for (unsigned as = 0; as < kAsCount; ++as) {
      if (!(raw & ((1u << as) | (1u << (16 + as))))) continue;
      const uint32_t status = gpu(asReg(as, kAsFaultStatus));
      if (!status) continue;   // the raw bits at 16 + as are command-completed noise: only a non-zero fault status is a fault
      const uint64_t address = (static_cast<uint64_t>(gpu(asReg(as, kAsFaultAddress) + 4)) << 32) | gpu(asReg(as, kAsFaultAddress));
      printf("mali: MMU %s in AS%u (%s): status %#x address %#llx, AS status %#x [%s]\n", (raw & (1u << (16 + as))) ? "bus fault" : "page fault", as,
             gAsCtx[as] ? "a client's" : "none", status, static_cast<unsigned long long>(address), gpu(asReg(as, kAsStatus)), when);
      ++shown;
    }
    gpuWrite(kMmuIntClear, raw);
  }
  for (unsigned slot = 0; slot < kMaxGroups; ++slot) {
    Group *g = gSlotGroup[slot];
    SlotView s;
    if (!g || !slotView(slot, s)) continue;
    for (unsigned q = 0; q < g->queues; ++q) {
      CsView c;
      if (!csView(s, q, c)) continue;
      const uint32_t fault = c.out->fault, fatal = c.out->fatal;
      if ((fault && fault != lastFault[slot][q]) || (fatal && fatal != lastFatal[slot][q])) {
        printf("mali: group %u (slot %u, AS%u) queue %u: fault %#x (info %#llx) fatal %#x (info %#llx), %llu of %llu jobs done [%s]\n", g->id, slot, g->ctx->as, q, fault,
               static_cast<unsigned long long>(c.out->faultInfo), fatal, static_cast<unsigned long long>(c.out->fatalInfo),
               static_cast<unsigned long long>(*reinterpret_cast<volatile uint64_t *>(g->sync.kva + q * 16)), static_cast<unsigned long long>(g->q[q].seqno), when);
        lastFault[slot][q] = fault; lastFatal[slot][q] = fatal;
        ++shown;
      }
    }
  }
}

// ---- M3a self-test
struct TestBufs {
  Mem code, data;   // code at kCodeVa (64 bytes per program), data at kDataVa
};

bool testBufsCreate(Ctx &c, TestBufs &b) {
  if (!memAlloc(b.code, 0x2000) || !memAlloc(b.data, 0x1000)) return false;
  volatile uint32_t *w = reinterpret_cast<volatile uint32_t *>(b.data.kva);
  for (unsigned i = 0; i < 1024; ++i) w[i] = 0xdeadbeef;
  __asm__ volatile("dsb sy" ::: "memory");
  return ctxBind(c, kCodeVa, b.code, kLeafData) && ctxBind(c, kDataVa, b.data, kLeafData);
}
void testBufsDestroy(TestBufs &b) { memFree(b.code); memFree(b.data); }

// A user command stream of eight instructions: store two words at kDataVa + off, wait for the stores.
void testProgram(TestBufs &b, unsigned idx, uint32_t off, uint32_t v0, uint32_t v1) {
  volatile uint64_t *p = reinterpret_cast<volatile uint64_t *>(b.code.kva + idx * 64);
  const uint64_t program[8] = {csSetSb(3, 0), csMove48(0, kDataVa + off), csMove32(2, v0), csMove32(3, v1), csStore(2, 0, 3, 0), csWait(1u << 0), 0, 0};
  for (unsigned i = 0; i < 8; ++i) p[i] = program[i];
  __asm__ volatile("dsb sy" ::: "memory");
}
bool testCheck(TestBufs &b, uint32_t off, uint32_t v0, uint32_t v1) {
  volatile uint32_t *w = reinterpret_cast<volatile uint32_t *>(b.data.kva + off);
  return w[0] == v0 && w[1] == v1;
}
void jobFailure(Group &g, unsigned q, const char *what) {
  SlotView s; CsView c;
  printf("mali: m3: %s failed on group %u queue %u\n", what, g.csg, q);
  if (g.csg != kNoSlot && slotView(g.csg, s) && csView(s, q, c))
    printf("mali: m3: cs ack %#x fault %#x fatal %#x (info %#llx), syncobj %llu of %llu\n", c.out->ack, c.out->fault, c.out->fatal, static_cast<unsigned long long>(c.out->fatalInfo),
           static_cast<unsigned long long>(*reinterpret_cast<volatile uint64_t *>(g.sync.kva + q * 16)), static_cast<unsigned long long>(g.q[q].seqno));
  faultReport();
}

// One job: program, submit, wait for its fence, compare.
bool runJob(Group &g, TestBufs &b, unsigned q, unsigned program, uint32_t off, uint32_t v0, uint32_t v1) {
  testProgram(b, program, off, v0, v1);
  uint64_t seq = 0;
  if (!groupSubmit(g, q, kCodeVa + program * 64, 64, 0, &seq)) { jobFailure(g, q, "submit"); return false; }
  if (!groupWait(g, q, seq, 1000000)) { jobFailure(g, q, "wait"); return false; }
  if (!testCheck(b, off, v0, v1)) { jobFailure(g, q, "data check"); return false; }
  return true;
}

bool m3() {
  printf("mali: m3: contexts, groups, jobs, fences\n");
  bool ok = true;
  Ctx *c1 = ctxCreate();
  if (!c1) return false;
  TestBufs b1{};
  Group *g1 = nullptr, *g2 = nullptr, *g3 = nullptr;
  Ctx *c2 = nullptr, *c3 = nullptr;
  TestBufs b2{}, b3{};
  if (!testBufsCreate(*c1, b1) || !(g1 = groupCreate(*c1, defaultParams(3, 0x1000)))) { printf("mali: m3: FAILED to set up context 1\n"); ok = false; goto out; }

  // 1: one job on each of the three queues
  for (unsigned q = 0; q < 3 && ok; ++q) ok = runJob(*g1, b1, q, q, q * 16, 0x600d0000 + q, 0x12340000 + q);
  printf("mali: m3: one job per queue: %s\n", ok ? "OK" : "FAILED");
  if (!ok) goto out;

  // 2: forty jobs through a 4 KiB ring (128 bytes each): the ring wraps once
  for (unsigned j = 0; j < 40 && ok; ++j) ok = runJob(*g1, b1, 1, 8 + j, 0x400 + j * 8, 0x1000 + j, 0x2000 + j);
  printf("mali: m3: ring wrap, 40 jobs, queue 1 insert offset %llu: %s\n", static_cast<unsigned long long>(g1->q[1].insert), ok ? "OK" : "FAILED");
  if (!ok) goto out;

  // 3: twenty jobs queued back to back, one wait at the end
  {
    uint64_t last = 0;
    for (unsigned j = 0; j < 20 && ok; ++j) {
      testProgram(b1, 64 + j, 0x800 + j * 8, 0x3000 + j, 0x4000 + j);
      ok = groupSubmit(*g1, 2, kCodeVa + (64 + j) * 64, 64, 0, &last);
    }
    ok = ok && groupWait(*g1, 2, last, 2000000);
    for (unsigned j = 0; j < 20 && ok; ++j) ok = testCheck(b1, 0x800 + j * 8, 0x3000 + j, 0x4000 + j);
    printf("mali: m3: 20 queued jobs, fence %llu: %s\n", static_cast<unsigned long long>(last), ok ? "OK" : "FAILED");
    if (!ok) { jobFailure(*g1, 2, "burst"); goto out; }
  }

  // 4: a second context and group at the same time, the same virtual addresses backed by other memory
  c2 = ctxCreate();
  if (!c2 || !testBufsCreate(*c2, b2) || !(g2 = groupCreate(*c2, defaultParams(3, 0x1000)))) { printf("mali: m3: FAILED to set up context 2\n"); ok = false; goto out; }
  {
    testProgram(b1, 100, 0xc00, 0xa1a1a1a1, 0xa2a2a2a2);
    testProgram(b2, 100, 0xc00, 0xb1b1b1b1, 0xb2b2b2b2);
    uint64_t s1 = 0, s2 = 0;
    ok = groupSubmit(*g1, 0, kCodeVa + 100 * 64, 64, 0, &s1) && groupSubmit(*g2, 0, kCodeVa + 100 * 64, 64, 0, &s2) && groupWait(*g1, 0, s1, 1000000) && groupWait(*g2, 0, s2, 1000000);
    ok = ok && testCheck(b1, 0xc00, 0xa1a1a1a1, 0xa2a2a2a2) && testCheck(b2, 0xc00, 0xb1b1b1b1, 0xb2b2b2b2);
    printf("mali: m3: two groups at once (same VA, different memory): %s\n", ok ? "OK" : "FAILED");
    if (!ok) { jobFailure(*g1, 0, "context 1"); jobFailure(*g2, 0, "context 2"); goto out; }
  }

  // 5: tear down the first group and context, the second keeps working, a third one reuses slot 0 and address space 1
  groupDestroy(g1); g1 = nullptr;
  ctxDestroy(c1); c1 = nullptr;
  testBufsDestroy(b1);
  ok = runJob(*g2, b2, 1, 101, 0xd00, 0xc1c1c1c1, 0xc2c2c2c2);
  printf("mali: m3: group 2 after group 1 was destroyed: %s\n", ok ? "OK" : "FAILED");
  if (!ok) goto out;
  c3 = ctxCreate();
  if (!c3 || !testBufsCreate(*c3, b3) || !(g3 = groupCreate(*c3, defaultParams(2, 0x2000)))) { printf("mali: m3: FAILED to set up context 3\n"); ok = false; goto out; }
  ok = runJob(*g3, b3, 1, 5, 0x20, 0xe1e1e1e1, 0xe2e2e2e2) && runJob(*g2, b2, 2, 102, 0xe00, 0xf1f1f1f1, 0xf2f2f2f2);
  printf("mali: m3: a new group in the freed slot (group %u, address space %u): %s\n", g3->csg, c3->as, ok ? "OK" : "FAILED");

out:
  groupDestroy(g3); ctxDestroy(c3); testBufsDestroy(b3);
  groupDestroy(g2); ctxDestroy(c2); testBufsDestroy(b2);
  groupDestroy(g1); ctxDestroy(c1); testBufsDestroy(b1);
  printf("mali: m3 %s\n", ok ? "PASSED: contexts, groups, kernel-built jobs and syncobj fences work" : "FAILED");
  return ok;
}

IOLock *gLock;   // the driver's state is global and single threaded: every user client call takes it (waiting for a fence does not)

bool m3b();               // the self-test through the user client's own entry point
bool serviceCreate();
void serviceRegister();

void m1() {
  if (!loadFirmware()) return;
  if (!asEnable(0, gMcuVm.pool.pa)) return;
  if (!startMcu()) return;
  if (!globalInterface()) return;
  if (gLevel >= 6) {
    // 6: M3a only. 7: M3a, then M3b through a user client made in the kernel, then publish. 8: publish only (what userland runs against).
    if (gLevel != 8) m3();
    if (gLevel >= 7) {
      gLock = IOLockAlloc();
      if (gLock && serviceCreate()) {
        if (gLevel == 7) m3b();
        serviceRegister();
      }
    }
  } else if (gLevel >= 3) {
    m2();
  }
}

}  // namespace

// ========================================================================================================== M3b: the IOKit service and its user client
// A service with no provider matching: mali_start() makes it by hand, attaches it to the platform expert and registers it, so
// IOServiceGetMatchingService(IOServiceMatching("Mali")) finds it. Each IOServiceOpen makes an MaliClient, which owns what its
// process made (buffer objects, address spaces, groups) and gives all of it back when the process closes the connection or dies.

class MaliClient : public IOUserClient {
  OSDeclareDefaultStructors(MaliClient)

  static constexpr unsigned kMaxBoHandles = 4096;
  task_t ownerTask = nullptr;   // the process that opened this connection (BO_CREATE_HOST wires its pages)
  Bo **bos = nullptr;      // handle - 1
  bool closed = false;
  char ownerName[17] = {};         // what the process is called, for the line the connection leaves when it closes
  uint64_t boLiveBytes = 0, boPeakBytes = 0, boHostBytes = 0, boCreated = 0;   // what it asked for: a process under a memory limit is killed for its footprint

  Bo *boFor(uint32_t handle) { return (bos && handle && handle <= kMaxBoHandles) ? bos[handle - 1] : nullptr; }
  Ctx *vmFor(uint32_t id) { return (id && id <= kMaxCtx && gCtx[id - 1].used && gCtx[id - 1].owner == this) ? &gCtx[id - 1] : nullptr; }
  Group *groupFor(uint32_t id) { return (id && id <= kMaxGroupObjs && gGroup[id - 1].used && gGroup[id - 1].owner == this) ? &gGroup[id - 1] : nullptr; }
  void releaseAll();
  IOReturn dispatch(uint32_t selector, const uint8_t *in, uint32_t inSize, uint8_t *out, uint32_t outCap, uint32_t *outSize);
  IOReturn fenceWait(const mali_fence_wait_in &i, mali_fence_wait_out *o);
  bool gopAllowed() const { return strcmp(ownerName, "backboardd") == 0; }
 public:
  const char *name() const { return ownerName; }
 private:
  IOReturn gopCall(const mali_gop_in &in, union mali_gop_out *out, uint32_t *outSize);

 public:
  using IOUserClient::initWithTask;
  using IOUserClient::clientMemoryForType;
  bool initWithTask(task_t owningTask, void *securityToken, UInt32 type) override;
  IOReturn clientClose() override;
  IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *arguments, IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) override;
  IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory) override;
  void free() override;
};

OSDefineMetaClassAndStructors(MaliClient, IOUserClient)

extern "C" kmod_info_t g_kernel_kmod_info;

class MaliService : public IOService {
  OSDeclareDefaultStructors(MaliService)
 public:
  using IOService::newUserClient;
  IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler) override;
};

OSDefineMetaClassAndStructors(MaliService, IOService)

namespace {

MaliService *gService;

uint64_t gpu64(uint32_t lo) { return static_cast<uint64_t>(gpu(lo)) | (static_cast<uint64_t>(gpu(lo + 4)) << 32); }

void fillInfo(mali_dev_info &i) {
  memset(&i, 0, sizeof i);
  i.gpu_id = gpu(kGpuId); i.gpu_rev = gpu(0x280); i.csf_id = gpu(kCsfId);
  i.l2_features = gpu(kL2Features); i.tiler_features = gpu(kTilerFeatures); i.mem_features = gpu(kMemFeatures);
  i.mmu_features = gpu(kMmuFeatures); i.thread_features = gpu(kThreadFeatures); i.max_threads = gpu(kThreadMax);
  i.thread_max_workgroup_size = gpu(0xa4); i.thread_max_barrier_size = gpu(0xa8); i.coherency_features = gpu(kCoherencyFeatures);
  for (unsigned n = 0; n < 4; ++n) i.texture_features[n] = gpu(0xb0 + 4 * n);
  i.as_present = gpu(kAsPresent);
  i.shader_present = gpu64(kShaderPresent); i.l2_present = gpu64(kL2Present); i.tiler_present = gpu64(kTilerPresent);
  i.core_features = gpu(kCoreFeatures);
  SlotView s; CsView c;
  i.csg_slot_count = gGlbControl->groupNum;
  i.unpreserved_cs_reg_count = 4;
  if (slotView(0, s) && csView(s, 0, c)) {
    i.cs_slot_count = s.ctl->streamNum;
    i.cs_reg_count = (c.ctl->features & 0xff) + 1;
    i.scoreboard_slot_count = (c.ctl->features >> 8) & 0xff;
  }
  uint64_t freq;
  __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
  i.timestamp_frequency = freq;
  i.fw_version = gGlbControl->version;
  i.abi_version = 1;
}

}  // namespace

extern "C" void proc_selfname(char *buf, int size);

IOReturn MaliService::newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler) {
  if (!handler) return kIOReturnBadArgument;
  {
    static unsigned seen;   // who opens a connection (the first few dozen): which processes got past the sandbox
    char name[17] = {};
    proc_selfname(name, sizeof name);
    if (seen < 64) { ++seen; printf("mali: connection opened by %s\n", name); }
  }
  MaliClient *client = OSTypeAlloc(MaliClient);
  if (!client) return kIOReturnNoMemory;
  bool attached = false;
  if (!client->initWithTask(owningTask, securityID, type) || !(attached = client->attach(this)) || !client->start(this)) {
    if (attached) client->detach(this);
    client->release();
    return kIOReturnError;
  }
  *handler = client;
  return kIOReturnSuccess;
}

bool MaliClient::initWithTask(task_t owningTask, void *securityToken, UInt32 type) {
  if (!IOUserClient::initWithTask(owningTask, securityToken, type)) return false;
  ownerTask = owningTask;
  proc_selfname(ownerName, sizeof ownerName);
  bos = IONewZero(Bo *, kMaxBoHandles);
  return bos != nullptr;
}

void MaliClient::releaseAll() {
  IOLockLock(gLock);
  if (!closed && boCreated)
    printf("mali: %s closes its connection: %llu buffers made, %llu MB of them alive at the end, %llu MB at most, %llu MB over its own pages\n", ownerName,
           (unsigned long long)boCreated, (unsigned long long)(boLiveBytes >> 20), (unsigned long long)(boPeakBytes >> 20), (unsigned long long)(boHostBytes >> 20));
  closed = true;
  for (unsigned g = 0; g < kMaxGroupObjs; ++g)
    if (gGroup[g].used && gGroup[g].owner == this) groupDestroy(&gGroup[g]);
  for (unsigned i = 0; i < kMaxCtx; ++i)
    if (gCtx[i].used && gCtx[i].owner == this) ctxDestroy(&gCtx[i]);
  for (unsigned h = 0; bos && h < kMaxBoHandles; ++h)
    if (bos[h]) { boRelease(bos[h]); bos[h] = nullptr; }
  IOLockUnlock(gLock);
}

IOReturn MaliClient::clientClose() {
  releaseAll();
  if (!isInactive()) (void)terminate();
  return kIOReturnSuccess;
}

void MaliClient::free() {
  releaseAll();
  if (bos) { IODelete(bos, Bo *, kMaxBoHandles); bos = nullptr; }
  IOUserClient::free();
}

IOReturn MaliClient::clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory) {
  if (!options || !memory) return kIOReturnBadArgument;
  IOLockLock(gLock);
  IOMemoryDescriptor *md = nullptr;
  IOOptionBits mapOptions = kIOMapWriteCombineCache;
  bool retained = false;
  if (!closed) {
    if (type == MALI_MAP_SCANOUT) {
      if (native_gop_handoff_memory && gopAllowed()) { md = native_gop_handoff_memory(); retained = md != nullptr; }   // retained by the display driver for us
    } else if (type & MALI_MAP_SYNCOBJ) {
      Group *g = groupFor(type & 0xff);
      if (g) { md = g->sync.md; mapOptions |= kIOMapReadOnly; }
    } else if (Bo *bo = boFor(type)) {
      md = bo->host ? nullptr : bo->md;   // pages the caller owns have a view of their own already
    }
  }
  if (md && !retained) md->retain();
  IOLockUnlock(gLock);
  if (!md) return kIOReturnNotFound;
  *options = mapOptions;
  *memory = md;
  return kIOReturnSuccess;
}

IOReturn MaliClient::gopCall(const mali_gop_in &in, union mali_gop_out *out, uint32_t *outSize) {
  if (!native_gop_handoff_call) return kIOReturnUnsupported;   // the display driver is not linked in
  if (!gopAllowed()) return kIOReturnNotPermitted;
  switch (in.op) {
    case MALI_GOP_MODE: {
      const IOReturn r = native_gop_handoff_call(0, ownerTask, nullptr, 0, &out->mode, sizeof out->mode);
      if (r == kIOReturnSuccess) *outSize = sizeof out->mode;
      return r;
    }
    case MALI_GOP_WAIT: {
      uint64_t timeoutMs = in.arg;
      const IOReturn r = native_gop_handoff_call(2, ownerTask, &timeoutMs, sizeof timeoutMs, &out->swap, sizeof out->swap);
      if (r == kIOReturnSuccess) *outSize = sizeof out->swap;
      return r;
    }
    case MALI_GOP_COMPLETE: {
      struct { uint64_t sequence; uint32_t status, reserved; } done = {in.arg, in.status, 0};
      return native_gop_handoff_call(3, ownerTask, &done, sizeof done, nullptr, 0);
    }
    default: return kIOReturnBadArgument;
  }
}

IOReturn MaliClient::fenceWait(const mali_fence_wait_in &i, mali_fence_wait_out *o) {
  volatile uint64_t *done = nullptr;
  IOLockLock(gLock);
  Group *g = closed ? nullptr : groupFor(i.group);
  if (g && i.queue < g->queues) done = reinterpret_cast<volatile uint64_t *>(g->sync.kva + i.queue * 16);
  IOLockUnlock(gLock);
  if (!done) return kIOReturnBadArgument;
  uint64_t waited = 0;
  while (*done < i.seqno && waited < i.timeout_us) {
    if (waited < 200) { IODelay(10); waited += 10; }
    else { IOSleep(1); waited += 1000; }
  }
  o->done_seqno = *done;
  o->signaled = *done >= i.seqno;
  if (!o->signaled) {
    IOLockLock(gLock);
    gpuFaultScan("a fence wait ran out");
    IOLockUnlock(gLock);
  }
  return kIOReturnSuccess;
}

// How long the calls take and how long they wait for the driver's lock. A poster extension that has not checked in within a few seconds of its
// launch is killed (RunningBoard, "extension check-in timeout"), and in gpu43 its main thread sat inside these calls. Counted per selector; the
// slow ones are named, with the process, as they happen, and a table of the totals follows every ten seconds.
namespace {
struct CallStat { uint64_t n, ticks, waitTicks, maxTicks; };
CallStat gCallStat[MALI_SELECTOR_COUNT];
uint64_t gCallStatShown;
uint64_t gCallStatFreq;

inline uint64_t cntvct() {
  uint64_t v;
  __asm__ volatile("isb\n\tmrs %0, cntvct_el0" : "=r"(v));
  return v;
}

// A group in a slot with jobs that do not finish: the user side polls the syncobj without telling us, so the driver looks for itself, at most once a
// second, on any call. Says where the group stands (slot, address space, per queue: jobs done / given, ring extract / insert, what the firmware reports).
void gpuStallScan() {
  static uint64_t last, since[kMaxGroupObjs], reported[kMaxGroupObjs];
  static unsigned shown;
  if (shown >= 60) return;
  if (!gCallStatFreq) __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(gCallStatFreq));
  const uint64_t now = cntvct();
  if (last && now - last < gCallStatFreq) return;
  last = now;
  for (unsigned slot = 0; slot < kMaxGroups; ++slot) {
    Group *g = gSlotGroup[slot];
    if (!g || g->id == 0 || g->id > kMaxGroupObjs) continue;
    const unsigned idx = g->id - 1;
    bool pending = false;
    for (unsigned q = 0; q < g->queues; ++q) {
      const volatile uint64_t *done = reinterpret_cast<volatile uint64_t *>(g->sync.kva + q * 16);
      if (*done < g->q[q].seqno) pending = true;
    }
    if (!pending) { since[idx] = 0; reported[idx] = 0; continue; }
    if (!since[idx]) { since[idx] = now; continue; }
    if (now - since[idx] < 2 * gCallStatFreq || (reported[idx] && now - reported[idx] < 4 * gCallStatFreq)) continue;
    reported[idx] = now;
    SlotView s;
    if (!slotView(slot, s)) continue;
    printf("mali: STALL group %u in slot %u (AS%u) for %llu s: csg req %#x ack %#x state %#x endpoint %#x/%#x\n", g->id, slot, g->ctx ? g->ctx->as : 0u,
           static_cast<unsigned long long>((now - since[idx]) / gCallStatFreq), s.in->req, s.out->ack, s.out->statusState, s.out->statusEndpointCurrent, s.out->statusEndpointReq);
    ++shown;
    for (unsigned q = 0; q < g->queues; ++q) {
      CsView c;
      if (!csView(s, q, c)) continue;
      const volatile RingInput *ri = reinterpret_cast<volatile RingInput *>(gSlotRes[slot].iface[q].kva);
      const volatile RingOutput *ro = reinterpret_cast<volatile RingOutput *>(gSlotRes[slot].iface[q].kva + 0x1000);
      printf("mali: STALL   queue %u: jobs %llu of %llu, ring extract %#llx insert %#llx active %u, cs req %#x ack %#x, cmd %#llx blocked %u wait %#x, fault %#x fatal %#x\n", q,
             static_cast<unsigned long long>(*reinterpret_cast<volatile uint64_t *>(g->sync.kva + q * 16)), static_cast<unsigned long long>(g->q[q].seqno),
             static_cast<unsigned long long>(ro->extract), static_cast<unsigned long long>(ri->insert), ro->active, c.in->req, c.out->ack,
             static_cast<unsigned long long>(c.out->statusCmdPtr), c.out->statusBlockedReason, c.out->statusWait, c.out->fault, c.out->fatal);
    }
    printf("mali: STALL   mmu AS%u: fault status %#x, status %#x; job irq raw %#x, mcu %u\n", g->ctx ? g->ctx->as : 0u, g->ctx ? gpu(asReg(g->ctx->as, kAsFaultStatus)) : 0u,
           g->ctx ? gpu(asReg(g->ctx->as, kAsStatus)) : 0u, gpu(kJobIntRaw), gpu(kMcuStatus));
  }
}

void callStatRecord(uint32_t selector, uint64_t began, uint64_t locked, uint64_t ended) {
  if (!gCallStatFreq) __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(gCallStatFreq));
  CallStat &c = gCallStat[selector];
  const uint64_t took = ended - began;
  c.n++; c.ticks += took; c.waitTicks += locked - began;
  if (took > c.maxTicks) c.maxTicks = took;
  const uint64_t ms = took * 1000 / gCallStatFreq;
  static unsigned shown;
  if (ms >= 40 && shown < 120) {
    char name[17] = {};
    proc_selfname(name, sizeof name);
    ++shown;
    printf("mali: slow call %s sel %u: %llu ms, waited %llu ms for the lock\n", name, selector, (unsigned long long)ms,
           (unsigned long long)((locked - began) * 1000 / gCallStatFreq));
  }
  if (!gCallStatShown) gCallStatShown = ended;
  if (ended - gCallStatShown >= 10 * gCallStatFreq) {
    gCallStatShown = ended;
    for (unsigned half = 0; half < 2; ++half) {
      char line[220];
      size_t used = 0;
      line[0] = 0;
      for (unsigned i = half * 7; i < MALI_SELECTOR_COUNT && i < half * 7 + 7; ++i) {
        const CallStat &t = gCallStat[i];
        if (!t.n) continue;
        used += snprintf(line + used, sizeof line - used, " %u:%llux %llu/%llu/%llums", i, (unsigned long long)t.n, (unsigned long long)(t.ticks * 1000 / gCallStatFreq),
                         (unsigned long long)(t.waitTicks * 1000 / gCallStatFreq), (unsigned long long)(t.maxTicks * 1000 / gCallStatFreq));
        if (used >= sizeof line) break;
      }
      if (line[0]) printf("mali: calls (selector:count total/lock-wait/max):%s\n", line);
      static uint64_t lastBind[2];
      if (gBindPages[0] != lastBind[0] || gBindPages[1] != lastBind[1]) {
        lastBind[0] = gBindPages[0]; lastBind[1] = gBindPages[1];
        printf("mali: client mappings so far: %llu pages cached, %llu pages uncached, of them %llu 2 MiB blocks (%llu MiB); buffers of 2 MiB or more: %llu in one aligned run, %llu in scattered pages; pool %u of %u granules in use (peak %u), %llu buffers from it, %llu it had no room for\n",
               static_cast<unsigned long long>(gBindPages[0]), static_cast<unsigned long long>(gBindPages[1]), static_cast<unsigned long long>(gBindBlocks),
               static_cast<unsigned long long>(gBindBlocks * 2), static_cast<unsigned long long>(gBoContig[0]), static_cast<unsigned long long>(gBoContig[1]), gBoPool.inUse, gBoPool.granules, gBoPool.peak,
               static_cast<unsigned long long>(gPoolHits), static_cast<unsigned long long>(gPoolMisses));
      }
    }
  }
}
}  // namespace

IOReturn MaliClient::externalMethod(uint32_t selector, IOExternalMethodArguments *a, IOExternalMethodDispatch *, OSObject *, void *) {
  if (!a || selector >= MALI_SELECTOR_COUNT) return kIOReturnBadArgument;
  uint32_t outSize = 0;
  IOReturn r;
  const uint64_t began = cntvct();
  uint64_t locked = began;
  if (selector == MALI_GOP) {
    // Like the fence wait, outside the driver's lock: a wait for a frame can take up to a second.
    mali_gop_in in;
    union mali_gop_out out;
    memset(&out, 0, sizeof out);
    if (a->structureInputSize < sizeof in || !a->structureInput) return kIOReturnBadArgument;
    memcpy(&in, a->structureInput, sizeof in);
    r = gopCall(in, &out, &outSize);
    if (r == kIOReturnSuccess && outSize) {
      if (!a->structureOutput || a->structureOutputSize < outSize) return kIOReturnBadArgument;
      memcpy(a->structureOutput, &out, outSize);
    }
  } else if (selector == MALI_FENCE_WAIT) {
    mali_fence_wait_in in;
    mali_fence_wait_out out;
    if (a->structureInputSize < sizeof in || a->structureOutputSize < sizeof out || !a->structureInput || !a->structureOutput) return kIOReturnBadArgument;
    memcpy(&in, a->structureInput, sizeof in);
    r = fenceWait(in, &out);
    if (r == kIOReturnSuccess) { memcpy(a->structureOutput, &out, sizeof out); outSize = sizeof out; }
  } else {
    IOLockLock(gLock);
    locked = cntvct();
    for (unsigned tries = 0;; ++tries) {
      r = closed ? static_cast<IOReturn>(kIOReturnNotAttached)
                 : dispatch(selector, static_cast<const uint8_t *>(a->structureInput), a->structureInputSize, static_cast<uint8_t *>(a->structureOutput), a->structureOutputSize, &outSize);
      if (selector == MALI_GROUP_SUBMIT || selector == MALI_GROUP_DESTROY) gpuFaultScan(selector == MALI_GROUP_SUBMIT ? "after a submit" : "after a group destroy");
      gpuStallScan();
      if (r != kIOReturnNotReady || tries >= 10000) break;   // a submit that found no slot: the busy groups finish on their own, without the lock
      IOLockUnlock(gLock);
      IOSleep(1);
      IOLockLock(gLock);
    }
    IOLockUnlock(gLock);
  }
  if (r == kIOReturnSuccess) a->structureOutputSize = outSize;
  if (selector != MALI_LOG && selector != MALI_GOP) callStatRecord(selector, began, locked, cntvct());   // (a wait for a frame is mostly idle time)
  return r;
}

#define NEED_IN(T, name) \
  T name; \
  if (!in || inSize < sizeof(T)) return kIOReturnBadArgument; \
  memcpy(&name, in, sizeof(T))
#define PUT_OUT(T, value) \
  do { \
    if (!out || outCap < sizeof(T)) return kIOReturnBadArgument; \
    memcpy(out, &(value), sizeof(T)); \
    *outSize = sizeof(T); \
  } while (0)

IOReturn MaliClient::dispatch(uint32_t selector, const uint8_t *in, uint32_t inSize, uint8_t *out, uint32_t outCap, uint32_t *outSize) {
  switch (selector) {
    case MALI_DEV_QUERY: {
      mali_dev_info info;
      fillInfo(info);
      PUT_OUT(mali_dev_info, info);
      return kIOReturnSuccess;
    }
    case MALI_VM_CREATE: {
      NEED_IN(mali_vm_create_in, i);
      Ctx *c = ctxCreate(i.user_va_range, this);
      if (!c) return kIOReturnNoResources;
      mali_vm_create_out o = {c->id, 0, c->userVaRange};
      PUT_OUT(mali_vm_create_out, o);
      return kIOReturnSuccess;
    }
    case MALI_VM_DESTROY: {
      NEED_IN(mali_vm_destroy_in, i);
      Ctx *c = vmFor(i.vm);
      if (!c) return kIOReturnBadArgument;
      for (unsigned g = 0; g < kMaxGroupObjs; ++g)
        if (gGroup[g].used && gGroup[g].ctx == c) return kIOReturnBusy;
      ctxDestroy(c);
      return kIOReturnSuccess;
    }
    case MALI_BO_CREATE: {
      NEED_IN(mali_bo_create_in, i);
      if (!i.size || i.size > (1ull << 32)) return kIOReturnBadArgument;
      unsigned h = 0;
      while (h < kMaxBoHandles && bos[h]) ++h;
      if (h == kMaxBoHandles) return kIOReturnNoResources;
      Bo *bo = boCreate(static_cast<size_t>(i.size), this);
      if (!bo) return kIOReturnNoMemory;
      bos[h] = bo;
      boCreated++;
      if (bo->size >= (4ull << 20)) {   // what a client's memory is made of: the big ones, by process and handle (the first few dozen)
        static unsigned big;
        if (big++ < 160) printf("mali: %s makes buffer %u of %llu KiB (%llu MiB alive now)\n", ownerName, h + 1, (unsigned long long)(bo->size >> 10), (unsigned long long)(boLiveBytes >> 20));
      }
      boLiveBytes += bo->size;
      if (boLiveBytes > boPeakBytes) boPeakBytes = boLiveBytes;
      mali_bo_create_out o = {h + 1, 0, bo->size};
      PUT_OUT(mali_bo_create_out, o);
      return kIOReturnSuccess;
    }
    case MALI_LOG: {
      NEED_IN(mali_log_in, i);
      i.text[sizeof i.text - 1] = 0;
      printf("mali: user: %s\n", i.text);
      return kIOReturnSuccess;
    }
    case MALI_BO_CREATE_HOST: {
      NEED_IN(mali_bo_create_host_in, i);
      unsigned h = 0;
      while (h < kMaxBoHandles && bos[h]) ++h;
      if (h == kMaxBoHandles) { printf("mali: host bo refused: all %u handles in use\n", (unsigned)kMaxBoHandles); return kIOReturnNoResources; }
      if (!i.size || (i.ptr | i.size) & 0xfff || i.size > (1ull << 30)) {
        printf("mali: host bo refused by the call: ptr %#llx size %#llx\n", (unsigned long long)i.ptr, (unsigned long long)i.size);
        return kIOReturnBadArgument;
      }
      Bo *bo = boCreateHost(ownerTask, i.ptr, i.size, this);
      if (!bo) return kIOReturnNoMemory;
      bos[h] = bo;
      boHostBytes += bo->size;
      mali_bo_create_out o = {h + 1, 0, bo->size};
      PUT_OUT(mali_bo_create_out, o);
      return kIOReturnSuccess;
    }
    case MALI_BO_DESTROY: {
      NEED_IN(mali_bo_destroy_in, i);
      Bo *bo = boFor(i.handle);
      if (!bo) return kIOReturnBadArgument;
      if (!bo->host && !bo->hostCopy) boLiveBytes -= bo->size;
      bos[i.handle - 1] = nullptr;
      boRelease(bo);
      return kIOReturnSuccess;
    }
    case MALI_VM_BIND: {
      NEED_IN(mali_vm_bind_in, i);
      Ctx *c = vmFor(i.vm);
      if (!c) return kIOReturnBadArgument;
      if (i.op == MALI_BIND_MAP) return ctxMapBo(*c, boFor(i.bo), i.bo_offset, i.va, i.size, i.flags);
      if (i.op == MALI_BIND_UNMAP) return ctxUnmap(*c, i.va, i.size);
      return kIOReturnBadArgument;
    }
    case MALI_GROUP_CREATE: {
      NEED_IN(mali_group_create_in, i);
      Ctx *c = vmFor(i.vm);
      const uint64_t shaders = gpu64(kShaderPresent), tilers = gpu64(kTilerPresent);
      if (!c || !i.queue_count || i.queue_count > kMaxQueues) return kIOReturnBadArgument;
      if ((i.compute_core_mask & ~shaders) || (i.fragment_core_mask & ~shaders) || (i.tiler_core_mask & ~tilers)) return kIOReturnBadArgument;
      if (!i.compute_core_mask || !i.fragment_core_mask || !i.tiler_core_mask) return kIOReturnBadArgument;
      if (i.max_compute_cores > __builtin_popcountll(i.compute_core_mask) || i.max_fragment_cores > __builtin_popcountll(i.fragment_core_mask) ||
          i.max_tiler_cores > __builtin_popcountll(i.tiler_core_mask) || i.group_priority > 15) return kIOReturnBadArgument;
      GroupParams p{};
      p.queues = i.queue_count;
      for (unsigned q = 0; q < kMaxQueues; ++q) { p.ringSize[q] = i.ring_size[q]; p.queuePriority[q] = i.priority[q] > 15 ? 15 : i.priority[q]; }
      p.computeMask = i.compute_core_mask; p.fragmentMask = i.fragment_core_mask; p.tilerMask = i.tiler_core_mask;
      p.maxCompute = i.max_compute_cores; p.maxFragment = i.max_fragment_cores; p.maxTiler = i.max_tiler_cores; p.priority = i.group_priority;
      Group *g = groupCreate(*c, p, this, false);   // it gets a slot when it first has work
      if (!g) return kIOReturnNoResources;
      mali_group_create_out o = {g->id, 0};
      PUT_OUT(mali_group_create_out, o);
      return kIOReturnSuccess;
    }
    case MALI_HEAP_CREATE: {
      NEED_IN(mali_heap_create_in, i);
      Ctx *c = vmFor(i.vm);
      if (!c) return kIOReturnBadArgument;
      mali_heap_create_out o = {};
      const int slot = heapCreate(*c, i.initial_chunk_count, i.chunk_size, i.max_chunks, this, &o.ctx_va, &o.first_chunk_va);
      if (slot < 0) return kIOReturnNoMemory;
      o.handle = static_cast<uint32_t>(slot) + 1;
      PUT_OUT(mali_heap_create_out, o);
      return kIOReturnSuccess;
    }
    case MALI_HEAP_DESTROY: {
      NEED_IN(mali_heap_destroy_in, i);
      if (!i.handle || i.handle > kMaxHeaps || !gHeap[i.handle - 1].used || gHeap[i.handle - 1].owner != this) return kIOReturnBadArgument;
      heapFree(gHeap[i.handle - 1]);
      return kIOReturnSuccess;
    }
    case MALI_GROUP_DESTROY: {
      NEED_IN(mali_group_destroy_in, i);
      Group *g = groupFor(i.group);
      if (!g) return kIOReturnBadArgument;
      groupDestroy(g);
      return kIOReturnSuccess;
    }
    case MALI_GROUP_SUBMIT: {
      NEED_IN(mali_group_submit_in, i);
      Group *g = groupFor(i.group);
      if (!g || i.queue >= g->queues || !i.cs_size || (i.cs_size & 7) || (i.cs_va & 63) || i.cs_va >= g->ctx->userVaRange) return kIOReturnBadArgument;
      mali_group_submit_out o = {0};
      const int act = groupActivate(*g);
      if (act > 0) return kIOReturnNotReady;   // every slot has a busy group in it: externalMethod waits and calls again
      if (act < 0) return kIOReturnNoResources;
      if (!groupSubmit(*g, i.queue, i.cs_va, i.cs_size, i.latest_flush, &o.seqno)) return kIOReturnNoSpace;
      PUT_OUT(mali_group_submit_out, o);
      return kIOReturnSuccess;
    }
    default:
      return kIOReturnUnsupported;
  }
}

namespace {

// ---- M3b self-test: the same entry point userland uses, called from the kernel with kernel buffers
IOReturn kcall(IOUserClient *client, uint32_t selector, const void *in, uint32_t inSize, void *out, uint32_t outSize) {
  IOExternalMethodArguments a;
  memset(&a, 0, sizeof a);
  a.version = kIOExternalMethodArgumentsCurrentVersion;
  a.selector = selector;
  a.structureInput = in; a.structureInputSize = inSize;
  a.structureOutput = out; a.structureOutputSize = outSize;
  return client->externalMethod(selector, &a, nullptr, nullptr, nullptr);
}
#define KIN(client, sel, in) kcall(client, sel, &(in), sizeof(in), nullptr, 0)
#define KINOUT(client, sel, in, out) kcall(client, sel, &(in), sizeof(in), &(out), sizeof(out))

struct KMap {
  IOMemoryDescriptor *md;
  IOMemoryMap *map;
  volatile uint32_t *p;
};
bool kmapType(IOUserClient *client, uint32_t type, KMap &m) {
  IOOptionBits options = 0;
  m = KMap{};
  if (client->clientMemoryForType(type, &options, &m.md) != kIOReturnSuccess || !m.md) return false;
  m.map = m.md->createMappingInTask(kernel_task, 0, kIOMapAnywhere | (options & (kIOMapCacheMask | kIOMapReadOnly)));
  if (!m.map) { m.md->release(); m.md = nullptr; return false; }
  m.p = reinterpret_cast<volatile uint32_t *>(m.map->getVirtualAddress());
  return true;
}
void kunmap(KMap &m) {
  if (m.map) m.map->release();
  if (m.md) m.md->release();
  m = KMap{};
}

constexpr uint64_t kUcDataVa = 0x30000000ULL;   // the 1 MiB buffer must not run into the code buffer at kCodeVa

bool m3b() {
  printf("mali: m3b: the user client, driven from the kernel\n");
  const unsigned boBefore = gBoLive;
  bool ok = true;
  IOUserClient *a = nullptr, *b = nullptr;
  if (gService->newUserClient(kernel_task, nullptr, 0, &a) != kIOReturnSuccess || gService->newUserClient(kernel_task, nullptr, 1, &b) != kIOReturnSuccess || !a || !b) {
    printf("mali: m3b: newUserClient failed\n");
    return false;
  }
  IOReturn r;
  mali_dev_info info;
  r = kcall(a, MALI_DEV_QUERY, nullptr, 0, &info, sizeof info);
  printf("mali: m3b: DEV_QUERY %#x: gpu id %#x, %u groups of %u queues, %u registers, %u scoreboards, shaders %#llx tiler %#llx, timestamp %llu Hz, firmware %#x\n", r, info.gpu_id,
         info.csg_slot_count, info.cs_slot_count, info.cs_reg_count, info.scoreboard_slot_count, static_cast<unsigned long long>(info.shader_present),
         static_cast<unsigned long long>(info.tiler_present), static_cast<unsigned long long>(info.timestamp_frequency), info.fw_version);
  ok = r == kIOReturnSuccess && info.gpu_id == gpu(kGpuId) && info.cs_reg_count >= 8;

  mali_vm_create_in vmIn = {0};
  mali_vm_create_out vm = {};
  r = KINOUT(a, MALI_VM_CREATE, vmIn, vm);
  printf("mali: m3b: VM_CREATE %#x: vm %u, user range %#llx\n", r, vm.vm, static_cast<unsigned long long>(vm.user_va_range));
  ok = ok && r == kIOReturnSuccess;

  mali_bo_create_in boIn = {0x2000, 0, 0};
  mali_bo_create_out code = {}, data = {};
  r = KINOUT(a, MALI_BO_CREATE, boIn, code);
  boIn.size = 0x100000;
  const IOReturn r2 = KINOUT(a, MALI_BO_CREATE, boIn, data);
  mali_bo_create_in zeroIn = {0, 0, 0};
  mali_bo_create_out junk = {};
  const IOReturn rz = KINOUT(a, MALI_BO_CREATE, zeroIn, junk);
  printf("mali: m3b: BO_CREATE code %#x (handle %u, %#llx B), data %#x (handle %u, %#llx B), a zero size %#x (wanted bad argument %#x)\n", r, code.handle,
         static_cast<unsigned long long>(code.size), r2, data.handle, static_cast<unsigned long long>(data.size), rz, kIOReturnBadArgument);
  ok = ok && r == kIOReturnSuccess && r2 == kIOReturnSuccess && rz == kIOReturnBadArgument && data.size == 0x100000;

  mali_vm_bind_in bind = {vm.vm, MALI_BIND_MAP, code.handle, MALI_BIND_READONLY | MALI_BIND_UNCACHED, 0, kCodeVa, 0x2000};
  const IOReturn rb1 = KIN(a, MALI_VM_BIND, bind);
  bind = {vm.vm, MALI_BIND_MAP, data.handle, MALI_BIND_UNCACHED | MALI_BIND_NOEXEC, 0, kUcDataVa, 0x100000};
  const IOReturn rb2 = KIN(a, MALI_VM_BIND, bind);
  bind = {vm.vm, MALI_BIND_MAP, data.handle, 0, 0, kUcDataVa + 0x80000, 0x1000};                 // inside the mapping above
  const IOReturn rb3 = KIN(a, MALI_VM_BIND, bind);
  bind = {vm.vm, MALI_BIND_MAP, data.handle, 0, 0x200000, kKernelVaBase - 0x10000, 0x1000};      // past the buffer's end
  const IOReturn rb4 = KIN(a, MALI_VM_BIND, bind);
  printf("mali: m3b: VM_BIND code %#x, data %#x, overlapping %#x (wanted no space %#x), past the end %#x (wanted bad argument %#x)\n", rb1, rb2, rb3, kIOReturnNoSpace, rb4, kIOReturnBadArgument);
  ok = ok && rb1 == kIOReturnSuccess && rb2 == kIOReturnSuccess && rb3 == kIOReturnNoSpace && rb4 == kIOReturnBadArgument;
  if (!ok) goto out;

  {
    KMap codeMap, dataMap, syncMap;
    const bool mapped = kmapType(a, code.handle, codeMap) && kmapType(a, data.handle, dataMap);
    printf("mali: m3b: the buffers mapped into the kernel's own map for checking: %s\n", mapped ? "OK" : "FAILED");
    if (!mapped) { kunmap(codeMap); kunmap(dataMap); ok = false; goto out; }
    for (unsigned i = 0; i < 0x100000 / 4; ++i) dataMap.p[i] = 0xdeadbeef;
    volatile uint64_t *prog = reinterpret_cast<volatile uint64_t *>(codeMap.p);
    auto program = [&](unsigned idx, uint64_t va, uint32_t v0, uint32_t v1) {
      const uint64_t p[8] = {csSetSb(3, 0), csMove48(0, va), csMove32(2, v0), csMove32(3, v1), csStore(2, 0, 3, 0), csWait(1u << 0), 0, 0};
      for (unsigned k = 0; k < 8; ++k) prog[idx * 8 + k] = p[k];
      __asm__ volatile("dsb sy" ::: "memory");
    };

    mali_group_create_in gIn = {};
    gIn.vm = vm.vm; gIn.queue_count = 3;
    for (unsigned q = 0; q < 3; ++q) gIn.ring_size[q] = 0x1000;
    gIn.compute_core_mask = gIn.fragment_core_mask = gpu(kShaderPresent); gIn.tiler_core_mask = gpu(kTilerPresent);
    gIn.max_compute_cores = gIn.max_fragment_cores = static_cast<uint8_t>(__builtin_popcount(gpu(kShaderPresent))); gIn.max_tiler_cores = 1;   // (1 on the G310) gIn.group_priority = 15;
    mali_group_create_out grp = {};
    r = KINOUT(a, MALI_GROUP_CREATE, gIn, grp);
    printf("mali: m3b: GROUP_CREATE %#x: group %u in slot %u\n", r, grp.group, grp.csg_slot);
    ok = r == kIOReturnSuccess;
    if (!ok) { kunmap(codeMap); kunmap(dataMap); goto out; }
    const bool syncOk = kmapType(a, MALI_MAP_SYNCOBJ | grp.group, syncMap);
    printf("mali: m3b: the group's syncobj page mapped: %s\n", syncOk ? "OK" : "FAILED");
    ok = syncOk;

    // submit by handle, wait through FENCE_WAIT, check the data through the mapping of the buffer
    auto job = [&](unsigned q, unsigned idx, uint64_t va, uint32_t v0, uint32_t v1, uint32_t bufOff) -> bool {
      program(idx, va, v0, v1);
      mali_group_submit_in s = {grp.group, q, kCodeVa + idx * 64, 64, 0};
      mali_group_submit_out so = {};
      const IOReturn rs = KINOUT(a, MALI_GROUP_SUBMIT, s, so);
      mali_fence_wait_in w = {grp.group, q, so.seqno, 1000000};
      mali_fence_wait_out wo = {};
      const IOReturn rw = rs == kIOReturnSuccess ? KINOUT(a, MALI_FENCE_WAIT, w, wo) : rs;
      const uint32_t off = bufOff / 4;
      const bool good = rs == kIOReturnSuccess && rw == kIOReturnSuccess && wo.signaled && dataMap.p[off] == v0 && dataMap.p[off + 1] == v1;
      if (!good) printf("mali: m3b: job on queue %u at %#llx: submit %#x wait %#x signaled %u done %llu seqno %llu data %#x %#x (wanted %#x %#x)\n", q, static_cast<unsigned long long>(va), rs, rw,
                        wo.signaled, static_cast<unsigned long long>(wo.done_seqno), static_cast<unsigned long long>(so.seqno), dataMap.p[off], dataMap.p[off + 1], v0, v1);
      return good;
    };
    for (unsigned q = 0; q < 3 && ok; ++q) ok = job(q, q, kUcDataVa + 0x100 + q * 16, 0x71000000 + q, 0x72000000 + q, 0x100 + q * 16);
    ok = ok && job(2, 3, kUcDataVa + 0xff000, 0x73000000, 0x74000000, 0xff000);                    // the last page of the 1 MiB buffer: the scatter list is right
    printf("mali: m3b: jobs by handle on three queues and the buffer's last page: %s\n", ok ? "OK" : "FAILED");
    for (unsigned j = 0; ok && j < 36; ++j) ok = job(1, 8 + j, kUcDataVa + 0x400 + j * 8, 0x75000000 + j, 0x76000000 + j, 0x400 + j * 8);
    printf("mali: m3b: 36 more jobs on queue 1 (the 4 KiB ring wraps), syncobj page says %llu: %s\n",
           static_cast<unsigned long long>(syncOk ? *reinterpret_cast<volatile uint64_t *>(syncMap.p + 4) : 0), ok ? "OK" : "FAILED");

    // a hole in the middle of the mapping, the rest still there; the hole's page mapped again elsewhere
    bind = {vm.vm, MALI_BIND_UNMAP, 0, 0, 0, kUcDataVa + 0x1000, 0x1000};
    const IOReturn ru1 = KIN(a, MALI_VM_BIND, bind);
    bind = {vm.vm, MALI_BIND_UNMAP, 0, 0, 0, kUcDataVa + 0x1000, 0x1000};
    const IOReturn ru2 = KIN(a, MALI_VM_BIND, bind);
    bind = {vm.vm, MALI_BIND_MAP, data.handle, MALI_BIND_UNCACHED | MALI_BIND_NOEXEC, 0x1000, kUcDataVa + 0x400000, 0x1000};
    const IOReturn rm = KIN(a, MALI_VM_BIND, bind);
    printf("mali: m3b: unmap a page %#x, unmap it again %#x (wanted not found %#x), map that page elsewhere %#x\n", ru1, ru2, kIOReturnNotFound, rm);
    ok = ok && ru1 == kIOReturnSuccess && ru2 == kIOReturnNotFound && rm == kIOReturnSuccess;
    ok = ok && job(0, 60, kUcDataVa + 0x400000 + 0x10, 0x77000000, 0x78000000, 0x1010);       // lands at offset 0x1000 + 0x10 of the buffer
    ok = ok && job(0, 61, kUcDataVa + 0x2000, 0x79000000, 0x7a000000, 0x2000) && job(0, 62, kUcDataVa + 0xfe000, 0x7b000000, 0x7c000000, 0xfe000);   // both sides of the hole still work
    printf("mali: m3b: jobs around the hole and through the remapped page: %s\n", ok ? "OK" : "FAILED");

    // another client may not touch these handles
    mali_group_submit_in foreign = {grp.group, 0, kCodeVa, 64, 0};
    mali_group_submit_out fo = {};
    const IOReturn rf1 = KINOUT(b, MALI_GROUP_SUBMIT, foreign, fo);
    mali_vm_destroy_in fv = {vm.vm, 0};
    const IOReturn rf2 = KIN(b, MALI_VM_DESTROY, fv);
    KMap stolen;
    const bool rf3 = kmapType(b, data.handle, stolen);
    kunmap(stolen);
    printf("mali: m3b: a second client: submit %#x, vm destroy %#x (both wanted bad argument %#x), mapping the first client's buffer %s (wanted refused)\n", rf1, rf2, kIOReturnBadArgument, rf3 ? "worked" : "refused");
    ok = ok && rf1 == kIOReturnBadArgument && rf2 == kIOReturnBadArgument && !rf3;

    kunmap(syncMap); kunmap(dataMap); kunmap(codeMap);
    // destroy in the wrong order: the vm refuses while a group uses it
    mali_vm_destroy_in dv = {vm.vm, 0};
    const IOReturn rbusy = KIN(a, MALI_VM_DESTROY, dv);
    printf("mali: m3b: VM_DESTROY while a group uses it: %#x (wanted busy %#x)\n", rbusy, kIOReturnBusy);
    ok = ok && rbusy == kIOReturnBusy;
    // leave the group, the address space and both buffers to the connection closing: that is what a dying process does
  }
out:
  r = a->clientClose();
  a->release();
  b->clientClose();
  b->release();
  bool clean = gBoLive == boBefore;
  for (unsigned g = 0; g < kMaxGroupObjs; ++g) clean = clean && !gGroup[g].used;
  for (unsigned i = 0; i < kMaxCtx; ++i) clean = clean && !gCtx[i].used;
  printf("mali: m3b: both connections closed (%#x): groups, address spaces and buffers all gone: %s (live buffers %u, wanted %u)\n", r, clean ? "yes" : "NO", gBoLive, boBefore);
  ok = ok && clean;
  printf("mali: m3b %s\n", ok ? "PASSED: the user client creates, binds, submits, waits, refuses strangers and cleans up after a closed connection" : "FAILED");
  return ok;
}

bool serviceCreate() {
  gService = OSTypeAlloc(MaliService);
  if (!gService || !gService->init() || !gService->attach(IOService::getResourceService()) || !gService->start(IOService::getResourceService())) {
    printf("mali: could not make the service\n");
    return false;
  }
  return true;
}

// The sandbox's profiles of the original OS name no class of this driver, so every process but launchd (the init process of the test
// program is the exception) is refused an open of the service. The service names the processes that get it in place of the sandbox's verdict
// (the kernel's native_sandbox_grant, UniXNU's zzw-native-adreno-sandbox-grant: a comma-separated list of process names). The test program
// and the child it forks; boot-arg mali-grant=name[,name...] names others (the render server will be one, M4).

// ---- The registry-visible accelerator for the original Metal discovery -------------------------------------------------------------------------------
// IOServiceMatching("IOAcceleratorES") -> MetalPluginName -> /System/Library/Extensions/<name>.bundle -> its principal class -> initWithAcceleratorPort:.
// The class is named after Apple's AGXAccelerator on purpose (the original sandbox profiles name that class), and derives from the original
// IOAcceleratorES exactly as the original base is laid out (nothing beyond IOService: 136 bytes, no operational overrides). Its metaclass is made at
// run time, after the original IOGPUFamily kext has loaded (the parent metaclass does not exist before that), so no symbol of that kext is bound.
// Same recipe as UniXNU-sheng's NativeAdrenoAccelerator + GpuPowerProbe::publishMetal, minus the static-initializer staging.
class AccelMeta;
const OSMetaClass *gAccelMeta;

class MaliAccelerator : public IOService {
 public:
  MaliAccelerator();
  virtual const OSMetaClass *getMetaClass() const override { return gAccelMeta; }
 protected:
  virtual ~MaliAccelerator() override;
};

class AccelMeta : public OSMetaClass {
 public:
  // The base class keeps operator new private (its instances are static members); this one is built in place.
  static void *operator new(size_t, void *where) { return where; }
  AccelMeta(const OSMetaClass *super) : OSMetaClass("AGXAccelerator", super, sizeof(IOService)) {}
  virtual OSObject *alloc() const override { return new MaliAccelerator; }
  void constructed() const { instanceConstructed(); }
  void destructed() const { instanceDestructed(); }
};

MaliAccelerator::MaliAccelerator() : IOService(gAccelMeta) { static_cast<const AccelMeta *>(gAccelMeta)->constructed(); }
MaliAccelerator::~MaliAccelerator() { static_cast<const AccelMeta *>(gAccelMeta)->destructed(); }

IOService *gAccelerator;

void publishAccelerator() {
  char plugin[32] = "NativeMaliMetal", klass[32] = "NativeMaliMetalDevice", grant[512] = "backboardd,iconservicesagen,Preferences,SpringBoard,PosterBoard,WidgetRenderer_D,AegirPoster,KaleidoscopePost,CollectionsPoste,PhotosPosterProv,MercuryPosterExt,GradientPosterEx,EmojiPosterExten,ExtragalacticPos,HelloPosterExten,PridePosterExten,UnityPosterExten";   // names are compared up to 16 characters: the wallpaper extensions render too (which of them is on the lock screen depends on the Data volume, so all of them are named)
  PE_parse_boot_argn("mali-plugin", plugin, sizeof plugin);
  PE_parse_boot_argn("mali-plugin-grant", grant, sizeof grant);
  const OSReturn loaded = OSKextLoadKextWithIdentifier("com.apple.iokit.IOGPUFamily");
  if (loaded) { printf("mali: metal: IOGPUFamily did not load (%#x)\n", loaded); return; }
  const OSSymbol *name = OSSymbol::withCString("IOAcceleratorES");
  const OSMetaClass *parent = name ? OSMetaClass::getMetaClassWithName(name) : nullptr;
  OSSafeReleaseNULL(name);
  if (!parent) { printf("mali: metal: no IOAcceleratorES\n"); return; }
  if (parent->getClassSize() != sizeof(IOService)) {
    printf("mali: metal: IOAcceleratorES is %u bytes, not the %lu of the original base\n", parent->getClassSize(), (unsigned long)sizeof(IOService));
    return;
  }
  void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
  if (!handle) { printf("mali: metal: preModLoad failed\n"); return; }
  alignas(16) static unsigned char metaStorage[sizeof(AccelMeta)];
  gAccelMeta = new (static_cast<void *>(metaStorage)) AccelMeta(parent);
  const OSReturn registered = OSMetaClass::postModLoad(handle);
  if (registered) { printf("mali: metal: could not register the class (%#x)\n", registered); return; }
  gAccelerator = static_cast<IOService *>(gAccelMeta->alloc());
  if (!gAccelerator || !gAccelerator->init()) { printf("mali: metal: no accelerator object\n"); return; }
  gAccelerator->setProperty("MetalPluginName", plugin);
  gAccelerator->setProperty("MetalPluginClassName", klass);
  gAccelerator->setProperty("NativeSandboxGrantProcess", grant);
  if (!gAccelerator->attach(IOService::getPlatform())) { printf("mali: metal: attach failed\n"); return; }
  gAccelerator->registerService();
  printf("mali: metal: AGXAccelerator (an IOAcceleratorES) registered, plugin %s class %s, sandbox grant %s\n", plugin, klass, grant);
}

void serviceRegister() {
  char grant[512] = "gpu-test,backboardd,iconservicesagen,Preferences,SpringBoard,PosterBoard,WidgetRenderer_D,AegirPoster,KaleidoscopePost,CollectionsPoste,PhotosPosterProv,MercuryPosterExt,GradientPosterEx,EmojiPosterExten,ExtragalacticPos,HelloPosterExten,PridePosterExten,UnityPosterExten";
  PE_parse_boot_argn("mali-grant", grant, sizeof grant);
  gService->setProperty("NativeSandboxGrantProcess", grant);
  gService->registerService();
  printf("mali: service \"%s\" registered: userland can open it (IOServiceGetMatchingService); sandbox grant: %s\n", MALI_SERVICE, grant);
  unsigned metal = 0;
  PE_parse_boot_argn("mali-metal", &metal, sizeof metal);
  if (metal) publishAccelerator();
}

}  // namespace

// The owner's test program and what it loads (MaliMetal's usertest/build.sh makes gpu_files.inc: signed Mach-Os, the big ones as zlib streams) for
// boot-arg evb10-exec=1 of the userland hook (UniXNU-evb10 patches/xnu/evb10-userland-mounts.patch), which writes them to the Data volume and starts the first one as
// the init process. Without the .inc there are no files.
struct TestFile {
  const char *name;
  const unsigned char *data;
  size_t size;      // as carried
  size_t rawSize;   // inflated size; 0: carried as is
};
#if __has_include("gpu_files.inc")
#include "gpu_files.inc"   // static const TestFile kTestFiles[]; static const unsigned kTestFileCount
#else
static const TestFile kTestFiles[1] = {{nullptr, nullptr, 0, 0}};
static const unsigned kTestFileCount = 0;
#endif

// The GPU addon the loader placed after the trust caches (/chosen/memory-map/GpuAddon; MaliMetal's metal-plugin/make-addon.py makes it, `unixnu build
// payload --gpu-addon` carries it): the Metal plugin bundle the kernel writes into the Data volume and union-mounts over /System/Library/Extensions.
// Boot memory, never freed here. Returns 0 without one. (UniXNU's NativeRamBlock has the same function; this kernel has no RamBlock driver.)
extern "C" int mali_gpu_addon(const uint8_t **data, uint64_t *bytes) {
  if (!data || !bytes) return 0;
  *data = nullptr;
  *bytes = 0;
  IORegistryEntry *map = IORegistryEntry::fromPath("/chosen/memory-map", gIODTPlane);
  if (!map) return 0;
  OSData *range = OSDynamicCast(OSData, map->getProperty("GpuAddon"));
  uint64_t physical = 0, size = 0;
  if (range && range->getLength() == 16) {
    memcpy(&physical, range->getBytesNoCopy(), 8);
    memcpy(&size, static_cast<const uint8_t *>(range->getBytesNoCopy()) + 8, 8);
  }
  map->release();
  if (!size || size > (UINT64_C(64) << 20) || (physical & 4095)) return 0;
  *data = reinterpret_cast<const uint8_t *>(ml_static_ptovirt(physical));
  if (!*data) return 0;
  *bytes = size;
  return 1;
}

extern "C" int mali_test_file(unsigned index, const char **name, const uint8_t **data, size_t *size, size_t *rawSize) {
  if (index >= kTestFileCount) return -1;
  *name = kTestFiles[index].name;
  *data = kTestFiles[index].data;
  *size = kTestFiles[index].size;
  *rawSize = kTestFiles[index].rawSize;
  return 0;
}

// ---- the display's fence: a swap may flip the panel to a surface only once the GPU work submitted before the swap is done ----
// mali_fence_take: the groups' queues that still have jobs running, as triples { serial, seqno, group << 8 | queue }, at most cap of them;
// returns the count, -1 when the driver is busy with a call (try again), -2 when there are more than cap (then the fence is "all idle", count ~0).
// mali_fence_done: 1 when all of them have reached their seqno (a group that is gone counts as done), 0 when not, -1 when the driver is busy.
extern "C" int mali_fence_take(uint64_t *entries, unsigned cap) {
  if (!gLock || !IOLockTryLock(gLock)) return -1;
  unsigned n = 0;
  int result = 0;
  for (unsigned g = 0; g < kMaxGroupObjs && result == 0; ++g) {
    Group &gr = gGroup[g];
    if (!gr.used || !gr.sync.kva) continue;
    // Only the render server's work is what a frame waits for: a poster extension or an app whose job is stalled (a GPU that waits on a
    // sync nobody signals, a group that is not running) must not hold the panel's frames back.
    if (!gr.owner || strcmp(static_cast<MaliClient *>(gr.owner)->name(), "backboardd") != 0) continue;
    for (unsigned q = 0; q < gr.queues; ++q) {
      const uint64_t done = *reinterpret_cast<volatile uint64_t *>(gr.sync.kva + q * 16);
      if (done >= gr.q[q].seqno) continue;
      if (n == cap) { result = -2; break; }
      entries[3 * n] = gr.serial; entries[3 * n + 1] = gr.q[q].seqno; entries[3 * n + 2] = (static_cast<uint64_t>(g) << 8) | q;
      ++n;
    }
  }
  IOLockUnlock(gLock);
  return result ? result : static_cast<int>(n);
}

extern "C" int mali_fence_done(const uint64_t *entries, unsigned count) {
  if (!gLock || !IOLockTryLock(gLock)) return -1;
  int pending = 0;
  if (count == ~0u) {
    for (unsigned g = 0; g < kMaxGroupObjs && !pending; ++g) {
      Group &gr = gGroup[g];
      if (!gr.used || !gr.sync.kva) continue;
      if (!gr.owner || strcmp(static_cast<MaliClient *>(gr.owner)->name(), "backboardd") != 0) continue;
      for (unsigned q = 0; q < gr.queues; ++q)
        if (*reinterpret_cast<volatile uint64_t *>(gr.sync.kva + q * 16) < gr.q[q].seqno) { pending = 1; break; }
    }
  } else {
    for (unsigned i = 0; i < count && !pending; ++i) {
      const unsigned g = static_cast<unsigned>(entries[3 * i + 2] >> 8), q = entries[3 * i + 2] & 0xff;
      if (g >= kMaxGroupObjs) continue;
      Group &gr = gGroup[g];
      if (!gr.used || gr.serial != entries[3 * i] || !gr.sync.kva || q >= gr.queues) continue;
      if (*reinterpret_cast<volatile uint64_t *>(gr.sync.kva + q * 16) < entries[3 * i + 1]) pending = 1;
    }
  }
  IOLockUnlock(gLock);
  return pending ? 0 : 1;
}

// What a fence that gave up was waiting for (the first few times only): which group of whom, how far it got, how long ago it last ran.
extern "C" void mali_fence_describe(const uint64_t *entries, unsigned count) {
  static unsigned told;
  if (told >= 12 || !gLock || !IOLockTryLock(gLock)) return;
  if (count == ~0u) {
    printf("mali: fence gave up: more waiting queues than it tracks\n");
    ++told;
  } else {
    for (unsigned i = 0; i < count && told < 12; ++i) {
      const unsigned g = static_cast<unsigned>(entries[3 * i + 2] >> 8), q = entries[3 * i + 2] & 0xff;
      if (g >= kMaxGroupObjs) continue;
      Group &gr = gGroup[g];
      if (!gr.used || gr.serial != entries[3 * i] || !gr.sync.kva || q >= gr.queues) continue;
      const uint64_t done = *reinterpret_cast<volatile uint64_t *>(gr.sync.kva + q * 16);
      if (done >= entries[3 * i + 1]) continue;
      printf("mali: fence gave up: group %u of %s queue %u at job %llu of %llu, slot %s%u, last scheduled %llu groups ago\n", gr.id,
             gr.owner ? static_cast<MaliClient *>(gr.owner)->name() : "?", q, static_cast<unsigned long long>(done),
             static_cast<unsigned long long>(entries[3 * i + 1]), gr.csg == kNoSlot ? "none " : "", gr.csg == kNoSlot ? 0u : gr.csg,
             static_cast<unsigned long long>(gUseClock - gr.lastUse));
      ++told;
    }
  }
  IOLockUnlock(gLock);
}

// The host's own cache clean (GPU_CMD FLUSH_CACHES: L2 and load/store cache clean), for the display's check that the GPU's output reached memory.
extern "C" bool mali_clean_caches(void) {
  if (!gLock || !gGpu) return false;
  IOLockLock(gLock);
  gpuWrite(kIntClear, 1u << 17);
  gpuWrite(kCmd, 4u | ((1u | (1u << 4)) << 8));
  const bool done = waitUs(200000, [] { return (gpu(kIntRaw) & (1u << 17)) != 0; });
  IOLockUnlock(gLock);
  return done;
}

// mali-sample=1: every two seconds, watch GPU_STATUS.ACTIVE for 200 ms as fast as the bus allows and say how much of the time the GPU
// was busy, how long one burst of work lasts and how long the gaps between bursts are. That tells whether the frames are bound by the GPU
// itself (busy nearly all the time) or by what feeds it (bursts short, gaps long).
thread_call_t gSampleCall;
void gpuActivityTick(thread_call_param_t, thread_call_param_t) {
  uint64_t freq;
  __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
  // The speed of the core this runs on: a chain of dependent adds retires one per cycle on the Cortex-A55 and A76 alike.
  unsigned long long mhz[3] = {};
  for (unsigned pass = 0; pass < 3; ++pass) {
    uint64_t chain = 0;
    const uint64_t t0 = cntvct();
    for (uint64_t i = 0; i < 4000000; ++i) __asm__ volatile("add %0, %0, #1" : "+r"(chain));
    const uint64_t t1 = cntvct();
    mhz[pass] = t1 > t0 ? 4000000ull * freq / (t1 - t0) / 1000000 : 0;
  }
  printf("mali: cpu %u speed %llu / %llu / %llu MHz (dependent adds)\n", static_cast<unsigned>(cpu_number()), mhz[0], mhz[1], mhz[2]);
  const uint64_t submits0 = gCallStat[MALI_GROUP_SUBMIT].n;
  const uint64_t begin = cntvct();
  uint64_t now = begin, runStart = 0, gapStart = begin, samples = 0, active = 0, runs = 0, runTicks = 0, longest = 0, gaps = 0, gapTicks = 0;
  bool was = false;
  do {
    now = cntvct();
    const bool on = (gpu(kStatus) & 1) != 0;
    ++samples;
    if (on) ++active;
    if (on && !was) { ++runs; runStart = now; gapTicks += now - gapStart; ++gaps; }
    if (!on && was) { const uint64_t r = now - runStart; runTicks += r; if (r > longest) longest = r; gapStart = now; }
    was = on;
  } while (now - begin < freq / 5);
  const uint64_t us = 1000000 / freq ? 1000000 / freq : 1;
  (void)us;
  const unsigned long long busyPermille = samples ? active * 1000 / samples : 0;
  printf("mali: activity over 200 ms: busy %llu.%llu%% (%llu samples), %llu bursts, burst mean %llu us longest %llu us, gap mean %llu us, submits %llu\n",
         busyPermille / 10, busyPermille % 10, static_cast<unsigned long long>(samples), static_cast<unsigned long long>(runs),
         static_cast<unsigned long long>(runs ? runTicks * 1000000 / freq / runs : 0), static_cast<unsigned long long>(longest * 1000000 / freq),
         static_cast<unsigned long long>(gaps > 1 ? gapTicks * 1000000 / freq / (gaps - 1) : 0),
         static_cast<unsigned long long>(gCallStat[MALI_GROUP_SUBMIT].n - submits0));
  uint64_t deadline;
  clock_interval_to_deadline(1800, NSEC_PER_MSEC, &deadline);
  thread_call_enter_delayed(gSampleCall, deadline);
}

extern "C" void mali_start(void) {
  unsigned on = 0;
  if (!PE_parse_boot_argn("mali", &on, sizeof on) || !on) return;
  gLevel = on;
  printf("mali: Mali-G310 bring-up, level %u\n", on);
  gClk = reinterpret_cast<volatile uint32_t *>(ml_io_map(kClkPhys, 0x1000));
  gReset = reinterpret_cast<volatile uint32_t *>(ml_io_map(kResetPhys, 0x1000));
  if (!gClk || !gReset) { printf("mali: cannot map the clock (%p) or reset (%p) controller\n", (void *)gClk, (void *)gReset); return; }
  {
    uint64_t size = 0;
    kFirmware = hakios_image_file("fw", "mali_csffw.bin", &size, nullptr);
    kFirmwareSize = static_cast<size_t>(size);
    if (!kFirmware && on >= 2) { printf("mali: no fw/mali_csffw.bin in the image\n"); on = gLevel = 1; }
  }
  if (!setClocks()) return;
  gGpu = reinterpret_cast<volatile uint32_t *>(ml_io_map(kGpuPhys, kGpuMapSize));
  if (!gGpu) { printf("mali: cannot map the GPU registers\n"); return; }
  identify();
  if (!softReset()) return;
  printf("mali: soft reset done\n");
  identify();
  unsigned l2 = 1;
  PE_parse_boot_argn("mali-l2", &l2, sizeof l2);
  PE_parse_boot_argn("mali-skipslot", &gSkipSlots, sizeof gSkipSlots);
  if (gSkipSlots) printf("mali: CSG slots %#x are not used (mali-skipslot)\n", gSkipSlots);
  if (l2) l2Test();
  pokeRegisters();
  {
    // What clock the GPU really runs at: its cycle counter (GPU_COMMAND_TIME_ENABLE starts it) against the architected counter (24 MHz).
    // CLKSEL_CON(158) is the clock (this probe reads 594 MHz at mali-div=1 and 198 at the default mali-div=5), and the heavy render passes scale with it
    // exactly (28.4 ms at 594 MHz, 84.8 ms at 198 MHz, mali-trace=print): a frame's GPU time follows the clock, so mali-pll / mali-div are worth trying.
    auto cycles = [] { uint32_t hi = gpu(0x94), lo = gpu(0x90); const uint32_t hi2 = gpu(0x94); if (hi2 != hi) { hi = hi2; lo = gpu(0x90); } return (static_cast<uint64_t>(hi) << 32) | lo; };
    auto counter = [] { uint64_t v; __asm__ volatile("isb\n\tmrs %0, cntvct_el0" : "=r"(v)); return v; };
    gpuWrite(kCmd, 0x103);
    IODelay(1000);
    const uint64_t c0 = cycles(), t0 = counter();
    IODelay(20000);
    const uint64_t c1 = cycles(), t1 = counter();
    uint64_t freq = 0; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    const uint64_t us = freq ? (t1 - t0) * 1000000 / freq : 0;
    printf("mali: clock probe: %llu GPU cycles in %llu us = %llu MHz (timestamp register %#x%08x)\n", static_cast<unsigned long long>(c1 - c0),
           static_cast<unsigned long long>(us), static_cast<unsigned long long>(us ? (c1 - c0) / us : 0), gpu(0x9c), gpu(0x98));
  }
  {
    // The CPU's own view of the memory system (cached write-back memory, 32 MiB, the kernel's physical aperture): a fill and a copy. This board's DRAM
    // is fast (about 50 GB/s fill, 20 GB/s copy), and up to 594 MHz the GPU's frame time follows the GPU's own clock, not the memory's.
    const size_t bytes = 32u << 20;
    IOBufferMemoryDescriptor *buffer = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut, bytes * 2, 4096);
    if (buffer) {
      uint8_t *p = static_cast<uint8_t *>(buffer->getBytesNoCopy());
      uint64_t freq = 0; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
      memset(p, 1, bytes * 2);
      uint64_t a, b, c;
      __asm__ volatile("isb\n\tmrs %0, cntvct_el0" : "=r"(a));
      for (int i = 0; i < 4; ++i) memset(p, i, bytes);
      __asm__ volatile("isb\n\tmrs %0, cntvct_el0" : "=r"(b));
      for (int i = 0; i < 4; ++i) memcpy(p + bytes, p, bytes);
      __asm__ volatile("isb\n\tmrs %0, cntvct_el0" : "=r"(c));
      printf("mali: CPU memory: fill %llu MB/s, copy %llu MB/s (32 MiB, cached)\n", static_cast<unsigned long long>(b > a ? 4ull * 32 * freq / (b - a) : 0),
             static_cast<unsigned long long>(c > b ? 4ull * 32 * freq / (c - b) : 0));
      buffer->release();
    }
  }
  {
    unsigned sample = 0;
    if (PE_parse_boot_argn("mali-sample", &sample, sizeof sample) && sample) {
      gSampleCall = thread_call_allocate(gpuActivityTick, nullptr);
      uint64_t deadline;
      clock_interval_to_deadline(30000, NSEC_PER_MSEC, &deadline);
      if (gSampleCall) thread_call_enter_delayed(gSampleCall, deadline);
    }
  }
  boPoolInit();
  printf("mali: M0 done\n");
  if (on >= 2) m1();
}
