//! AxVM AArch64 adapter.
//!
//! This module owns the AxVM/ArceOS glue for the OS-neutral `arm_vcpu` core:
//! `AxvmArmHostOps` supplies host IRQ/GIC operations, while this module handles
//! `arm_vcpu` exits inside the AArch64 architecture boundary.

use alloc::boxed::Box;
use core::time::Duration;

use arm_vcpu::{
    ArmAccessWidth, ArmGuestPhysAddr, ArmHostOps, ArmNestedPagingConfig, ArmPerCpu, ArmSysRegAddr,
    ArmVcpu, ArmVcpuCreateConfig, ArmVcpuError, ArmVcpuResult, ArmVcpuSetupConfig, ArmVmExit,
};
use arm_vgic::host::ArmVgicHostIf;
use ax_crate_interface::impl_interface;
use ax_memory_addr::{PhysAddr, VirtAddr};
use axvm_types::{
    AccessWidth, GuestPhysAddr, InterruptTriggerMode, NestedPagingConfig, SysRegAddr, VCpuId, VMId,
    VmArchPerCpuOps, VmArchVcpuOps, VmBackendError as BackendError,
    VmBackendResult as BackendResult,
};

use super::{ArchOps, BoundVcpuExit, HypercallExit, MmioReadExit, MmioWriteExit, VcpuRunAction};
use crate::{
    AxVmResult, ax_err,
    host::{HostCpu, HostMemory, HostTime, default_host},
};

mod capabilities;
#[path = "../../architecture/cpu_up.rs"]
mod cpu_up;
pub(crate) mod fdt;
mod gic;
mod images;
mod ipi;
mod npt;
#[path = "../../architecture/sysreg.rs"]
mod sysreg;
mod vm;
mod vtimer;

pub use capabilities::{host_fdt_bootarg, host_phys_to_virt};
use cpu_up::{CpuUpExit, CpuUpOps};
pub use images::ImageLoader;
use ipi::SendIpiExit;
use sysreg::{SysRegReadExit, SysRegWriteExit};
pub(crate) use vm::register_device_factories;

pub(crate) struct Aarch64Arch;

#[derive(Clone, Copy, Debug)]
pub(crate) enum Aarch64DeferredRunWork {
    ExternalInterrupt { vector: usize },
}

impl CpuUpOps for Aarch64Arch {}

impl ArchOps for Aarch64Arch {
    type VCpu = AxvmArmVcpu;
    type PerCpu = AxvmArmPerCpu;
    type DeferredRunWork = Aarch64DeferredRunWork;
    type NestedPageTable = npt::NestedPageTable<crate::HostPagingHandler>;

    fn has_hardware_support() -> bool {
        arm_vcpu::has_hardware_support()
    }

    fn before_first_run(_vm: &crate::AxVMRef, _vcpu: &crate::vm::AxVCpuRef<Self::VCpu>) {
        gic::enable_virtual_interrupt_interface();
    }

    fn register_platform_irq_injector() {
        crate::irq::register_aarch64_virtual_irq_injector(inject_virtual_irq);
    }

    fn clean_dcache_range(addr: VirtAddr, size: usize) {
        aarch64_cpu_ext::cache::dcache_range(
            aarch64_cpu_ext::cache::CacheOp::Clean,
            addr.as_usize(),
            size,
        );
    }

    fn handle_vcpu_exit_bound(
        vm: &crate::AxVMRef,
        vcpu: &crate::vm::AxVCpuRef<Self::VCpu>,
        exit: <Self::VCpu as VmArchVcpuOps>::Exit,
    ) -> AxVmResult<BoundVcpuExit<Self::DeferredRunWork>> {
        match exit {
            ArmVmExit::Hypercall { nr, args } => super::handle_hypercall(
                vm,
                vcpu,
                HypercallExit { nr, args },
                crate::runtime::hvc::HyperCallAbi::AArch64,
            ),
            ArmVmExit::MmioRead {
                addr,
                width,
                reg,
                reg_width,
                signed_ext,
            } => super::handle_mmio_read(
                vm,
                vcpu,
                MmioReadExit {
                    addr: arm_guest_phys_addr_to_ax(addr),
                    width: arm_access_width_to_ax(width),
                    reg,
                    reg_width: arm_access_width_to_ax(reg_width),
                    signed_ext,
                },
            ),
            ArmVmExit::MmioWrite { addr, width, data } => super::handle_mmio_write::<Self>(
                vm,
                MmioWriteExit {
                    addr: arm_guest_phys_addr_to_ax(addr),
                    width: arm_access_width_to_ax(width),
                    data,
                },
            ),
            ArmVmExit::SysRegRead { addr, reg } => handle_sysreg_read(
                vm,
                vcpu,
                SysRegReadExit {
                    addr: arm_sys_reg_addr_to_ax(addr),
                    reg,
                },
            ),
            ArmVmExit::SysRegWrite { addr, value } => sysreg::handle_write(
                vm,
                SysRegWriteExit {
                    addr: arm_sys_reg_addr_to_ax(addr),
                    value,
                },
            ),
            ArmVmExit::ExternalInterrupt { vector } => {
                debug!("VM[{}] run VCpu[{}] get irq {vector}", vm.id(), vcpu.id());
                Ok(BoundVcpuExit::Defer(
                    Aarch64DeferredRunWork::ExternalInterrupt {
                        vector: vector as usize,
                    },
                ))
            }
            ArmVmExit::CpuDown { state } => {
                warn!(
                    "VM[{}] run VCpu[{}] CpuDown state {state:#x}",
                    vm.id(),
                    vcpu.id()
                );
                Ok(BoundVcpuExit::Complete(VcpuRunAction {
                    waits_for_event: true,
                    stop_reason: None,
                    resets_vm: false,
                    exits_vcpu: false,
                }))
            }
            ArmVmExit::CpuUp {
                target_cpu,
                entry_point,
                arg,
            } => cpu_up::handle::<Self>(
                vm,
                vcpu,
                CpuUpExit {
                    target_cpu,
                    entry_point: arm_guest_phys_addr_to_ax(entry_point),
                    arg,
                },
            ),
            ArmVmExit::SystemDown => {
                warn!("VM[{}] run VCpu[{}] SystemDown", vm.id(), vcpu.id());
                Ok(BoundVcpuExit::Complete(VcpuRunAction {
                    waits_for_event: false,
                    stop_reason: Some(crate::StopReason::SystemDown),
                    resets_vm: false,
                    exits_vcpu: false,
                }))
            }
            ArmVmExit::SendIPI {
                target_cpu,
                target_cpu_aux,
                send_to_all,
                send_to_self,
                vector,
            } => ipi::handle(
                vm,
                vcpu.id(),
                SendIpiExit {
                    target_cpu,
                    target_cpu_aux,
                    send_to_all,
                    send_to_self,
                    vector,
                },
            ),
            ArmVmExit::Nothing => Ok(BoundVcpuExit::Complete(VcpuRunAction {
                waits_for_event: false,
                stop_reason: None,
                resets_vm: false,
                exits_vcpu: false,
            })),
            _ => ax_err!(Unsupported, "unsupported AArch64 VM exit"),
        }
    }

    fn finish_deferred_run_work(
        vm: &crate::AxVMRef,
        vcpu: &crate::vm::AxVCpuRef<Self::VCpu>,
        work: Self::DeferredRunWork,
    ) -> AxVmResult<VcpuRunAction> {
        match work {
            Aarch64DeferredRunWork::ExternalInterrupt { vector } => {
                Self::after_external_interrupt(vm, vcpu, vector);
            }
        }
        Ok(VcpuRunAction {
            waits_for_event: false,
            stop_reason: None,
            resets_vm: false,
            exits_vcpu: false,
        })
    }
}

fn inject_virtual_irq(irq_id: usize) -> bool {
    const CNTV_PPI: usize = 27;

    if irq_id != CNTV_PPI {
        trace!("skip AArch64 virtual IRQ {irq_id}: only CNTV PPI is forwarded");
        return false;
    }

    let Some(vm_id) = crate::current_vm_id() else {
        trace!("skip AArch64 virtual IRQ {irq_id}: no current VM context");
        return false;
    };
    let Some(vcpu_id) = crate::current_vcpu_id() else {
        trace!("skip AArch64 virtual IRQ {irq_id}: no current vCPU context");
        return false;
    };

    if let Err(err) = crate::runtime::vcpus::queue_interrupt(vm_id, vcpu_id, irq_id) {
        warn!("failed to inject AArch64 virtual IRQ {irq_id}: {err:?}");
        return false;
    }
    true
}

fn handle_sysreg_read(
    vm: &crate::AxVMRef,
    vcpu: &crate::vm::AxVCpuRef<AxvmArmVcpu>,
    exit: SysRegReadExit,
) -> AxVmResult<BoundVcpuExit<Aarch64DeferredRunWork>> {
    if let Some(value) = read_virtualized_id_register(exit.addr.addr()) {
        vcpu.set_gpr(exit.reg, value as usize);
        return Ok(BoundVcpuExit::Continue);
    }

    sysreg::handle_read(vm, vcpu, exit)
}

fn read_virtualized_id_register(addr: usize) -> Option<u64> {
    let value = match addr {
        ID_AA64PFR0_EL1_SYSREG => Some(virtualize_id_aa64pfr0_el1(read_id_aa64pfr0_el1())),
        ID_AA64PFR1_EL1_SYSREG => Some(read_id_aa64pfr1_el1()),
        ID_AA64DFR0_EL1_SYSREG => Some(virtualize_id_aa64dfr0_el1(read_id_aa64dfr0_el1())),
        ID_AA64ISAR0_EL1_SYSREG => Some(read_id_aa64isar0_el1()),
        ID_AA64ISAR1_EL1_SYSREG => Some(read_id_aa64isar1_el1()),
        ID_AA64MMFR0_EL1_SYSREG => Some(read_id_aa64mmfr0_el1()),
        ID_AA64MMFR1_EL1_SYSREG => Some(read_id_aa64mmfr1_el1()),
        ID_AA64MMFR2_EL1_SYSREG => Some(read_id_aa64mmfr2_el1()),
        _ if is_id_aa64_feature_register(addr) => Some(0),
        _ => None,
    }?;
    Some(value)
}

const ID_AA64PFR0_EL1_SYSREG: usize = 0x300008;
const ID_AA64PFR1_EL1_SYSREG: usize = 0x320008;
const ID_AA64DFR0_EL1_SYSREG: usize = 0x30000a;
const ID_AA64ISAR0_EL1_SYSREG: usize = 0x30000c;
const ID_AA64ISAR1_EL1_SYSREG: usize = 0x32000c;
const ID_AA64MMFR0_EL1_SYSREG: usize = 0x30000e;
const ID_AA64MMFR1_EL1_SYSREG: usize = 0x32000e;
const ID_AA64MMFR2_EL1_SYSREG: usize = 0x34000e;

fn is_id_aa64_feature_register(addr: usize) -> bool {
    const ID_AA64_OP0_OP1_CRN: usize = 0x300000;
    const CRN_MASK: usize = 0x3fc000;
    const CRM_MASK: usize = 0x1e;
    const CRM_PFR: usize = 0x8;
    const CRM_DFR: usize = 0xa;
    const CRM_ISAR: usize = 0xc;
    const CRM_MMFR: usize = 0xe;

    addr & CRN_MASK == ID_AA64_OP0_OP1_CRN
        && matches!(addr & CRM_MASK, CRM_PFR | CRM_DFR | CRM_ISAR | CRM_MMFR)
}

fn virtualize_id_aa64pfr0_el1(value: u64) -> u64 {
    const EL2_SHIFT: u32 = 8;
    const FEATURE_MASK: u64 = 0xf;
    const NOT_IMPLEMENTED: u64 = 0xf;

    (value & !(FEATURE_MASK << EL2_SHIFT)) | (NOT_IMPLEMENTED << EL2_SHIFT)
}

fn virtualize_id_aa64dfr0_el1(value: u64) -> u64 {
    const PMUVER_SHIFT: u32 = 8;
    const FEATURE_MASK: u64 = 0xf;

    value & !(FEATURE_MASK << PMUVER_SHIFT)
}

#[cfg(target_arch = "aarch64")]
fn read_id_aa64pfr0_el1() -> u64 {
    let value: u64;
    // SAFETY: This reads an architectural ID register from the host CPU while
    // handling a trapped guest read. It has no side effects.
    unsafe {
        core::arch::asm!("mrs {value}, ID_AA64PFR0_EL1", value = out(reg) value);
    }
    value
}

#[cfg(not(target_arch = "aarch64"))]
fn read_id_aa64pfr0_el1() -> u64 {
    0
}

macro_rules! read_id_register {
    ($name:ident, $register:literal) => {
        #[cfg(target_arch = "aarch64")]
        fn $name() -> u64 {
            let value: u64;
            // SAFETY: This reads an architectural ID register from the host CPU
            // while handling a trapped guest read. It has no side effects.
            unsafe {
                core::arch::asm!(concat!("mrs {value}, ", $register), value = out(reg) value);
            }
            value
        }

        #[cfg(not(target_arch = "aarch64"))]
        fn $name() -> u64 {
            0
        }
    };
}

read_id_register!(read_id_aa64pfr1_el1, "ID_AA64PFR1_EL1");
read_id_register!(read_id_aa64dfr0_el1, "ID_AA64DFR0_EL1");
read_id_register!(read_id_aa64isar0_el1, "ID_AA64ISAR0_EL1");
read_id_register!(read_id_aa64isar1_el1, "ID_AA64ISAR1_EL1");
read_id_register!(read_id_aa64mmfr0_el1, "ID_AA64MMFR0_EL1");
read_id_register!(read_id_aa64mmfr1_el1, "ID_AA64MMFR1_EL1");
read_id_register!(read_id_aa64mmfr2_el1, "ID_AA64MMFR2_EL1");

struct AxvmArmHostOps;

impl ArmHostOps for AxvmArmHostOps {
    fn inject_virtual_interrupt(vector: u8) -> ArmVcpuResult {
        gic::inject_interrupt(vector as usize);
        Ok(())
    }

    fn fetch_pending_host_irq() -> Option<usize> {
        Some(gic::fetch_irq())
    }

    fn handle_current_host_irq() {
        gic::handle_current_irq();
    }
}

pub(crate) struct AxvmArmVcpu(ArmVcpu<AxvmArmHostOps>);

impl VmArchVcpuOps for AxvmArmVcpu {
    type CreateConfig = ArmVcpuCreateConfig;
    type SetupConfig = ArmVcpuSetupConfig;
    type Exit = ArmVmExit;

    fn guest_mpidr_from_create_config(config: &Self::CreateConfig) -> Option<u64> {
        Some(config.mpidr_el1 as u64)
    }

    fn new(vm_id: VMId, vcpu_id: VCpuId, config: Self::CreateConfig) -> BackendResult<Self> {
        arm_result(ArmVcpu::new(vm_id, vcpu_id, config)).map(Self)
    }

    fn set_entry(&mut self, entry: GuestPhysAddr) -> BackendResult {
        arm_result(self.0.set_entry(ax_guest_phys_addr_to_arm(entry)))
    }

    fn set_nested_page_table(&mut self, config: NestedPagingConfig) -> BackendResult {
        arm_result(
            self.0
                .set_nested_page_table(ax_nested_paging_to_arm(config)),
        )
    }

    fn setup(&mut self, config: Self::SetupConfig) -> BackendResult {
        arm_result(self.0.setup(config))
    }

    fn run(&mut self) -> BackendResult<Self::Exit> {
        arm_result(self.0.run())
    }

    fn bind(&mut self) -> BackendResult {
        arm_result(self.0.bind())
    }

    fn unbind(&mut self) -> BackendResult {
        arm_result(self.0.unbind())
    }

    fn set_gpr(&mut self, reg: usize, val: usize) {
        self.0.set_gpr(reg, val);
    }

    fn inject_interrupt(&mut self, vector: usize) -> BackendResult {
        arm_result(self.0.inject_interrupt(vector))
    }

    fn inject_interrupt_with_trigger(
        &mut self,
        vector: usize,
        trigger: InterruptTriggerMode,
    ) -> BackendResult {
        // The Arm Router/VGIC consumes line trigger semantics before emitting
        // an INTID. The GIC list-register injection itself is mode-agnostic.
        match trigger {
            InterruptTriggerMode::EdgeTriggered | InterruptTriggerMode::LevelTriggered => {
                arm_result(self.0.inject_interrupt(vector))
            }
        }
    }

    fn set_return_value(&mut self, val: usize) {
        self.0.set_return_value(val);
    }
}

pub(crate) struct AxvmArmPerCpu(ArmPerCpu);

impl VmArchPerCpuOps for AxvmArmPerCpu {
    fn new(cpu_id: usize) -> BackendResult<Self> {
        arm_result(ArmPerCpu::new(cpu_id)).map(Self)
    }

    fn is_enabled(&self) -> bool {
        self.0.is_enabled()
    }

    fn hardware_enable(&mut self) -> BackendResult {
        arm_result(self.0.hardware_enable::<AxvmArmHostOps>())
    }

    fn hardware_disable(&mut self) -> BackendResult {
        arm_result(self.0.hardware_disable())
    }

    fn max_guest_page_table_levels(&self) -> usize {
        self.0.max_guest_page_table_levels()
    }

    fn guest_phys_addr_bits(&self) -> usize {
        self.0.guest_phys_addr_bits()
    }
}

fn arm_result<T>(result: ArmVcpuResult<T>) -> BackendResult<T> {
    result.map_err(arm_error_to_backend)
}

fn arm_error_to_backend(err: ArmVcpuError) -> BackendError {
    match err {
        ArmVcpuError::InvalidInput => BackendError::InvalidInput,
        ArmVcpuError::Unsupported => BackendError::Unsupported,
        ArmVcpuError::BadState => BackendError::InvalidState,
    }
}

fn ax_guest_phys_addr_to_arm(addr: GuestPhysAddr) -> ArmGuestPhysAddr {
    ArmGuestPhysAddr::from_usize(addr.as_usize())
}

fn arm_guest_phys_addr_to_ax(addr: ArmGuestPhysAddr) -> GuestPhysAddr {
    GuestPhysAddr::from(addr.as_usize())
}

fn ax_nested_paging_to_arm(config: NestedPagingConfig) -> ArmNestedPagingConfig {
    ArmNestedPagingConfig::new(
        config.root_paddr.as_usize(),
        config.levels,
        config.gpa_bits,
        config.mode,
    )
}

fn arm_access_width_to_ax(width: ArmAccessWidth) -> AccessWidth {
    match width {
        ArmAccessWidth::Byte => AccessWidth::Byte,
        ArmAccessWidth::Word => AccessWidth::Word,
        ArmAccessWidth::Dword => AccessWidth::Dword,
        ArmAccessWidth::Qword => AccessWidth::Qword,
    }
}

fn arm_sys_reg_addr_to_ax(addr: ArmSysRegAddr) -> SysRegAddr {
    SysRegAddr::new(addr.addr())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converts_arm_vcpu_errors_to_backend_errors() {
        assert_eq!(
            arm_error_to_backend(ArmVcpuError::InvalidInput),
            BackendError::InvalidInput
        );
        assert_eq!(
            arm_error_to_backend(ArmVcpuError::Unsupported),
            BackendError::Unsupported
        );
        assert_eq!(
            arm_error_to_backend(ArmVcpuError::BadState),
            BackendError::InvalidState
        );
    }

    fn assert_arm_exit_type<T: VmArchVcpuOps<Exit = ArmVmExit>>() {}

    #[test]
    fn axvm_arm_vcpu_uses_arm_exit_type() {
        assert_arm_exit_type::<AxvmArmVcpu>();
    }

    #[test]
    fn converts_arm_value_types_to_axvm_value_types() {
        assert_eq!(
            arm_guest_phys_addr_to_ax(ArmGuestPhysAddr::from_usize(0x4000)).as_usize(),
            0x4000
        );
        assert_eq!(
            arm_access_width_to_ax(ArmAccessWidth::Dword),
            AccessWidth::Dword
        );
        assert_eq!(
            arm_access_width_to_ax(ArmAccessWidth::Qword),
            AccessWidth::Qword
        );
        assert_eq!(
            arm_sys_reg_addr_to_ax(ArmSysRegAddr::new(0x3a_3016)).addr(),
            0x3a_3016
        );
    }

    #[test]
    fn virtualized_id_aa64pfr0_hides_el2_from_guest() {
        const EL2_SHIFT: u32 = 8;
        const FEATURE_MASK: u64 = 0xf;
        let host_value = 0x1234_5678_9abc_def0;

        let guest_value = virtualize_id_aa64pfr0_el1(host_value);

        assert_eq!((guest_value >> EL2_SHIFT) & FEATURE_MASK, 0xf);
        assert_eq!(
            guest_value & !(FEATURE_MASK << EL2_SHIFT),
            host_value & !(FEATURE_MASK << EL2_SHIFT)
        );
    }

    #[test]
    fn virtualized_id_registers_cover_linux_early_feature_reads() {
        assert!(read_virtualized_id_register(ID_AA64PFR0_EL1_SYSREG).is_some());
        assert!(read_virtualized_id_register(ID_AA64DFR0_EL1_SYSREG).is_some());
        assert!(read_virtualized_id_register(ID_AA64MMFR0_EL1_SYSREG).is_some());
        assert_eq!(read_virtualized_id_register(0x36000c), Some(0));
        assert!(read_virtualized_id_register(0x3f_ffff).is_none());
    }

    #[test]
    fn virtualized_id_aa64dfr0_hides_unvirtualized_pmu_from_guest() {
        const PMUVER_SHIFT: u32 = 8;
        const FEATURE_MASK: u64 = 0xf;
        let host_value = 0x1234_5678_9abc_def0;

        let guest_value = virtualize_id_aa64dfr0_el1(host_value);

        assert_eq!((guest_value >> PMUVER_SHIFT) & FEATURE_MASK, 0);
        assert_eq!(
            guest_value & !(FEATURE_MASK << PMUVER_SHIFT),
            host_value & !(FEATURE_MASK << PMUVER_SHIFT)
        );
    }
}

struct ArmVgicHostIfImpl;

#[impl_interface]
impl ArmVgicHostIf for ArmVgicHostIfImpl {
    fn alloc_contiguous_frames(frame_count: usize, frame_align: usize) -> Option<PhysAddr> {
        default_host().alloc_contiguous_frames(frame_count, frame_align)
    }

    fn dealloc_contiguous_frames(start_paddr: PhysAddr, frame_count: usize) {
        default_host().dealloc_contiguous_frames(start_paddr, frame_count);
    }

    fn phys_to_virt(paddr: PhysAddr) -> VirtAddr {
        default_host().phys_to_virt(paddr)
    }

    fn host_cpu_num() -> usize {
        default_host().cpu_count()
    }

    fn current_vcpu_id() -> usize {
        crate::current_vcpu_id().expect("current AArch64 vCPU is not set")
    }

    fn current_vm_id() -> usize {
        crate::current_vm_id().expect("current AArch64 VM is not set")
    }

    fn queue_virtual_interrupt(vm_id: usize, vcpu_id: usize, vector: u8) {
        if let Err(err) = crate::runtime::vcpus::queue_interrupt(vm_id, vcpu_id, vector as usize) {
            warn!(
                "failed to queue VM[{vm_id}] vCPU[{vcpu_id}] virtual interrupt {vector}: {err:?}"
            );
        }
    }

    fn current_time_nanos() -> u64 {
        default_host().monotonic_time().as_nanos() as u64
    }

    fn register_timer(
        deadline: Duration,
        callback: Box<dyn FnOnce(Duration) + Send + 'static>,
    ) -> usize {
        crate::timer::register_timer(deadline.as_nanos() as u64, callback)
    }

    fn cancel_timer(token: usize) {
        crate::timer::cancel_timer(token);
    }

    fn read_vgicd_iidr() -> u32 {
        gic::read_gicd_iidr()
    }

    fn read_vgicd_typer() -> u32 {
        gic::read_gicd_typer()
    }

    fn get_host_gicd_base() -> PhysAddr {
        gic::host_gicd_base()
    }

    fn get_host_gicr_base() -> PhysAddr {
        gic::host_gicr_base()
    }

    fn hardware_inject_virtual_interrupt(vector: u8) {
        gic::inject_interrupt(vector as usize);
    }
}

#[cfg(all(test, feature = "host-test"))]
mod guest_mpidr_tests {
    use super::*;

    #[test]
    fn guest_mpidr_from_real_arm_create_config() {
        let config = ArmVcpuCreateConfig {
            mpidr_el1: 0x100,
            dtb_addr: 0x4000_0000,
        };

        assert_eq!(
            <AxvmArmVcpu as VmArchVcpuOps>::guest_mpidr_from_create_config(&config),
            Some(0x100),
        );
    }
}
