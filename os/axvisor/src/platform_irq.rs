struct RiscvPlatformIrqInjector;

#[ax_crate_interface::impl_interface]
impl axvm::irq::PlatformIrqInjectorIf for RiscvPlatformIrqInjector {
    fn register_virtual_irq_injector(injector: fn(usize) -> bool) {
        axplat_dyn::register_virtual_irq_injector(injector);
    }

    fn set_virtual_irq_targets(cpu_id: usize, irq_sources: &[u32]) {
        #[cfg(not(target_arch = "riscv64"))]
        let (_cpu_id, _irq_sources) = (cpu_id, irq_sources);

        #[cfg(target_arch = "riscv64")]
        axplat_dyn::set_virtual_irq_targets(cpu_id, irq_sources);
    }
}
