//! AxVisor device adapter that connects [`VirtioMmioNetDevice`] to the AxVM
//! MMIO dispatch router.
//!
//! The adapter owns the device model, its edge-triggered [`IrqLine`] and the
//! stable resource list the router uses for address dispatch. It translates
//! [`BusAccess`] MMIO exits into device `mmio_read`/`mmio_write` calls and, when
//! a write reports [`DeviceEvent::InterruptPending`], pulses the IRQ so the
//! backend interrupt reaches the target vCPU through the VM-local queued sink.

use alloc::sync::Arc;

use axdevice_base::{BusAccess, BusResponse, Device, DeviceAccess, DeviceError, IrqLine, Resource};
use axvirtio_net::{ManagedVirtioNetDevice, VirtioMmioNetDevice};
use axvm::AxvmGuestMemoryAccessor;

use super::backend::AxvisorNetworkBackend;
use super::config::VirtioNetDeviceSpec;
use super::raw_uplink::PortAttachment;

/// AxVisor adapter wrapping one virtio-net MMIO device model.
pub struct VirtioNetDeviceAdapter {
    managed: ManagedVirtioNetDevice<AxvisorNetworkBackend, AxvmGuestMemoryAccessor>,
    backend: AxvisorNetworkBackend,
    /// RAII switch-port attachment for raw-uplink devices. Dropping the adapter
    /// detaches the port (deactivate -> unregister); `None` for the
    /// deterministic echo backend, which owns no switch port. The field is
    /// drop-only by design, so it is never read directly.
    _attachment: Option<PortAttachment>,
}

impl VirtioNetDeviceAdapter {
    /// Creates a new adapter from its prepared components.
    pub(super) fn new(
        spec: VirtioNetDeviceSpec,
        device: Arc<VirtioMmioNetDevice<AxvisorNetworkBackend, AxvmGuestMemoryAccessor>>,
        irq: IrqLine,
        backend: AxvisorNetworkBackend,
        attachment: Option<PortAttachment>,
    ) -> Self {
        Self {
            managed: ManagedVirtioNetDevice::new(
                spec.name,
                device,
                irq,
                spec.base_gpa as u64,
                spec.length as u64,
                spec.irq_id as u32,
            ),
            backend,
            _attachment: attachment,
        }
    }

    /// Returns the device model handle (shared with the RX worker).
    pub fn device(
        &self,
    ) -> &Arc<VirtioMmioNetDevice<AxvisorNetworkBackend, AxvmGuestMemoryAccessor>> {
        self.managed.model()
    }

    /// Returns the interrupt line used to signal RX/TX completions.
    pub fn irq(&self) -> &IrqLine {
        self.managed.irq()
    }

    /// Returns the backend handle (shared with the RX worker).
    pub fn backend(&self) -> &AxvisorNetworkBackend {
        &self.backend
    }
}

impl Device for VirtioNetDeviceAdapter {
    fn name(&self) -> &str {
        self.managed.name()
    }

    fn resources(&self) -> &[Resource] {
        self.managed.resources()
    }

    fn access(
        &self,
        access: &BusAccess,
        context: &mut dyn DeviceAccess,
    ) -> Result<BusResponse, DeviceError> {
        self.managed.access(access, context)
    }
}
