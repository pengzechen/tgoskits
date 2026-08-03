// Copyright 2025 The Axvisor Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use aarch64_cpu::registers::{CNTVCT_EL0, Readable};
use aarch64_sysreg::SystemRegType;
use axdevice_base::{
    AccessWidth, BusAccess, BusKind, BusResponse, Device, DeviceAccess, DeviceError, DeviceResult,
    Resource,
};
use log::debug;

const CNTVCT_EL0_ADDR: u32 = SystemRegType::CNTVCT_EL0 as u32;

impl SysCntvctEl0 {
    /// Reads CNTVCT_EL0.
    pub fn read_register(&self, _width: AccessWidth) -> DeviceResult<usize> {
        Ok(CNTVCT_EL0.get() as usize)
    }

    /// Ignores guest writes to the read-only CNTVCT_EL0 register.
    pub fn write_register(&self, _width: AccessWidth, val: usize) -> DeviceResult {
        debug!("Write to read-only virtual counter register CNTVCT_EL0, value: {val}");
        Ok(())
    }
}

/// System register emulation for CNTVCT_EL0.
///
/// Provides virtualization support for the virtual counter register.
pub struct SysCntvctEl0 {
    resources: [Resource; 1],
}

impl SysCntvctEl0 {
    /// Creates a new CNTVCT_EL0 register emulator.
    pub fn new() -> Self {
        Self {
            resources: [Resource::SysReg {
                addr: CNTVCT_EL0_ADDR,
                count: 1,
            }],
        }
    }
}

impl Default for SysCntvctEl0 {
    fn default() -> Self {
        Self::new()
    }
}

impl Device for SysCntvctEl0 {
    fn name(&self) -> &str {
        "aarch64-cntvct-el0"
    }

    fn resources(&self) -> &[Resource] {
        &self.resources
    }

    fn access(
        &self,
        access: &BusAccess,
        _context: &mut dyn DeviceAccess,
    ) -> Result<BusResponse, DeviceError> {
        if access.kind != BusKind::SysReg || access.addr != CNTVCT_EL0_ADDR as u64 {
            return Err(DeviceError::OutOfRange { addr: access.addr });
        }
        if access.is_read {
            self.read_register(access.width)
                .map(|value| BusResponse::Read {
                    value: value as u64,
                })
        } else {
            self.write_register(access.width, access.data as usize)
                .map(|_| BusResponse::Write)
        }
    }
}
