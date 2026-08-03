use std::{env, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-env-changed=CARGO_CFG_TARGET_OS");

    if env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("none") {
        let linker = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../components/percpu/percpu/host-test.ld");
        println!("cargo:rerun-if-changed={}", linker.display());
        println!("cargo:rustc-link-arg=-T{}", linker.display());
    }
}
