fn main() {
    // This is needed if your flash or ram addresses are not aligned to 0x10000 in memory.x
    // See https://github.com/rust-embedded/cortex-m-quickstart/pull/95
    println!("cargo:rustc-link-arg-bins=--nmagic");
    // LLD (shipped with the Rust toolchain) is used as the default linker
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    // For defmt logging, we need to add this one:
    // See: https://defmt.ferrous-systems.com/setup-app.html
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
