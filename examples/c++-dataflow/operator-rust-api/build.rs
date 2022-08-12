fn main() {
    cxx_build::bridge("src/lib.rs") // returns a cc::Build
        .file("src/operator.cc")
        .flag_if_supported("-std=c++14")
        .compile("cxx-example-dataflow-operator");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/operator.cc");
    println!("cargo:rerun-if-changed=src/operator.h");
}
