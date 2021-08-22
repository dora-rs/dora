#![allow(
    improper_ctypes,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    invalid_codeblock_attributes,
    deref_nullptr,
    clippy::all,
    rustdoc::bare_urls,
    rustdoc::broken_intra_doc_links,
    rustdoc::invalid_rust_codeblocks
)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
