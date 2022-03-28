#![feature(test)]
mod bench {
    extern crate test;
    use std::{collections::BTreeMap, sync::Arc};

    use dora_rs::python::binding::{call, init};
    use test::Bencher;

    #[bench]
    fn init_python(b: &mut Bencher) {
        b.iter(|| {
            init("source", "produce").unwrap();
        });
    }

    #[bench]
    fn call_python(b: &mut Bencher) {
        let function = Arc::new(init("source", "produce").unwrap());
        let state = BTreeMap::new();
        b.iter(|| call(function.clone(), &state));
    }
}
