use super::{OperatorEvent, OperatorInput};
use eyre::{bail, Context};
use libloading::Symbol;
use std::{
    ffi::c_void,
    panic::{catch_unwind, AssertUnwindSafe},
    path::Path,
    slice, thread,
};
use tokio::sync::mpsc::{Receiver, Sender};

pub fn spawn(
    path: &Path,
    events_tx: Sender<OperatorEvent>,
    inputs: Receiver<OperatorInput>,
) -> eyre::Result<()> {
    let library = unsafe {
        libloading::Library::new(path)
            .wrap_err_with(|| format!("failed to load shared library at `{}`", path.display()))?
    };

    thread::spawn(move || {
        let closure = AssertUnwindSafe(|| {
            let bindings = Bindings::init(&library)?;

            let operator = SharedLibraryOperator {
                events_tx: events_tx.clone(),
                inputs,
                bindings,
            };

            operator.run()
        });
        match catch_unwind(closure) {
            Ok(Ok(())) => {}
            Ok(Err(err)) => {
                let _ = events_tx.blocking_send(OperatorEvent::Error(err));
            }
            Err(panic) => {
                let _ = events_tx.blocking_send(OperatorEvent::Panic(panic));
            }
        }
    });

    Ok(())
}

struct SharedLibraryOperator<'lib> {
    events_tx: Sender<OperatorEvent>,
    inputs: Receiver<OperatorInput>,

    bindings: Bindings<'lib>,
}

impl<'lib> SharedLibraryOperator<'lib> {
    fn run(mut self) -> eyre::Result<()> {
        while let Some(input) = self.inputs.blocking_recv() {
            let id_start = input.id.as_bytes().as_ptr();
            let id_len = input.id.as_bytes().len();
            let data_start = input.value.as_slice().as_ptr();
            let data_len = input.value.len();

            println!("Received input {}", input.id);
            let output = |id: &str, data: &[u8]| -> isize {
                let result = self.events_tx.blocking_send(OperatorEvent::Output {
                    id: id.to_owned().into(),
                    value: data.to_owned(),
                });
                match result {
                    Ok(()) => 0,
                    Err(_) => -1,
                }
            };
            let (output_fn, output_ctx) = wrap_closure(&output);

            let result = unsafe {
                (self.bindings.on_input)(
                    id_start, id_len, data_start, data_len, output_fn, output_ctx,
                )
            };
            if result != 0 {
                bail!("on_input failed with exit code {result}");
            }
        }
        Ok(())
    }
}

/// Wrap a closure with an FFI-compatible trampoline function.
///
/// Returns a C compatible trampoline function and a data pointer that
/// must be passed as when invoking the trampoline function.
fn wrap_closure<F>(closure: &F) -> (OutputFn, *const c_void)
where
    F: Fn(&str, &[u8]) -> isize,
{
    /// Rust closures are just compiler-generated structs with a `call` method. This
    /// trampoline function is generic over the closure type, which means that the
    /// compiler's monomorphization step creates a different copy of that function
    /// for each closure type.
    ///
    /// The trampoline function expects the pointer to the corresponding closure
    /// struct as `context` argument. It casts that pointer back to a closure
    /// struct pointer and invokes its call method.
    unsafe extern "C" fn trampoline<F: Fn(&str, &[u8]) -> isize>(
        id_start: *const u8,
        id_len: usize,
        data_start: *const u8,
        data_len: usize,
        context: *const c_void,
    ) -> isize {
        let id_raw = unsafe { slice::from_raw_parts(id_start, id_len) };
        let data = unsafe { slice::from_raw_parts(data_start, data_len) };
        let id = match std::str::from_utf8(id_raw) {
            Ok(s) => s,
            Err(_) => return -1,
        };
        unsafe { (*(context as *const F))(id, data) }
    }

    (trampoline::<F>, closure as *const F as *const c_void)
}

struct Bindings<'lib> {
    on_input: Symbol<'lib, OnInputFn>,
}

impl<'lib> Bindings<'lib> {
    fn init(library: &'lib libloading::Library) -> Result<Self, eyre::Error> {
        let bindings = unsafe {
            Bindings {
                on_input: library
                    .get(b"dora_on_input")
                    .wrap_err("failed to get `dora_on_input`")?,
            }
        };
        Ok(bindings)
    }
}

type OnInputFn = unsafe extern "C" fn(
    id_start: *const u8,
    id_len: usize,
    data_start: *const u8,
    data_len: usize,
    output: OutputFn,
    output_context: *const c_void,
) -> isize;

type OutputFn = unsafe extern "C" fn(
    id_start: *const u8,
    id_len: usize,
    data_start: *const u8,
    data_len: usize,
    output_context: *const c_void,
) -> isize;
