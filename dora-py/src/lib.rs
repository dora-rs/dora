use async_std::task::sleep;
use futures::prelude::*;
use futures::select;
use pollster::FutureExt;
use pyo3::types::{PyDict, PyTuple};
use pyo3::{prelude::*, wrap_pyfunction};
use std::time::Duration;
use zenoh::config::Config;
use zenoh::net::protocol::io::SplitBuffer;
use zenoh::subscriber::Subscriber;
use zenoh::Session;

/// A function decorator that keeps track how often it is called.
///
/// It otherwise doesn't do anything special.
#[pyclass(name = "PyDoraSession")]
pub struct PyDoraSession {
    // We use `#[pyo3(get)]` so that python can read the count but not mutate it.
    #[pyo3(get)]
    count: u64,

    #[pyo3(get)]
    topic: String,
    session: Session,

    // This is the actual function being wrapped.
    wraps: Py<PyAny>,
}

#[pymethods]
impl PyDoraSession {
    // Note that we don't validate whether `wraps` is actually callable.
    //
    // While we could use `PyAny::is_callable` for that, it has some flaws:
    //    1. It doesn't guarantee the object can actually be called successfully
    //    2. We still need to handle any exceptions that the function might raise
    #[new]
    fn __new__(wraps: Py<PyAny>) -> Self {
        let config = Config::default();
        let session = pollster::block_on(zenoh::open(config)).unwrap();

        PyDoraSession {
            count: 0,
            wraps,
            topic: "".to_string(),
            session,
        }
    }

    #[args(args = "*", kwargs = "**")]
    fn __call__(&mut self, py: Python, args: &PyTuple, kwargs: Option<&PyDict>) -> PyResult<()> {
        let subscriber = pollster::block_on(self.session.subscribe("a")).unwrap();

        // After doing something, we finally forward the call to the wrapped function
        let ret = self.wraps.call(py, args, kwargs)?;

        // We could do something with the return value of
        // the function before returning it
        Ok(())
    }
}

#[pyfunction]
fn register(py: Python, function: PyObject) -> PyResult<&pyo3::PyAny> {
    env_logger::init();
    let config = Config::default();
    pyo3_asyncio::async_std::future_into_py(py, async {
        let session = pollster::block_on(zenoh::open(config)).unwrap();
        let mut subscriber = pollster::block_on(session.subscribe("a")).unwrap();

        let result = subscriber.next().block_on().unwrap();
        let key = result.key_expr.clone();

        Ok(key.as_str().to_string())
    })
}

#[pymodule]
fn dora(py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(register, m)?)?;
    Ok(())
}
