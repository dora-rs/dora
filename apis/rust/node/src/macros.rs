/// Create a [`NodeFailureError`][crate::NodeFailureError] with file, line, and column information.
///
/// Sets the `file`, `line`, and `column` fields of the error to the location where the macro is
/// invoked.
///
/// Expects a single expression that can be converted into a `String`, which will be used as the
/// `summary` field of the `NodeFailureError`.
///
/// ## Example
///
/// ```
/// use dora_node_api::node_failure_error;
///
/// let mut error = node_failure_error!("an error occurred");
/// error.detailed = Some("something error details".into());
/// assert_eq!(error.summary, "an error occurred");
/// assert_eq!(error.file.unwrap(), file!());
/// assert_eq!(error.line.unwrap(), line!() - 4);
/// ```
#[macro_export]
macro_rules! node_failure_error {
    ($summary:expr) => {{
        let file = file!().into();
        let line = line!();
        let column = column!();
        let mut error = $crate::NodeFailureError::new($summary);
        error.file = Some(file);
        error.line = Some(line);
        error.column = Some(column);
        error
    }};
}
