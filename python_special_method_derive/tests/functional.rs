use pyo3::pyclass;
use python_special_method_derive::{DirHelper, StrReprHelper};

#[pyclass]
#[derive(DirHelper, StrReprHelper)]
#[allow(dead_code)]
struct WithFields {
    dora: u32,
    my: String,
    name: f32,
}

#[test]
fn test_with_dir() {
    let dir = WithFields {
        dora: 0,
        my: "".to_string(),
        name: 0.0,
    }
    .__dir__();
    assert_eq!(
        vec!["dora".to_string(), "my".to_string(), "name".to_string()],
        dir
    );
}

#[test]
fn test_with_str() {
    let res = WithFields {
        dora: 299792458,
        my: "Hello world".to_string(),
        name: 3.14159,
    }
    .__str__();
    // TODO: Is this a good __str__ output? How can we better show it or should they be different?
    assert_eq!(
        "WithFields(dora=`299792458`, my=`Hello world`, name=`3.14159`)",
        &res
    );
}

#[test]
fn test_with_repr() {
    let res = WithFields {
        dora: 299792458,
        my: "Hello world".to_string(),
        name: 3.14159,
    }
    .__repr__();
    // TODO: Is this a good __repr__ output? How can we better show it or should they be different?
    assert_eq!(
        "WithFields(dora=299792458, my=\"Hello world\", name=3.14159)",
        &res
    );
}

#[pyclass]
#[derive(DirHelper)]
#[allow(dead_code)]
struct UnitNoFields;

#[test]
fn test_no_fields() {
    let fields: Vec<String> = UnitNoFields.__dir__();
    assert_eq!(Vec::<String>::new(), fields);
}
