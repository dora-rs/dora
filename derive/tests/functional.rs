use derive::DirHelper;
use pyo3::pyclass;

#[derive(DirHelper)]
#[pyclass]
#[allow(dead_code)]
struct WithFields {
    hello: (),
    dora: u32,
    my: String,
    name: f32,
}

#[test]
fn test_with_fields() {
    let fields = WithFields {
        hello: (),
        dora: 0,
        my: "".to_string(),
        name: 0.0,
    }
    .__dir__();
    assert_eq!(
        vec![
            "hello".to_string(),
            "dora".to_string(),
            "my".to_string(),
            "name".to_string()
        ],
        fields
    );
}

#[derive(DirHelper)]
#[pyclass]
#[allow(dead_code)]
struct UnitNoFields;

#[test]
fn test_no_fields() {
    let fields: Vec<String> = UnitNoFields.__dir__();
    assert_eq!(Vec::<String>::new(), fields);
}
