use std::path::PathBuf;

pub fn create(args: crate::CommandNew) -> eyre::Result<()> {
    let crate::CommandNew {
        kind,
        lang: _,
        name,
        path,
    } = args;

    match kind {
        crate::Kind::Operator => create_operator(name, path),
        crate::Kind::CustomNode => create_custom_node(name, path),
        crate::Kind::Dataflow => todo!(),
    }
}

fn create_operator(_name: String, _path: Option<PathBuf>) -> Result<(), eyre::ErrReport> {
    todo!()
}

fn create_custom_node(_name: String, _path: Option<PathBuf>) -> Result<(), eyre::ErrReport> {
    todo!()
}
