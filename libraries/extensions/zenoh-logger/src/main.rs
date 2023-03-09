use zenoh::prelude::{sync::SyncResolve, Config};

fn main() {
    let zenoh = zenoh::open(Config::default()).res_sync().unwrap();
    let sub = zenoh
        .declare_subscriber("/**")
        .reliable()
        .res_sync()
        .unwrap();

    loop {
        let msg = sub.recv().unwrap();
        println!("{}", msg.key_expr);
    }
}
