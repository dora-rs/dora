use zenoh::prelude::sync::SyncResolve;

fn main() {
    let zenoh = zenoh::open(zenoh_config::Config::default())
        .res_sync()
        .unwrap();
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
