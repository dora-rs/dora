use zenoh::prelude::{Receiver, ZFuture};

fn main() {
    let zenoh = zenoh::open(zenoh_config::Config::default()).wait().unwrap();
    let mut sub = zenoh.subscribe("/**").wait().unwrap();

    loop {
        let msg = sub.receiver().recv().unwrap();
        println!("{}", msg.key_expr);
    }
}
