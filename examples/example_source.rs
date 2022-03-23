use std::time::Duration;

fn main() {
    for i in 0..100 {
        println!("{i}");
        std::thread::sleep(Duration::from_millis(100));
    }
}
