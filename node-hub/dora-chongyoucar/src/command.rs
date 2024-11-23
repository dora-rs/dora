// 差速小车
pub fn send_speed_to_x4chassis(x: f64, _y: f64, w: f64) -> Vec<u8> {
    let mut data = vec![];

    let speed_offset = 10.0; // 速度偏移值 10m/s，把速度转换成正数发送

    data.push(0xAE_u8);
    data.push(0xEA);
    data.push(0x0B);
    data.push(0xF3);
    let x = ((x + speed_offset) * 100.0) as u16;
    data.push((x >> 8) as u8);
    data.push(x as u8);
    data.push(0x00);
    data.push(0x00);
    let w = ((w + speed_offset) * 100.0) as u16;
    data.push((w >> 8) as u8);
    data.push(w as u8);
    data.push(0x00);
    data.push(0x00);
    let len = data.len();
    data[2] = len as u8 - 1;

    let mut count = 0;
    for &d in data.iter().take(len).skip(2) {
        count += d as u16;
    }
    // 数据校验位
    data.push(count as u8);

    data.push(0xEF);
    data.push(0xFE);

    data
}
