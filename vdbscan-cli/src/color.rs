use vdbscan::ClusterLabel;

pub fn label_to_rgb(label: ClusterLabel) -> [u8; 3] {
    match label {
        None => [128, 128, 128],
        Some(id) => {
            let hue = (id.get() as f32 * 0.618033988749895) % 1.0;
            hsv_to_rgb(hue, 0.8, 0.9)
        }
    }
}

pub fn hsv_to_rgb(h: f32, s: f32, v: f32) -> [u8; 3] {
    let i = (h * 6.0) as u32;
    let f = h * 6.0 - i as f32;
    let p = v * (1.0 - s);
    let q = v * (1.0 - f * s);
    let t = v * (1.0 - (1.0 - f) * s);
    let (r, g, b) = match i % 6 {
        0 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        _ => (v, p, q),
    };
    [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8]
}
