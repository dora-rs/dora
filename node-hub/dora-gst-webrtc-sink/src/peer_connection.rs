use gstreamer::prelude::*;
use gstreamer_app::AppSrc;

#[derive(Debug, Clone)]
pub struct PeerConnection {
    pub _id: String,
    pub pipeline: gstreamer::Pipeline,
    pub webrtcbin: gstreamer::Element,
    pub appsrc: AppSrc,
    caps_set: std::sync::Arc<std::sync::Mutex<bool>>,
    frame_count: std::sync::Arc<std::sync::Mutex<u32>>,
    first_frame_time: std::sync::Arc<std::sync::Mutex<Option<std::time::Instant>>>,
    detected_framerate: std::sync::Arc<std::sync::Mutex<Option<gstreamer::Fraction>>>,
}

impl PeerConnection {
    pub fn new(
        id: String,
        pipeline: gstreamer::Pipeline,
        webrtcbin: gstreamer::Element,
        appsrc: AppSrc,
    ) -> Self {
        Self {
            _id: id,
            pipeline,
            webrtcbin,
            appsrc,
            caps_set: std::sync::Arc::new(std::sync::Mutex::new(false)),
            frame_count: std::sync::Arc::new(std::sync::Mutex::new(0)),
            first_frame_time: std::sync::Arc::new(std::sync::Mutex::new(None)),
            detected_framerate: std::sync::Arc::new(std::sync::Mutex::new(None)),
        }
    }

    pub fn send_frame(&self, frame_data: &[u8]) -> Result<(), gstreamer::FlowError> {
        // Track frame timing for framerate detection
        let now = std::time::Instant::now();
        let mut frame_count = self.frame_count.lock().unwrap();
        let mut first_frame_time = self.first_frame_time.lock().unwrap();
        let mut detected_framerate = self.detected_framerate.lock().unwrap();

        if first_frame_time.is_none() {
            *first_frame_time = Some(now);
        }
        *frame_count += 1;

        // Detect framerate after receiving enough frames (e.g., 30 frames)
        if *frame_count >= 30 && detected_framerate.is_none() {
            if let Some(start_time) = *first_frame_time {
                let elapsed = now.duration_since(start_time).as_secs_f64();
                if elapsed > 0.0 {
                    let fps = (*frame_count as f64 - 1.0) / elapsed;
                    let fps_rounded = fps.round() as i32;

                    // Common framerates: 15, 24, 25, 30, 50, 60
                    let common_fps = [15, 24, 25, 30, 50, 60];
                    let mut best_fps = 30;
                    let mut min_diff = i32::MAX;

                    for &candidate in &common_fps {
                        let diff = (fps_rounded - candidate).abs();
                        if diff < min_diff {
                            min_diff = diff;
                            best_fps = candidate;
                        }
                    }

                    *detected_framerate = Some(gstreamer::Fraction::new(best_fps, 1));
                    log::info!(
                        "Detected framerate: {} fps (measured: {:.2} fps)",
                        best_fps,
                        fps
                    );
                }
            }
        }

        // Set caps on first frame or when framerate is detected
        let mut caps_set = self.caps_set.lock().unwrap();
        if !*caps_set || (*caps_set && detected_framerate.is_some() && *frame_count == 30) {
            // Detect resolution from frame size (RGB = 3 bytes per pixel)
            let pixels = frame_data.len() / 3;

            // Common resolutions
            let resolutions = [
                (640, 480),
                (800, 600),
                (1024, 768),
                (1280, 720),
                (1280, 960),
                (1280, 1024),
                (1920, 1080),
                (2560, 1440),
                (3840, 2160),
            ];

            let mut width = 640;
            let mut height = 480;

            for (w, h) in resolutions.iter() {
                if (*w as usize) * (*h as usize) == pixels {
                    width = *w;
                    height = *h;
                    break;
                }
            }

            // Build caps with detected or default framerate
            let mut caps_builder = gstreamer::Caps::builder("video/x-raw")
                .field("format", "RGB")
                .field("width", width as i32)
                .field("height", height as i32);

            if let Some(framerate) = &*detected_framerate {
                caps_builder = caps_builder.field("framerate", framerate);
            } else {
                // Use a default framerate initially
                caps_builder = caps_builder.field("framerate", gstreamer::Fraction::new(30, 1));
            }

            let caps = caps_builder.build();
            self.appsrc.set_caps(Some(&caps));
            *caps_set = true;

            if *frame_count == 30 {
                log::info!("Updated caps with detected framerate");
            }
        }

        let mut buffer = gstreamer::Buffer::from_slice(frame_data.to_vec());

        // Set the buffer timestamp
        let buffer_ref = buffer.get_mut().unwrap();
        let clock = gstreamer::SystemClock::obtain();
        let base_time = self.pipeline.base_time();
        let now = clock.time();
        if let (Some(now), Some(base_time)) = (now, base_time) {
            buffer_ref.set_pts(now - base_time);
        }

        match self.appsrc.push_buffer(buffer) {
            Ok(_) => Ok(()),
            Err(e) => Err(e),
        }
    }

    pub fn shutdown(&self) {
        let _ = self.pipeline.set_state(gstreamer::State::Null);
    }
}
