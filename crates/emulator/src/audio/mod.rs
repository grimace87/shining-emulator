
use std::fs::File;
use std::io::{BufReader, BufWriter};

pub trait AudioController {
    fn new() -> Self;
    fn reset(&mut self, running_speed: u64);
    fn stop_all(&mut self);
    fn re_enable_audio(&mut self);
    fn update_routing_masks(&mut self);
    fn simulate(&mut self, ticks: u64);
    fn update_waveform_data(&mut self, from_slice: &[u8]);
    fn restart_channel_1(&mut self);
    fn restart_channel_2(&mut self);
    fn restart_channel_3(&mut self);
    fn restart_channel_4(&mut self);
    fn on_audio_thread_needing_data(&mut self, buffer: &mut [u16], frame_count: u32);
    fn persist_state(&mut self, stream: &BufReader<File>);
    fn load_state(&self, stream: &BufWriter<File>);
}

pub struct DummyAudioController {
    running_speed: u64
}

impl AudioController for DummyAudioController {

    fn new() -> Self {
        Self { running_speed: 1 }
    }

    fn reset(&mut self, running_speed: u64) {
        self.running_speed = running_speed;
    }

    fn stop_all(&mut self) {}

    fn re_enable_audio(&mut self) {}

    fn update_routing_masks(&mut self) {}

    fn simulate(&mut self, _ticks: u64) {}

    fn update_waveform_data(&mut self, _from_slice: &[u8]) {}

    fn restart_channel_1(&mut self) {}

    fn restart_channel_2(&mut self) {}

    fn restart_channel_3(&mut self) {}

    fn restart_channel_4(&mut self) {}

    fn on_audio_thread_needing_data(&mut self, buffer: &mut [u16], frame_count: u32) {
        let sample_count = frame_count as usize * 2;
        for i in 0..(sample_count as usize) {
            buffer[i] = 0;
        }
    }

    fn persist_state(&mut self, _stream: &BufReader<File>) {}

    fn load_state(&self, _stream: &BufWriter<File>) {}
}
