//! Library with Native Webots Bindings
//! - direct FFI bindings to Webots C API with no external create usage 
//! - sensor fusion logic 

use byteorder::{BigEndian, LittleEndian, ReadBytesExt, WriteBytesExt};
use serde::{Deserialize, Serialize};
use std::ffi::CStr;
use std::io::{self, Cursor, Read, Write};
use std::os::raw::{c_char, c_double, c_int};
use std::time::{SystemTime, UNIX_EPOCH};

// Webots C API Bindings 

/// raw FFI bindings to Webots Controller library
pub mod webots_sys {
    use std::os::raw::{c_char, c_double, c_int};

    #[link(name = "Controller")]
    extern "C" {
        // functions 
        pub fn wb_robot_init();
        pub fn wb_robot_cleanup();
        pub fn wb_robot_step(time_step: c_int) -> c_int;
        pub fn wb_robot_get_time() -> c_double;
        pub fn wb_robot_get_basic_time_step() -> c_double;
        pub fn wb_robot_get_name() -> *const c_char;
    }
}

/// safe Webots wrapper 
pub struct Robot {
    time_step: i32,
}

impl Robot {
    /// initialize the robot controller
    pub fn new() -> Self {
        unsafe {
            webots_sys::wb_robot_init();
        }
        
        let time_step = unsafe {
            webots_sys::wb_robot_get_basic_time_step() as i32
        };
        
        Robot { time_step }
    }

    /// get the simulation time step in milliseconds
    pub fn time_step(&self) -> i32 {
        self.time_step
    }

    /// get current simulation time in seconds
    pub fn get_time(&self) -> f64 {
        unsafe { webots_sys::wb_robot_get_time() }
    }

    /// get current simulation time in milliseconds
    pub fn get_time_ms(&self) -> f64 {
        self.get_time() * 1000.0
    }

    /// get robot name
    pub fn get_name(&self) -> String {
        unsafe {
            let name_ptr = webots_sys::wb_robot_get_name();
            if name_ptr.is_null() {
                return String::new();
            }
            CStr::from_ptr(name_ptr)
                .to_string_lossy()
                .into_owned()
        }
    }

    /// step the simulation. Returns false if simulation is ending.
    pub fn step(&self) -> bool {
        let result = unsafe { webots_sys::wb_robot_step(self.time_step) };
        result != -1
    }

    /// step the simulation with custom time step
    pub fn step_ms(&self, ms: i32) -> bool {
        let result = unsafe { webots_sys::wb_robot_step(ms) };
        result != -1
    }
}

impl Drop for Robot {
    fn drop(&mut self) {
        unsafe {
            webots_sys::wb_robot_cleanup();
        }
    }
}

impl Default for Robot {
    fn default() -> Self {
        Self::new()
    }
}

//Cconfiguration 

pub const THERMAL_PORT: u16 = 5001;
pub const BBOX_PORT: u16 = 5002;

pub const THERMAL_WIDTH: usize = 32;
pub const THERMAL_HEIGHT: usize = 24;
pub const THERMAL_SIZE: usize = THERMAL_WIDTH * THERMAL_HEIGHT;

pub const IMAGE_WIDTH: u32 = 640;
pub const IMAGE_HEIGHT: u32 = 480;

pub const FIRE_TEMP_THRESHOLD: f32 = 100.0;
pub const FUSION_TIME_WINDOW_MS: i64 = 2000;

pub const SEND_INTERVAL_MS: u64 = 500;

pub const MSG_TYPE_THERMAL: u8 = 1;
pub const MSG_TYPE_BBOX: u8 = 2;

// Simulated fire locations
pub const FIRE_LOCATIONS: [(f64, f64); 2] = [(3.0, 4.0), (-2.0, 6.0)];

// Simulated objects for bbox detection
pub const SIMULATED_OBJECTS: [(f64, f64, &str, f64); 4] = [
    (3.0, 4.0, "FIRE", 1.0),
    (-2.0, 6.0, "FIRE", 0.8),
    (7.0, -3.0, "LANDMARK", 2.0),
    (-6.0, 5.0, "VEGETATION", 1.5),
];

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundingBox {
    pub label: String,
    pub confidence: f64,
    pub x: i32,
    pub y: i32,
    pub width: i32,
    pub height: i32,
    #[serde(default)]
    pub world_x: f64,
    #[serde(default)]
    pub world_z: f64,
}

#[derive(Debug, Clone)]
pub struct ThermalData {
    pub sequence: u32,
    pub timestamp_ms: i64,
    pub width: u32,
    pub height: u32,
    pub temperatures: Vec<f32>,
}

impl ThermalData {
    pub fn max_temp(&self) -> f32 {
        self.temperatures.iter().cloned().fold(f32::NEG_INFINITY, f32::max)
    }

    pub fn hotspot_position(&self) -> (usize, usize) {
        let mut max_idx = 0;
        let mut max_val = f32::NEG_INFINITY;
        for (i, &temp) in self.temperatures.iter().enumerate() {
            if temp > max_val {
                max_val = temp;
                max_idx = i;
            }
        }
        (max_idx % self.width as usize, max_idx / self.width as usize)
    }

    pub fn has_fire(&self) -> bool {
        self.max_temp() > FIRE_TEMP_THRESHOLD
    }
}

#[derive(Debug, Clone)]
pub struct BBoxData {
    pub sequence: u32,
    pub timestamp_ms: i64,
    pub image_width: u32,
    pub image_height: u32,
    pub detections: Vec<BoundingBox>,
}

impl BBoxData {
    pub fn fire_detections(&self) -> Vec<&BoundingBox> {
        self.detections.iter().filter(|d| d.label == "FIRE").collect()
    }

    pub fn has_fire(&self) -> bool {
        self.detections.iter().any(|d| d.label == "FIRE")
    }

    pub fn best_fire_detection(&self) -> Option<&BoundingBox> {
        self.detections
            .iter()
            .filter(|d| d.label == "FIRE")
            .max_by(|a, b| a.confidence.partial_cmp(&b.confidence).unwrap())
    }
}

#[derive(Debug, Clone)]
pub struct ConfirmedFire {
    pub fusion_id: u32,
    pub timestamp_ms: i64,
    pub thermal_max_temp: f32,
    pub thermal_hotspot: (usize, usize),
    pub bbox: BoundingBox,
    pub confidence: FusionConfidence,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FusionConfidence {
    High,
    Medium,
    Low,
}

impl std::fmt::Display for FusionConfidence {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FusionConfidence::High => write!(f, "HIGH"),
            FusionConfidence::Medium => write!(f, "MEDIUM"),
            FusionConfidence::Low => write!(f, "LOW"),
        }
    }
}

// Protocol Encoding 

pub fn encode_thermal_message(data: &ThermalData) -> io::Result<Vec<u8>> {
    let mut buffer = Vec::with_capacity(21 + data.temperatures.len() * 4);

    buffer.write_u8(MSG_TYPE_THERMAL)?;
    buffer.write_u32::<BigEndian>(data.sequence)?;
    buffer.write_i64::<BigEndian>(data.timestamp_ms)?;
    buffer.write_u32::<BigEndian>(data.width)?;
    buffer.write_u32::<BigEndian>(data.height)?;

    for &temp in &data.temperatures {
        buffer.write_f32::<LittleEndian>(temp)?;
    }

    Ok(buffer)
}

pub fn encode_bbox_message(data: &BBoxData) -> io::Result<Vec<u8>> {
    let json_bytes = serde_json::to_vec(&data.detections)?;

    let mut buffer = Vec::with_capacity(29 + json_bytes.len());

    buffer.write_u8(MSG_TYPE_BBOX)?;
    buffer.write_u32::<BigEndian>(data.sequence)?;
    buffer.write_i64::<BigEndian>(data.timestamp_ms)?;
    buffer.write_u32::<BigEndian>(data.image_width)?;
    buffer.write_u32::<BigEndian>(data.image_height)?;
    buffer.write_u32::<BigEndian>(data.detections.len() as u32)?;
    buffer.write_u32::<BigEndian>(json_bytes.len() as u32)?;
    buffer.write_all(&json_bytes)?;

    Ok(buffer)
}

// Protocol Decoding 

pub fn decode_thermal_message(data: &[u8]) -> io::Result<ThermalData> {
    let mut cursor = Cursor::new(data);

    let msg_type = cursor.read_u8()?;
    if msg_type != MSG_TYPE_THERMAL {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("Expected msg_type {}, got {}", MSG_TYPE_THERMAL, msg_type),
        ));
    }

    let sequence = cursor.read_u32::<BigEndian>()?;
    let timestamp_ms = cursor.read_i64::<BigEndian>()?;
    let width = cursor.read_u32::<BigEndian>()?;
    let height = cursor.read_u32::<BigEndian>()?;

    let num_temps = (width * height) as usize;
    let mut temperatures = Vec::with_capacity(num_temps);
    for _ in 0..num_temps {
        temperatures.push(cursor.read_f32::<LittleEndian>()?);
    }

    Ok(ThermalData {
        sequence,
        timestamp_ms,
        width,
        height,
        temperatures,
    })
}

pub fn decode_bbox_message(data: &[u8]) -> io::Result<BBoxData> {
    let mut cursor = Cursor::new(data);

    let msg_type = cursor.read_u8()?;
    if msg_type != MSG_TYPE_BBOX {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("Expected msg_type {}, got {}", MSG_TYPE_BBOX, msg_type),
        ));
    }

    let sequence = cursor.read_u32::<BigEndian>()?;
    let timestamp_ms = cursor.read_i64::<BigEndian>()?;
    let image_width = cursor.read_u32::<BigEndian>()?;
    let image_height = cursor.read_u32::<BigEndian>()?;
    let _num_detections = cursor.read_u32::<BigEndian>()?;
    let json_len = cursor.read_u32::<BigEndian>()?;

    let mut json_bytes = vec![0u8; json_len as usize];
    cursor.read_exact(&mut json_bytes)?;

    let detections: Vec<BoundingBox> = serde_json::from_slice(&json_bytes)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;

    Ok(BBoxData {
        sequence,
        timestamp_ms,
        image_width,
        image_height,
        detections,
    })
}

// Sensor Fusion 

pub struct SensorFusion {
    thermal_data: Option<ThermalData>,
    bbox_data: Option<BBoxData>,
    pub confirmed_fires: Vec<ConfirmedFire>,
    pub fusion_count: u32,
}

impl SensorFusion {
    pub fn new() -> Self {
        Self {
            thermal_data: None,
            bbox_data: None,
            confirmed_fires: Vec::new(),
            fusion_count: 0,
        }
    }

    pub fn update_thermal(&mut self, data: ThermalData) -> Option<ConfirmedFire> {
        self.thermal_data = Some(data);
        self.try_fusion()
    }

    pub fn update_bbox(&mut self, data: BBoxData) -> Option<ConfirmedFire> {
        self.bbox_data = Some(data);
        self.try_fusion()
    }

    fn try_fusion(&mut self) -> Option<ConfirmedFire> {
        let thermal = self.thermal_data.as_ref()?;
        let bbox = self.bbox_data.as_ref()?;

        let time_diff = (thermal.timestamp_ms - bbox.timestamp_ms).abs();
        if time_diff > FUSION_TIME_WINDOW_MS {
            return None;
        }

        if thermal.has_fire() && bbox.has_fire() {
            self.fusion_count += 1;

            let best_fire = bbox.best_fire_detection()?;
            let hotspot = thermal.hotspot_position();

            let confirmed = ConfirmedFire {
                fusion_id: self.fusion_count,
                timestamp_ms: thermal.timestamp_ms.max(bbox.timestamp_ms),
                thermal_max_temp: thermal.max_temp(),
                thermal_hotspot: hotspot,
                bbox: best_fire.clone(),
                confidence: FusionConfidence::High,
            };

            self.confirmed_fires.push(confirmed.clone());
            return Some(confirmed);
        }

        None
    }
}

impl Default for SensorFusion {
    fn default() -> Self {
        Self::new()
    }
}

/
pub fn current_timestamp_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_millis() as i64
}

pub fn calculate_latency_ms(timestamp_ms: i64) -> i64 {
    current_timestamp_ms() - timestamp_ms
}

// Print Statistics 

#[derive(Debug, Default, Clone)]
pub struct LatencyStats {
    values: Vec<i64>,
}

impl LatencyStats {
    pub fn new() -> Self {
        Self { values: Vec::new() }
    }

    pub fn add(&mut self, latency_ms: i64) {
        self.values.push(latency_ms);
    }

    pub fn count(&self) -> usize {
        self.values.len()
    }

    pub fn min(&self) -> Option<i64> {
        self.values.iter().copied().min()
    }

    pub fn max(&self) -> Option<i64> {
        self.values.iter().copied().max()
    }

    pub fn avg(&self) -> Option<f64> {
        if self.values.is_empty() {
            return None;
        }
        Some(self.values.iter().sum::<i64>() as f64 / self.values.len() as f64)
    }

    pub fn percentile(&self, p: f64) -> Option<i64> {
        if self.values.is_empty() {
            return None;
        }
        let mut sorted = self.values.clone();
        sorted.sort();
        let idx = ((p / 100.0) * (sorted.len() - 1) as f64).round() as usize;
        Some(sorted[idx.min(sorted.len() - 1)])
    }

    pub fn p95(&self) -> Option<i64> {
        self.percentile(95.0)
    }
}
