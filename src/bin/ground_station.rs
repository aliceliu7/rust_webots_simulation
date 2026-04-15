//! Ground station: receives data from both drones and performs sensor fusion 
//! 1. receives thermal data (port 5001) and bounding box data (port 5002) from drones.
//! 2. performs sensor fusion to detect if a wildfire is present 

use fire_detection::*;
use std::collections::HashMap;
use std::io::Read;
use std::net::{TcpListener, TcpStream};

fn main() {
    // Initialize Webots robot
    let robot = Robot::new();
    
    println!();
    println!("Ground Station");
    println!("Name: {}", robot.get_name());
    println!("Time step: {}ms", robot.time_step());
    println!("Thermal port: {}", THERMAL_PORT);
    println!("BBox port: {}", BBOX_PORT);
    println!("Fire threshold: {}C", FIRE_TEMP_THRESHOLD);
    println!("Fusion window: {}ms", FUSION_TIME_WINDOW_MS);
    println!("Waiting for drone connections");
    println!();

    // start TCP servers
    let thermal_server = match TcpListener::bind(format!("0.0.0.0:{}", THERMAL_PORT)) {
        Ok(l) => {
            l.set_nonblocking(true).ok();
            println!("Thermal server listening on port {}", THERMAL_PORT);
            Some(l)
        }
        Err(e) => {
            println!("Failed to start thermal server: {}", e);
            None
        }
    };

    let bbox_server = match TcpListener::bind(format!("0.0.0.0:{}", BBOX_PORT)) {
        Ok(l) => {
            l.set_nonblocking(true).ok();
            println!("BBox server listening on port {}", BBOX_PORT);
            Some(l)
        }
        Err(e) => {
            println!("[Failed to start bbox server: {}", e);
            None
        }
    };

    // client connections
    let mut thermal_client: Option<TcpStream> = None;
    let mut bbox_client: Option<TcpStream> = None;
    let mut thermal_connected = false;
    let mut bbox_connected = false;

    // receive buffers
    let mut thermal_buffer: Vec<u8> = Vec::with_capacity(8192);
    let mut bbox_buffer: Vec<u8> = Vec::with_capacity(4096);

    // statistics for latency 
    let mut thermal_count: u32 = 0;
    let mut bbox_count: u32 = 0;
    let mut thermal_stats = LatencyStats::new();
    let mut bbox_stats = LatencyStats::new();

    // sensor fusion
    let mut fusion = SensorFusion::new();

    // timing
    let mut last_stats_time: f64 = 0.0;

    // main simulation loop
    while robot.step() {
        let current_time = robot.get_time();

        // accept thermal connection
        if !thermal_connected {
            if let Some(ref server) = thermal_server {
                if let Ok((stream, addr)) = server.accept() {
                    stream.set_nonblocking(true).ok();
                    stream.set_nodelay(true).ok();
                    thermal_client = Some(stream);
                    thermal_connected = true;
                    println!("Thermal drone connected from {}", addr);
                }
            }
        }

        // accept bbox connection
        if !bbox_connected {
            if let Some(ref server) = bbox_server {
                if let Ok((stream, addr)) = server.accept() {
                    stream.set_nonblocking(true).ok();
                    stream.set_nodelay(true).ok();
                    bbox_client = Some(stream);
                    bbox_connected = true;
                    println!("BBox drone connected from {}", addr);
                }
            }
        }

        // receive thermal data
        if thermal_connected {
            if let Some(ref mut client) = thermal_client {
                let mut temp_buf = [0u8; 4096];
                match client.read(&mut temp_buf) {
                    Ok(0) => {
                        println!("Thermal drone disconnected");
                        thermal_connected = false;
                        thermal_client = None;
                        thermal_buffer.clear();
                    }
                    Ok(n) => {
                        thermal_buffer.extend_from_slice(&temp_buf[..n]);

                        // process complete messages
                        const MSG_SIZE: usize = 21 + THERMAL_SIZE * 4;
                        while thermal_buffer.len() >= MSG_SIZE {
                            let message: Vec<u8> = thermal_buffer.drain(..MSG_SIZE).collect();
                            match decode_thermal_message(&message) {
                                Ok(data) => {
                                    let latency = calculate_latency_ms(data.timestamp_ms);
                                    thermal_stats.add(latency);
                                    thermal_count += 1;

                                    let max_temp = data.max_temp();
                                    let indicator = if data.has_fire() { "Fire" } else { "N/A" };

                                    println!(
                                        "seq={}, latency={}ms, max={:.1}C {}",
                                        data.sequence, latency, max_temp, indicator
                                    );

                                    if let Some(fire) = fusion.update_thermal(data) {
                                        print_confirmed_fire(&fire);
                                    }
                                }
                                Err(e) => {
                                    println!("Thermal decode error: {}", e);
                                }
                            }
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        // No data available
                    }
                    Err(e) => {
                        println!("[Thermal read error: {}", e);
                        thermal_connected = false;
                        thermal_client = None;
                    }
                }
            }
        }

        // receive bbox data
        if bbox_connected {
            if let Some(ref mut client) = bbox_client {
                let mut temp_buf = [0u8; 4096];
                match client.read(&mut temp_buf) {
                    Ok(0) => {
                        println!("BBox drone disconnected");
                        bbox_connected = false;
                        bbox_client = None;
                        bbox_buffer.clear();
                    }
                    Ok(n) => {
                        bbox_buffer.extend_from_slice(&temp_buf[..n]);

                        // process complete messages
                        while bbox_buffer.len() >= 29 {
                            let json_len = u32::from_be_bytes([
                                bbox_buffer[25],
                                bbox_buffer[26],
                                bbox_buffer[27],
                                bbox_buffer[28],
                            ]) as usize;

                            let total_size = 29 + json_len;
                            if bbox_buffer.len() < total_size {
                                break;
                            }

                            let message: Vec<u8> = bbox_buffer.drain(..total_size).collect();
                            match decode_bbox_message(&message) {
                                Ok(data) => {
                                    let latency = calculate_latency_ms(data.timestamp_ms);
                                    bbox_stats.add(latency);
                                    bbox_count += 1;

                                    let mut label_counts: HashMap<&str, usize> = HashMap::new();
                                    for det in &data.detections {
                                        *label_counts.entry(det.label.as_str()).or_insert(0) += 1;
                                    }

                                    let summary: String = if label_counts.is_empty() {
                                        "none".to_string()
                                    } else {
                                        label_counts
                                            .iter()
                                            .map(|(k, v)| format!("{}:{}", k, v))
                                            .collect::<Vec<_>>()
                                            .join(", ")
                                    };

                                    println!(
                                        "seq={}, latency={}ms, detections: {}",
                                        data.sequence, latency, summary
                                    );

                                    if let Some(fire) = fusion.update_bbox(data) {
                                        print_confirmed_fire(&fire);
                                    }
                                }
                                Err(e) => {
                                    println!("[BBox decode error: {}", e);
                                }
                            }
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        // no data available
                    }
                    Err(e) => {
                        println!("BBox read error: {}", e);
                        bbox_connected = false;
                        bbox_client = None;
                    }
                }
            }
        }

        // print statistics every 15 seconds
        if current_time - last_stats_time >= 15.0 {
            if thermal_count > 0 || bbox_count > 0 {
                print_statistics(thermal_count, bbox_count, &thermal_stats, &bbox_stats, &fusion);
            }
            last_stats_time = current_time;
        }
    }
}

fn print_confirmed_fire(fire: &ConfirmedFire) {
    println!("Confirmed Wildfire");
    println!("Fusion ID: {}", fire.fusion_id);
    println!(
        "Thermal: {:.1}C at grid ({}, {})",
        fire.thermal_max_temp, fire.thermal_hotspot.0, fire.thermal_hotspot.1
    );
    println!(
        "Visual: BBox at ({}, {}) size {}x{}",
        fire.bbox.x, fire.bbox.y, fire.bbox.width, fire.bbox.height
    );
    println!("Visual confidence: {:.0}%", fire.bbox.confidence * 100.0);
    println!(
        "  World coords: ({:.1}, {:.1})",
        fire.bbox.world_x, fire.bbox.world_z
    );
    println!("Fusion confidence: {} (thermal + visual)", fire.confidence);
}

fn print_statistics(
    thermal_count: u32,
    bbox_count: u32,
    thermal_stats: &LatencyStats,
    bbox_stats: &LatencyStats,
    fusion: &SensorFusion,
) {
    println!("Ground Station Statistics)");
    println!("Thermal messages: {}", thermal_count);
    println!("BBox messages: {}", bbox_count);
    println!("Sensor fusions: {}", fusion.fusion_count);
    println!("Confirmed fires: {}", fusion.confirmed_fires.len());

    if let (Some(min), Some(avg), Some(max), Some(p95)) = (
        thermal_stats.min(),
        thermal_stats.avg(),
        thermal_stats.max(),
        thermal_stats.p95(),
    ) {
        println!("Thermal Latency (ms):");
        println!(" Min: {}, Avg: {:.1}, Max: {}, P95: {}", min, avg, max, p95);
    }

    if let (Some(min), Some(avg), Some(max), Some(p95)) = (
        bbox_stats.min(),
        bbox_stats.avg(),
        bbox_stats.max(),
        bbox_stats.p95(),
    ) {
        println!();
        println!("BBox Latency (ms):");
        println!("Min: {}, Avg: {:.1}, Max: {}, P95: {}", min, avg, max, p95);
    }
    println!();
}
