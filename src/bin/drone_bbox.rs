//! Drone 2: Sends Bounding Boxes to GS  
//! 1. generates simulated bounding box detections and sends to ground station.
//! 2. transmits bounding boxes via TCP on port 5002.

use fire_detection::*;
use rand::Rng;
use std::io::Write;
use std::net::TcpStream;
use std::time::Duration;

const GROUND_STATION_HOST: &str = "127.0.0.1";

// drone position (currently stationary)
const DRONE_X: f64 = 5.0;
const DRONE_Z: f64 = 0.0;
const FOV_HORIZONTAL: f64 = 12.0;
const FOV_VERTICAL: f64 = 9.0;

fn main() {
    // Initialize Webots robot
    let robot = Robot::new();
    
    println!("Drone 2: Sends Bounding Boxes");
    println!("Name: {}", robot.get_name());
    println!("Time step: {}ms", robot.time_step());
    println!("Image size: {}x{} (simulated)", IMAGE_WIDTH, IMAGE_HEIGHT);
    println!("Send interval: {}ms", SEND_INTERVAL_MS);
    println!("Target: ground station port {}", BBOX_PORT);
    println!("Waiting for ground station");
    println!();

    let mut socket: Option<TcpStream> = None;
    let mut connected = false;
    let mut last_send_time: f64 = 0.0;
    let mut last_connect_attempt: f64 = 0.0;
    let mut sequence: u32 = 0;

    // main simulation loop
    while robot.step() {
        let current_time = robot.get_time_ms();

        // connect to gs 
        if !connected {
            if current_time - last_connect_attempt >= 2000.0 {
                match TcpStream::connect_timeout(
                    &format!("{}:{}", GROUND_STATION_HOST, BBOX_PORT).parse().unwrap(),
                    Duration::from_secs(1),
                ) {
                    Ok(stream) => {
                        stream.set_nodelay(true).ok();
                        socket = Some(stream);
                        connected = true;
                        println!("Connected to ground station at port {}", BBOX_PORT);
                    }
                    Err(_) => {
                        println!("Connection failed");
                    }
                }
                last_connect_attempt = current_time;
            }
            continue;
        }

        // send at interval
        if current_time - last_send_time >= SEND_INTERVAL_MS as f64 {
            sequence += 1;
            let detections = generate_bounding_boxes();

            let data = BBoxData {
                sequence,
                timestamp_ms: current_timestamp_ms(),
                image_width: IMAGE_WIDTH,
                image_height: IMAGE_HEIGHT,
                detections,
            };

            // build status message
            let fire_count = data.fire_detections().len();
            let other_count = data.detections.len() - fire_count;

            let mut status = String::new();
            if fire_count > 0 {
                status.push_str(&format!("[Fire:{}]", fire_count));
            }
            if other_count > 0 {
                if !status.is_empty() {
                    status.push(' ');
                }
                status.push_str(&format!("[Other:{}]", other_count));
            }
            if status.is_empty() {
                status = "no detections".to_string();
            }

            match encode_bbox_message(&data) {
                Ok(message) => {
                    if let Some(ref mut stream) = socket {
                        match stream.write_all(&message) {
                            Ok(_) => {
                                println!(
                                    "Sent seq={}, {} detections: {}",
                                    sequence,
                                    data.detections.len(),
                                    status
                                );
                            }
                            Err(e) => {
                                println!("Send error: {}. Reconnecting...", e);
                                connected = false;
                                socket = None;
                            }
                        }
                    }
                }
                Err(e) => {
                    println!("Encode error: {}", e);
                }
            }

            last_send_time = current_time;
        }
    }
}

fn generate_bounding_boxes() -> Vec<BoundingBox> {
    let mut rng = rand::thread_rng();
    let mut detections = Vec::new();

    for &(obj_x, obj_z, obj_type, obj_size) in &SIMULATED_OBJECTS {
        let rel_x = obj_x - DRONE_X;
        let rel_z = obj_z - DRONE_Z;

        if rel_x.abs() > FOV_HORIZONTAL / 2.0 || rel_z.abs() > FOV_VERTICAL / 2.0 {
            continue;
        }

        let distance = (rel_x.powi(2) + rel_z.powi(2)).sqrt();
        if distance > 15.0 {
            continue;
        }

        let img_x = ((rel_x / FOV_HORIZONTAL + 0.5) * IMAGE_WIDTH as f64) as i32;
        let img_y = ((rel_z / FOV_VERTICAL + 0.5) * IMAGE_HEIGHT as f64) as i32;

        let mut bbox_size = (obj_size * 100.0 / (distance + 1.0)) as i32;
        bbox_size = bbox_size.clamp(20, 150);

        let noise_x: i32 = rng.gen_range(-5..=5);
        let noise_y: i32 = rng.gen_range(-5..=5);
        let noise_size: i32 = rng.gen_range(-3..=3);

        let mut confidence = (1.0 / (1.0 + distance * 0.1)).min(0.95);
        confidence += rng.gen_range(-0.05..=0.05);
        confidence = confidence.clamp(0.5, 0.99);

        let mut x = img_x - bbox_size / 2 + noise_x;
        let mut y = img_y - bbox_size / 2 + noise_y;
        let width = bbox_size + noise_size;
        let height = bbox_size + noise_size;

        x = x.clamp(0, IMAGE_WIDTH as i32 - width);
        y = y.clamp(0, IMAGE_HEIGHT as i32 - height);

        detections.push(BoundingBox {
            label: obj_type.to_string(),
            confidence,
            x,
            y,
            width,
            height,
            world_x: obj_x,
            world_z: obj_z,
        });
    }

    detections
}
