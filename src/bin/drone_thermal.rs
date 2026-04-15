//! Drone 1: Sends Thermal Readings to GS 
//! 1. transmits 32x24 temperature array via TCP on port 5001.
//! 2. sends to ground station

use fire_detection::*;
use rand_distr::{Distribution, Normal};
use std::io::Write;
use std::net::TcpStream;
use std::time::Duration;

const GROUND_STATION_HOST: &str = "127.0.0.1";

// drone position (stationary)
const DRONE_X: f64 = -5.0;
const DRONE_Z: f64 = 0.0;
const DRONE_ALTITUDE: f64 = 5.0;

fn main() {
    // initialize Webots robot
    let robot = Robot::new();
    
    println!("Drone 2: Sends Thermal Data");
    println!("Robot name: {}", robot.get_name());
    println!("Time step: {}ms", robot.time_step());
    println!("Thermal grid: {}x{}", THERMAL_WIDTH, THERMAL_HEIGHT);
    println!("Send interval: {}ms", SEND_INTERVAL_MS);
    println!("Target: ground station port {}", THERMAL_PORT);
    println!("Waiting for ground station...");
    println!();

    let mut socket: Option<TcpStream> = None;
    let mut connected = false;
    let mut last_send_time: f64 = 0.0;
    let mut last_connect_attempt: f64 = 0.0;
    let mut sequence: u32 = 0;

    // main simulation loop
    while robot.step() {
        let current_time = robot.get_time_ms();

        // connect if needed
        if !connected {
            if current_time - last_connect_attempt >= 2000.0 {
                match TcpStream::connect_timeout(
                    &format!("{}:{}", GROUND_STATION_HOST, THERMAL_PORT).parse().unwrap(),
                    Duration::from_secs(1),
                ) {
                    Ok(stream) => {
                        stream.set_nodelay(true).ok();
                        socket = Some(stream);
                        connected = true;
                        println!("Connected to ground station at port {}", THERMAL_PORT);
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
            let temps = generate_thermal_data();

            let data = ThermalData {
                sequence,
                timestamp_ms: current_timestamp_ms(),
                width: THERMAL_WIDTH as u32,
                height: THERMAL_HEIGHT as u32,
                temperatures: temps,
            };

            let max_temp = data.max_temp();
            let indicator = if data.has_fire() { "[Fire]" } else { "[N/A]" };

            match encode_thermal_message(&data) {
                Ok(message) => {
                    if let Some(ref mut stream) = socket {
                        match stream.write_all(&message) {
                            Ok(_) => {
                                println!(
                                    "Sent seq={}, max={:.1}C {}",
                                    sequence, max_temp, indicator
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

fn generate_thermal_data() -> Vec<f32> {
    let mut rng = rand::thread_rng();
    let normal = Normal::new(22.0, 1.5).unwrap();

    // base ambient temperature with noise
    let mut temps: Vec<f32> = (0..THERMAL_SIZE)
        .map(|_| normal.sample(&mut rng) as f32)
        .collect();

    // field of view based on altitude
    let fov_size = DRONE_ALTITUDE * 2.5;

    // add fire heat signatures
    for &(fire_x, fire_z) in &FIRE_LOCATIONS {
        let dist_to_fire = ((DRONE_X - fire_x).powi(2) + (DRONE_Z - fire_z).powi(2)).sqrt();

        if dist_to_fire < 15.0 {
            let rel_x = fire_x - DRONE_X;
            let rel_z = fire_z - DRONE_Z;

            let grid_x = ((rel_x / fov_size + 0.5) * THERMAL_WIDTH as f64) as i32;
            let grid_y = ((rel_z / fov_size + 0.5) * THERMAL_HEIGHT as f64) as i32;

            for dy in -5..=5 {
                for dx in -5..=5 {
                    let gx = grid_x + dx;
                    let gy = grid_y + dy;

                    if gx >= 0 && gx < THERMAL_WIDTH as i32 && gy >= 0 && gy < THERMAL_HEIGHT as i32 {
                        let pixel_dist = ((dx * dx + dy * dy) as f64).sqrt();
                        let heat = 350.0 * (-pixel_dist / 2.5).exp() * (-dist_to_fire / 12.0).exp();

                        let idx = gy as usize * THERMAL_WIDTH + gx as usize;
                        temps[idx] = temps[idx].max(22.0 + heat as f32);
                    }
                }
            }
        }
    }

    temps
}
