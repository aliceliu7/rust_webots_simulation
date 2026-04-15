//! Build script to find and link the Webots Controller library

use std::env;
use std::path::PathBuf;

fn main() {
    // get WEBOTS_HOME from environment
    let webots_home = env::var("WEBOTS_HOME").unwrap_or_else(|_| {
        // yuse common locations
        let candidates = [
            "/Applications/Webots.app/Contents",  // macOS
            "/usr/local/webots",                
            "/opt/webots",                     
            "C:\\Program Files\\Webots",       
        ];
        
        for candidate in &candidates {
            let path = PathBuf::from(candidate);
            if path.join("lib").join("controller").exists() {
                return candidate.to_string();
            }
        }
        
        panic!(
            "WEBOTS_HOME not set and Webots not found in standard locations.\n\
             Please set WEBOTS_HOME environment variable."
        );
    });

    let webots_path = PathBuf::from(&webots_home);
    
    // find library directory
    let lib_dir = webots_path.join("lib").join("controller");
    if !lib_dir.exists() {
        panic!(
            "Webots controller library not found at {:?}\n\
             WEBOTS_HOME={}\n\
             Expected: {}/lib/controller/",
            lib_dir, webots_home, webots_home
        );
    }

    // tell cargo to link the Controller library
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=dylib=Controller");
    
    // set rpath for runtime library loading 
    #[cfg(target_os = "macos")]
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
    
    #[cfg(target_os = "linux")]
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());

    // tell cargo to re-run if WEBOTS_HOME changes
    println!("cargo:rerun-if-env-changed=WEBOTS_HOME");
    
    // output the path for debugging
    eprintln!("Using WEBOTS_HOME: {}", webots_home);
    eprintln!("Library directory: {}", lib_dir.display());
}
