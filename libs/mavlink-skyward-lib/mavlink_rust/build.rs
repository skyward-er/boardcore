#![recursion_limit = "256"]

use std::env;

#[cfg(feature = "reflection")]
use std::{collections::HashSet, path::Path};

const DEFINITIONS_DIR: &str = "../message_definitions";

pub fn main() {
    let out_path = env::var("OUT_DIR").unwrap();

    let result = mavlink_bindgen::generate(DEFINITIONS_DIR, out_path)
        .expect("Failed to generate Rust MAVLink bindings");

    #[cfg(feature = "reflection")]
    serialize_message_definitions();

    mavlink_bindgen::format_generated_code(&result);
    mavlink_bindgen::emit_cargo_build_messages(&result);
}

// Parse again definition files and serialize the result to a file
#[cfg(feature = "reflection")]
fn serialize_message_definitions() {
    let out_path = env::var("OUT_DIR").unwrap();

    // iter through files in DEFINITIONS_DIR
    for entry in std::fs::read_dir(DEFINITIONS_DIR).unwrap() {
        let entry = entry.unwrap();
        let path = entry.path();
        if path.is_file() {
            let file_name = path.file_name().unwrap();
            let mut parsed_files = HashSet::new();
            let profile = mavlink_bindgen::parser::parse_profile(
                Path::new(&DEFINITIONS_DIR),
                Path::new(file_name),
                &mut parsed_files,
            );
            if let Ok(profile) = profile {
                let serialized = serde_json::to_string(&profile).unwrap();
                let out_file = format!(
                    "{}/{}_profile.json",
                    out_path,
                    path.file_stem().unwrap().to_str().unwrap()
                );
                std::fs::write(out_file, serialized).unwrap();
            }
        }
    }
}
