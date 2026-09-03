mod bindings {
    include!(concat!(env!("OUT_DIR"), "/mod.rs"));
}

pub use bindings::*;
pub use mavlink_core as mavlink;

#[cfg(feature = "reflection")]
/// This module contains all the message definitions for the MAVLink dialects enabled.
/// This can be useful for reflection and parsing of definition messages.
pub mod reflection {
    use paste::paste;
    macro_rules! include_message_definitions {
        ($name:ident, $feature:literal) => {
            #[cfg(feature = $feature)]
            paste! {
                pub static [<$name _MAVLINK_PROFILE_SERIALIZED>]: &'static str = include_str!(concat!(env!("OUT_DIR"), "/", $feature, "_profile.json"));
            }
        };
    }

    include_message_definitions!(ORION, "orion");
    include_message_definitions!(LYRA, "lyra");
    include_message_definitions!(GEMINI, "gemini");
    include_message_definitions!(PYXIS, "pyxis");
    include_message_definitions!(LYNX, "lynx");
    include_message_definitions!(HERMES, "hermes");
    include_message_definitions!(R2A, "r2a");
    include_message_definitions!(TEST, "test");
}
