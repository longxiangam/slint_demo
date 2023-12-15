#![cfg_attr(not(feature = "simulator"), no_std)]
#![cfg_attr(not(feature = "simulator"), no_main)]

#[cfg(not(feature = "simulator"))]
mod lcd;
#[cfg(not(feature = "simulator"))]
mod my_platform;
#[cfg(not(feature = "simulator"))]
mod epd;

extern crate alloc;


#[cfg(not(feature = "simulator"))]
use hal::entry;
use slint::platform::Platform;
use slint::platform::software_renderer::{ TargetPixel};

slint::include_modules!();


fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");

    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_counter(ui.get_counter() + 1);
    });
    ui
}

#[cfg(feature = "simulator")]
fn main() -> Result<(), slint::PlatformError> {
    create_slint_app().run()
}
#[cfg(feature = "lcd")]
#[entry]
fn main() -> !{
    lcd::run()
}

#[cfg(feature = "epd")]
#[entry]
fn main() -> !{
    epd::run()
}
