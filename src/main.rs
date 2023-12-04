#![cfg_attr(not(feature = "simulator"), no_std)]
#![cfg_attr(not(feature = "simulator"), no_main)]
extern crate alloc;

#[cfg(not(feature = "simulator"))]
use hal::entry;

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
#[cfg(not(feature = "simulator"))]
#[entry]
fn main() -> !{
    use esp_backtrace as _;
    use esp_println::println;
    use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc, IO, gpio::{Gpio0, Input, PullDown}, Delay, esp_riscv_rt, interrupt, peripherals, riscv};
    use fugit::RateExtU32;
    use slint::platform::WindowEvent;
    use hal::gpio::Unknown;

    // -------- Setup Allocator --------
    const HEAP_SIZE: usize = 200 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    #[global_allocator]
    static ALLOCATOR: embedded_alloc::Heap = embedded_alloc::Heap::empty();
    unsafe { ALLOCATOR.init(&mut HEAP as *const u8 as usize, core::mem::size_of_val(&HEAP)) };


    println!("Hello, world!");
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    loop{

    }
}
