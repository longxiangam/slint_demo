
extern crate alloc;

use alloc::rc::Rc;
use core::cell::RefCell;
use embedded_graphics::image::{Image, ImageRaw, ImageRawLE};
use embedded_graphics::mono_font::ascii::FONT_6X9;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::primitives::{PrimitiveStyle, Triangle};
use embedded_graphics::text::Text;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::Drawable;
use embedded_graphics_core::geometry::Point;
use embedded_graphics_core::pixelcolor::{BinaryColor, Rgb565};
use embedded_graphics_core::prelude::RgbColor;
use fugit::HertzU32;

use hal::entry;
use hal::systimer::SystemTimer;
use slint::platform::Platform;
use slint::platform::software_renderer::{MinimalSoftwareWindow, PremultipliedRgbaColor, TargetPixel};

slint::include_modules!();

use esp_backtrace as _;
use esp_println::println;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc, IO, gpio::{Gpio0, Input, PullDown}, Delay, esp_riscv_rt, interrupt, peripherals, riscv, gpio};
use fugit::RateExtU32;
use slint::platform::WindowEvent;
use hal::gpio::{Event, Gpio1, GpioPin, GpioProperties, InterruptStatusRegisterAccess, Unknown};
use hal::riscv::_export::critical_section;
use hal::riscv::_export::critical_section::Mutex;
use st7735_lcd::Orientation;
use crate::my_platform::MyPlatform;

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");

    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_counter(ui.get_counter() + 1);
    });
    ui
}
static BUTTON: Mutex<RefCell<Option<Gpio0<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));
static BUTTON1: Mutex<RefCell<Option<Gpio1<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn GPIO(context: &mut esp_riscv_rt::TrapFrame) {
    critical_section::with(|cs| {
        println!("GPIO interrupt");
        println!("{:?}",context);

        let gpio0_status = <Gpio0<Input<PullDown>>  as GpioProperties>::InterruptStatus::app_cpu_interrupt_status_read();
        println!("GPIO0 status{}",gpio0_status);


        let  button_is_high =  BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_input_high();

        let  button1_is_high =  BUTTON1
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_input_high();

        if(gpio0_status == 1){
            println!("按钮0 按下");
        }

        if(gpio0_status == 2){
            println!("按钮1 按下");
        }
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();

        BUTTON1
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();

    });
}

#[cfg(feature = "lcd")]
pub(crate) fn run() -> !{


    // -------- Setup Allocator --------
    const HEAP_SIZE: usize = 300 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    #[global_allocator]
    static ALLOCATOR: embedded_alloc::Heap = embedded_alloc::Heap::empty();
    unsafe { ALLOCATOR.init(&mut HEAP as *const u8 as usize, core::mem::size_of_val(&HEAP)) };


    println!("进入系统 !");
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

    //按钮事件
    // Set GPIO0 as an input
    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);
    let mut button = io.pins.gpio0.into_pull_down_input();
    button.listen(Event::FallingEdge);

    critical_section::with(|cs| BUTTON.borrow_ref_mut(cs).replace(button));

    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();

    let mut button1 = io.pins.gpio1.into_pull_down_input();
    button1.listen(Event::FallingEdge);

    critical_section::with(|cs| BUTTON1.borrow_ref_mut(cs).replace(button1));

    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();

    unsafe {
        riscv::interrupt::enable();
    }


    let mut delay = Delay::new(&clocks);



    let sclk = io.pins.gpio2;
    let miso = io.pins.gpio12;
    let mosi = io.pins.gpio3;
    let cs = io.pins.gpio7.into_push_pull_output();


    let mut spi = hal::spi::master::Spi::new(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        cs,
        32u32.MHz(),
        hal::spi::SpiMode::Mode0,
        &clocks,
    );
    let rst =io.pins.gpio10.into_push_pull_output();
    let dc = io.pins.gpio6.into_push_pull_output();

    let rgb = true;
    let inverted = false;
    const WIDTH:u32 = 128;
    const HEIGHT:u32 = 160;

    let mut display = st7735_lcd::ST7735::new(spi, dc, rst, rgb, inverted, WIDTH, HEIGHT);
    display.init(&mut delay).unwrap();
    display.clear( Rgb565::BLACK).unwrap();
    display
        .set_orientation(&Orientation::Portrait)
        .unwrap();
    display.set_offset(0, 0);

    let mut buffer = [slint::platform::software_renderer::Rgb565Pixel(0);(WIDTH * HEIGHT) as usize];

    /* let image_raw: ImageRawLE<Rgb565> =
         ImageRaw::new(include_bytes!("../assets/ferris.raw"), 86);
     let image = Image::new(&image_raw, Point::new(26, 8));
     image.draw(&mut display).unwrap();*/
    /*  let thin_stroke = PrimitiveStyle::with_stroke(Rgb565::WHITE, 1);
      Triangle::new(
          Point::new(16, 16 + yoffset),
          Point::new(16 + 16, 16 + yoffset),
          Point::new(16 + 8, yoffset),
      )
          .into_styled(thin_stroke)
          .draw(&mut display).expect("绘制失败");*/

    let character_style = MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE);

    let text = Text::new("Hello e-g", Point::new(10, 11), character_style);
    text.draw(&mut display).expect("");


    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default());
    slint::platform::set_platform(alloc::boxed::Box::new(MyPlatform {
        window: window.clone()
    })) .unwrap();

    let _ui = create_slint_app();
    let mut line = [slint::platform::software_renderer::Rgb565Pixel(0); 320];
    loop{
        slint::platform::update_timers_and_animations();

        window.draw_if_needed(|renderer| {
            use embedded_graphics_core::prelude::*;
            struct DisplayWrapper<'a, T>(
                &'a mut T,
                &'a mut [slint::platform::software_renderer::Rgb565Pixel],
            );
            impl<T: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>
            slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, T>
            {
                type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
                fn process_line(
                    &mut self,
                    line: usize,
                    range: core::ops::Range<usize>,
                    render_fn: impl FnOnce(&mut [Self::TargetPixel]),
                ) {
                    let rect = embedded_graphics_core::primitives::Rectangle::new(
                        Point::new(range.start as _, line as _),
                        Size::new(range.len() as _, 1),
                    );
                    render_fn(&mut self.1[range.clone()]);
                    // NOTE! this is not an efficient way to send pixel to the screen, but it is kept simple on this template.
                    // It would be much faster to use the DMA to send pixel in parallel.
                    // See the example in https://github.com/slint-ui/slint/blob/master/examples/mcu-board-support/pico_st7789.rs
                    self.0
                        .fill_contiguous(
                            &rect,
                            self.1[range.clone()].iter().map(|p| {
                                embedded_graphics_core::pixelcolor::raw::RawU16::new(p.0).into()
                            }),
                        )
                        .map_err(drop)
                        .unwrap();
                }
            }
            //一行行的绘制，ram占用较小，速度慢
            //renderer.render_by_line(DisplayWrapper(&mut display, &mut line));
            //一次性绘制，ram占用较大，速度快
            renderer.render(&mut  buffer,WIDTH as usize);
            println!("renderer",);

            display.set_pixels_buffered(0,
                                        0,
                                        WIDTH as u16 - 1,
                                        HEIGHT as u16 - 1,buffer.iter().map(|p|{
                    p.0
                })).unwrap();

        });
    }
}

