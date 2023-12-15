use embedded_graphics_core::pixelcolor::BinaryColor;
use embedded_graphics::{
    pixelcolor::BinaryColor::On as Black,
    pixelcolor::BinaryColor::{Off as White},

};
use epd_waveshare::{epd2in9::*, graphics::DisplayRotation, prelude::*};
use epd_waveshare::epd2in9::{Display2in9, Epd2in9, HEIGHT, WIDTH};
use epd_waveshare::prelude::{Display, WaveshareDisplay};
use esp_println::println;
use esp_println::print;
use fugit::RateExtU32;
use hal::clock::ClockControl;
use hal::{Delay, entry, IO, Rtc};
use hal::peripherals::Peripherals;
use hal::prelude::_esp_hal_system_SystemExt;
use hal::timer::TimerGroup;
use slint::platform::software_renderer::{PremultipliedRgbaColor, TargetPixel};
use crate::my_platform::MyPlatform;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics::mono_font::ascii::FONT_6X9;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::{ MonoTextStyleBuilder};
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text, TextStyleBuilder};
#[cfg(feature = "epd")]
pub(crate) fn run() -> !{

    // -------- Setup Allocator --------
    const HEAP_SIZE: usize = 300 * 1024;
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

    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);


    //墨水屏
    let epd_sclk = io.pins.gpio2;
    let epd_mosi = io.pins.gpio3;
    let epd_cs = io.pins.gpio7.into_push_pull_output();
    let epd_rst =io.pins.gpio10.into_push_pull_output();
    let epd_dc = io.pins.gpio6.into_push_pull_output();
    let mut spi = hal::spi::master::Spi::new_no_cs_no_miso(
        peripherals.SPI2,
        epd_sclk,
        epd_mosi,
        32u32.MHz(),
        hal::spi::SpiMode::Mode0,
        &clocks,
    );
    let busy_in = io.pins.gpio11.into_pull_up_input();

    let mut epd = Epd2in9::new(&mut spi, epd_cs, busy_in, epd_dc, epd_rst, &mut delay).unwrap();

    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default());
    slint::platform::set_platform(alloc::boxed::Box::new(MyPlatform {
        window: window.clone()
    })) .unwrap();

    let _ui = crate::create_slint_app();

    const DISPLAY_WIDTH:usize = 128;
    const DISPLAY_HEIGHT:usize = 296;

    let mut buffer = [WBPixel(BinaryColor::Off); DISPLAY_WIDTH * DISPLAY_HEIGHT];

    loop{
        slint::platform::update_timers_and_animations();

        window.draw_if_needed(|renderer| {
            use embedded_graphics_core::prelude::*;
            renderer.render(&mut buffer, DISPLAY_WIDTH );

            let mut display = Display2in9::default();
            println!("Drawing rotated text...");
            display.set_rotation(DisplayRotation::Rotate0);



            let mut x = 0;
            let mut y = 0;
            let mut color  = BinaryColor::On;
            for pixel in buffer {

                display.draw_helper(DISPLAY_WIDTH as u32,DISPLAY_HEIGHT as u32, Pixel(Point{x,y }, pixel.0) );

                x+= 1;
                if(x == DISPLAY_WIDTH as i32){
                    x = 0;
                    y += 1;
                }
                if(y > 20){
                    color = BinaryColor::Off;
                }
            }
            //draw_text(&mut display,"abcdef",10,10);

            let _= epd.clear_frame(&mut spi, &mut delay);

            let _= epd.update_frame(&mut spi, &display.buffer(), &mut delay);
            let _= epd.display_frame(&mut spi, &mut delay);
            let _= epd.sleep(&mut spi, &mut delay);
        });
    }
}
pub fn draw_text(display: &mut Display2in9, text: &str, x: i32, y: i32) {

    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::iso_8859_16::FONT_10X20)
        .text_color(Black)
        .background_color(White)
        .build();

    let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();

    let _ = Text::with_text_style(text, Point::new(x, y), style, text_style).draw(display);

}



#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct WBPixel(pub BinaryColor);

impl WBPixel {

}

impl TargetPixel for WBPixel {
    fn blend(&mut self, color: PremultipliedRgbaColor) {
        let a = (u8::MAX - color.alpha) as u32;
        // convert to 5 bits
        let a = (a + 4) >> 3;


        let  gray = ( (color.red as u32*299 + color.green as u32 *587 + color.blue as u32 *114 + 500) / 1000 ) as u32;
        let gray_u8 = gray as u8;
        if gray_u8 >= 255 / 2 {
            self.0 = BinaryColor::Off;
        }else {
            self.0 = BinaryColor::On;
        }
      /*  print!("r:{},g:{},b:{},wb:{}",color.red,color.green,color.blue,self.0.is_off());*/
    }

    fn from_rgb(r: u8, g: u8, b: u8) -> Self {

        let  gray = ( (r as u32*299 +g as u32 *587 + b as u32 *114 + 500) / 1000 ) as u32;
        let gray_u8 = gray as u8;
        let  wb =  gray_u8 / 1 << 7;
        if wb >= 1 {
            WBPixel(BinaryColor::Off)
        }else {
            WBPixel( BinaryColor::On)
        }
    }
}
