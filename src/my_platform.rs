use alloc::rc::Rc;
use hal::systimer::SystemTimer;
use slint::platform::Platform;
use slint::platform::software_renderer::MinimalSoftwareWindow;

pub struct MyPlatform  {
    pub(crate) window: Rc<MinimalSoftwareWindow>,

}

impl Platform  for MyPlatform {
    fn create_window_adapter(&self) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        // Since on MCUs, there can be only one window, just return a clone of self.window.
        // We'll also use the same window in the event loop.
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {

        core::time::Duration::from_micros(SystemTimer::now() )
    }
    // optional: You can put the event loop there, or in the main function, see later
    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        todo!();
    }
}