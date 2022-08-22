#![no_std]
#![no_main]

pub mod mybuf;

// For string formatting.
use core::fmt::Write;
use core::iter::repeat;

// The macro for our start-up function
use cortex_m_rt::entry;

// Time handling traits:
use embedded_time::duration::*;
use embedded_time::rate::Extensions;

// CountDown timer for the counter on the display:
use embedded_hal::timer::CountDown;
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use embedded_hal::digital::v2::InputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use hal::{pac, Clock};
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// The macro for marking our interrupt functions
use hal::pac::interrupt;

// For in the graphics drawing utilities like the font
// and the drawing routines:
use embedded_graphics::{
    image::Image,
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::*,
    text::{Baseline, Text},
};
use tinybmp::Bmp;

// use display_interface;
use display_interface_spi;
// The display driver:
use ssd1306::{prelude::*, Ssd1306};

// led drivers
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

// pio drivers used for ws2812
use hal::gpio::FunctionPio0;
use hal::pio::PIOExt;
use hal::Sio;

// usb drivers
use usb_device::{
    class_prelude::{UsbBusAllocator, UsbClass},
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
// multiple usb classes
use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::HIDClass,
};
use usbd_serial::SerialPort;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core: cortex_m::Peripherals = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    // unsafe {
    //     // Note (safety): This is safe as interrupts haven't been started yet
    //     USB_BUS = Some(usb_bus);
    // }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    // let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    let mut serial = SerialPort::new(&usb_bus);

    // Set up the USB HID Class Device driver, providing Keyboard Reports
    let mut usb_hid = HIDClass::new(&usb_bus, KeyboardReport::desc(), 60);
    // unsafe {
    //     // Note (safety): This is safe as interrupts haven't been started yet.
    //     USB_HID = Some(usb_hid);
    // }
    // Create a USB device with a fake VID and PID
    // Set up the USB Communications Class Device driver
    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27da))
        .manufacturer("Adam Mills")
        .product("(Cl|H)ack Pad")
        .serial_number("TEST")
        .device_class(0x00)
        .composite_with_iads()
        .build();
    // unsafe {
    //     // Note (safety): This is safe as interrupts haven't been started yet
    //     USB_DEVICE = Some(usb_dev);
    // }
    // ! interrupt code
    // unsafe {
    //     // Enable the USB interrupt
    //     pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    // };
    // ! interrupt code

    /* Configuring our pins */

    // Configure the addressable LEDs
    let led: hal::gpio::Pin<_, FunctionPio0> = pins.gpio3.into_mode();
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(led, &mut pio, sm0, 125_000_000.Hz(), timer.count_down());
    let mut wheel_pos: u8 = 128;

    // These are implicitly used by the spi driver if they are in the correct mode
    let _sck = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    let _tx = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
    let dc = pins.gpio16.into_push_pull_output();
    let cs = pins.gpio17.into_push_pull_output();

    let usb_test = pins.gpio27.into_pull_up_input();
    let right = pins.gpio2.into_pull_up_input();
    let center = pins.gpio8.into_pull_up_input();
    let left = pins.gpio14.into_pull_up_input();

    // rotary encoder pins
    let r_btn = pins.gpio20.into_pull_up_input();
    let r_cw = pins.gpio22.into_pull_up_input();
    let r_cc = pins.gpio21.into_pull_up_input();

    // Create an SPI driver instance for the SPI0 device
    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        125_000_000u32.Hz(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let interface = display_interface_spi::SPIInterface::new(spi, dc, cs);
    // Create a driver instance and initialize:
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // Create a text style for drawing the font:
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // counts of presses
    let mut btn = 0;
    let mut cc = 0;
    let mut cw = 0;

    // booleans for detecting presses
    let mut test_trigger: bool = false;
    let mut btn_trigger: bool = false;
    let mut cc_trigger: bool = false;
    let mut cw_trigger: bool = false;
    let mut rotary_turning: bool = false;
    let mut rotary_counted: bool = false;
    // true == cw  false == cc
    let mut dir: bool = false;

    // init display properties
    let up_bmp =
        Bmp::<BinaryColor>::from_slice(include_bytes!("../assets/bongo_up_inverted.bmp")).unwrap();
    let left_bmp =
        Bmp::<BinaryColor>::from_slice(include_bytes!("../assets/bongo_left_inverted.bmp"))
            .unwrap();
    let right_bmp =
        Bmp::<BinaryColor>::from_slice(include_bytes!("../assets/bongo_right_inverted.bmp"))
            .unwrap();
    let both_bmp =
        Bmp::<BinaryColor>::from_slice(include_bytes!("../assets/bongo_both_inverted.bmp"))
            .unwrap();

    #[derive(PartialEq)]
    enum bongo {
        Up,
        Left,
        Right,
        Both,
    }
    let bongo_up = Image::new(&up_bmp, Point::zero());
    let bongo_left = Image::new(&left_bmp, Point::zero());
    let bongo_right = Image::new(&right_bmp, Point::zero());
    let bongo_both = Image::new(&both_bmp, Point::zero());

    let display_rect = embedded_graphics::primitives::rectangle::Rectangle::new(
        Point::new(1, 1),
        Size::new(126, 62),
    )
    .into_styled(
        PrimitiveStyleBuilder::new()
            .stroke_width(2)
            .stroke_color(BinaryColor::On)
            .build(),
    );

    let mut position = bongo::Up;
    bongo_up.draw(&mut display).unwrap();
    display_rect.draw(&mut display).unwrap();

    display.flush();

    let mut buf = mybuf::FmtBuf::new();
    let mut buf_write = mybuf::FmtBuf::new();
    // let mut debug = FmtBuf::new();

    let mut btn_toggle: bool = false;
    let mut said_hello: bool = false;
    loop {
        delay.delay_ms(1);
        // reset display text buffer
        buf.reset();
        buf_write.reset();
        // debug.reset();
        // Empty the display:
        // display.clear();

        // led rainbow control
        let colours = repeat(wheel(wheel_pos)).take(12);
        let final_rgb = brightness(colours, 100);
        ws.write(final_rgb).unwrap();
        if (btn_toggle) {
            wheel_pos = wheel_pos.wrapping_add(1);
        }

        if (left.is_low().unwrap() && right.is_low().unwrap()) || center.is_low().unwrap() {
            if position != bongo::Both {
                position = bongo::Both;
                bongo_both.draw(&mut display).unwrap();
            }
        } else if left.is_low().unwrap() {
            if position != bongo::Left {
                position = bongo::Left;
                bongo_left.draw(&mut display).unwrap();
            }
        } else if right.is_low().unwrap() {
            if position != bongo::Right {
                position = bongo::Right;
                bongo_right.draw(&mut display).unwrap();
            }
        } else {
            if position != bongo::Up {
                position = bongo::Up;
                bongo_up.draw(&mut display).unwrap();
            }
        }
        // // rotary checks
        // } else if r_cc.is_low().unwrap() && r_cw.is_low().unwrap() {
        //     bongo_both.draw(&mut display).unwrap();
        // } else if r_cc.is_low().unwrap() {
        //     bongo_left.draw(&mut display).unwrap();
        // } else if r_cw.is_low().unwrap() {
        //     bongo_right.draw(&mut display).unwrap();
        // } else {
        //     bongo_up.draw(&mut display).unwrap();
        // }
        // Display the image
        display_rect.draw(&mut display).unwrap();
        display.flush();

        // rotary updating
        if !btn_trigger && r_btn.is_low().unwrap() {
            btn_trigger = true;
            btn += 1;
            btn_toggle = !btn_toggle;
        } else if !r_btn.is_low().unwrap() {
            btn_trigger = false;
        }

        // rotary encoder logic
        if !cw_trigger && r_cw.is_low().unwrap() {
            cw_trigger = true;
            // if this is first signal of the rotary encoder set direction
            if !rotary_turning {
                dir = true;
            }
            rotary_turning = true;
        } else if !r_cw.is_low().unwrap() {
            cw_trigger = false;
        }

        if !cc_trigger && r_cc.is_low().unwrap() {
            cc_trigger = true;
            // if this is first signal of the rotary encoder set direction
            if !rotary_turning {
                dir = false;
            }
            rotary_turning = true;
        } else if !r_cc.is_low().unwrap() {
            cc_trigger = false;
        }

        // if the turning hasn't been counted yet, and both signals are low (ie triggered) we count
        if !rotary_counted && cc_trigger && cw_trigger {
            rotary_counted = true;
            // true == cw  false == cc
            // match dir {
            //     true => cw += 1,
            //     false => cc += 1,
            // }
            match dir {
                true => wheel_pos = wheel_pos.wrapping_add(3),
                false => wheel_pos = wheel_pos.wrapping_sub(3),
            }
        // if both signals are high (ie not triggered) we may reset
        } else if rotary_turning && !cc_trigger && !cw_trigger {
            rotary_turning = false;
            rotary_counted = false;
            dir = false;
        }

        // * usb testing
        buf_write.reset();

        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 2_000_000 {
            said_hello = true;
            write!(buf_write, "(Cl|H)ack Pad connected!\n\n");
            let _ = serial.write(buf_write.as_bytes());
            buf_write.reset();
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial, &mut usb_hid]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                // WouldBlock - No bytes available for reading.
                Err(usb_device::UsbError::WouldBlock) => {}
                Ok(0) => {}
                Err(e) => {
                    // Do nothing
                    write!(&mut buf_write, "{:?} \n\r", e);
                    let mut wr_ptr = buf_write.as_bytes();
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                        // serial.write(&buf_write.as_bytes()).unwrap();
                    }
                    serial.flush();
                    buf_write.reset();
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            Err(_) => break,
                        };
                    }
                }
            }
        }

        // HID actions
        {
            let mut key_code: u8 = 0;
            if usb_test.is_low().unwrap() {
                key_code = 21;
                // write!(&mut buf_write, "usb_test low \n\r");
                // serial.write(&buf_write.as_bytes()).unwrap();
                // buf_write.reset();
            }
            let report = KeyboardReport {
                modifier: 0,
                reserved: 0,
                leds: 0,
                keycodes: [key_code, 0, 0, 0, 0, 0],
            };
            match usb_hid.push_input(&report) {
                Ok(s) => {}
                Err(e) => {}
            };
        }
        // * usb testing

        // Wait a bit:
        // delay.start(20.milliseconds());
        // let _ = nb::block!(delay.wait());
    }
}

// Convert a number from `0..=255` to an RGB color triplet.
//
// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}

// /// Submit a new keyboard report to the USB stack.
// ///
// /// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
// fn push_keyboard_event(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
//     cortex_m::interrupt::free(|_| unsafe {
//         // Now interrupts are disabled, grab the global variable and, if
//         // available, send it a HID report
//         USB_HID.as_mut().map(|hid| hid.push_input(&report))
//     })
//     .unwrap()
// }

// /// This function is called whenever the USB Hardware generates an Interrupt
// /// Request.
// #[allow(non_snake_case)]
// #[interrupt]
// unsafe fn USBCTRL_IRQ() {
//     // Handle USB request
//     let usb_dev = USB_DEVICE.as_mut().unwrap();
//     let usb_hid = USB_HID.as_mut().unwrap();
//     usb_dev.poll(&mut [usb_hid]);
// }
