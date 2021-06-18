//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]
#![feature(core_intrinsics)]
#![feature(alloc)]
#![feature(alloc_error_handler)]

extern crate panic_abort;

use core::convert::TryInto;
use core::fmt::Write;
use hal::gpio::gpioc::PC6;
use rtic::Mutex;

extern crate alloc;
use alloc::string::ToString;
use alloc::vec;
use core::alloc::Layout;

use alloc_cortex_m::CortexMHeap;
use cortex_m::asm;

// use core::intrinsics::powif32;
use nb::block;
use rtic::export::wfi;
use rtic::{
    app,
    cyccnt::{Instant, U32Ext},
};
use stm32f1::stm32f103::Interrupt;
use stm32f1xx_hal::{
    adc,
    gpio::{
        gpioa::{PA1, PA2, PA3, PA4},
        gpiob::{PB13, PB14, PB15, PB5, PB6, PB7, PB8},
        Output, PushPull
    },
    i2c::{BlockingI2c, DutyCycle},
    prelude::*,
    rtc::Rtc,
    serial::{Config, Rx, Serial, Tx},
    spi::{Mode as SpiMode, NoMiso, NoSck, Phase, Polarity, Spi, Spi2NoRemap},
    stm32,
    timer::{CountDownTimer, Event, Timer},
};

use stm32f1xx_hal as hal;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};

use bitbang_hal;

// used for encoding data to send over serial
use byteorder::{ByteOrder, LE};
// used to convert bytes to string
use core::str;

use ds1307::{Ds1307, NaiveDate, NaiveDateTime, NaiveTime, Rtcc, Timelike};
use max7219::*; // NaiveDateTime,

use lite_json::json::{JsonValue, NumberValue};
use lite_json::json_parser::parse_json;
use lite_json::Serialize;

// use crate::mappings;
use crate::mappings::{
    display_matrix_int, encode_char, encode_digit, encode_num, SingleDisplayData,
};
pub mod mappings;

enum DisplayKind {
    MinutesSeconds,
    HourMinutes,
    TestData,
}

type SpiType = hal::spi::Spi<
    stm32f1xx_hal::pac::SPI2,
    stm32f1xx_hal::spi::Spi2NoRemap,
    (
        hal::gpio::gpiob::PB13<hal::gpio::Alternate<hal::gpio::PushPull>>,
        hal::gpio::gpiob::PB14<hal::gpio::Input<hal::gpio::Floating>>,
        hal::gpio::gpiob::PB15<hal::gpio::Alternate<hal::gpio::PushPull>>,
    ),
>;

//     type SpiType = stm32f1xx_hal::spi::Spi<stm32f1xx_hal::pac::SPI2, stm32f1xx_hal::spi::Spi2NoRemap,
// (stm32f1xx_hal::gpio::gpiob::PB13<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>>,
//     stm32f1xx_hal::gpio::gpiob::PB14<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::Floating>>,
//     stm32f1xx_hal::gpio::gpiob::PB15<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>>)>;

type DisplayType = MAX7219<
    max7219::connectors::SpiConnectorSW<
        SpiType,
        stm32f1xx_hal::gpio::gpiob::PB12<
            stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>,
        >,
    >,
>;

type Ds1307Type = ds1307::Ds1307<
    bitbang_hal::i2c::I2cBB<
        stm32f1xx_hal::gpio::gpiob::PB6<
            stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>,
        >,
        stm32f1xx_hal::gpio::gpiob::PB7<
            stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>,
        >,
        stm32f1xx_hal::timer::CountDownTimer<stm32::TIM4>,
    >,
>;

type RtcStm = stm32f1xx_hal::rtc::Rtc;

// Output GPIO for switch external devices:
type GpioType01 = stm32f1xx_hal::gpio::gpioc::PC6<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>;
type GpioType02 = stm32f1xx_hal::gpio::gpioc::PC7<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>;
// const Gpio01: stm32f1xx_hal::gpio::gpioc::PC6 = ;

// this is the allocator the application will use
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const HEAP_SIZE: usize = 2048; // in bytes

const CARRIAGE_RETURN: u8 = 13;
const NEW_LINE: u8 = 10;

// ****************************************************************
// TO DO: replace it flexible alerts!!!

const alarm_hour_1: u16 = 7;
const alarm_min_1: u16 = 30;
const alarm_duration_1: u16 = 20; // sec

const alarm_hour_2: u16 = 20;
const alarm_min_2: u16 = 30;
const alarm_duration_2: u16 = 20; // sec


// ****************************************************************


// Size of buffer for received serial data
// max length of gcode line is supposed to be 256 and we need to hold a min of 2 lines
// TODO: should probably increase this since utf-8 is not 1 byte per character, seems to be working well though
const RX_SZ: usize = 512;

// ------------------------- Alerts:
const ALERTS_SZ: usize = 20;

// TO DO:............................
// enum GPIOActions {
    
// }
// enum ActionTypes {
//     SetGPIO ()
// }

// pub struct Action {
//     action_id: u16,
//     action_type: ActionTypes
// }

#[derive(Clone, Copy)]
pub enum AlertTimes {
    UnixTimestamp (u32), // seconds
    SimpleTime(NaiveTime),
    FullDateTime(NaiveDateTime),
    Period (u32) // ms
}



#[derive(Clone, Copy)]
pub struct Alert {
    duration: u32, // duration of the alert (ms)
    alert_time: AlertTimes,
    last_alert_time: u32,
    is_fired: bool,
    // action: GpioType01 // TO DO: make flexible
}

impl Alert {
    pub const fn new() -> Alert {
        Alert {duration: 0, alert_time: AlertTimes::UnixTimestamp(0), last_alert_time: 0, is_fired: false}
    }
}

pub struct AlertsList {
    index: usize,
    alerts: [Alert; ALERTS_SZ],
}

impl AlertsList {
    const fn new() -> AlertsList {
        AlertsList {
            index: 0,
            alerts: [Alert::new(); ALERTS_SZ],
        }
    }

    pub fn len(&self) -> usize {
        self.index
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn push(&mut self, data: Alert) -> Result<(), ()> {
        if self.index < ALERTS_SZ {
            self.alerts[self.index] = data;
            self.index += 1;
            return Ok(());
        }
        Err(())
    }

    pub fn read_and_reset(&mut self) -> Option<AlertsList> {
        if self.index > 0 {
            let tmp = self.index;
            self.index = 0;
            Some(AlertsList {
                index: tmp,
                alerts: self.alerts,
            })
        } else {
            None
        }
    }

    // check all alerts for given timestamp, and set fired attribute for fired events
    pub fn check_and_fire(&self, checking_time: AlertTimes) -> bool {

        true

    }

}

impl Default for AlertsList {
    fn default() -> Self {
        Self::new()
    }
}

impl Iterator for AlertsList {
    type Item = Alert;

    fn next(&mut self) -> Option<Alert> {
        if self.index != ALERTS_SZ {
            match &mut self.alerts[self.index].alert_time {
                // TO DO: check different types 
                AlertTimes::UnixTimestamp(x) => {
                    self.index = (self.index + 1) % ALERTS_SZ;
                    Some(self.alerts[self.index])
                }
                AlertTimes::SimpleTime(_) => {
                    self.index = (self.index + 1) % ALERTS_SZ;
                    Some(self.alerts[self.index])
                },
                AlertTimes::FullDateTime(_) => {
                    self.index = (self.index + 1) % ALERTS_SZ;
                    Some(self.alerts[self.index])
                },
                AlertTimes::Period(_) => {
                    self.index = (self.index + 1) % ALERTS_SZ;
                    Some(self.alerts[self.index])
                },
            }
        } else {
            None
        }
    }
}

// ----------------------- Buffers:
// Serial receive buffer
pub struct Buffer {
    index: usize,
    buffer: [u8; RX_SZ],
}

impl Buffer {
    const fn new() -> Buffer {
        Buffer {
            index: 0,
            buffer: [0; RX_SZ],
        }
    }

    pub fn push(&mut self, data: u8) -> Result<(), ()> {
        if self.index < RX_SZ {
            self.buffer[self.index] = data;
            self.index += 1;
            return Ok(());
        }
        Err(())
    }

    pub fn read(&mut self) -> Option<Buffer> {
        if self.index > 0 {
            let tmp = self.index;
            self.index = 0;
            Some(Buffer {
                index: tmp,
                buffer: self.buffer,
            })
        } else {
            None
        }
    }

    pub fn split(&mut self) -> Option<(Buffer, Buffer)> {
        if let Some(index) = self
            .buffer
            .iter()
            .position(|&x| x == CARRIAGE_RETURN || x == NEW_LINE)
        {
            let mut line = Buffer::new();
            let mut remainder = Buffer::new();
            for c in &self.buffer[0..index] {
                // TODO: handle error here
                if line.push(*c).is_err() {}
            }
            for c in &self.buffer[index + 1..self.index] {
                // TODO: handle error here
                if remainder.push(*c).is_err() {}
            }
            Some((line, remainder))
        } else {
            None
        }
    }
}

// CONNECTIONS
// serial tx and rx
// type TX = Tx<stm32f1xx_hal::pac::USART2>;
// type RX = Rx<stm32f1xx_hal::pac::USART2>;
type TX = stm32f1xx_hal::serial::Tx<stm32f1xx_hal::pac::USART2>;
type RX = stm32f1xx_hal::serial::Rx<stm32f1xx_hal::pac::USART2>;

#[app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        #[init(Buffer::new())]
        RX_BUF: Buffer,
        #[init(Buffer::new())]
        CMD_BUF: Buffer,
        // TIMER_X: CountDownTimer<stm32f1xx_hal::pac::TIM2>,
        // TIMER_Y: CountDownTimer<stm32f1xx_hal::pac::TIM3>,
        TX: TX,
        RX: RX,
        EXTI: stm32f1xx_hal::pac::EXTI,
        #[init(0)]
        SLEEP: u32,

        #[init(1)]
        beat: u32,
        #[init(1)]
        number: u32,
        #[init(1000)]
        last_brightness: u16,

        // late resources -- these must be initialized in the `init` task and
        // returned in `init::LateResources`
        led1: stm32f1xx_hal::gpio::gpioc::PC11<
            stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>,
        >,
        tmr2: stm32f1xx_hal::timer::CountDownTimer<stm32::TIM2>,
        tmr3: stm32f1xx_hal::timer::CountDownTimer<stm32::TIM3>,
        // tx: hal::serial::Tx<stm32f1xx_hal::pac::USART2>,
        // rx: hal::serial::Rx<stm32f1xx_hal::pac::USART2>,
        rtc: RtcStm,
        adc1: stm32f1xx_hal::adc::Adc<stm32f1xx_hal::pac::ADC1>,
        ch11: stm32f1xx_hal::gpio::gpioc::PC0<stm32f1xx_hal::gpio::Analog>, // stm32f1xx_hal::gpio::Analog, //
        display: DisplayType,
        Gpio01: GpioType01,
        Gpio02: GpioType02
    }

    // #[init(schedule = [perf])]
    #[init(schedule = [process, perf])]
    fn init(cx: init::Context) -> init::LateResources {
        // Initialize the allocator BEFORE you use it
        unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) }

        let device = cx.device;
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let exti = device.EXTI;

        // TODO: test performance and see what this needs to actually be set to
        // set the system clock to max for this chip
        let clocks = rcc
            .cfgr
            .sysclk(64.mhz())
            .pclk1(32.mhz())
            .freeze(&mut flash.acr);
        // // Now we can set the controllers frequency to 8 MHz:
        // let clocks = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr);

        // These timers are used to control the step speed of the steppers
        // the initial update freq doesn't matter since they won't start until a move is received
        // just don't set to 0hz or you get a divide by zero error
        // let tim2 = Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1).start_count_down(1.hz());
        // let tim3 = Timer::tim3(device.TIM3, &clocks, &mut rcc.apb1).start_count_down(1.hz());

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);

        // configure PB0 and PB1 for limit switches on X and Y axis
        gpiob.pb0.into_pull_up_input(&mut gpiob.crl);
        gpiob.pb1.into_pull_up_input(&mut gpiob.crl);

        // configure interrupts, PB0 to EXTI0, PB1 to EXTI1 for limit switches
        // enable interrupt mask for line 0 and 1
        exti.imr.write(|w| {
            w.mr0().set_bit();
            w.mr1().set_bit()
        });

        // set to falling edge triggering
        exti.ftsr.write(|w| {
            w.tr0().set_bit();
            w.tr1().set_bit()
        });

        // set exti0 and 1 to gpio bank B
        // TODO: submit patch to stm32f1 crate to make this call safe
        afio.exticr1.exticr1().write(|w| unsafe {
            w.exti0().bits(1);
            w.exti1().bits(1)
        });

        // **********************************************************************************************
        let adc1 = adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);

        // Configure pc5 as an analog input
        // let ch15 = gpioc.pc5.into_analog(&mut gpioc.crl);
        let ch11 = gpioc.pc0.into_analog(&mut gpioc.crl);

        let tmr = Timer::tim4(device.TIM4, &clocks, &mut rcc.apb1).start_count_down(200.khz());
        let scl = gpiob.pb6.into_open_drain_output(&mut gpiob.crl);
        let sda = gpiob.pb7.into_open_drain_output(&mut gpiob.crl);
        let i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, tmr);
        let mut rtc = Ds1307::new(i2c);

        let datetime = match rtc.get_datetime() {
            Ok(datetime) => datetime,
            Err(_error) => {
                let datetime = NaiveDate::from_ymd(2021, 4, 28).and_hms(01, 22, 15);
                rtc.set_datetime(&datetime).unwrap();
                rtc.set_running().unwrap();
                datetime
            }
        };

        let mut pwr = device.PWR;
        let mut backup_domain = rcc.bkp.constrain(device.BKP, &mut rcc.apb1, &mut pwr);
        let mut rtc = Rtc::rtc(device.RTC, &mut backup_domain);
        rtc.set_time(datetime.timestamp().try_into().unwrap());

        // let _datetime = rtc.get_datetime().unwrap();

        // DISPLAY:
        let pins = (
            gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh),
            gpiob.pb14.into_floating_input(&mut gpiob.crh),
            gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
        );

        let spi_cs = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
        let spi_mode = SpiMode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let spi = Spi::spi2(
            device.SPI2,
            pins,
            spi_mode,
            100.khz(),
            clocks,
            &mut rcc.apb1,
        );
        let mut display = MAX7219::from_spi_cs(5, spi, spi_cs).unwrap();
        // make sure to wake the display up
        display.power_on().unwrap();
        display.set_decode_mode(1, DecodeMode::NoDecode).unwrap();

        display.clear_display(0).unwrap();
        display.clear_display(1).unwrap();
        display.clear_display(2).unwrap();
        display.clear_display(3).unwrap();
        display.clear_display(4).unwrap();
        // write given octet of ASCII characters with dots specified by 3rd param bits
        let cd = encode_char('S');
        // display.write_str(1, &cd, 0b00000001).unwrap();
        display.write_raw(1, &cd).unwrap();

        display.write_str(0, &encode_char('N'), 0b00000010).unwrap();
        // set display intensity lower
        display.set_intensity(0, 0x3).unwrap();
        display.set_intensity(1, 0x3).unwrap();
        display.set_intensity(2, 0x3).unwrap();
        display.set_intensity(3, 0x3).unwrap();
        display.set_intensity(4, 0x3).unwrap();

        let led1 = gpioc.pc11.into_push_pull_output(&mut gpioc.crh);


// TO DO: flexible init actions GPIOs:
        let Gpio01 = gpioc.pc6.into_push_pull_output(&mut gpioc.crl);
        let mut Gpio02 = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);

        Gpio02.set_low().unwrap();

        // Use TIM2 for the beat counter task
        let mut tmr2 = Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1).start_count_down(1.hz());
        tmr2.listen(Event::Update);

        // Use TIM3 for the LED blinker task
        let mut tmr3 = Timer::tim3(device.TIM3, &clocks, &mut rcc.apb1).start_count_down(2.hz());
        tmr3.listen(Event::Update);

        // *************************** Serial setup:
        // USART2
        // Configure pa2 as a push_pull output, this will be the tx pin
        let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        // Take ownership over pa3
        let rx = gpioa.pa3;
        let mut serial = Serial::usart2(
            device.USART2,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(9600.bps()), // 57600
            clocks,
            &mut rcc.apb1,
        );
        serial.listen(stm32f1xx_hal::serial::Event::Rxne);
        let (mut tx, rx) = serial.split();

        rtic::pend(stm32f1xx_hal::pac::Interrupt::USART2);

        // *******************************************************************************************

        // // SERIAL pins for USART 1
        // let pa9 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        // let pa10 = gpioa.pa10;

        // let mut serial = Serial::usart1(
        //     device.USART1,
        //     (pa9, pa10),
        //     &mut afio.mapr,
        //     Config {
        //         baudrate: 9_600.bps(),
        //         ..Default::default()
        //     },
        //     clocks,
        //     &mut rcc.apb2,
        // );

        // // Enable RX interrupt
        // serial.listen(stm32f1xx_hal::serial::Event::Rxne);

        // let (mut tx, rx) = serial.split();

        // TODO: look in to changing serial TX back to DMA
        for c in b"\r\ninitialized\r\n" {
            block!(tx.write(*c)).ok();
        }

        // schedule tasks to process gcode and send out performance data
        // TODO: tweak how often this runs
        cx.schedule
            .process(Instant::now() + 64_000_000.cycles())
            .unwrap();
        cx.schedule
            .perf(Instant::now() + 640_000_000.cycles())
            .unwrap();

        // pend all used interrupts
        rtic::pend(Interrupt::USART2);
        rtic::pend(Interrupt::EXTI0);
        rtic::pend(Interrupt::EXTI1);
        rtic::pend(Interrupt::TIM2);
        rtic::pend(Interrupt::TIM3);

        init::LateResources {
            // TIMER_X: tim2,
            // TIMER_Y: tim3,
            TX: tx,
            RX: rx,
            EXTI: exti,
            led1,
            tmr2,
            tmr3,
            // spi,
            rtc,
            display,
            adc1,
            ch11,
            Gpio01,
            Gpio02
        }
    }

    #[idle(resources = [SLEEP])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            // record when this loop starts
            let before = Instant::now();
            // wait for an interrupt (sleep)
            wfi();
            // after interrupt is fired add sleep time to the sleep tracker
            cx.resources
                .SLEEP
                .lock(|sleep| *sleep += before.elapsed().as_cycles());
        }
    }

    #[task(schedule = [process], resources = [RX_BUF, CMD_BUF, SLEEP, TX, rtc])]
    fn process(mut cx: process::Context) {
        // check if any commands have been received, if so process it
        if let Some(data) = cx.resources.RX_BUF.lock(Buffer::read) {
            // add the data to the command buffer
            for c in &data.buffer[0..data.index] {
                // TODO: handle error here
                if cx.resources.CMD_BUF.push(*c).is_err() {}
            }

            if let Some(buf_tmp) = cx.resources.CMD_BUF.read() {
                match str::from_utf8(&buf_tmp.buffer[0..buf_tmp.index]) {
                    Err(e) => {} // TODO: error processing //.expect("utf error");

                    Ok(s) => {
                        match parse_json(&s) {
                            Err(e) => {} // TODO: error processing //.expect("Invalid JSON specified!")

                            Ok(json_data) => match json_data {
                                JsonValue::Object(json_root) => {
                                    let json_root_name = &json_root[0].0;
                                    let json_root_obj = &json_root[0].1;
                                    // root element must be "ssn":
                                    if json_root_name.eq(&['s', 's', 'n']) {
                                        match json_root_obj {
                                            JsonValue::Object(command_obj) => {
                                            let command_name = &command_obj[0].0;
                                            let command_data = &command_obj[0].1;
                                            match &command_name.as_slice() {
                                                &['s', 'e', 't','d','a','t','e','t','i','m','e'] => {
                                                    match command_data {
                                                        JsonValue::Number(num) => {
                                                            let x = num.integer;
                                                            cx.resources.rtc.set_time(x as u32);
                                                            let y = x;
                                                        }
                                                        JsonValue::String(s) => {
                                                            let z = &s.clone();
                                                        }
                                                        // ignore other types:
                                                        (_) => (),
                                                    }
                                                }
                                                (_) => (),

                                            }
                                        }
                                          // root element must be object with command, other types are ignore:
                                            (_) => (),
                                        }
                                    }
                                }
                                JsonValue::Array(arr) => {
                                    if arr.len() > 0 {
                                        let bbb = arr;
                                    }
                                }
                                JsonValue::String(_) => todo!(),
                                JsonValue::Number(_) => todo!(),
                                JsonValue::Boolean(_) => todo!(),
                                JsonValue::Null => todo!(),
                            },
                        }
                    }
                }
            }
        }
        // TODO: tweak how often this runs
        cx.schedule
            .process(cx.scheduled + 64_000_000.cycles())
            .unwrap();
    }

    #[task(schedule = [perf], resources = [SLEEP, TX])]
    fn perf(mut cx: perf::Context) {
        // send performance info once a second
        let mut data = [0; 12];
        let mut buf: [u8; 13] = [0; 13];

        LE::write_u32(&mut data[0..4], *cx.resources.SLEEP);
        cobs::encode(&data, &mut buf);
        *cx.resources.SLEEP = 0;

        // cx.resources.TX.lock(|tx| print_serial(tx));
        for c in b"\r\n test \r\n" {
            cx.resources.TX.lock(|tx| block!(tx.write(*c)).ok());
        }

        for byte in &buf {
            cx.resources.TX.lock(|tx| block!(tx.write(*byte)).ok());
        }
        cx.schedule
            .perf(cx.scheduled + 640_000_000.cycles())
            .unwrap();
    }

    // Update the beat counter and periodically display the current count
    // on the RTT channel
    #[task(resources = [last_brightness, beat, display, rtc, adc1, ch11, Gpio02], spawn = [print_out, print_time, display_matrix])]
    fn beat_update(cx: beat_update::Context) {
        // if *cx.resources.beat % 5 == 0 {
        //     // hprintln!("TIM2 beat = {}", *cx.resources.beat);
        //     // writeln!(*cx.resources.TX, "Hello formatted string {}", *cx.resources.beat).unwrap();
        //     Some(cx.spawn.print_out(*cx.resources.beat * 10));
        // };

        // let seconds = cx.resources.rtc.get_seconds().unwrap();
        // let time = cx.resources.rtc.get_time(); //.unwrap() as NaiveTime;
        let time_ts = cx.resources.rtc.current_time(); // Unix timestamp
        let time = NaiveDateTime::from_timestamp(time_ts.into(), 0);
        // let time = match time {
        //     Ok(time) => time,
        //     Err(_error) => {NaiveTime::from_hms(0, 0, 0)},
        // };
        let h = time.hour() as u8;
        let m = time.minute() as u8;
        let s = time.second() as u8;


        // Check ALARM state:
        let alarm_state: bool = if ((h as u16 == alarm_hour_1) && (m as u16 >= alarm_min_1) && ((m as u16 *60 + s as u16 ) <= (alarm_min_1 * 60 + alarm_duration_1)) )
        || ((h as u16 == alarm_hour_2) && (m as u16 >= alarm_min_2) && ((m as u16 *60 + s as u16 ) <= (alarm_min_2 * 60 + alarm_duration_2)) ) {
            cx.resources.Gpio02.set_high().unwrap();

            true

        } else { 
            cx.resources.Gpio02.set_low().unwrap();
            false 
        };

        let div_char = if alarm_state { '*' } else { ':' };

        let attrDisp = DisplayKind::HourMinutes; // TO DO: type of the display - 0: min:sec, 1 - hour:min ...

        // let m = (datetime - NaiveDate::from_ymd(2020, 5, 2).and_hms(10, 21, 34)).num_minutes();
        // cx.spawn.print_out(x as u8).unwrap();
        cx.spawn.print_time(h, m, s).unwrap();

        // cx.resources.display.write_raw(1, &encode_num(*cx.resources.beat % 10)).unwrap();

        let data_adc: u16 = cx.resources.adc1.read(cx.resources.ch11).unwrap();
        // let data_adc: u16 = 1000;

        // if ((*cx.resources.last_brightness - data_adc) as i16).abs() >= 100 {
        if i16::abs(*cx.resources.last_brightness as i16 - (data_adc as i16)) >= 100 {
            let new_brt = data_adc / 170;
            let new_brt = (8 - if new_brt <= 1 {
                1
            } else if new_brt >= 7 {
                7
            } else {
                new_brt
            }) as u8;
            cx.resources.display.set_intensity(0, new_brt).unwrap();
            cx.resources.display.set_intensity(1, new_brt).unwrap();
            cx.resources.display.set_intensity(2, new_brt).unwrap();
            cx.resources.display.set_intensity(3, new_brt).unwrap();
            cx.resources.display.set_intensity(4, new_brt).unwrap();
        }
        *cx.resources.last_brightness = data_adc;

        let matrixArray = match attrDisp {
            DisplayKind::HourMinutes => [
                encode_digit((h / 10) % 10),
                encode_digit(h % 10),
                encode_char(if s % 2 == 0 { ' ' } else { div_char }),
                encode_digit((m / 10) % 10),
                encode_digit(m % 10),
            ],
            DisplayKind::MinutesSeconds => [
                encode_digit((m / 10) % 10),
                encode_digit(m % 10),
                encode_char(if s % 2 == 0 { ' ' } else { div_char }),
                encode_digit((s / 10) % 10),
                encode_digit(s % 10),
            ],
            DisplayKind::TestData => display_matrix_int(data_adc),
        };

        Some(cx.spawn.display_matrix(matrixArray));
    }

    // Interrupt task for TIM2, the beat counter timer
    #[task(binds = TIM2, priority = 1, resources = [tmr2, number], spawn = [beat_update, print_out])]
    fn tim2(cx: tim2::Context) {
        // Delegate the state update to a software task
        Some(cx.spawn.beat_update()); //.unwrap();

        *cx.resources.number += 1;
        // writeln!(*cx.resources.tx, "Hello formatted string {}", *cx.resources.number).unwrap();
        // cx.spawn.print_out(*cx.resources.number).unwrap();

        // Restart the timer and clear the interrupt flag
        cx.resources.tmr2.start(1.hz());
        cx.resources.tmr2.clear_update_interrupt_flag();
    }

    // Interrupt task for TIM3, the LED blink timer
    #[task(binds = TIM3, priority = 1, resources = [led1, tmr3, beat])]
    fn tim3(cx: tim3::Context) {
        *cx.resources.beat += 1;
        cx.resources.led1.toggle().unwrap();
        cx.resources.tmr3.start(2.hz());
        cx.resources.tmr3.clear_update_interrupt_flag();
    }

    #[task(priority = 1, capacity = 4, resources = [TX])]
    fn print_out(mut cx: print_out::Context, x: u8) {
        for c in b"\r\n test \r\n" {
            cx.resources.TX.lock(|tx| block!(tx.write(*c)).ok());
        }
        // writeln!(*cx.resources.TX, "\r\nFormatted string {}", x).unwrap();
    }

    #[task(capacity = 4, resources = [TX])]
    fn print_time(cx: print_time::Context, h: u8, m: u8, s: u8) {
        // writeln!(*cx.resources.TX, "\r\n {}:{}:{}", h, m, s).unwrap();
    }

    #[task(capacity = 1, resources = [display])]
    fn display_matrix(cx: display_matrix::Context, matrix: [SingleDisplayData; 5]) {
        cx.resources.display.write_raw(0, &matrix[4]).unwrap();
        cx.resources.display.write_raw(1, &matrix[3]).unwrap();
        cx.resources.display.write_raw(2, &matrix[2]).unwrap();
        cx.resources.display.write_raw(3, &matrix[1]).unwrap();
        cx.resources.display.write_raw(4, &matrix[0]).unwrap();
    }

    // #[task(binds = USART2, priority = 1, resources = [RX,RX_BUF,TX], spawn = [display_matrix])]
    // fn usart2(cx: usart2::Context) {
    //     // let val = cx.resources.rx.read().unwrap();
    //     // cx.spawn.display_matrix(display_matrix_int(val as u16)).unwrap();
    //     match cx.resources.RX.read() {
    //         Ok(c) => {
    //             // TODO: handle buffer being full
    //             block!(cx.resources.TX.write(c)).ok();
    //             if cx.resources.RX_BUF.push(c).is_ok() {}
    //         }
    //         Err(e) => {
    //             match e {
    //                 // no data available
    //                 nb::Error::WouldBlock => {
    //                     /*
    //                     for c in b"blocking\r\n" {
    //                         block!(resources.TX.write(*c)).ok();
    //                     }
    //                     */
    //                 }
    //                 nb::Error::Other(stm32f1xx_hal::serial::Error::Overrun) => {
    //                     for c in b"rx buffer overrun error\r\n" {
    //                         block!(cx.resources.TX.write(*c)).ok();
    //                     }
    //                 }
    //                 _ => {
    //                     for c in b"other error\r\n" {
    //                         block!(cx.resources.TX.write(*c)).ok();
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    // Interrupt handler for serial receive, needs to be high priority or the receive buffer overruns
    #[task(binds=USART2, priority = 2, resources=[RX,RX_BUF,TX])]
    fn USART2(cx: USART2::Context) {
        // Read each character from serial as it comes in and add it to the rx buffer
        match cx.resources.RX.read() {
            Ok(c) => {
                // TODO: handle buffer being full
                if cx.resources.RX_BUF.push(c).is_ok() {}
            }
            Err(e) => {
                match e {
                    // no data available
                    nb::Error::WouldBlock => {
                        // for c in b"blocking\r\n" {
                        //     block!(cx.resources.TX.write(*c)).ok();
                        // }
                    }
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Overrun) => {
                        for c in b"rx buffer overrun error\r\n" {
                            block!(cx.resources.TX.write(*c)).ok();
                        }
                    }
                    _ => {
                        for c in b"other error\r\n" {
                            block!(cx.resources.TX.write(*c)).ok();
                        }
                    }
                }
            }
        }
    }

    // X axis limit switch interrupt handler
    #[task(binds=EXTI0, resources=[EXTI])]
    fn EXTI0(cx: EXTI0::Context) {
        // cx.resources.CURRENT.x = None;
        // cx.resources.LOCATION.x = 0.0;
        // cx.resources.VIRT_LOCATION.x = 0.0;

        // clear the interrupt pending bit or the interrupt will keep firing
        // TODO: see if there's a way to clear this without having an instance of the EXTI registers
        // reference https://github.com/rust-embedded/cortex-m/pull/138
        cx.resources.EXTI.pr.write(|w| w.pr0().set_bit());
    }

    // Y axis limit switch interrupt handler
    #[task(binds=EXTI1, resources=[EXTI])]
    fn EXTI1(cx: EXTI1::Context) {
        // cx.resources.CURRENT.y = None;
        // cx.resources.LOCATION.y = 0.0;
        // cx.resources.VIRT_LOCATION.y = 0.0;

        // clear the interrupt pending bit or the interrupt will keep firing
        cx.resources.EXTI.pr.write(|w| w.pr1().set_bit());
    }

    // spare interrupt used for scheduling software tasks
    extern "C" {
        fn TAMPER();
        fn DMA1_CHANNEL1();
    }
};

// fn print_serial(tx: TX) {
//     for c in b"\r\n test \r\n" {
//         tx.write(*c).ok();
//     }
//     // writeln!(*cx.resources.TX, "\r\nFormatted string {}", x).unwrap();
// }

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt();

    loop {}
}
