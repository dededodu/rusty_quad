#![no_std]
#![no_main]

use arduino_hal::{prelude::_unwrap_infallible_UnwrapInfallible, Delay};
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        100000,
    );

    arduino_hal::delay_ms(1000);

    // Init BNO055 IMU
    let mut imu = bno055::Bno055::new(i2c).with_alternative_address();
    let mut delay = Delay::new();
    match imu.init(&mut delay) {
        Ok(_) => {}
        Err(bno055::Error::I2c(e)) => {
            ufmt::uwriteln!(&mut serial, "\r\nIMU initialization failed: I2C {:?}\r", e)
                .unwrap_infallible()
        }
        Err(bno055::Error::AccConfig(_)) => {
            ufmt::uwriteln!(&mut serial, "\r\nIMU initialization failed: AccConfig\r")
                .unwrap_infallible()
        }
        Err(bno055::Error::InvalidChipId(chip_id)) => ufmt::uwriteln!(
            &mut serial,
            "\r\nIMU initialization failed: InvalidChipId {}\r",
            chip_id
        )
        .unwrap_infallible(),
        Err(bno055::Error::InvalidMode) => {
            ufmt::uwriteln!(&mut serial, "\r\nIMU initialization failed: InvalidMode\r")
                .unwrap_infallible()
        }
    }
    arduino_hal::delay_ms(1000);

    // Enable 9-degrees-of-freedom sensor fusion mode with fast magnetometer calibration
    imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
        .unwrap();
    arduino_hal::delay_ms(1000);

    /*
    ufmt::uwriteln!(&mut serial, "Beginning BNO055 IMU calibration\r").unwrap_infallible();
    while !imu.is_fully_calibrated().unwrap() {
        let status = imu.get_calibration_status().unwrap();
        ufmt::uwriteln!(&mut serial, "Calibration status: {}, {}, {}, {}", status.acc, status.gyr, status.mag, status.sys).unwrap_infallible();
        arduino_hal::delay_ms(1000);
    }

    ufmt::uwriteln!(&mut serial, "TEST 4\r").unwrap_infallible();
    let calib = imu.calibration_profile(&mut delay).unwrap();
    ufmt::uwriteln!(&mut serial, "TEST 5\r").unwrap_infallible();

    imu.set_calibration_profile(calib, &mut delay).unwrap();
    ufmt::uwriteln!(&mut serial, "Calibration complete!").unwrap_infallible();
    */

    loop {
        arduino_hal::delay_ms(100);
        let a = imu.euler_angles().unwrap().a;
        let b = imu.euler_angles().unwrap().b;
        let c = imu.euler_angles().unwrap().c;
        ufmt::uwriteln!(
            &mut serial,
            "A = {}, B = {}, C = {}\r",
            a as i32,
            b as i32,
            c as i32
        )
        .unwrap_infallible();
    }
}
