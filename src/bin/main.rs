#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use bt_hci::{
    controller::ExternalController,
    uuid::{characteristic::MEDIA_CONTROL_POINT, service::GENERIC_MEDIA_CONTROL},
};
use defmt::{Debug2Format, error, info, unwrap};
use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_time::Timer;
use embedded_graphics::{
    image::{Image, ImageRaw},
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_hal::{
    clock::CpuClock,
    i2c::master::{Config, I2c},
    rng::{Trng, TrngSource},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use panic_rtt_target as _;
use rtt_target::{ChannelMode, DownChannel, set_defmt_channel, set_print_channel};
use sequential_storage::{
    cache::{KeyCacheImpl, NoCache},
    map::{Key, MapConfig, MapStorage, SerializationError, Value},
};
use ssd1306::{I2CDisplayInterface, Ssd1306Async, prelude::*};
use trouble_host::prelude::*;

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

const SCREEN_WIDTH: i32 = 128;
const SCREEN_HEIGHT: i32 = 64;

const PLAY_IMG: ImageRaw<BinaryColor> = ImageRaw::new(include_bytes!("../../images/play.raw"), 24);
const PAUSE_IMG: ImageRaw<BinaryColor> =
    ImageRaw::new(include_bytes!("../../images/pause.raw"), 24);
const FORWARD_IMG: ImageRaw<BinaryColor> =
    ImageRaw::new(include_bytes!("../../images/skip-forward.raw"), 24);
const BACKWARD_IMG: ImageRaw<BinaryColor> =
    ImageRaw::new(include_bytes!("../../images/skip-back.raw"), 24);

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(clippy::large_stack_frames, reason = "GATT is better on a stack")]
#[gatt_server]
struct Server {}

#[derive(Debug, Clone, PartialEq, Eq)]
struct StoredAddr(BdAddr);

impl Key for StoredAddr {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 6 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..6].copy_from_slice(self.0.raw());
        Ok(6)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.len() < 6 {
            Err(SerializationError::BufferTooSmall)
        } else {
            Ok((StoredAddr(BdAddr::new(buffer[0..6].try_into().unwrap())), 6))
        }
    }
}

#[derive(Debug)]
struct StoredBondInformation {
    ltk: LongTermKey,
    irk: Option<IdentityResolvingKey>,
    security_level: SecurityLevel,
}

impl<'a> Value<'a> for StoredBondInformation {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 33 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..16].copy_from_slice(self.ltk.to_le_bytes().as_slice());

        if let Some(key) = self.irk {
            buffer[16..32].copy_from_slice(key.to_le_bytes().as_slice());
        } else {
            buffer[16..32].fill(0);
        }

        buffer[32] = match self.security_level {
            SecurityLevel::NoEncryption => 0,
            SecurityLevel::Encrypted => 1,
            SecurityLevel::EncryptedAuthenticated => 2,
        };
        Ok(33)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 33 {
            Err(SerializationError::BufferTooSmall)
        } else {
            let ltk = LongTermKey::from_le_bytes(buffer[0..16].try_into().unwrap());

            let raw_irk: [u8; 16] = buffer[16..32].try_into().unwrap();
            let irk = if raw_irk == [0u8; 16] {
                None
            } else {
                Some(IdentityResolvingKey::from_le_bytes(raw_irk))
            };

            let security_level = match buffer[32] {
                0 => SecurityLevel::NoEncryption,
                1 => SecurityLevel::Encrypted,
                2 => SecurityLevel::EncryptedAuthenticated,
                _ => return Err(SerializationError::InvalidData),
            };
            Ok((
                StoredBondInformation {
                    ltk,
                    irk,
                    security_level,
                },
                33,
            ))
        }
    }
}

async fn host_runner_task<C: Controller, P: PacketPool>(runner: &mut Runner<'_, C, P>) {
    loop {
        runner.run().await.ok();
    }
}

async fn write_bond<K: Key, S: embedded_storage_async::nor_flash::NorFlash, C: KeyCacheImpl<K>>(
    map_storage: &mut MapStorage<K, S, C>,
    key: K,
    bond_info: BondInformation,
) {
    let mut buffer = [0; 39];
    let value = StoredBondInformation {
        ltk: bond_info.ltk,
        irk: bond_info.identity.irk,
        security_level: bond_info.security_level,
    };
    map_storage.erase_all().await.unwrap();
    map_storage
        .store_item(&mut buffer, &key, &value)
        .await
        .unwrap();
    info!("Wrote bond_info keys to NVS");
}

async fn gatt_client_task<T: Controller, P: PacketPool, const M: usize>(
    client: &GattClient<'_, T, P, M>,
) {
    loop {
        client.task().await.unwrap();
    }
}

#[allow(clippy::large_stack_frames, reason = "This is a big function")]
async fn app_logic<T: Controller, P: PacketPool, const M: usize, D: DrawTarget>(
    input: &mut DownChannel,
    client: &GattClient<'_, T, P, M>,
    display: &mut D,
) {
    let services = client
        .services_by_uuid(&GENERIC_MEDIA_CONTROL.into())
        .await
        .unwrap();

    let service = services.first().unwrap().clone();
    info!("Fetched service!");

    loop {
        let mut buf = [0u8; 64];
        let count = input.read(&mut buf);
        let cmd = core::str::from_utf8(&buf[..count]).unwrap_or("").trim();

        match cmd {
            "help" => {
                info!(
                    r#"
                                        title - what is playing rn 
                                        play  - play media 
                                        pause - pause media 
                                        "#
                )
            }
            "title" => {
                let c = client
                    .characteristic_by_uuid::<str>(&service, &Uuid::new_short(0x2B97))
                    .await
                    .unwrap();

                let mut data = [0u8; 64];
                let amnt = client.read_characteristic(&c, &mut data).await.unwrap();

                info!("{:?}", str::from_utf8(&data[..amnt]).unwrap());
                info!("{:?}", Debug2Format(&c));
            }
            "play" => {
                let c = client
                    .characteristic_by_uuid::<u8>(&service, &MEDIA_CONTROL_POINT.into())
                    .await
                    .unwrap();

                client.write_characteristic(&c, &[0x01]).await.unwrap();
            }
            "pause" => {
                let c = client
                    .characteristic_by_uuid::<u8>(&service, &MEDIA_CONTROL_POINT.into())
                    .await
                    .unwrap();

                client.write_characteristic(&c, &[0x02]).await.unwrap();
            }
            "" => {}
            _ => info!("Don't know what that is!"),
        }
    }
}

async fn advertise<'values, 'server, C: Controller>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> GattConnection<'values, 'server, DefaultPacketPool> {
    let mut adv_data = [0u8; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(b"TrouBLE"),
        ],
        &mut adv_data,
    )
    .unwrap();
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &[],
            },
        )
        .await
        .unwrap();

    info!("Advertsing!");
    let conn = advertiser
        .accept()
        .await
        .unwrap()
        .with_attribute_server(server)
        .unwrap();

    conn.raw().set_bondable(true).unwrap();

    info!("Someone connected!");
    conn
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> () {
    // generator version: 1.3.0
    // generator parameters: --chip esp32s3 -o esp32s3-wroom-2 -o unstable-hal -o alloc -o wifi -o embassy -o ble-trouble -o probe-rs -o defmt -o panic-rtt-target -o wokwi -o vscode

    // Configure RTT channels
    let channels = rtt_target::rtt_init! {
        up: {
            0: { size: 256,  mode: ChannelMode::NoBlockSkip, name: "terminal", section: ".rtt_data" }
            1: { size: 1024, mode: ChannelMode::NoBlockSkip, name: "defmt", section: ".rtt_data" }
            2: { size: 256,  mode: ChannelMode::NoBlockSkip, name: "binary", section: ".rtt_data" }
        }
        down: {
            0: { size: 16, name: "terminal", section: ".rtt_data" }
        }
        section_cb: ".rtt_data"
    };

    let mut input = channels.down.0;

    set_print_channel(channels.up.0);
    set_defmt_channel(channels.up.1);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The following pins are used to bootstrap the chip. They are available
    // for use, but check the datasheet of the module for more information on them.
    // - GPIO0
    // - GPIO3
    // - GPIO45
    // - GPIO46
    // These GPIO pins are in use by some feature of the module and should not be used.
    let _ = peripherals.GPIO33;
    let _ = peripherals.GPIO34;
    let _ = peripherals.GPIO35;
    let _ = peripherals.GPIO36;
    let _ = peripherals.GPIO37;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 1>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    // RNG source for security
    let _trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1);
    let mut trng = Trng::try_new().unwrap(); // Ok when there's a TrngSource accessible

    // Bluetooth Stack
    let stack =
        trouble_host::new(ble_controller, &mut resources).set_random_generator_seed(&mut trng);

    // NVS Flash
    let flash_storage = FlashStorage::new(peripherals.FLASH).multicore_auto_park();
    let erase_size = <FlashStorage as NorFlash>::ERASE_SIZE as u32;
    let capacity = flash_storage.capacity() as u32;
    let storage_range = (capacity - erase_size * 2)..capacity;
    let flash = embassy_embedded_hal::adapter::BlockingAsync::new(flash_storage);

    // I2C Bus
    let i2c = I2c::new(
        peripherals.I2C0,
        Config::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO4)
    .with_scl(peripherals.GPIO5)
    .into_async();

    // SSD1306 Display (can change)
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(
        interface,
        ssd1306::size::DisplaySize128x64,
        ssd1306::rotation::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    display.init().await.unwrap();
    info!("Initalized display");

    // Storage logic
    let mut map_storage =
        MapStorage::<StoredAddr, _, _>::new(flash, MapConfig::new(storage_range), NoCache::new());

    // Check if bond info there
    let mut buf = [0u8; 64];
    let mut iter = map_storage.fetch_all_items(&mut buf).await.unwrap();

    // TODO: Maybe a loop?
    if let Some((key, value)) = unwrap!(iter.next::<StoredBondInformation>(&mut buf).await) {
        info!(
            "Bond stored. Address: {}, Value: {:?}",
            key.0,
            Debug2Format(&value)
        );
        stack
            .add_bond_information(BondInformation {
                identity: Identity {
                    bd_addr: key.0,
                    irk: value.irk,
                },
                security_level: value.security_level,
                is_bonded: true,
                ltk: value.ltk,
            })
            .unwrap();
        true
    } else {
        false
    };

    stack.set_io_capabilities(IoCapabilities::NoInputNoOutput);
    let host = stack.build();
    let mut runner = host.runner;
    let mut peripheral = host.peripheral;

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Widget",
        appearance: &appearance::REMOTE_CONTROL,
    }))
    .unwrap();

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    Text::with_alignment(
        "Connect to \"Widget\"\non Bluetooth",
        Point::new(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2),
        style,
        Alignment::Center,
    )
    .draw(&mut display)
    .unwrap();

    display.flush().await.unwrap();

    loop {
        display.clear(BinaryColor::Off).unwrap();

        Text::with_alignment(
            "Song Title",
            Point::new(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 4),
            style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        // TODO: Rounded rectangles
        let percentage = 0.3;
        let width = (SCREEN_WIDTH as f32 * 0.8) as i32;
        let done = (percentage * width as f32) as i32;
        let margin = (SCREEN_WIDTH as f32 * 0.8) as i32;

        Rectangle::new(Point::new(margin, SCREEN_HEIGHT / 2), Size::new(done, 10));

        Image::with_center(
            &PLAY_IMG,
            Point::new(SCREEN_WIDTH / 2, 3 * SCREEN_HEIGHT / 4),
        )
        .draw(&mut display)
        .unwrap();

        Image::with_center(
            &BACKWARD_IMG,
            Point::new(SCREEN_WIDTH / 4, 3 * SCREEN_HEIGHT / 4),
        )
        .draw(&mut display)
        .unwrap();

        Image::with_center(
            &FORWARD_IMG,
            Point::new(3 * SCREEN_WIDTH / 4, 3 * SCREEN_HEIGHT / 4),
        )
        .draw(&mut display)
        .unwrap();

        display.flush().await.unwrap();
    }

    let conn = advertise(&mut peripheral, &server).await;

    info!("Starting!");

    let _ = join(host_runner_task(&mut runner), async {
        loop {
            let client =
                GattClient::<ExternalController<_, 1>, DefaultPacketPool, 32>::new_no_mtu_bs(
                    &stack,
                    conn.raw(),
                )
                .await
                .unwrap();

            info!("Created Gatt Client");

            select(
                async {
                    loop {
                        match conn.next().await {
                            GattConnectionEvent::Disconnected { reason } => {
                                info!("Disconnected: {:?}", Debug2Format(&reason));
                                break;
                            }
                            GattConnectionEvent::Gatt { event } => {
                                event.accept().ok();
                            }
                            GattConnectionEvent::PairingComplete {
                                security_level: _,
                                bond,
                            } => {
                                info!("Pairing complete: {:?}", Debug2Format(&bond));
                                if bond.is_none() {
                                    info!("No binding info");
                                    return;
                                }
                                let info = bond.unwrap();
                                write_bond(
                                    &mut map_storage,
                                    StoredAddr(info.identity.bd_addr),
                                    info,
                                )
                                .await;
                            }
                            GattConnectionEvent::PairingFailed(err) => {
                                error!("[gatt] pairing error: {:?}", Debug2Format(&err));
                            }
                            _ => {}
                        }
                    }
                },
                join(gatt_client_task(&client), async {
                    loop {
                        if conn.raw().security_level().unwrap().encrypted() {
                            break;
                        }
                        Timer::after_secs(2).await;
                    }

                    app_logic(&mut input, &client, &mut display).await;
                }),
            )
            .await;
        }
    })
    .await;

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}
