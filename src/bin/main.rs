#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use alloc::format;

use bt_hci::{
    controller::ExternalController,
    uuid::{
        characteristic::{
            MEDIA_CONTROL_POINT, MEDIA_STATE, TRACK_DURATION, TRACK_POSITION, TRACK_TITLE,
        },
        service::GENERIC_MEDIA_CONTROL,
    },
};
use defmt::{Debug2Format, error, info, unwrap};
use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::Timer;
use embedded_graphics::{
    image::{Image, ImageRaw},
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Alignment, Text},
};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_hal::{
    Async,
    clock::CpuClock,
    gpio::{Input, InputConfig, Pull},
    i2c::master::{Config, I2c},
    rng::{Trng, TrngSource},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use panic_rtt_target as _;
use rtt_target::rtt_init_defmt;
use sequential_storage::{
    cache::{KeyCacheImpl, NoCache},
    map::{Key, MapConfig, MapStorage, SerializationError, Value},
};
use ssd1306::{I2CDisplayInterface, Ssd1306Async, mode::BufferedGraphicsModeAsync, prelude::*};
use trouble_host::prelude::*;

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3;

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

async fn draw_screen(
    song_title: &str,
    percentage: f32,
    playing: bool,
    display: &mut Ssd1306Async<
        I2CInterface<I2c<'_, Async>>,
        DisplaySize128x64,
        BufferedGraphicsModeAsync<DisplaySize128x64>,
    >,
) {
    display.clear(BinaryColor::Off).unwrap();

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    let display_text = if song_title.len() > 15 {
        format!("{}...", &song_title[..12])
    } else {
        song_title.into()
    };

    Text::with_alignment(
        &display_text,
        Point::new(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 4),
        text_style,
        Alignment::Center,
    )
    .draw(display)
    .unwrap();

    // TODO: Rounded rectangles
    let width = (SCREEN_WIDTH as f32 * 0.8) as i32;
    let done = (percentage * width as f32) as i32;
    let margin = (SCREEN_WIDTH as f32 * 0.1) as i32;

    Rectangle::new(
        Point::new(margin, SCREEN_HEIGHT / 2),
        Size::new(done as u32, 4),
    )
    .into_styled(
        PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::On)
            .stroke_width(1)
            .fill_color(BinaryColor::On)
            .build(),
    )
    .draw(display)
    .unwrap();

    Rectangle::new(
        Point::new(margin + done, SCREEN_HEIGHT / 2),
        Size::new((width - done) as u32, 4),
    )
    .into_styled(
        PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::On)
            .stroke_width(1)
            .fill_color(BinaryColor::Off)
            .build(),
    )
    .draw(display)
    .unwrap();

    Image::with_center(
        if playing { &PAUSE_IMG } else { &PLAY_IMG },
        Point::new(SCREEN_WIDTH / 2, 3 * SCREEN_HEIGHT / 4),
    )
    .draw(display)
    .unwrap();

    Image::with_center(
        &BACKWARD_IMG,
        Point::new(SCREEN_WIDTH / 4, 3 * SCREEN_HEIGHT / 4),
    )
    .draw(display)
    .unwrap();

    Image::with_center(
        &FORWARD_IMG,
        Point::new(3 * SCREEN_WIDTH / 4, 3 * SCREEN_HEIGHT / 4),
    )
    .draw(display)
    .unwrap();

    display.flush().await.unwrap();
}

async fn app_logic<T: Controller, P: PacketPool, const M: usize>(
    button_matrix: (&Input<'_>, &Input<'_>, &Input<'_>),
    client: &GattClient<'_, T, P, M>,
    display: &mut Ssd1306Async<
        I2CInterface<I2c<'_, Async>>,
        DisplaySize128x64,
        BufferedGraphicsModeAsync<DisplaySize128x64>,
    >,
) {
    let services = client
        .services_by_uuid(&GENERIC_MEDIA_CONTROL.into())
        .await
        .unwrap();

    let service = services.first().unwrap().clone();
    info!("Fetched service!");

    let mcp_chr = client
        .characteristic_by_uuid::<u8>(&service, &MEDIA_CONTROL_POINT.into())
        .await
        .unwrap();
    info!("Fetched charactertistic");

    // Extremely weird fix: for some reason, trouble doesn't allow GATT requests at once and will
    // timeout. Using mutex to ensure these two futures are mutually exclusive

    let one_only: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

    join(
        async {
            loop {
                let lock = one_only.lock().await;

                let track_title_chr = client
                    .characteristic_by_uuid::<str>(&service, &TRACK_TITLE.into())
                    .await
                    .unwrap();

                let mut data = [0u8; 64];
                let amnt = client
                    .read_characteristic(&track_title_chr, &mut data)
                    .await
                    .unwrap();

                let track_title = core::str::from_utf8(&data[..amnt]).unwrap_or("");

                info!("Track title: {}", track_title);

                let track_duration_chr = client
                    .characteristic_by_uuid::<i32>(&service, &TRACK_DURATION.into())
                    .await
                    .unwrap();
                let mut data = [0u8; 64];
                let _ = client
                    .read_characteristic(&track_duration_chr, &mut data)
                    .await
                    .unwrap();

                // TODO: Maybe check?
                let track_duration =
                    i32::from_le_bytes(data[..4].try_into().unwrap()) as f32 * 0.01;

                let track_position_chr = client
                    .characteristic_by_uuid::<i32>(&service, &TRACK_POSITION.into())
                    .await
                    .unwrap();
                let mut data = [0u8; 64];
                let _ = client
                    .read_characteristic(&track_position_chr, &mut data)
                    .await
                    .unwrap();

                let mut data = [0u8; 64];
                client
                    .read_characteristic_by_uuid(&service, &MEDIA_STATE.into(), &mut data)
                    .await
                    .unwrap();
                let playing = *data.first().unwrap_or(&0u8) == 0x01u8;

                // TODO: Maybe check?
                let track_position =
                    i32::from_le_bytes(data[..4].try_into().unwrap()) as f32 * 0.01;
                info!("Track pos: {}", track_position);

                draw_screen(
                    track_title,
                    track_position / track_duration,
                    playing,
                    display,
                )
                .await;

                drop(lock);

                Timer::after_secs(1).await;
            }
        },
        async {
            loop {
                if button_matrix.0.is_low() {
                    info!("left");

                    let _lock = one_only.lock().await;
                    client
                        .write_characteristic(&mcp_chr, &[0x30])
                        .await
                        .unwrap();
                }
                if button_matrix.1.is_low() {
                    info!("mid");

                    let _lock = one_only.lock().await;
                    let mut buf = [0u8; 8];
                    client
                        .read_characteristic_by_uuid(&service, &MEDIA_STATE.into(), &mut buf)
                        .await
                        .unwrap();

                    // Playing
                    let opcode = if *buf.first().unwrap_or(&0u8) == 0x01u8 {
                        0x02
                    } else {
                        0x01
                    };

                    client
                        .write_characteristic(&mcp_chr, &[opcode])
                        .await
                        .unwrap();
                }
                if button_matrix.2.is_low() {
                    info!("right");

                    let _lock = one_only.lock().await;
                    client
                        .write_characteristic(&mcp_chr, &[0x31])
                        .await
                        .unwrap();
                }

                Timer::after_millis(20).await;
            }
        },
    )
    .await;
}

async fn advertise<'values, 'server, C: Controller>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> GattConnection<'values, 'server, DefaultPacketPool> {
    let mut adv_data = [0u8; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(b"Widget"),
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
    rtt_init_defmt!();

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

    // Storage logic
    let mut map_storage =
        MapStorage::<StoredAddr, _, _>::new(flash, MapConfig::new(storage_range), NoCache::new());

    info!("Storage Initialized");

    // GPIO Buttons
    let config = InputConfig::default().with_pull(Pull::Up);
    let button_left = Input::new(peripherals.GPIO16, config);
    let button_mid = Input::new(peripherals.GPIO17, config);
    let button_right = Input::new(peripherals.GPIO18, config);

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
    }

    stack.set_io_capabilities(IoCapabilities::NoInputNoOutput);
    let host = stack.build();
    let mut runner = host.runner;
    let mut peripheral = host.peripheral;

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Widget",
        appearance: &appearance::TAG,
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

    info!("Starting!");

    let _ = join(host_runner_task(&mut runner), async {
        loop {
            let conn = advertise(&mut peripheral, &server).await;

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
                            info!("Session now encrypted!");
                            break;
                        }
                        Timer::after_secs(2).await;
                    }

                    app_logic(
                        (&button_left, &button_mid, &button_right),
                        &client,
                        &mut display,
                    )
                    .await;
                }),
            )
            .await;
        }
    })
    .await;

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}
