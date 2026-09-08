use super::result::SendResult;
use super::spi::Spi;
use super::wap::Wap;
use crate::unwrap_send;
use num_enum::{FromPrimitive, IntoPrimitive};

#[derive(IntoPrimitive, FromPrimitive, PartialEq)]
#[repr(u8)]
enum Command {
    #[num_enum(default)]
    None = 0x00,
    Init1 = 0x10,
    Init2 = 0x3d,
    Unknown = 0x11,
    GetSomeValue = 0x13,
    Broadcast = 0x16,
    Setup = 0x17,
    StartHost = 0x19,
    AcceptConnections = 0x1a,
    EndHost = 0x1b,
    BroadcastReadStart = 0x1c,
    BroadcastReadPoll = 0x1d,
    BroadcastReadEnd = 0x1e,
    Connect = 0x1f,
    IsConnecting = 0x20,
    FinishConnecting = 0x21,
    SendData = 0x24,
    SendDataAndWait = 0x25,
    ReceiveData = 0x26,
    ReceiveDataAndWait = 0x27,
    ReceiveDataAndWaitResponse = 0x28,
}

pub struct Router {
    wap: Wap,
    /// GetSomeValue (0x13) alternates between two values on successive calls, matching a
    /// real adapter (upstream web-app router.ts + pico_host.py PeerSim).
    some_value_high: bool,
    /// Connect handshake state for the relayed peer: report "connecting" a few polls, then
    /// "connected". Matches PeerSim's connect_polls model.
    connecting_polls: u8,
    connected: bool,
}

impl Router {
    pub fn new(spi: Spi) -> Self {
        Self {
            wap: Wap::new(spi),
            some_value_high: false,
            connecting_polls: 0,
            connected: false,
        }
    }

    fn login(&mut self) {
        self.wap.login()
    }

    fn reset(&mut self) {
        self.connecting_polls = 0;
        self.connected = false;
        self.wap.reset();
    }

    /// Answer a post-login adapter command LOCALLY, like a real Wireless Adapter does. This
    /// is what lets the GBA enter the Union Room with NO peer/Switch connected — the default
    /// hardware behaviour: control commands succeed, peer-discovery reports "no peers", and
    /// the player simply waits in the room. Crucially it NEVER blocks on an external host,
    /// which is what caused the connection errors (the old code waited on a USB reply that,
    /// with no Linux/Switch answering, never came).
    ///
    /// Writes reply words into `out` and returns the count. Peer/trade DATA now comes from the
    /// ESP32 (Switch side) over the UART relay (`crate::relay`): when the Switch is present the
    /// relay reports a peer, so BroadcastReadPoll lists it and the GBA can connect. With no
    /// relayed peer these return empty = "solo, no peers", so standalone entry still works.
    fn local_respond(&mut self, command: Command, out: &mut [u32]) -> usize {
        match command {
            Command::GetSomeValue => {
                // 0x13: alternate 0x0200abcd / 0x00000000.
                self.some_value_high = !self.some_value_high;
                out[0] = if self.some_value_high { 0x0200abcd } else { 0x00000000 };
                1
            }
            Command::Unknown => {
                // 0x11: adapter returns 0x000000ff.
                out[0] = 0x000000ff;
                1
            }
            // Peer discovery: list the Switch-relayed peer if present (peer_id + 6-word beacon),
            // else empty ("nobody here"). The GBA infers peer count from the word count / 7.
            Command::BroadcastReadPoll | Command::BroadcastReadEnd => crate::relay::get_peer(out),
            Command::Connect => {
                // Begin connecting to the relayed peer (if any).
                self.connecting_polls = 0;
                self.connected = false;
                0
            }
            Command::IsConnecting | Command::FinishConnecting => match crate::relay::peer_id() {
                Some(id) => {
                    if !self.connected && self.connecting_polls < 3 {
                        self.connecting_polls += 1;
                        out[0] = 0x01000000; // still connecting
                        1
                    } else {
                        self.connected = true;
                        out[0] = id; // connected: report the peer id
                        1
                    }
                }
                None => 0,
            },
            // Received trade data from the Switch (relayed), served to the GBA. 0x26 ReceiveData
            // and 0x28 ReceiveDataAndWaitResponse both pull the latest relayed slot.
            Command::ReceiveData | Command::ReceiveDataAndWaitResponse => crate::relay::take_slot(out),
            // Everything else (Init/Setup/Broadcast/StartHost/AcceptConnections/…): a bare
            // success ack with no data, same as a real adapter acknowledging the command.
            _ => 0,
        }
    }

    fn handle_req(&mut self, _res_buf: &mut [u32]) -> SendResult<()> {
        let req = unwrap_send!(self.wap.recv_req());
        let command = Command::from(req.command());

        // Ignore responses
        if req.is_response() {
            return SendResult::Data(());
        }

        // Mirror every outgoing command to the ESP32 (Switch side) and to USB for logging.
        // Both are fire-and-forget: a real adapter answers within the GBA's ~800us deadline,
        // so we must NOT wait on any external round-trip here.
        crate::uart_link::send32(req.raw());
        crate::serial_usb::send_only32(req.raw());

        match command {
            Command::SendDataAndWait => {
                // 0x25 = ID_DATA_TX_AND_CHANGE_REQ: the child's outgoing 14-byte trade slot
                // rides here (already forwarded above). Fake the completion + clock-change
                // locally so the GBA's timing is met.
                self.wap.async_ack()
            }
            Command::ReceiveDataAndWait => {
                // 0x27 = ID_MS_CHANGE_REQ: payload-less clock master/slave change.
                self.wap.async_ack()
            }
            // All other commands: answer locally, immediately (no blocking on USB/Switch).
            _ => {
                let mut reply = [0u32; 8];
                let n = self.local_respond(command, &mut reply);
                self.wap.reply_req(&reply[..n])
            }
        }
    }

    pub fn run(&mut self) -> ! {
        let mut res_buf = [0u32; 0xff];

        loop {
            self.reset();
            self.login();

            loop {
                if self.handle_req(&mut res_buf) == SendResult::Reset {
                    break;
                };
                // Pull the ESP32->Pico relay stream (peer presence / received slots) here on
                // core1, between GBA commands, so UART access never contends with core1's own
                // timing-critical SPI path or with core0.
                crate::relay::poll();
            }
        }
    }
}
