"""User-facing help text, kept separate from shell routing."""

OVERVIEW = """Incident Recorder Host Tool
Commands are either HOST commands (run on this PC) or DEVICE commands (sent to the
recorder over UART). Type 'help <command>' for details, e.g. 'help config'.

SEE WHAT HAPPENED
  status                    device state, self-check result, UART error counters
  journal [N]               last N incidents, newest first (default 5, max 30)
  journal id ID             one incident by its ID
  journal all               every stored incident (device must be DISARMED)
  photos                    recent incidents and whether each one has a photo

DOWNLOAD PHOTOS  (device must be DISARMED; files go to the output folder)
  photos list               table of downloadable photos (+ photo_index.json)
  photo id ID               download one photo as PNG
  photo all                 download every photo that is not saved yet

CHANGE THRESHOLDS  (device must be DISARMED and UNSEALED)
  config                    show current mode and thresholds
  config mode cargo|site    choose the working mode
  config cargo shock MG | tilt DEG | temp MIN MAX
  config site vibration MG | temp MIN MAX
  unseal / seal             unlock / lock the settings with a password
                            (just type the word; the password is asked with * echo)

MAINTENANCE
  photo status              camera and photo storage status
  photo selftest            read-only photo metadata/CRC self-test
  led test                  blink the incident LED pattern
  erase config|media|all    wipe data (asks for the password and a YES)

THIS PC
  ports                     list serial ports (FT232 adapter is marked)
  connect [PORT] | disconnect | reconnect
  settings [show | set KEY VALUE | save | load | reset]
  log on [FILE] | log off | log show
  raw COMMAND               send a command to the device exactly as typed
  help [topic]  |  exit

More: help device | photo | settings | <any command above>
"""

HOST = OVERVIEW

DEVICE = """Device commands (sent to the STM32 CLI, USART1 115200 8-N-1):
  status                      state, startup self-check, error counters
  config                      show Cargo/Site profiles and storage status
  config mode cargo|site
  config cargo shock MG|tilt DEG|temp MIN MAX
  config site vibration MG|temp MIN MAX
  journal [1..30] | journal id ID | journal all
  photos [1..30] | photos list
  photo status | photo id ID | photo all | photo selftest
  seal PASSWORD | unseal PASSWORD     (8..32 printable characters, no spaces)
  erase config|media|all PASSWORD     (needs SEALED configuration and a YES)
  led test
Only DISARMED devices accept configuration, photo download and erase.
"""

SETTINGS = """PC-side settings (these affect only this program, not the device):
  settings                      show the current values
  settings set KEY VALUE        change a value for this session
  settings save                 write the values to ~/.incident_recorder/config.json
  settings load                 reload them from that file
  settings reset                back to defaults (in memory; 'settings save' to keep)

Keys and defaults:
  port=auto                     serial port name, or 'auto' to find the FT232
  baud=115200
  read_timeout=1.0              serial read slice, seconds
  command_timeout=5.0           time allowed for a normal command to answer
  photo_timeout=5.0             max silence between photo bytes
  retries=3                     photo retries after the first attempt (0..3)
  rgb565_byte_order=big|little
  keep_rgb565=false             also keep the raw RGB565 file
  keep_metadata=false           also write event_ID_WxH.json (CRC, size, PNG hash) next to each PNG
  output_dir=incident_photos    where PNG files and photo_index.json are written
  png_scale=1                   enlarge PNG by an integer factor (1..16)
  overwrite=false               replace files that already exist
  auto_reconnect=true  reconnect_timeout=30.0  reconnect_interval=1.0
  log_file=null                 session log file; 'log on'/'log off' change it too (passwords are never written)
Command-line flags (--port, --baud ...) override the saved values for one run.
"""

PHOTO = """Photos:
  photos            quick list of recent incidents; says if a photo exists and, if not, why
  photos list       table of all downloadable photos, also saved as photo_index.json
  photo id ID       download one photo -> <output_dir>/event_ID_WxH.png
  photo all         download all photos; files that are already valid are skipped
  photo list        same as 'photos list'

If an incident has no photo (for example because of the 3 s cooldown) the device says
why and nothing is downloaded. Ctrl+C during a download cancels it safely.
Protocol: IRPHOTO4 (per-chunk ACK/NACK + CRC32); PNG is written atomically.
"""

TOPICS = {
    "host": OVERVIEW,
    "overview": OVERVIEW,
    "device": DEVICE,
    "settings": SETTINGS,
    "photo": PHOTO,
    "photos": PHOTO,
    "status": "status\n  Shows DISARMED/ARMED state, startup self-check, how long the device has run and\n  UART error counters (rx_overflow, tx_drop ...). Counters should stay 0.\n",
    "config": """config                          show mode, thresholds and whether settings are sealed
config mode cargo|site          choose the working mode
config cargo shock MG           shock threshold, 100..16000 mg
config cargo tilt DEG           tilt threshold, 1..180 degrees
config cargo temp MIN MAX       allowed temperature range, -40..85 C, MIN < MAX
config site vibration MG        vibration RMS threshold, 100..16000 mg
config site temp MIN MAX        allowed temperature range, -40..85 C

Changes need a DISARMED device and an UNSEALED configuration ('unseal' first).
The tool waits until the device confirms that the change was saved to flash.
""",
    "journal": """journal [N]        newest N incidents (1..30, default 5)
journal id ID      one incident by its persistent ID
journal all        every stored incident, newest first (device must be DISARMED)
""",
    "seal": """seal / unseal
  Type the word alone, e.g. 'unseal'. The password is requested with * echo.
  'seal' asks for the new password twice. Password: 8..32 printable characters,
  no spaces. 'unseal' lets you change thresholds; 'seal' locks them again
  (erase also requires a SEALED configuration).
  Typing 'unseal PASSWORD' on one line works too, but your terminal then shows the
  password in clear text.
""",
    "unseal": None,  # filled below
    "erase": """erase config|media|all
  config  restore factory thresholds and erase the old password
  media   erase the whole journal and photo area
  all     both
  Requires DISARMED and SEALED configuration. You are asked for the password and must
  then type YES. While erasing, the device ignores input; progress is shown.
""",
    "led": "led test\n  Runs the incident LED pattern once (device must be DISARMED).\n",
    "connect": "connect [PORT]\n  Opens the serial port from the settings (port=auto looks for the FT232),\n  or PORT (for example COM4 or /dev/ttyUSB0), and synchronises with the device.\n  Related: ports, disconnect, reconnect.\n",
    "raw": "raw COMMAND\n  Sends COMMAND to the device without any host-side checks or formatting.\n  Not for 'erase' (use the erase command) or photo/journal downloads.\n",
    "log": "log on [FILE] | log off | log show\n  Writes everything shown in this session to FILE. Passwords are never logged.\n",
}
TOPICS["unseal"] = TOPICS["seal"]
