# USB speaker setup (demo audio)

The demo plays its prompts with pygame, which sends sound to the Pi's **default audio output**. The code doesn't pick
a device. If the default is HDMI (the screens), the prompts play where you can't hear them, and nothing shows up in
the logs.

Do this once per Pi (the setting is saved), and again whenever the speaker goes quiet.

All commands work over SSH. SSH in as the same user that's logged in to the Pi's desktop (`heartbridge`). Check with
`whoami`.

## 1. Check where the sound is going

```bash
wpctl status
```

Under **Audio > Sinks**, `*` marks the default output. This is the problem:

```
 ├─ Sinks:
 │  *   35. Built-in Audio Digital Stereo (HDMI) [vol: 1.00]
 │      57. Built-in Audio Stereo               [vol: 1.00]
 │      67. USB PnP Audio Device Analog Stereo  [vol: 1.50]
```

Sound goes to HDMI (35), not the USB speaker (67).

## 2. Make the USB speaker the default

```bash
wpctl set-default 67    # the USB speaker's number in wpctl status
```

The number can change after a reboot, so read it from `wpctl status` first.

The choice is saved. In `wpctl status`, **Settings > Default Configured Devices** should now name the USB device
(`alsa_output.usb-...`) instead of `...hdmi-stereo`. If the speaker is unplugged, the Pi falls back to HDMI, and it
switches back when the speaker is plugged in again.

On the Pi's desktop you can do the same thing by right-clicking the speaker icon in the taskbar and choosing
**USB PnP Audio Device**.

## 3. Test without the demo

From the `Raspberry Pi code` folder:

```bash
pw-play src/Audio/startup.wav
```

If you hear it, the demo will play too. If this is silent as well, the code isn't the problem: see Troubleshooting.

## 4. Volume

Keep the speaker at 1.0 (100%) in software. Above 100% the signal is boosted digitally, which can distort the prompts.

```bash
wpctl set-volume 67 1.0
```

Turn up the speaker's own volume instead.

## Running the demo

```bash
cd "Raspberry Pi code"
python demo/main.py
```

Don't start it with `sudo`. That runs it as root, which can't reach your user's sound system.

## Troubleshooting

Is the speaker detected?

```bash
aplay -l    # should list: card N: Device [USB PnP Audio Device]
```

What pygame can see (should include `USB PnP Audio Device Analog Stereo`):

```bash
python3 -c "import pygame; pygame.init(); import pygame._sdl2.audio as a; print(a.get_audio_device_names(False))"
```

USB connections and power. The speaker shares the Pi's USB power with the USB-CAN adapter, which already drops out
when the screens are connected:

```bash
lsusb                                                  # every USB device
lsusb -t                                               # which port each device is on, and its speed
sudo lsusb -v 2>/dev/null | grep -E "^Bus|MaxPower"    # current each device asks for
sudo dmesg -w                                          # live plug/unplug, "over-current" and "disconnect" messages
ls -l /dev/serial/by-id/                               # the USB-CAN adapter
vcgencmd get_throttled                                 # anything other than 0x0 means under-voltage
```

Leave `sudo dmesg -w` running while you plug in the screens. If the speaker or the CAN adapter disconnects at that
moment, it's a power problem, not the code.
