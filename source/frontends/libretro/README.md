# libretro core

There is a [libretro](https://docs.libretro.com/development/cores/developing-cores/) core.

## RetroPad

A retropad can be plugged in port 1 (with or without analog stick).

| Device | Axis | ID | Default action | Remapped |
| ------ | ------ | ------- | ---- | --- |
| Standard | | A | Button 0 | |
| Standard | | B | Button 1 | |
| Standard | | X | | |
| Standard | | Y | | |
| Standard | | Left | PDL(0) = 0 | |
| Standard | | Right | PDL(0) = 255 | |
| Standard | | Up | PDL(1) = 0 | |
| Standard | | Down | PDL(1) = 255 | |
| Standard | | L | | Space |
| Standard | | R | | Enter |
| Standard | | L2 | | Left|
| Standard | | R2 | | Right|
| Standard | | L3 | | |
| Standard | | R3 | | |
| Standard | | Select | | |
| Standard | | Start | | |
| Analog | Left | X | PDL(0) | |
| Analog | Left | Y | PDL(1) | |
| Mouse  | | Left | Button 0 | |
| Mouse  | | Right | Button 1 | |
| Mouse  | | X | PDL(0) | |
| Mouse  | | Y | PDL(1) | |

All *standard* buttons can be remapped to keys (in which case they lose their default action): buttons are remapped on all connected devices.

The speed of the mouse pointer can be tuned with an option.

## Keyboard

* ``END``: equivalent to ``F2`` to reset the machine
* ``HOME``: save configuration to `/tmp/applewin.retro.conf`

In order to have a better experience with the keyboard, one should probably enable *Game Focus Mode* (normally Scroll-Lock) to disable hotkeys. Even better set *Auto Enable 'Game Focus' Mode* to *Detect*.

## Misc

Easiest way to run from the ``build`` folder:
``retroarch -L source/frontends/libretro/applewin_libretro.so ../bin/MASTER.DSK``

The core can be statically linked in Linux and MSYS2, pass `-DSTATIC_LINKING=ON` to `cmake`, or disable networking `-DENABLE_NETWORKING=OFF`.

## M3U Playlists and Disk Control

The core cupports libretro [Disk Control Interface](https://docs.libretro.com/guides/disc-swapping/), allowing disk swaps from the frontend UI (the drive is selectable via a core option).

### M3U format

A `.m3u` file lists disk images, one per line. Paths are relative to the M3U file's directory, similar to [VICE](https://docs.libretro.com/library/vice/#m3u-and-disk-control).

```
Ultima V - Disk A.dsk
Ultima V - Disk B.dsk
```

### Supported directives

| Directive | Description |
| --------- | ----------- |
| `#` | Comment (line is ignored) |
| `#SAVEDISK:` | Creates a writable blank disk in the save directory |
| `#LABEL:` | Sets the display label for the next file entry |
| `#EXTINF:` | Standard M3U extended info - label follows the first comma |
| `\|` (pipe in filename) | `path.dsk\|Label` sets a display label for that entry |

Label priority: `#LABEL:` / `#EXTINF:` override pipe labels.

#### `#SAVEDISK:`

Adds a writable save disk to the playlist. The disk image is created (as a blank `.dsk`) in the save directory on first use and reused on subsequent launches.

```
Game - Side A.dsk
Game - Side B.dsk
#SAVEDISK:Character
#SAVEDISK:
```

The label after `#SAVEDISK:` is optional. If omitted, an automatic index is used. The resulting label shown in the frontend is always prefixed with "Save Disk ".

Note: the created disk is unformatted (all zeros). Most Apple II games that request a save disk will format it themselves.

#### Pipe-delimited labels

```
Game - Side A (Disk 1 of 3).dsk|Side A
Game - Side B (Disk 2 of 3).dsk|Side B
```

The text after `|` becomes the display label in the frontend's disk control UI instead of the filename stem.

### MultiDrive

When enabled, the second disk in the playlist is automatically inserted into DRIVE_2 at load time. This is useful for games that expect both drives populated (e.g. Ultima, Wizardry).

MultiDrive is triggered by either:
- Naming the M3U file with `(MD)` in the stem, e.g. `Ultima V (MD).m3u`
- Enabling the **Floppy MultiDrive** core option

Save disks (`#SAVEDISK:`) are never auto-inserted into DRIVE_2.

### Core options

| Option | Values | Description |
| ------ | ------ | ----------- |
| Disk Control Drive | Drive 1, 2 | Active drive for the Disk Control Interface. |
| Playlist Start Disk | First, Previous | Whether to start from disk 1 or resume from the previously used disk |
| Floppy MultiDrive | Disabled, Enabled | Auto-insert second disk into DRIVE_2 for all playlists |
| Write-protect Game Disks | Enabled, Disabled | When disabled, a game can save to its own disks: each disk is copied to the save folder and written there, and the original image is left unchanged |
