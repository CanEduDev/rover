project(
    'Rover',
    'c',
    meson_version: '>=1.5.2',
    version: '0.1',
    default_options: [
        'optimization=2',
        'default_library=static',
        'warning_level=3',
        'werror=true',
        'c_std=c17',
    ],
)

add_project_arguments('-DLFS_NO_MALLOC', language: 'c')
add_project_arguments('-O0', '-g', language: 'c', native: true)

cc_native = meson.get_compiler('c', native: true)
math_dep = cc_native.find_library('m')

objcopy = find_program('objcopy')
size = find_program('size')
clang_tidy = find_program('clang-tidy')
cat = find_program('cat')
python = find_program('python3')
openocd = find_program('openocd')

stm32f3_hal_dep = dependency('stm32f3-hal')
freertos_dep = dependency('freertos')
littlefs_dep = dependency('littlefs')
fff_dep = dependency('fff')

common_deps = [stm32f3_hal_dep, freertos_dep, littlefs_dep]

# Populated in subdirs
release_files = []
tidy_files = []

# Order is important
fs = import('fs')
subdir('libs')
subdir('apps')
subdir('bootloader')
subdir('docs')

# Build binaries with bootloader. Creates binaries with both the bootloader and the app, using cat.
foreach app_bin : app_binaries
    bin_name = app_bin.full_path().split('/')[-1].split('.')[0]
    release_files += custom_target(
        '@0@-with-bootloader.bin'.format(bin_name),
        build_by_default: true,
        capture: true,  # This makes stdout of cat be captured and written to output file.
        command: [cat, '@INPUT@'],
        input: [bootloader_bin, app_bin],
        output: ['@0@-with-bootloader.bin'.format(bin_name)],
        depends: [bootloader_bin, app_bin],
    )
endforeach

# SWD flash targets for all elf files
foreach elf : app_elfs + [bootloader_elf]
    elf_name = elf.name().split('.')[0]
    run_target(
        'swd-flash-@0@'.format(elf_name),
        command: [
            openocd,
            '-f',
            files('openocd.cfg'),
            '-c',
            'program @0@ verify'.format(bootloader_elf.full_path()),
            '-c',
            'program @0@ verify reset exit'.format(elf.full_path()),
        ],
        depends: [bootloader_elf, elf],
    )
endforeach

# Override default meson clang-tidy targets
run_target(
    'clang-tidy',
    command: [
        clang_tidy,
        '--config-file',
        meson.project_source_root() / '.clang-tidy',
        '--quiet',
        '-p',
        meson.project_build_root(),
        tidy_files,
    ],
)

run_target(
    'clang-tidy-fix',
    command: [
        clang_tidy,
        '--fix-errors',
        '-extra-arg=-ferror-limit=0',
        '--config-file',
        meson.project_source_root() / '.clang-tidy',
        '--quiet',
        '-p',
        meson.project_build_root(),
        tidy_files,
    ],
)

run_target(
    'check',
    command: [
        files('scripts' / 'check-src.sh'),
        meson.project_source_root(),
        meson.project_build_root(),
    ],
)

# Binary size reporting
run_target(
    'size',
    command: [size, app_elfs, bootloader_elf],
    depends: [app_elfs, bootloader_elf],
)

config_dir = custom_target(
    'config',
    output: 'config',
    command: [
        python,
        files('scripts' / 'gen-system-conf.py'),
        '--force',
        '--battery-monitor-bin',
        battery_monitor_bin,
        '--light-array-bin',
        light_array_bin,
        '--obstacle-detector-bin',
        obstacle_detector_bin,
        '--sbus-receiver-bin',
        sbus_receiver_bin,
        '--servo-bin',
        servo_bin,
        '--wheel-bin',
        wheel_bin,
        '-o',
        '@OUTPUT@',
    ],
    build_always_stale: true,
)

release_script = files('scripts' / 'create-release.sh')
release_files += fs.copyfile('rover.dbc')
flasher_path = meson.project_source_root() / 'rover_py'
release_dir = 'release'

release_tgt = custom_target(
    'release',
    input: release_files,
    output: release_dir,
    command: [
        release_script,
        '--config-dir',
        config_dir,
        '--release-dir',
        release_dir,
        '--flasher-path',
        flasher_path,
        '@INPUT@',
    ],
    build_always_stale: true,
    build_by_default: true,
    depends: config_dir,
)

run_target('ros-gateway', command: [files('scripts' / 'build-ros-gateway.sh')])

# Try to find python3.exe first, in case we're running in WSL.
python_flash = find_program('python3.exe', 'python3', required: false)

run_target(
    'can-flash-all',
    command: [
        python_flash,
        release_dir / 'fw_update.py',
        'system',
        '--config',
        config_dir.full_path() / 'system.json',
        '--binary-dir',
        release_dir / 'binaries',
    ],
    depends: [config_dir, release_tgt],
)
